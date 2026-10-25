/*
 * ESP32 4WD Line Follower + Manual Control + Ultrasonic Obstacle Stop
 * + RFID Delivery (X -> Y -> base) with J1 junction
 * -------------------------------------------------------------------
 * Motor driver: L298N (Direct PWM on INA/INB/INC/IND)
 * IR sensor: 5-channel array (32, 33, 34, 35, 39)
 * Ultrasonic: HC-SR04 (TRIG = 25, ECHO = 26)
 * RFID (RC522): SS=5, RST=17, SCK=21, MOSI=13, MISO=14, VCC=3.3V
 * WiFi AP: LineBott_ESP32 / 12345678 -> http://192.168.4.1
 * Requires ESP32 Arduino core 3.x + "MFRC522" library
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>

/* ================= ROBOT STATE ================= */
enum RobotState {
  STATE_STANDBY,
  STATE_GOING_TO_X,
  STATE_AT_X,
  STATE_GOING_TO_Y,
  STATE_AT_Y,
  STATE_RETURNING
};

/* ================= TIMED PHASES ================= */
// PH_NONE : normal line follow (IR active)
// PH_WAIT : robot stopped for 3 s after a station tag, IR ignored
// PH_TURN : timed junction spin, IR ignored
// PH_UTURN: timed 180 degree spin, IR ignored
enum Phase { PH_NONE, PH_WAIT, PH_TURN, PH_UTURN };
enum WaitReason { WAIT_NONE, WAIT_J1, WAIT_X, WAIT_Y };

Phase phase = PH_NONE;
WaitReason waitReason = WAIT_NONE;
unsigned long phaseStart = 0;
unsigned long phaseLen   = 0;
char turnDir = 'S';                        // 'L', 'R' for timed spins
RobotState stateAfterUTurn = STATE_STANDBY;

const unsigned long STATION_WAIT_MS = 3000;   // delay after any station tag

/* ================= JUNCTION ROUTE TABLE =================
 * L = turn left, R = turn right, S = go straight
 * STEM   = robot reaches J1 from the base side (going out)
 * BRANCH = robot reaches J1 from the X or Y side (after U-turn)
 */
char ROUTE_STEM_TO_X      = 'L';
char ROUTE_STEM_TO_Y      = 'R';
char ROUTE_STEM_TO_BASE   = 'S';
char ROUTE_BRANCH_TO_X    = 'L';
char ROUTE_BRANCH_TO_Y    = 'L';
char ROUTE_BRANCH_TO_BASE = 'S';

bool comingFromBranch = false;   // true after a U-turn, until J1 is passed

/* ================= WIFI ================= */
const char* AP_SSID = "LineBott_ESP32";
const char* AP_PASS = "12345678";
WebServer server(80);

/* ============== 5-CHANNEL IR SENSOR PINS (Left -> Right) ============== */
const int irPin[5] = {32, 33, 34, 35, 39};
int irVal[5];

bool BLACK_LINE = true;

/* ============== MOTOR DRIVER PINS ============== */
#define INA 18
#define INB 19
#define INC 22
#define IND 23

#define PWM_FREQ 5000
#define PWM_RES  8

bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;
int  STEER_DIR    = 1;

/* ============== ULTRASONIC PINS ============== */
#define TRIG_PIN 25
#define ECHO_PIN 26

const float OBSTACLE_DISTANCE_CM = 10.0;
const unsigned long US_READ_INTERVAL = 60;

float distanceCM = 999;
bool  obstacleDetected = false;
unsigned long lastUSRead = 0;

/* ============== RFID PINS ============== */
#define RFID_SS   5
#define RFID_RST  17
#define RFID_SCK  21
#define RFID_MOSI 13
#define RFID_MISO 14

MFRC522 rfid(RFID_SS, RFID_RST);
bool rfidReady = false;

/* ============== RFID TAGS ============== */
struct Station {
  const char* name;
  byte uid[4];
};

Station tags[] = {
  {"J1", {0x3E, 0xA3, 0x46, 0x03}},   // blue tag 1 = junction
  {"J2", {0x0E, 0x17, 0x45, 0x03}},   // blue tag 2 = spare junction
  {"X",  {0x64, 0xD5, 0x30, 0x03}},   // white card 1 = sender
  {"Y",  {0xE9, 0x8F, 0x36, 0x03}}    // white card 2 = receiver
};
const int TAG_COUNT = sizeof(tags) / sizeof(tags[0]);

String lastTag = "--";
const unsigned long RFID_POLL_INTERVAL = 30;
const unsigned long SAME_TAG_IGNORE    = 2000;
unsigned long lastRFIDPoll = 0;
unsigned long lastTagTime  = 0;
byte lastUid[4];
bool haveLastUid = false;

/* ============== DELIVERY (AUTO) STATE ============== */
RobotState robotState = STATE_STANDBY;
bool autoModeActive   = false;
String deliveryMsg    = "Idle";

unsigned long JUNCTION_TURN_MS = 500;   // J1 timed spin, tune on track
unsigned long UTURN_MS         = 1000;  // 180 spin, tune on track

// Base marker: all 5 sensors black for this long
unsigned long baseMarkerStart = 0;
bool baseMarkerTiming = false;
const unsigned long BASE_MARKER_MS = 150;

/* ============== CONTROL VARIABLES ============== */
String mode = "STOP";          // LINE / MANUAL / STOP
String manualCmd = "S";        // F B L R S
int baseSpeed   = 150;
int manualSpeed = 180;

float Kp = 35.0, Ki = 0.0, Kd = 20.0;
float lastError = 0, integral = 0;
float lastKnownError = 0;

/* ================= MOTOR HELPERS ================= */
void leftMotor(int spd) {
  spd = constrain(spd, -255, 255);
  if (INVERT_LEFT) spd = -spd;
  if (spd >= 0) { ledcWrite(INA, spd);   ledcWrite(INB, 0); }
  else          { ledcWrite(INA, 0);     ledcWrite(INB, -spd); }
}

void rightMotor(int spd) {
  spd = constrain(spd, -255, 255);
  if (INVERT_RIGHT) spd = -spd;
  if (spd >= 0) { ledcWrite(INC, spd);   ledcWrite(IND, 0); }
  else          { ledcWrite(INC, 0);     ledcWrite(IND, -spd); }
}

void driveStop() {
  leftMotor(0);
  rightMotor(0);
}

void spinLeft(int s)  { leftMotor(-s); rightMotor(s); }
void spinRight(int s) { leftMotor(s);  rightMotor(-s); }

/* ================= ULTRASONIC ================= */
float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return 999;

  return duration * 0.0343 / 2.0;
}

void updateUltrasonic() {
  unsigned long now = millis();
  if (now - lastUSRead < US_READ_INTERVAL) return;
  lastUSRead = now;

  distanceCM = readUltrasonicCM();
  obstacleDetected = (distanceCM > 0 && distanceCM <= OBSTACLE_DISTANCE_CM);
}

/* ================= SENSOR READ (5-ARRAY) ================= */
const float weight[5] = {-4.0, -2.0, 0.0, 2.0, 4.0};

bool readSensors(float &error) {
  float sum = 0;
  int count = 0;
  for (int i = 0; i < 5; i++) {
    int raw = digitalRead(irPin[i]);
    irVal[i] = BLACK_LINE ? (raw == LOW ? 1 : 0) : (raw == HIGH ? 1 : 0);
    if (irVal[i]) {
      sum += weight[i];
      count++;
    }
  }
  if (count == 0) return false;
  error = sum / count;
  return true;
}

/* ================= LINE FOLLOW ================= */
void lineFollow() {
  float error;
  if (!readSensors(error)) {
    int s = baseSpeed * 0.8;
    if (lastKnownError * STEER_DIR < 0) { leftMotor(-s); rightMotor(s);  }
    else                                { leftMotor(s);  rightMotor(-s); }
    return;
  }
  lastKnownError = error;

  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  correction *= STEER_DIR;

  leftMotor(baseSpeed + correction);
  rightMotor(baseSpeed - correction);
}

/* ================= MANUAL DRIVE ================= */
void manualDrive() {
  int s = manualSpeed;
  if      (manualCmd == "F") { leftMotor(s);  rightMotor(s);  }
  else if (manualCmd == "B") { leftMotor(-s); rightMotor(-s); }
  else if (manualCmd == "R") { leftMotor(-s); rightMotor(s);  }
  else if (manualCmd == "L") { leftMotor(s);  rightMotor(-s); }
  else driveStop();
}

/* ================= RFID HELPERS ================= */
String uidToText(byte *uid, byte size) {
  String s = "";
  for (byte i = 0; i < size; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
    if (i < size - 1) s += " ";
  }
  s.toUpperCase();
  return s;
}

int findStation(byte *uid, byte size) {
  if (size != 4) return -1;
  for (int i = 0; i < TAG_COUNT; i++) {
    if (memcmp(uid, tags[i].uid, 4) == 0) return i;
  }
  return -1;
}

const char* stateName(RobotState s) {
  switch (s) {
    case STATE_STANDBY:    return "STANDBY";
    case STATE_GOING_TO_X: return "GOING_TO_X";
    case STATE_AT_X:       return "AT_X";
    case STATE_GOING_TO_Y: return "GOING_TO_Y";
    case STATE_AT_Y:       return "AT_Y";
    case STATE_RETURNING:  return "RETURNING";
  }
  return "?";
}

// Which way to turn at J1, based on goal and arrival side
char junctionDirection() {
  if (robotState == STATE_GOING_TO_X) return comingFromBranch ? ROUTE_BRANCH_TO_X    : ROUTE_STEM_TO_X;
  if (robotState == STATE_GOING_TO_Y) return comingFromBranch ? ROUTE_BRANCH_TO_Y    : ROUTE_STEM_TO_Y;
  if (robotState == STATE_RETURNING)  return comingFromBranch ? ROUTE_BRANCH_TO_BASE : ROUTE_STEM_TO_BASE;
  return 'S';
}

/* ================= PHASE HELPERS ================= */
void beginWait(WaitReason r) {
  phase = PH_WAIT;
  waitReason = r;
  phaseStart = millis();
  phaseLen = STATION_WAIT_MS;
  driveStop();
  Serial.println("[WAIT] 3 s, IR ignored");
}

void beginTimedSpin(Phase p, unsigned long len, char dir) {
  phase = p;
  phaseStart = millis();
  phaseLen = len;
  turnDir = dir;
}

// Called when the 3 s station wait is over
void finishWait() {
  phase = PH_NONE;

  if (waitReason == WAIT_J1) {
    char d = junctionDirection();
    comingFromBranch = false;            // J1 is passed now
    if (d == 'S') {
      integral = 0; lastError = 0;
      deliveryMsg = "J1: going straight";
      Serial.println("[J1] Straight, IR on");
    } else {
      beginTimedSpin(PH_TURN, JUNCTION_TURN_MS, d);
      deliveryMsg = String("J1: turning ") + (d == 'L' ? "left" : "right");
      Serial.print("[J1] Turning: "); Serial.println(d);
    }
  }
  else if (waitReason == WAIT_X) {
    robotState = STATE_AT_X;
    deliveryMsg = "Arrived at X. Press ITEM LOADED.";
    Serial.println("[STATE] AT_X");
  }
  else if (waitReason == WAIT_Y) {
    robotState = STATE_AT_Y;
    deliveryMsg = "Arrived at Y. Press RECEIVED.";
    Serial.println("[STATE] AT_Y");
  }
  waitReason = WAIT_NONE;
}

void startUTurn(RobotState nextState) {
  stateAfterUTurn = nextState;
  beginTimedSpin(PH_UTURN, UTURN_MS, 'R');
  Serial.println("[UTURN] Starting 180-degree turn");
}

/* ================= RFID CHECK ================= */
void checkRFID() {
  if (!rfidReady) return;
  if (phase != PH_NONE) return;          // ignore tags during wait / timed moves

  unsigned long now = millis();
  if (now - lastRFIDPoll < RFID_POLL_INTERVAL) return;
  lastRFIDPoll = now;

  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  byte uid[4];
  byte size = rfid.uid.size;
  for (byte i = 0; i < 4 && i < size; i++) uid[i] = rfid.uid.uidByte[i];

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  if (haveLastUid && memcmp(uid, lastUid, 4) == 0 && now - lastTagTime < SAME_TAG_IGNORE) return;
  memcpy(lastUid, uid, 4);
  haveLastUid = true;
  lastTagTime = now;

  Serial.print("Card UID: "); Serial.println(uidToText(uid, size));

  int idx = findStation(uid, size);
  if (idx < 0) {
    lastTag = "Unknown";
    Serial.println("Unknown tag");
    return;
  }
  String name = tags[idx].name;
  lastTag = name;
  Serial.println("Station: " + name);

  if (!autoModeActive) {
    deliveryMsg = "Passed " + name;
    return;
  }

  if (name == "J1") {
    if (robotState == STATE_GOING_TO_X || robotState == STATE_GOING_TO_Y || robotState == STATE_RETURNING) {
      beginWait(WAIT_J1);
      deliveryMsg = "J1 found, waiting 3 s";
    }
  }
  else if (name == "X" && robotState == STATE_GOING_TO_X) {
    beginWait(WAIT_X);
    deliveryMsg = "X found, waiting 3 s";
  }
  else if (name == "Y" && robotState == STATE_GOING_TO_Y) {
    beginWait(WAIT_Y);
    deliveryMsg = "Y found, waiting 3 s";
  }
}

/* ================= BASE MARKER ================= */
bool checkBaseMarker() {
  bool allBlack = true;
  for (int i = 0; i < 5; i++) if (irVal[i] == 0) { allBlack = false; break; }

  if (allBlack) {
    if (!baseMarkerTiming) { baseMarkerTiming = true; baseMarkerStart = millis(); }
    if (millis() - baseMarkerStart >= BASE_MARKER_MS) return true;
  } else {
    baseMarkerTiming = false;
  }
  return false;
}

/* ================= WEB PAGE ================= */
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 LineBot (5-Array + RFID)</title>
<style>
 *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
 body{margin:0;font-family:system-ui,Arial;background:#101418;color:#e8eef2}
 header{padding:14px;text-align:center;background:#161d24;font-weight:600;letter-spacing:1px}
 .tabs{display:flex}
 .tab{flex:1;padding:14px;text-align:center;background:#1b242d;cursor:pointer;font-weight:600;font-size:14px}
 .tab.on{background:#0aa06e;color:#fff}
 .panel{display:none;padding:18px}
 .panel.on{display:block}
 button{border:0;border-radius:14px;font-size:17px;font-weight:700;color:#fff;padding:18px;width:100%}
 button:disabled{background:#2a3540;color:#6b7a86}
 .go{background:#0aa06e}.stop{background:#d0342c}.dir{background:#2b6cb0;height:78px}
 .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:auto}
 .sp{margin:18px 0}
 input[type=range]{width:100%}
 select{width:100%;padding:12px;font-size:16px;border-radius:10px;background:#1b242d;color:#e8eef2;border:1px solid #2a3540}
 .sens{display:flex;gap:10px;justify-content:center;margin-top:16px}
 .dot{width:32px;height:32px;border-radius:8px;background:#2a3540}
 .dot.hit{background:#0aa06e}
 .st{text-align:center;margin-top:12px;font-size:14px;color:#8fa3b0}
 .dist{text-align:center;margin-top:8px;font-size:15px}
 .dist.warn{color:#ff5b5b;font-weight:700}
 .msg{text-align:center;margin-top:10px;font-size:16px;font-weight:700;color:#4fd1a1;min-height:22px}
</style></head><body>
<header>ESP32 4WD LINE BOT (5-ARRAY + RFID)</header>
<div class="tabs">
  <div class="tab on" id="t1" onclick="tab(1)">LINE FOLLOW</div>
  <div class="tab" id="t2" onclick="tab(2)">MANUAL</div>
  <div class="tab" id="t3" onclick="tab(3)">DELIVERY</div>
</div>

<div class="panel on" id="p1">
  <button class="go" onclick="send('/mode?m=LINE')">START LINE FOLLOWING</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/mode?m=STOP')">STOP</button>
  <div class="sp">Base Speed: <b id="bsv">150</b>
    <input type="range" min="60" max="255" value="150" oninput="bsv.innerText=this.value" onchange="send('/set?base='+this.value)">
  </div>
  <div class="sp">Kp: <b id="kpv">35</b>
    <input type="range" min="0" max="100" value="35" oninput="kpv.innerText=this.value" onchange="send('/set?kp='+this.value)">
  </div>
  <div class="sp">Kd: <b id="kdv">20</b>
    <input type="range" min="0" max="100" value="20" oninput="kdv.innerText=this.value" onchange="send('/set?kd='+this.value)">
  </div>
  <div class="sens" id="sens"></div>
  <div class="st" id="st">mode: --</div>
  <div class="dist" id="dist">distance: -- cm</div>
</div>

<div class="panel" id="p2">
  <div class="pad">
    <div></div>
    <button class="dir" onmousedown="mv('F')" ontouchstart="mv('F')" onmouseup="mv('S')" ontouchend="mv('S')">&#9650;</button>
    <div></div>
    <button class="dir" onmousedown="mv('L')" ontouchstart="mv('L')" onmouseup="mv('S')" ontouchend="mv('S')">&#9664;</button>
    <button class="stop" style="height:78px" onclick="mv('S')">&#9632;</button>
    <button class="dir" onmousedown="mv('R')" ontouchstart="mv('R')" onmouseup="mv('S')" ontouchend="mv('S')">&#9654;</button>
    <div></div>
    <button class="dir" onmousedown="mv('B')" ontouchstart="mv('B')" onmouseup="mv('S')" ontouchend="mv('S')">&#9660;</button>
    <div></div>
  </div>
  <div class="sp">Manual Speed: <b id="msv">180</b>
    <input type="range" min="60" max="255" value="180" oninput="msv.innerText=this.value" onchange="send('/set?man='+this.value)">
  </div>
  <div class="dist" id="dist2">distance: -- cm</div>
</div>

<div class="panel" id="p3">
  <div class="st" style="font-size:16px">Robot state: <b id="rstate">--</b></div>
  <div class="sp">Sender:
    <select id="senderSel"><option value="X" selected>X</option><option value="Y">Y</option></select>
  </div>
  <div class="sp">Receiver:
    <select id="receiverSel"><option value="X">X</option><option value="Y" selected>Y</option></select>
  </div>
  <button class="go" id="btnCall" onclick="callRobot()">CALL ROBOT</button>
  <div class="sp"></div>
  <button class="go" id="btnLoaded" onclick="send('/itemLoaded')" disabled>ITEM LOADED (at X)</button>
  <div class="sp"></div>
  <button class="go" id="btnReceived" onclick="send('/received')" disabled>RECEIVED (at Y)</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/cancel')">CANCEL / RESET TO STANDBY</button>
  <div class="sp">J1 turn ms: <b id="jtv">500</b>
    <input type="range" min="200" max="1500" value="500" oninput="jtv.innerText=this.value" onchange="send('/set?jturn='+this.value)">
  </div>
  <div class="sp">U-turn ms: <b id="utv">1000</b>
    <input type="range" min="400" max="2500" value="1000" oninput="utv.innerText=this.value" onchange="send('/set?uturn='+this.value)">
  </div>
  <div class="msg" id="dmsg">Idle</div>
  <div class="st" id="rf">last tag: -- | rfid: --</div>
  <div class="dist" id="dist3">distance: -- cm</div>
</div>

<script>
let box=document.getElementById('sens');
for(let i=0;i<5;i++){let d=document.createElement('div');d.className='dot';d.id='d'+i;box.appendChild(d);}
function tab(n){
  t1.className='tab'+(n==1?' on':'');t2.className='tab'+(n==2?' on':'');t3.className='tab'+(n==3?' on':'');
  p1.className='panel'+(n==1?' on':'');p2.className='panel'+(n==2?' on':'');p3.className='panel'+(n==3?' on':'');
  if (n!=3) send('/mode?m='+(n==1?'STOP':'MANUAL'));
}
function send(u){fetch(u);}
function mv(c){fetch('/manual?c='+c);}
function callRobot(){
  const s=senderSel.value, r=receiverSel.value;
  if (s===r) { alert('Sender and receiver cannot be the same.'); return; }
  fetch('/call?sender='+s+'&receiver='+r).then(resp=>{
    if (resp.status===409) alert('A delivery is already running. Cancel it first.');
    else if (!resp.ok) alert('This route is not built yet. Only X -> Y works now.');
  });
}
document.addEventListener('keydown',e=>{const k={ArrowUp:'F',ArrowDown:'B',ArrowLeft:'L',ArrowRight:'R'}[e.key];if(k)mv(k);});
document.addEventListener('keyup',e=>{if(e.key.startsWith('Arrow'))mv('S');});
setInterval(()=>{fetch('/status').then(r=>r.json()).then(j=>{
  for(let i=0;i<5;i++)document.getElementById('d'+i).className='dot'+(j.s[i]?' hit':'');
  st.innerText='mode: '+j.mode;
  let txt='distance: '+j.dist.toFixed(1)+' cm'+(j.obs?'  OBSTACLE!':'');
  dist.innerText=txt; dist.className='dist'+(j.obs?' warn':'');
  dist2.innerText=txt; dist2.className='dist'+(j.obs?' warn':'');
  dist3.innerText=txt; dist3.className='dist'+(j.obs?' warn':'');
  rstate.innerText=j.robotState+(j.auto?' (AUTO)':' (idle)');
  rf.innerText='last tag: '+j.tag+' | rfid: '+(j.rfidOk?'OK':'NOT FOUND');
  dmsg.innerText=j.msg;
  btnCall.disabled=j.auto;
  btnLoaded.disabled=(j.robotState!=='AT_X');
  btnReceived.disabled=(j.robotState!=='AT_Y');
});},250);
</script></body></html>
)HTML";

/* ================= HANDLERS ================= */
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleMode() {
  if (autoModeActive) { server.send(409, "text/plain", "delivery running"); return; }
  mode = server.arg("m");
  integral = 0; lastError = 0;
  if (mode != "LINE" && mode != "MANUAL") { mode = "STOP"; driveStop(); }
  server.send(200, "text/plain", "ok");
}

void handleManual() {
  if (autoModeActive) { server.send(409, "text/plain", "delivery running"); return; }
  manualCmd = server.arg("c");
  mode = "MANUAL";
  server.send(200, "text/plain", "ok");
}

void handleSet() {
  if (server.hasArg("base")) baseSpeed   = server.arg("base").toInt();
  if (server.hasArg("man"))  manualSpeed = server.arg("man").toInt();
  if (server.hasArg("kp"))   Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd"))   Kd = server.arg("kd").toFloat();
  if (server.hasArg("jturn")) JUNCTION_TURN_MS = (unsigned long)server.arg("jturn").toInt();
  if (server.hasArg("uturn")) UTURN_MS = (unsigned long)server.arg("uturn").toInt();
  server.send(200, "text/plain", "ok");
}

void handleCall() {
  if (autoModeActive) { server.send(409, "text/plain", "running"); return; }
  String s = server.hasArg("sender")   ? server.arg("sender")   : "X";
  String r = server.hasArg("receiver") ? server.arg("receiver") : "Y";
  if (!(s == "X" && r == "Y")) { server.send(400, "text/plain", "route not built"); return; }

  mode = "STOP";
  autoModeActive = true;
  robotState = STATE_GOING_TO_X;
  phase = PH_NONE;
  waitReason = WAIT_NONE;
  comingFromBranch = false;
  integral = 0; lastError = 0;
  deliveryMsg = "Going to X (sender)";
  Serial.println("[APP] Call received: X -> Y");
  server.send(200, "text/plain", "ok");
}

void handleItemLoaded() {
  if (robotState == STATE_AT_X && phase == PH_NONE) {
    startUTurn(STATE_GOING_TO_Y);
    deliveryMsg = "Item loaded. Turning, then going to Y.";
    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "not at X");
  }
}

void handleReceived() {
  if (robotState == STATE_AT_Y && phase == PH_NONE) {
    startUTurn(STATE_RETURNING);
    deliveryMsg = "Received. Turning, then returning to base.";
    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "not at Y");
  }
}

void handleCancel() {
  autoModeActive = false;
  robotState = STATE_STANDBY;
  phase = PH_NONE;
  waitReason = WAIT_NONE;
  comingFromBranch = false;
  mode = "STOP";
  driveStop();
  deliveryMsg = "Cancelled. Standby.";
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  String j = "{\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 5; i++) { j += String(irVal[i]); if (i < 4) j += ","; }
  j += "],\"dist\":" + String(distanceCM, 1);
  j += ",\"obs\":" + String(obstacleDetected ? "true" : "false");
  j += ",\"tag\":\"" + lastTag + "\"";
  j += ",\"rfidOk\":" + String(rfidReady ? "true" : "false");
  j += ",\"robotState\":\"" + String(stateName(robotState)) + "\"";
  j += ",\"auto\":" + String(autoModeActive ? "true" : "false");
  j += ",\"msg\":\"" + deliveryMsg + "\"}";
  server.send(200, "application/json", j);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < 5; i++) pinMode(irPin[i], INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  ledcAttach(INA, PWM_FREQ, PWM_RES);
  ledcAttach(INB, PWM_FREQ, PWM_RES);
  ledcAttach(INC, PWM_FREQ, PWM_RES);
  ledcAttach(IND, PWM_FREQ, PWM_RES);
  driveStop();

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();
  delay(50);
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("RC522 Version: 0x"); Serial.println(version, HEX);
  rfidReady = (version != 0x00 && version != 0xFF);
  Serial.println(rfidReady ? "RFID OK" : "RFID NOT FOUND (line follow still works)");

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Access Point started! IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.on("/call", handleCall);
  server.on("/itemLoaded", handleItemLoaded);
  server.on("/received", handleReceived);
  server.on("/cancel", handleCancel);
  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  server.handleClient();

  updateUltrasonic();
  checkRFID();

  // ---------- DELIVERY (AUTO) MODE ----------
  if (autoModeActive) {
    float e;
    readSensors(e);   // keep IR values live for the UI

    if (obstacleDetected) {
      driveStop();
    }
    else if (phase == PH_WAIT) {
      // 3 s stop after a station tag. IR is ignored.
      driveStop();
      if (millis() - phaseStart >= phaseLen) finishWait();
    }
    else if (phase == PH_TURN) {
      // Timed junction spin. IR is ignored.
      if (millis() - phaseStart < phaseLen) {
        if (turnDir == 'L') spinLeft(baseSpeed);
        else                spinRight(baseSpeed);
      } else {
        phase = PH_NONE;
        integral = 0; lastError = 0;
        Serial.println("[J1] Turn done, IR on");
      }
    }
    else if (phase == PH_UTURN) {
      // Timed 180 spin. IR is ignored.
      if (millis() - phaseStart < phaseLen) {
        spinRight(baseSpeed);
      } else {
        phase = PH_NONE;
        robotState = stateAfterUTurn;
        comingFromBranch = true;   // next J1 is reached from the X/Y branch
        integral = 0; lastError = 0;
        Serial.print("[UTURN] Done. State: "); Serial.println(stateName(robotState));
      }
    }
    else {
      switch (robotState) {
        case STATE_GOING_TO_X:
        case STATE_GOING_TO_Y:
          lineFollow();
          break;

        case STATE_RETURNING:
          if (checkBaseMarker()) {
            driveStop();
            robotState = STATE_STANDBY;
            autoModeActive = false;
            comingFromBranch = false;
            deliveryMsg = "Back at base. Standby.";
            Serial.println("[STATE] Base reached, STANDBY");
          } else {
            lineFollow();
          }
          break;

        default:   // AT_X, AT_Y, STANDBY: wait for app button
          driveStop();
          break;
      }
    }
  }
  // ---------- MANUAL / LINE MODE ----------
  else {
    bool blockDrive = obstacleDetected && !(mode == "MANUAL" && manualCmd == "B");
    if (blockDrive) {
      driveStop();
      if (mode == "LINE") { float e; readSensors(e); }
    }
    else if (mode == "LINE")   lineFollow();
    else if (mode == "MANUAL") manualDrive();
    else {
      float e;
      readSensors(e);
      driveStop();
    }
  }
}