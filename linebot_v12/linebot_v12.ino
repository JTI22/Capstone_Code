/*
 * ESP32 4WD Line Follower + Manual + Ultrasonic + RFID Delivery (v12)
 * -------------------------------------------------------------------
 * Pin configuration taken from Last_working_code:
 *   IR 5-channel array : 32, 33, 34, 35, 39   (Black line = true)
 *   L298N (direct PWM) : INA 18, INB 19, INC 22, IND 23
 *   HC-SR04 ultrasonic : TRIG 25, ECHO 26
 *   RC522 RFID         : SS 5, RST 17, SCK 21, MOSI 13, MISO 14, VCC 3.3V
 *
 * Delivery flow (base is always the sender):
 *   Base -> J1         : stop 3 s on J1, turn LEFT for X / RIGHT for Y
 *   -> cabin X or Y    : stop, wait for ITEM RECEIVED (web UI)
 *   -> 180 turn -> J1  : stop 3 s, turn RIGHT back from X / LEFT back from Y
 *   -> base (RFID card C1:0D:BB:0A): stop 3 s -> 180 turn -> STANDBY
 *
 * WiFi AP: LineBot_ESP32 / 12345678  ->  http://192.168.4.1
 * RGY status LEDs:
 *   RED    D15 : obstacle detected, robot stopped
 *   YELLOW D2  : robot running
 *   GREEN  D4  : robot stopped at an RFID station (J1, cabin, base)
 *
 * Requires ESP32 Arduino core 3.x and the MFRC522 library.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>

/* ============== STATE MACHINE (enums defined early) ============== */
enum RobotState {
  STATE_STANDBY,
  STATE_GOING_OUT,          // base -> J1 -> cabin
  STATE_AT_CABIN,           // stopped at X or Y, waiting for ITEM RECEIVED
  STATE_UTURN_CABIN,        // 180 turn at cabin
  STATE_RETURNING_TO_J1,    // cabin -> J1
  STATE_JUNCTION_WAIT,      // stopped 3 s on J1
  STATE_JUNCTION_TURN,      // timed turn on J1
  STATE_RETURNING_TO_BASE,  // J1 -> base
  STATE_BASE_WAIT,          // stopped 3 s on base marker
  STATE_BASE_UTURN          // 180 turn at base, then standby
};

/* ================= WIFI ================= */
const char* AP_SSID = "LineBot_ESP32";
const char* AP_PASS = "12345678";
WebServer server(80);

/* ============== 5-CHANNEL IR SENSOR PINS (Left -> Right) ============== */
const int irPin[5] = {32, 33, 34, 35, 39};
int irVal[5];

/* Black line on white floor -> true ; White line on black floor -> false */
bool BLACK_LINE = true;

/* ============== MOTOR DRIVER PINS (L298N, direct PWM) ============== */
#define INA 18   // Left  forward
#define INB 19   // Left  backward
#define INC 22   // Right forward
#define IND 23   // Right backward

#define PWM_FREQ 5000
#define PWM_RES  8      // 0-255

bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;

/* ============== ULTRASONIC PINS (HC-SR04) ============== */
#define TRIG_PIN 25
#define ECHO_PIN 26

const float OBSTACLE_DISTANCE_CM = 10.0;
const unsigned long US_READ_INTERVAL = 60;

float distanceCM = 999;
bool  obstacleDetected = false;
unsigned long lastUSRead = 0;

/* ============== RGY STATUS LEDS ============== */
#define LED_RED    15   // D15
#define LED_YELLOW  2   // D2
#define LED_GREEN   4   // D4

/* ============== RFID (RC522) PINS ============== */
#define RFID_SS   5
#define RFID_RST  17
#define RFID_SCK  21
#define RFID_MOSI 13
#define RFID_MISO 14

MFRC522 rfid(RFID_SS, RFID_RST);

byte UID_J1[] = {0x64, 0xD5, 0x30, 0x03};   // Junction J1 card
byte UID_X[]  = {0xAC, 0x71, 0x8F, 0x02};   // Cabin 1 (X)
byte UID_Y[]  = {0x09, 0xB2, 0x30, 0x03};   // Cabin 2 (Y)
byte UID_BASE[] = {0xC1, 0x0D, 0xBB, 0x0A}; // Base station card

String lastRfidLocation = "NONE";
bool rfidReady = false;
byte lastActedUid[10];
byte lastActedUidLen = 0;
unsigned long lastActedUidTime = 0;
const unsigned long RFID_SAME_TAG_MS = 2000;   // one physical tag pass -> one action
unsigned long lastRfidProbe = 0;

bool uidMatches(byte *scanned, byte size, byte *known, byte knownSize) {
  if (size != knownSize) return false;
  for (byte i = 0; i < size; i++) if (scanned[i] != known[i]) return false;
  return true;
}

/* ============== DELIVERY SETTINGS ============== */
char target = 'X';                       // receiver chosen in the web UI ('X' or 'Y')
bool autoModeActive = false;             // true from ITEM LOADED until back on standby
RobotState robotState = STATE_STANDBY;

// Turn directions at J1 (tune if the robot turns the wrong way)
char J1_TURN_OUT_X  = 'L';   // base -> X
char J1_TURN_OUT_Y  = 'R';   // base -> Y
char J1_TURN_BACK_X = 'R';   // X -> base
char J1_TURN_BACK_Y = 'L';   // Y -> base

char pendingTurn = 'S';
RobotState pendingNext = STATE_GOING_OUT;
unsigned long phaseStart = 0;

const unsigned long JUNCTION_WAIT_MS = 3000;   // stop on J1 before turning
const unsigned long BASE_WAIT_MS     = 3000;   // stop on base before 180 turn
unsigned long JUNCTION_TURN_MS = 500;          // ~90 degree pivot, tune on track
unsigned long UTURN_MS         = 1000;         // ~180 degree pivot, tune on track
bool uTurnFlip = false;

unsigned long j1IgnoreUntil = 0;               // J1 tag ignored for a while after it is handled
const unsigned long J1_IGNORE_MS = 6000;

const char* stateName(RobotState s) {
  switch (s) {
    case STATE_STANDBY:            return "STANDBY";
    case STATE_GOING_OUT:          return "GOING_OUT";
    case STATE_AT_CABIN:           return "AT_CABIN";
    case STATE_UTURN_CABIN:        return "UTURN_CABIN";
    case STATE_RETURNING_TO_J1:    return "RETURNING_TO_J1";
    case STATE_JUNCTION_WAIT:      return "J1_WAIT";
    case STATE_JUNCTION_TURN:      return "J1_TURN";
    case STATE_RETURNING_TO_BASE:  return "RETURNING_TO_BASE";
    case STATE_BASE_WAIT:          return "BASE_WAIT";
    case STATE_BASE_UTURN:         return "BASE_UTURN";
  }
  return "?";
}

/* ============== CONTROL VARIABLES ============== */
String mode = "STOP";          // LINE / MANUAL / STOP
String manualCmd = "S";        // F B L R S
int baseSpeed   = 150;
int manualSpeed = 180;

float Kp = 35.0, Ki = 0.0, Kd = 20.0;   // PID from the working code
float lastError = 0, integral = 0;
float lastKnownError = 0;

/* ================= MOTOR HELPERS ================= */
void leftMotor(int spd) {                  // -255 .. 255
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

void driveStop() { leftMotor(0); rightMotor(0); }

// Timed pivot: 'L' = left turn, 'R' = right turn
void spin(char dir, int s) {
  if (dir == 'L')      { leftMotor(s);  rightMotor(-s); }
  else if (dir == 'R') { leftMotor(-s); rightMotor(s);  }
  else                 driveStop();
}

void uTurnSpin() {
  if (uTurnFlip) spin('R', baseSpeed);
  else           spin('L', baseSpeed);
}

/* ================= ULTRASONIC ================= */
float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);   // 25 ms timeout (~4 m)
  if (duration == 0) return 999;                      // no echo -> clear
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
    if (irVal[i]) { sum += weight[i]; count++; }
  }
  if (count == 0) return false;            // line lost
  error = sum / count;
  return true;
}

/* ================= LINE FOLLOW ================= */
void lineFollow() {
  float error;
  if (!readSensors(error)) {
    int s = baseSpeed * 0.8;               // line hariye gele khuje
    if (lastKnownError < 0) { leftMotor(-s); rightMotor(s); }
    else                    { leftMotor(s);  rightMotor(-s); }
    return;
  }
  lastKnownError = error;

  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  leftMotor(baseSpeed + correction);
  rightMotor(baseSpeed - correction);
}

/* ================= MANUAL DRIVE ================= */
void manualDrive() {
  int s = manualSpeed;
  if      (manualCmd == "F") { leftMotor(s);  rightMotor(s);  }
  else if (manualCmd == "B") { leftMotor(-s); rightMotor(-s); }
  else if (manualCmd == "L") { leftMotor(s);  rightMotor(-s); }
  else if (manualCmd == "R") { leftMotor(-s); rightMotor(s);  }
  else driveStop();
}

/* ================= RFID ================= */
bool sameAsLastActed(byte *uid, byte size) {
  if (size == 0 || size != lastActedUidLen) return false;
  for (byte i = 0; i < size; i++) if (uid[i] != lastActedUid[i]) return false;
  return true;
}

void rememberActedUid(byte *uid, byte size) {
  if (size > sizeof(lastActedUid)) size = sizeof(lastActedUid);
  for (byte i = 0; i < size; i++) lastActedUid[i] = uid[i];
  lastActedUidLen = size;
  lastActedUidTime = millis();
}

bool probeRfid() {
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("[RFID] Version: 0x"); Serial.println(v, HEX);
  rfidReady = (v != 0x00 && v != 0xFF);
  if (rfidReady) Serial.println("[RFID] Reader OK.");
  else Serial.println("[RFID] Reader not detected. Line follow still works.");
  lastRfidProbe = millis();
  return rfidReady;
}

void startJunctionWait() {
  robotState = STATE_JUNCTION_WAIT;
  phaseStart = millis();
  j1IgnoreUntil = millis() + J1_IGNORE_MS;
  Serial.print("[J1] Reached. Stopping 3 s, then turn "); Serial.println(pendingTurn);
}

void handleCabinTag(char c) {
  if (autoModeActive && robotState == STATE_GOING_OUT && c == target) {
    robotState = STATE_AT_CABIN;
    Serial.print("[CABIN] "); Serial.print(c); Serial.println(" reached. Waiting for ITEM RECEIVED.");
  } else {
    Serial.print("[CABIN] "); Serial.print(c); Serial.println(" seen, not the target right now. Ignored.");
  }
}

void checkRFID() {
  if (!rfidReady) return;
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  if (sameAsLastActed(rfid.uid.uidByte, rfid.uid.size) &&
      (millis() - lastActedUidTime < RFID_SAME_TAG_MS)) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }

  if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_BASE, sizeof(UID_BASE))) {
    lastRfidLocation = "BASE";
    if (autoModeActive && robotState == STATE_RETURNING_TO_BASE) {
      robotState = STATE_BASE_WAIT;
      phaseStart = millis();
      Serial.println("[BASE] Card C1:0D:BB:0A detected. Stop 3 s, then 180 turn.");
    } else {
      Serial.println("[BASE] Base card seen, no action.");
    }
  }
  else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_J1, sizeof(UID_J1))) {
    lastRfidLocation = "J1";
    if (autoModeActive && millis() >= j1IgnoreUntil) {
      if (robotState == STATE_GOING_OUT) {
        pendingTurn = (target == 'X') ? J1_TURN_OUT_X : J1_TURN_OUT_Y;
        pendingNext = STATE_GOING_OUT;
        startJunctionWait();
      } else if (robotState == STATE_RETURNING_TO_J1) {
        pendingTurn = (target == 'X') ? J1_TURN_BACK_X : J1_TURN_BACK_Y;
        pendingNext = STATE_RETURNING_TO_BASE;
        startJunctionWait();
      } else {
        Serial.println("[RFID] J1 seen, not driving. Ignored.");
      }
    } else if (!autoModeActive) {
      Serial.println("[RFID] J1 seen (standby, no action).");
    }
  }
  else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_X, sizeof(UID_X))) {
    lastRfidLocation = "X";
    handleCabinTag('X');
  }
  else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_Y, sizeof(UID_Y))) {
    lastRfidLocation = "Y";
    handleCabinTag('Y');
  }
  else {
    Serial.print("[RFID] Unknown tag, UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i], HEX); Serial.print(" ");
    }
    Serial.println();
  }

  rememberActedUid(rfid.uid.uidByte, rfid.uid.size);
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

/* ================= DELIVERY HELPERS ================= */
void enterTurnaround(RobotState next) {
  // Starts the 180 turn at a cabin. Ends in RETURNING_TO_J1.
  pendingNext = next;
  robotState = STATE_UTURN_CABIN;
  phaseStart = millis();
}

/* ================= WEB PAGE ================= */
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 LineBot</title>
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
<header>ESP32 4WD LINE BOT v12</header>
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
  <div class="st" style="font-size:16px">Base (sender) -> <b id="rstate">--</b></div>
  <div class="sp">Receiver:
    <select id="receiverSel">
      <option value="X" selected>Cabin 1 (X)</option>
      <option value="Y">Cabin 2 (Y)</option>
    </select>
  </div>
  <div class="sp">Delivery speed: <b id="spd3v">150</b>
    <input type="range" min="60" max="255" value="150" oninput="spd3v.innerText=this.value" onchange="send('/set?base='+this.value)">
  </div>
  <button class="go" id="btnLoaded" onclick="itemLoaded()">ITEM LOADED - SEND TO X</button>
  <div class="sp"></div>
  <button class="go" id="btnReceived" onclick="send('/received')" disabled>ITEM RECEIVED</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/cancelAuto')">CANCEL / RESET TO STANDBY</button>
  <div class="sp">J1 turn ms: <b id="jtv">500</b>
    <input type="range" min="200" max="1500" value="500" oninput="jtv.innerText=this.value" onchange="send('/set?jturn='+this.value)">
  </div>
  <div class="sp">U-turn ms: <b id="utv">1000</b>
    <input type="range" min="400" max="2500" value="1000" oninput="utv.innerText=this.value" onchange="send('/set?uturn='+this.value)">
  </div>
  <button class="dir" id="btnFlip" onclick="flipU()">FLIP U-TURN DIRECTION</button>
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
function itemLoaded(){
  const r = receiverSel.value;
  fetch('/call?receiver='+r).then(resp=>{
    if (resp.status === 409) alert('A delivery is already running. Cancel it first.');
    else if (!resp.ok) alert('Receiver not valid.');
  });
}
let uFlip = 0;
function flipU(){
  uFlip = uFlip ? 0 : 1;
  send('/set?flip='+uFlip);
  btnFlip.innerText = uFlip ? 'U-TURN FLIPPED (tap to restore)' : 'FLIP U-TURN DIRECTION';
}
function mv(c){fetch('/manual?c='+c);}
document.addEventListener('keydown',e=>{const k={ArrowUp:'F',ArrowDown:'B',ArrowLeft:'L',ArrowRight:'R'}[e.key];if(k)mv(k);});
document.addEventListener('keyup',e=>{if(e.key.startsWith('Arrow'))mv('S');});
setInterval(()=>{fetch('/status').then(r=>r.json()).then(j=>{
  for(let i=0;i<5;i++)document.getElementById('d'+i).className='dot'+(j.s[i]?' hit':'');
  st.innerText='mode: '+j.mode;
  let txt='distance: '+j.dist.toFixed(1)+' cm'+(j.obs?'  OBSTACLE!':'');
  dist.innerText=txt; dist.className='dist'+(j.obs?' warn':'');
  dist2.innerText=txt; dist2.className='dist'+(j.obs?' warn':'');
  dist3.innerText=txt; dist3.className='dist'+(j.obs?' warn':'');
  rstate.innerText=j.robotState+' (receiver '+j.receiver+')'+(j.auto?' AUTO':' idle');
  rf.innerText='last tag: '+j.tag+' | rfid: '+(j.rfidOk?'OK':'NOT FOUND');
  dmsg.innerText=j.msg;
  btnLoaded.innerText='ITEM LOADED - SEND TO '+receiverSel.value;
  btnLoaded.disabled=j.auto;
  receiverSel.disabled=j.auto;
  btnReceived.disabled=(j.robotState!=='AT_CABIN');
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
  if (server.hasArg("base"))  baseSpeed   = server.arg("base").toInt();
  if (server.hasArg("man"))   manualSpeed = server.arg("man").toInt();
  if (server.hasArg("kp"))    Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd"))    Kd = server.arg("kd").toFloat();
  if (server.hasArg("jturn")) JUNCTION_TURN_MS = (unsigned long)server.arg("jturn").toInt();
  if (server.hasArg("uturn")) UTURN_MS = (unsigned long)server.arg("uturn").toInt();
  if (server.hasArg("flip"))  uTurnFlip = server.arg("flip").toInt() != 0;
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  String j = "{\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 5; i++) { j += String(irVal[i]); if (i < 4) j += ","; }
  j += "],\"dist\":" + String(distanceCM, 1) + ",\"obs\":" + String(obstacleDetected ? "true" : "false");
  j += ",\"tag\":\"" + lastRfidLocation + "\"";
  j += ",\"rfidOk\":" + String(rfidReady ? "true" : "false");
  j += ",\"robotState\":\"" + String(stateName(robotState)) + "\"";
  j += ",\"auto\":" + String(autoModeActive ? "true" : "false");
  j += ",\"receiver\":\"" + String(target) + "\"";
  j += ",\"msg\":\"" + String(autoModeActive ? "Delivery running" : "Idle") + "\"}";
  server.send(200, "application/json", j);
}

void handleCall() {
  if (autoModeActive) {
    Serial.println("[APP] Item loaded ignored, delivery already running.");
    server.send(409, "text/plain", "delivery already running");
    return;
  }
  String r = server.arg("receiver");
  if (r != "X" && r != "Y") {
    server.send(400, "text/plain", "receiver must be X or Y");
    return;
  }
  target = r[0];
  mode = "STOP";
  robotState = STATE_GOING_OUT;
  autoModeActive = true;
  j1IgnoreUntil = 0;
  integral = 0; lastError = 0;
  Serial.print("[APP] Item loaded. Base -> "); Serial.println(target);
  server.send(200, "text/plain", "ok");
}

void handleReceived() {
  if (robotState == STATE_AT_CABIN) {
    enterTurnaround(STATE_RETURNING_TO_J1);
    Serial.println("[APP] Item received. 180 turn, then back to J1.");
    server.send(200, "text/plain", "ok");
  } else {
    server.send(400, "text/plain", "not at a cabin");
  }
}

void handleCancelAuto() {
  autoModeActive = false;
  robotState = STATE_STANDBY;
  driveStop();
  Serial.println("[APP] Delivery cancelled, standby.");
  server.send(200, "text/plain", "ok");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < 5; i++) pinMode(irPin[i], INPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  ledcAttach(INA, PWM_FREQ, PWM_RES);
  ledcAttach(INB, PWM_FREQ, PWM_RES);
  ledcAttach(INC, PWM_FREQ, PWM_RES);
  ledcAttach(IND, PWM_FREQ, PWM_RES);
  driveStop();

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  probeRfid();
  Serial.println("LineBot v12 ready. Base sends, receiver X or Y chosen in the web UI.");

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.on("/call", handleCall);
  server.on("/received", handleReceived);
  server.on("/cancelAuto", handleCancelAuto);
  server.begin();
}

/* ================= RGY LEDS ================= */
bool isMovingNow() {
  if (autoModeActive) {
    switch (robotState) {
      case STATE_GOING_OUT:
      case STATE_RETURNING_TO_J1:
      case STATE_RETURNING_TO_BASE:
      case STATE_JUNCTION_TURN:
      case STATE_UTURN_CABIN:
      case STATE_BASE_UTURN:
        return true;
      default:
        return false;
    }
  }
  if (mode == "LINE") return true;
  if (mode == "MANUAL") return manualCmd != "S";
  return false;
}

bool isRfidStopped() {
  return autoModeActive && (robotState == STATE_AT_CABIN ||
                            robotState == STATE_JUNCTION_WAIT ||
                            robotState == STATE_BASE_WAIT);
}

void updateLeds() {
  bool red    = obstacleDetected;          // obstacle -> stop
  bool yellow = !red && isMovingNow();     // running
  bool green  = !red && isRfidStopped();   // stopped at RFID station
  digitalWrite(LED_RED,    red    ? HIGH : LOW);
  digitalWrite(LED_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(LED_GREEN,  green  ? HIGH : LOW);
}

/* ================= LOOP ================= */
void loop() {
  server.handleClient();

  updateUltrasonic();
  if (!rfidReady && mode != "LINE" && !(mode == "MANUAL" && manualCmd != "S") && !autoModeActive) {
    if (millis() - lastRfidProbe >= 8000) probeRfid();
  }
  checkRFID();
  updateLeds();

  bool obstacleBlock = obstacleDetected && !(mode == "MANUAL" && manualCmd == "B");
  unsigned long now = millis();

  // ---------- DELIVERY MODE ----------
  if (autoModeActive) {
    switch (robotState) {
      case STATE_JUNCTION_WAIT:
        driveStop();
        if (now - phaseStart >= JUNCTION_WAIT_MS) {
          robotState = STATE_JUNCTION_TURN;
          phaseStart = now;
        }
        break;

      case STATE_JUNCTION_TURN:
        if (now - phaseStart < JUNCTION_TURN_MS) {
          spin(pendingTurn, baseSpeed);
        } else {
          driveStop();
          integral = 0; lastError = 0;
          robotState = pendingNext;
          Serial.println("[J1] Turn complete, line follow again.");
        }
        break;

      case STATE_UTURN_CABIN:
        if (now - phaseStart < UTURN_MS) {
          uTurnSpin();
        } else {
          driveStop();
          integral = 0; lastError = 0;
          robotState = pendingNext;
          Serial.println("[UTURN] Done, heading back to J1.");
        }
        break;

      case STATE_BASE_WAIT:
        driveStop();
        if (now - phaseStart >= BASE_WAIT_MS) {
          robotState = STATE_BASE_UTURN;
          phaseStart = now;
        }
        break;

      case STATE_BASE_UTURN:
        if (now - phaseStart < UTURN_MS) {
          uTurnSpin();
        } else {
          driveStop();
          robotState = STATE_STANDBY;
          autoModeActive = false;
          Serial.println("[STATE] Back on standby at base.");
        }
        break;

      case STATE_GOING_OUT:
      case STATE_RETURNING_TO_J1:
        if (obstacleBlock) driveStop();
        else lineFollow();
        break;

      case STATE_RETURNING_TO_BASE:
        // Base card (RFID) moves us to STATE_BASE_WAIT inside checkRFID()
        if (obstacleBlock) driveStop();
        else lineFollow();
        break;

      case STATE_AT_CABIN:
      case STATE_STANDBY:
      default:
        driveStop();
        break;
    }
  }
  // ---------- MANUAL / LINE TEST MODE ----------
  else {
    if (obstacleBlock) {
      driveStop();
    }
    else if (mode == "LINE")   lineFollow();
    else if (mode == "MANUAL") manualDrive();
    else {
      float e; readSensors(e);
      driveStop();
    }
  }
}
