#define MPU_SDA 21
#define MPU_SCL 22
/*
 * ESP32 Mini Quadcopter Flight Controller
 * ----------------------------------------
 * Board   : ESP32 DevKit V1
 * IMU     : MPU6050 (I2C, SDA=21, SCL=22)
 * Motors  : 4x 8520 Coreless via SI2302 N-MOSFET (GPIO 13/25/26/27)
 * Control : WebSocket joystick from mobile browser over ESP32 AP Wi-Fi
 *
 * Motor layout (top-down view):
 *   FL(CW)  [13]  |  FR(CCW) [25]
 *   BL(CCW) [26]  |  BR(CW)  [27]
 *
 * PID axes:
 *   Roll  → FL/BR vs FR/BL
 *   Pitch → FL/FR vs BL/BR
 *   Yaw   → (FL+BR) vs (FR+BL)  [reaction torque]
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define MOTOR_FL  13
#define MOTOR_FR  25
#define MOTOR_BL  26
#define MOTOR_BR  27

// ── LEDC (ESP32 PWM) channels ────────────────────────────────────────────────
#define PWM_FREQ      20000   // 20 kHz — inaudible, good for coreless motors
#define PWM_RES       8       // 8-bit: 0-255
#define CH_FL         0
#define CH_FR         1
#define CH_BL         2
#define CH_BR         3

// ── Wi-Fi AP credentials ─────────────────────────────────────────────────────

// Forward declarations
inline int clamp(int val, int lo, int hi);
void setMotors(int fl, int fr, int bl, int br);
void cutMotors();
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
function connectWS();
function sendState();
function map(v,il,ih,ol,oh);
function clamp(v,lo,hi);
function drawStick(id, stick);
function stickFromTouch(e,canvas,stick,lockY);
function setupCanvas(id, stick, lockY);

const char* AP_SSID     = "Drone_WiFi";
const char* AP_PASSWORD = "drone1234";   // min 8 chars

// ── PID tuning (start conservative, tune on bench) ──────────────────────────
float KP_ROLL  = 1.2f,  KI_ROLL  = 0.04f,  KD_ROLL  = 0.8f;
float KP_PITCH = 1.2f,  KI_PITCH = 0.04f,  KD_PITCH = 0.8f;
float KP_YAW   = 2.0f,  KI_YAW   = 0.02f,  KD_YAW   = 0.0f;

// ── Throttle / safety constants ──────────────────────────────────────────────
#define THROTTLE_MIN     30    // Minimum armed throttle (keeps motors barely spinning)
#define THROTTLE_MAX    240    // Maximum out of 255 — leaves headroom for corrections
#define MOTOR_ARM_MIN    10    // Below this the motor is off (disarmed)
#define FAILSAFE_MS     500    // Cut motors if no WS message within this period

// ── Runtime state ────────────────────────────────────────────────────────────
MPU6050 mpu(Wire);
AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");

volatile float   g_throttle = 0;   // 0-100 from joystick
volatile float   g_yaw      = 0;   // -100..+100
volatile float   g_pitch    = 0;   // -100..+100
volatile float   g_roll     = 0;   // -100..+100
volatile uint32_t g_lastWsMsg = 0; // millis() of last WS packet

// PID state
float errRollPrev  = 0, intRoll  = 0;
float errPitchPrev = 0, intPitch = 0;
float errYawPrev   = 0, intYaw   = 0;
uint32_t lastLoopUs = 0;

// ── Helpers ──────────────────────────────────────────────────────────────────
inline int clamp(int val, int lo, int hi) {
  return (val < lo) ? lo : (val > hi) ? hi : val;
}

void setMotors(int fl, int fr, int bl, int br) {
  ledcWrite(CH_FL, clamp(fl, 0, 255));
  ledcWrite(CH_FR, clamp(fr, 0, 255));
  ledcWrite(CH_BL, clamp(bl, 0, 255));
  ledcWrite(CH_BR, clamp(br, 0, 255));
}

void cutMotors() {
  ledcWrite(CH_FL, 0);
  ledcWrite(CH_FR, 0);
  ledcWrite(CH_BL, 0);
  ledcWrite(CH_BR, 0);
}

// ── WebSocket event handler ──────────────────────────────────────────────────
// Expected message format (plain text):  "T:50,Y:-10,P:5,R:3"
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len) {
      String msg = String((char *)data).substring(0, len);
      // Parse "T:XX,Y:XX,P:XX,R:XX"
      int tIdx = msg.indexOf("T:");
      int yIdx = msg.indexOf(",Y:");
      int pIdx = msg.indexOf(",P:");
      int rIdx = msg.indexOf(",R:");
      if (tIdx >= 0 && yIdx > tIdx && pIdx > yIdx && rIdx > pIdx) {
        g_throttle = msg.substring(tIdx + 2, yIdx).toFloat();
        g_yaw      = msg.substring(yIdx + 3, pIdx).toFloat();
        g_pitch    = msg.substring(pIdx + 3, rIdx).toFloat();
        g_roll     = msg.substring(rIdx + 3).toFloat();
        g_lastWsMsg = millis();
      }
    }
  }
}

// ── Served HTML/JS page ──────────────────────────────────────────────────────
// Dual virtual joystick UI rendered via HTML5 Canvas + Touch Events
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Drone Control</title>
<style>
  body{background:#111;display:flex;flex-direction:column;align-items:center;
       font-family:sans-serif;color:#eee;margin:0;padding:10px;touch-action:none;}
  h2{margin:6px 0;font-size:18px;letter-spacing:2px;}
  #status{font-size:13px;color:#0f0;margin-bottom:6px;}
  .stick-wrap{display:flex;gap:20px;}
  canvas{border:2px solid #444;border-radius:50%;background:#1a1a1a;}
  #labels{display:flex;gap:20px;font-size:11px;color:#888;margin-top:4px;}
  .lbl{width:150px;text-align:center;}
</style>
</head>
<body>
<h2>&#9654; DRONE</h2>
<div id="status">Connecting…</div>
<div class="stick-wrap">
  <canvas id="leftStick"  width="150" height="150"></canvas>
  <canvas id="rightStick" width="150" height="150"></canvas>
</div>
<div id="labels"><div class="lbl">Throttle / Yaw</div><div class="lbl">Pitch / Roll</div></div>

<script>
var ws, pingTimer;
var left  = {x:75,y:75,touch:null};   // Throttle(y) + Yaw(x)
var right = {x:75,y:75,touch:null};   // Pitch(y)    + Roll(x)
var leftLock = false; // lock throttle axis on Y so it stays

function connectWS(){
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen  = function(){ document.getElementById('status').textContent='Connected ✓'; };
  ws.onclose = function(){ document.getElementById('status').textContent='Disconnected – retrying…'; setTimeout(connectWS,1000); };
  ws.onerror = function(){ ws.close(); };
}

function sendState(){
  if(ws && ws.readyState===1){
    var throttle = Math.round(map(left.y,  150, 0, 0, 100));
    var yaw      = Math.round(map(left.x,  0, 150, -100, 100));
    var pitch    = Math.round(map(right.y, 150, 0, -50, 50));
    var roll     = Math.round(map(right.x, 0,  150, -50, 50));
    throttle = clamp(throttle,0,100);
    yaw      = clamp(yaw,-100,100);
    pitch    = clamp(pitch,-50,50);
    roll     = clamp(roll,-50,50);
    ws.send('T:'+throttle+',Y:'+yaw+',P:'+pitch+',R:'+roll);
  }
}

function map(v,il,ih,ol,oh){ return (v-il)/(ih-il)*(oh-ol)+ol; }
function clamp(v,lo,hi){ return Math.max(lo,Math.min(hi,v)); }

function drawStick(id, stick){
  var c = document.getElementById(id);
  var ctx = c.getContext('2d');
  ctx.clearRect(0,0,150,150);
  ctx.beginPath(); ctx.arc(75,75,72,0,2*Math.PI);
  ctx.strokeStyle='#444'; ctx.lineWidth=2; ctx.stroke();
  // crosshair
  ctx.strokeStyle='#333';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(75,5);ctx.lineTo(75,145);ctx.stroke();
  ctx.beginPath();ctx.moveTo(5,75);ctx.lineTo(145,75);ctx.stroke();
  // knob
  ctx.beginPath(); ctx.arc(stick.x,stick.y,18,0,2*Math.PI);
  ctx.fillStyle='#0af'; ctx.fill();
}

function stickFromTouch(e,canvas,stick,lockY){
  var r = canvas.getBoundingClientRect();
  var touches = e.changedTouches;
  for(var i=0;i<touches.length;i++){
    var t = touches[i];
    if(stick.touch!==null && stick.touch!==t.identifier) continue;
    var nx = clamp(t.clientX - r.left, 5, 145);
    var ny = clamp(t.clientY - r.top,  5, 145);
    stick.x = nx;
    if(!lockY) stick.y = ny;
    else stick.y = ny; // throttle allows free Y
    stick.touch = t.identifier;
  }
}

function setupCanvas(id, stick, lockY){
  var c = document.getElementById(id);
  c.addEventListener('touchstart', function(e){
    e.preventDefault(); stick.touch=null; stickFromTouch(e,c,stick,lockY); sendState(); drawStick(id,stick);
  },{passive:false});
  c.addEventListener('touchmove', function(e){
    e.preventDefault(); stickFromTouch(e,c,stick,lockY); sendState(); drawStick(id,stick);
  },{passive:false});
  c.addEventListener('touchend', function(e){
    e.preventDefault();
    // Right stick snaps back to centre; left stick holds throttle Y
    stick.touch=null; stick.x=75; if(!lockY) stick.y=75;
    sendState(); drawStick(id,stick);
  },{passive:false});
}

setupCanvas('leftStick', left, true);
setupCanvas('rightStick', right, false);

setInterval(function(){ sendState(); drawStick('leftStick',left); drawStick('rightStick',right); }, 50);

connectWS();
drawStick('leftStick', left);
drawStick('rightStick', right);
</script>
</body>
</html>
)rawliteral";

// ── setup() ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[DRONE] Booting…");

  // ── LEDC motor PWM ──────────────────────────────────────────────────────
  ledcSetup(CH_FL, PWM_FREQ, PWM_RES);  ledcAttachPin(MOTOR_FL, CH_FL);
  ledcSetup(CH_FR, PWM_FREQ, PWM_RES);  ledcAttachPin(MOTOR_FR, CH_FR);
  ledcSetup(CH_BL, PWM_FREQ, PWM_RES);  ledcAttachPin(MOTOR_BL, CH_BL);
  ledcSetup(CH_BR, PWM_FREQ, PWM_RES);  ledcAttachPin(MOTOR_BR, CH_BR);
  cutMotors();

  // ── I2C + MPU6050 ────────────────────────────────────────────────────────
  Wire.begin(MPU_SDA, MPU_SCL);
  byte status = mpu.begin();
  while (status != 0) {
    Serial.print("[IMU] Init failed, status="); Serial.println(status);
    delay(500);
    status = mpu.begin();
  }
  Serial.println("[IMU] MPU6050 OK");

  Serial.println("[IMU] Calibrating — keep drone flat…");
  mpu.calcOffsets(true, true);   // gyro + accel offsets
  Serial.println("[IMU] Calibration done");

  // ── Wi-Fi AP ─────────────────────────────────────────────────────────────
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[WIFI] AP IP: "); Serial.println(WiFi.softAPIP());

  // ── WebSocket ────────────────────────────────────────────────────────────
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // ── Serve root HTML ──────────────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.begin();
  Serial.println("[HTTP] Server started");

  lastLoopUs = micros();
  g_lastWsMsg = millis();
}

// ── loop() ──────────────────────────────────────────────────────────────────
void loop() {
  // ── Failsafe: cut motors if no WS packet within FAILSAFE_MS ──────────────
  if ((millis() - g_lastWsMsg) > FAILSAFE_MS) {
    cutMotors();
    ws.cleanupClients();
    return;
  }

  // ── Read IMU ─────────────────────────────────────────────────────────────
  mpu.update();
  float angleRoll  = mpu.getAngleX();
  float anglePitch = mpu.getAngleY();
  float angleYawRate = mpu.getGyroZ();  // yaw: use gyro rate (no magnetometer)

  // ── Compute delta-time ────────────────────────────────────────────────────
  uint32_t nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1e6f;
  lastLoopUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;  // guard against bad dt

  // ── Desired angles from joystick (right stick = pitch/roll setpoints) ─────
  float desiredPitch = g_pitch * 0.3f;   // ±50 joystick → ±15° setpoint
  float desiredRoll  = g_roll  * 0.3f;

  // ── PID: Roll ─────────────────────────────────────────────────────────────
  float errRoll  = desiredRoll - angleRoll;
  intRoll       += errRoll * dt;
  intRoll        = clamp((int)intRoll, -50, 50);  // anti-windup
  float dRoll    = (errRoll - errRollPrev) / dt;
  float pidRoll  = KP_ROLL*errRoll + KI_ROLL*intRoll + KD_ROLL*dRoll;
  errRollPrev    = errRoll;

  // ── PID: Pitch ────────────────────────────────────────────────────────────
  float errPitch  = desiredPitch - anglePitch;
  intPitch       += errPitch * dt;
  intPitch        = clamp((int)intPitch, -50, 50);
  float dPitch    = (errPitch - errPitchPrev) / dt;
  float pidPitch  = KP_PITCH*errPitch + KI_PITCH*intPitch + KD_PITCH*dPitch;
  errPitchPrev    = errPitch;

  // ── PID: Yaw (rate control) ───────────────────────────────────────────────
  float desiredYawRate = g_yaw * 1.5f;   // ±100 joystick → ±150 °/s target
  float errYaw   = desiredYawRate - angleYawRate;
  intYaw        += errYaw * dt;
  intYaw         = clamp((int)intYaw, -30, 30);
  float dYaw     = (errYaw - errYawPrev) / dt;
  float pidYaw   = KP_YAW*errYaw + KI_YAW*intYaw + KD_YAW*dYaw;
  errYawPrev     = errYaw;

  // ── Map throttle joystick to base PWM ─────────────────────────────────────
  // g_throttle is 0-100; map to THROTTLE_MIN..THROTTLE_MAX
  int baseThrottle = (int)((g_throttle / 100.0f) * (THROTTLE_MAX - THROTTLE_MIN) + THROTTLE_MIN);
  if (g_throttle < 2.0f) {
    cutMotors();   // fully off — no idle spin
    return;
  }

  // ── Mix PID outputs onto motors ──────────────────────────────────────────
  // Standard X-quad motor mixing:
  //   FL(CW)  = base + pitch + roll - yaw
  //   FR(CCW) = base + pitch - roll + yaw
  //   BL(CCW) = base - pitch + roll + yaw
  //   BR(CW)  = base - pitch - roll - yaw
  int fl = baseThrottle + (int)(pidPitch + pidRoll - pidYaw);
  int fr = baseThrottle + (int)(pidPitch - pidRoll + pidYaw);
  int bl = baseThrottle + (int)(-pidPitch + pidRoll + pidYaw);
  int br = baseThrottle + (int)(-pidPitch - pidRoll - pidYaw);

  setMotors(fl, fr, bl, br);

  // ── Cleanup stale WebSocket clients periodically ──────────────────────────
  static uint32_t lastCleanup = 0;
  if (millis() - lastCleanup > 2000) {
    ws.cleanupClients();
    lastCleanup = millis();
  }

  // ── Telemetry over Serial (200 Hz, every 5ms) ─────────────────────────────
  static uint32_t lastTelem = 0;
  if (millis() - lastTelem >= 5) {
    Serial.printf("R:%.1f P:%.1f | pidR:%.1f pidP:%.1f pidY:%.1f | FL:%d FR:%d BL:%d BR:%d\n",
                  angleRoll, anglePitch, pidRoll, pidPitch, pidYaw, fl, fr, bl, br);
    lastTelem = millis();
  }
}