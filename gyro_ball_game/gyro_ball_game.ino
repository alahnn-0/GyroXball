/*
  GYRO BALL — ESP32 + MPU6050 Tilt Game Server
  ----------------------------------------------
  Reads pitch/roll from an MPU6050 and streams it over a WebSocket
  to a web page (served from this same ESP32) that renders a
  bouncing-ball balance game.

  WIRING (ESP32 <-> MPU6050, I2C):
    MPU6050 VCC  -> ESP32 3V3
    MPU6050 GND  -> ESP32 GND
    MPU6050 SCL  -> ESP32 GPIO 22  (default I2C SCL)
    MPU6050 SDA  -> ESP32 GPIO 21  (default I2C SDA)

  LIBRARIES REQUIRED (install via Arduino Library Manager):
    - "Adafruit MPU6050"        by Adafruit
    - "Adafruit Unified Sensor" by Adafruit (dependency)
    - "ESPAsyncWebServer"       by lacamera / ESP32Async (or me-no-dev fork)
    - "AsyncTCP"                by ESP32Async / me-no-dev (dependency for ESP32)

  HOW TO USE:
    1. Fill in WIFI_SSID and WIFI_PASSWORD below.
    2. Flash this sketch to your ESP32.
    3. Open Serial Monitor (115200 baud) - it will print the IP address
       once connected, e.g. "Game ready at: http://192.168.1.45"
    4. Open that address in a browser on any device on the same WiFi network.
    5. Tilt the breadboard/sensor to roll the ball and keep it from
       falling off the edges!

  NOTE: The full game HTML/CSS/JS lives in data.h (PROGMEM string),
        generated alongside this sketch. Make sure data.h is in the
        same folder as this .ino file.
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESPAsyncWebServer.h>
#include "./data.h"   // contains GAME_HTML (the web page, stored in flash)

// ----------------- USER CONFIG -----------------
const char* WIFI_SSID     = "RABBITSQUARE_4G";
const char* WIFI_PASSWORD = "rsq@9846";

// How often to push sensor data to the browser (milliseconds)
const uint32_t SEND_INTERVAL_MS = 20;   // ~50Hz, smooth and responsive

// Complementary filter strength (0..1). Higher = trust gyro more (smoother
// but can drift), Lower = trust accelerometer more (noisier but stable).
const float FILTER_ALPHA = 0.96f;

// Calibration: set true once at boot to zero out resting sensor bias
const bool AUTO_CALIBRATE_ON_BOOT = true;
const int  CALIBRATION_SAMPLES    = 400;
// -------------------------------------------------

Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

float pitch = 0.0f;     // tilt forward/back, degrees
float roll  = 0.0f;     // tilt left/right, degrees
float pitchOffset = 0.0f;
float rollOffset  = 0.0f;

uint32_t lastSendTime = 0;
uint32_t lastSensorTime = 0;

// ---------- WebSocket event handling ----------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

// ---------- Calibration: average resting tilt to use as zero point ----------
void calibrateSensor() {
  Serial.println("Calibrating... keep the board FLAT and STILL.");
  double pitchSum = 0, rollSum = 0;
  int validSamples = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float p = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float r = atan2(ay, az) * 180.0 / PI;

    pitchSum += p;
    rollSum += r;
    validSamples++;
    delay(5);
  }

  pitchOffset = pitchSum / validSamples;
  rollOffset  = rollSum / validSamples;
  Serial.printf("Calibration done. Offsets -> pitch: %.2f  roll: %.2f\n",
                pitchOffset, rollOffset);
}

void setup() {
  Serial.begin(9600);
  delay(300);
  Serial.println("\n=== GYRO BALL GAME SERVER ===");

  // ---- I2C + MPU6050 init ----
  Wire.begin(21, 22);   // SDA, SCL (default ESP32 pins)
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring (SDA=21, SCL=22) and power.");
    while (1) { delay(1000); }
  }
  Serial.println("MPU6050 connected.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (AUTO_CALIBRATE_ON_BOOT) {
    calibrateSensor();
  }

  // ---- WiFi ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Game ready at: http://");
  Serial.println(WiFi.localIP());

  // ---- Web server routes ----
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", GAME_HTML);
  });

  // Simple JSON fallback endpoint (polling, if WebSocket unavailable)
  server.on("/tilt", HTTP_GET, [](AsyncWebServerRequest *request) {
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"pitch\":%.2f,\"roll\":%.2f}", pitch, roll);
    request->send(200, "application/json", buf);
  });

  server.begin();
  lastSensorTime = millis();
}

void loop() {
  uint32_t now = millis();

  // ---- Read sensor + complementary filter fusion ----
  // Run sensor fusion every loop iteration for accuracy, but only
  // broadcast over WebSocket at SEND_INTERVAL_MS to avoid flooding clients.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float dt = (now - lastSensorTime) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  lastSensorTime = now;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float gx = g.gyro.x * 180.0 / PI;   // rad/s -> deg/s
  float gy = g.gyro.y * 180.0 / PI;

  // Accelerometer-derived angle (absolute, but noisy)
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float accRoll  = atan2(ay, az) * 180.0 / PI;

  // Complementary filter: blend gyro integration (smooth) with accel (stable)
  pitch = FILTER_ALPHA * (pitch + gy * dt) + (1.0f - FILTER_ALPHA) * accPitch;
  roll  = FILTER_ALPHA * (roll  + gx * dt) + (1.0f - FILTER_ALPHA) * accRoll;

  float pitchOut = pitch - pitchOffset;
  float rollOut  = roll  - rollOffset;

  // ---- Broadcast to connected browser(s) ----
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;
    if (ws.count() > 0) {
      char msg[96];
      snprintf(msg, sizeof(msg), "{\"pitch\":%.2f,\"roll\":%.2f}", pitchOut, rollOut);
      ws.textAll(msg);
    }
  }
}
