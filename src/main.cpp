#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

const unsigned long WAITING_MESSAGE_INTERVAL_MS = 2000;
const unsigned long RUNNING_DURATION_MS = 30000;
unsigned long lastWaitingMessageMillis = 0;
unsigned long runningStartMillis = 0;

enum SensorState {
  STANDBY,
  RUNNING
};

SensorState currentState = STANDBY;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print("Received: ");
  Serial.write(incomingData, len);
  Serial.println();

  if (len == 5 && memcmp(incomingData, "START", 5) == 0) {
    if (currentState == STANDBY) {
      currentState = RUNNING;
      runningStartMillis = millis();
      Serial.println("START received");
      Serial.println("State: RUNNING");
    } else {
      Serial.println("START ignored: already RUNNING");
    }
  } else if (len == 4 && memcmp(incomingData, "STOP", 4) == 0) {
    if (currentState == RUNNING) {
      currentState = STANDBY;
      Serial.println("STOP received");
      Serial.println("State: STANDBY");
    } else {
      Serial.println("STOP ignored: already STANDBY");
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  Serial.println("ESP32 Sensor Ready");
  Serial.print("ESP32 Sensor MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("ESP-NOW Receiver Ready");
    Serial.println("Waiting for command...");
  } else {
    Serial.println("ESP-NOW Init Failed");
  }
}

void loop() {
  unsigned long now = millis();

  if (currentState == RUNNING &&
      now - runningStartMillis >= RUNNING_DURATION_MS) {
    currentState = STANDBY;
    Serial.println("30 seconds completed");
    Serial.println("State: STANDBY");
  }

  if (now - lastWaitingMessageMillis >= WAITING_MESSAGE_INTERVAL_MS) {
    lastWaitingMessageMillis = now;
    Serial.println("Waiting for command...");
    Serial.print("ESP32 Sensor MAC: ");
    Serial.println(WiFi.macAddress());
  }
}
