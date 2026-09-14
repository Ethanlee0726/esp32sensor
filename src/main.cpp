#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

const unsigned long WAITING_MESSAGE_INTERVAL_MS = 2000;
unsigned long lastWaitingMessageMillis = 0;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print("Received: ");
  Serial.write(incomingData, len);
  Serial.println();
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

  if (now - lastWaitingMessageMillis >= WAITING_MESSAGE_INTERVAL_MS) {
    lastWaitingMessageMillis = now;
    Serial.println("Waiting for command...");
    Serial.print("ESP32 Sensor MAC: ");
    Serial.println(WiFi.macAddress());
  }
}
