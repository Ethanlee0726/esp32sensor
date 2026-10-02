#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "audio_config.h"
#include "i2s_audio.h"
#include "wav_writer.h"

const unsigned long WAITING_MESSAGE_INTERVAL_MS = 2000;
const unsigned long RUNNING_DURATION_MS = 60000;
unsigned long lastWaitingMessageMillis = 0;
unsigned long runningStartMillis = 0;

enum SensorState {
  STANDBY,
  RUNNING
};

SensorState currentState = STANDBY;

uint8_t startSenderMac[6] = {};
bool hasStartSenderMac = false;
volatile bool pendingUnicastTest = false;

enum PendingCommandType : uint8_t {
  PENDING_NONE,
  PENDING_START,
  PENDING_STOP
};

struct PendingCommand {
  PendingCommandType type = PENDING_NONE;
  char command[64] = {};
  char date[9] = {};
  char time[7] = {};
  uint8_t senderMac[6] = {};
  unsigned long receivedMillis = 0;
  bool queuedWhileSdNotReady = false;
  uint8_t sdRetryFailures = 0;
};

PendingCommand pendingCommand;
volatile bool pendingCommandReady = false;
portMUX_TYPE pendingCommandMux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint8_t SD_SCK_PIN = 14;
constexpr uint8_t SD_MISO_PIN = 12;
constexpr uint8_t SD_MOSI_PIN = 15;
constexpr uint8_t SD_CS_PIN = 16;
const unsigned long SD_RETRY_INTERVAL_MS = 2000;
constexpr uint8_t SD_START_RETRY_LIMIT = 10;

volatile bool sdCardReady = false;
bool sdTestAttemptFailed = false;
bool sdCardBeginAttempted = false;
bool sdSpiStarted = false;
unsigned long nextSdRetryMillis = 0;
bool startSdCheckRequired = false;
bool audioReady = false;
unsigned long sdReadyMillis = 0;
bool startPeerConfigured = false;

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint8_t ESPNOW_RETRY_LIMIT = 3;
constexpr unsigned long ESPNOW_CALLBACK_TIMEOUT_MS = 1000;
volatile bool sendCallbackReceived = false;
volatile esp_now_send_status_t sendCallbackStatus = ESP_NOW_SEND_FAIL;
volatile uint8_t sendCallbackMac[6] = {};
portMUX_TYPE sendCallbackMux = portMUX_INITIALIZER_UNLOCKED;

FsFile recordingWavFile;
FsFile eventsFile;
WavWriter wavWriter;
int16_t audioBuffer[FRAME_SAMPLES];
String recordingBaseName;
char recordingStartDate[9] = {};
char recordingStartTime[7] = {};
unsigned long recordingStartMillis = 0;

SPIClass sdSpi(FSPI);
SdFat sdCard;

enum RecordingsDirectoryStatus : uint8_t {
  RECORDINGS_READY,
  RECORDINGS_NOT_A_DIRECTORY,
  RECORDINGS_IO_ERROR
};

void reportSdInitError(const char *message) {
  Serial.printf("[SD] ERROR: %s\n", message);
  sdCard.initErrorPrint(&Serial);
}

void reportSdOperationError(const char *message) {
  Serial.printf("[SD] ERROR: %s\n", message);
  sdCard.errorPrint(&Serial);
}

void closeSdFilesForRecovery() {
  if (eventsFile.isOpen()) {
    eventsFile.close();
  }

  if (recordingWavFile.isOpen()) {
    recordingWavFile.close();
  }
}

void unmountSdCard() {
  closeSdFilesForRecovery();

  if (sdCardBeginAttempted) {
    sdCard.end();
    sdCardBeginAttempted = false;
  }

  if (sdSpiStarted) {
    sdSpi.end();
    sdSpiStarted = false;
  }

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  sdCardReady = false;
  sdReadyMillis = 0;
}

void markSdCardNeedsRemount(const char *reason) {
  Serial.printf("[SD] Remount required: %s\n", reason);

  if (currentState == RUNNING) {
    currentState = STANDBY;
    hasStartSenderMac = false;
    pendingUnicastTest = false;
  }

  recordingBaseName = "";
  unmountSdCard();
  nextSdRetryMillis = millis() + SD_RETRY_INTERVAL_MS;
}

RecordingsDirectoryStatus ensureRecordingsDirectory() {
  FsFile directory = sdCard.open("/recordings", O_RDONLY);
  if (directory) {
    const bool isDirectory = directory.isDir();
    directory.close();
    if (isDirectory) {
      return RECORDINGS_READY;
    }

    reportSdOperationError(
        "/recordings exists, but it is not a directory");
    return RECORDINGS_NOT_A_DIRECTORY;
  }

  // A failed open is ambiguous: the directory may be missing, or the card
  // may have failed to read the directory entry. Verify the root first.
  FsFile rootDirectory = sdCard.open("/", O_RDONLY);
  if (!rootDirectory || !rootDirectory.isDir()) {
    if (rootDirectory) {
      rootDirectory.close();
    }
    reportSdOperationError("Could not read SD card root directory");
    return RECORDINGS_IO_ERROR;
  }
  rootDirectory.close();

  if (!sdCard.mkdir("/recordings")) {
    reportSdOperationError("Could not create /recordings");
    return RECORDINGS_IO_ERROR;
  }

  FsFile createdDirectory = sdCard.open("/recordings", O_RDONLY);
  const bool createdDirectoryIsValid =
      createdDirectory && createdDirectory.isDir();
  if (createdDirectory) {
    createdDirectory.close();
  }

  if (!createdDirectoryIsValid) {
    reportSdOperationError(
        "Created /recordings, but could not verify it as a directory");
    return RECORDINGS_IO_ERROR;
  }

  return RECORDINGS_READY;
}

bool runSdCardTest() {
  Serial.println("[SD] Initializing SD card...");
  unmountSdCard();

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdSpiStarted = true;
  SdSpiConfig sdConfig(
      SD_CS_PIN,
      SHARED_SPI,
      SD_SCK_MHZ(4),
      &sdSpi);

  sdCardBeginAttempted = true;
  if (!sdCard.begin(sdConfig)) {
    reportSdInitError("Card mount failed");
    unmountSdCard();
    return false;
  }

  Serial.println("[SD] Card mount OK");

  if (ensureRecordingsDirectory() != RECORDINGS_READY) {
    unmountSdCard();
    return false;
  }

  Serial.println("[SD] Directory ready: /recordings");

  const char *testText = "ESP32 SD card test OK";
  const size_t testTextLength = strlen(testText);
  FsFile testFile = sdCard.open(
      "/recordings/sd_test.txt",
      O_WRONLY | O_CREAT | O_TRUNC);

  if (!testFile) {
    reportSdOperationError(
        "Could not open /recordings/sd_test.txt for writing");
    unmountSdCard();
    return false;
  }

  const size_t bytesWritten = testFile.print(testText);
  const bool testWriteSynced = testFile.sync();
  testFile.close();

  if (bytesWritten != testTextLength || !testWriteSynced) {
    reportSdOperationError("Test file write failed");
    unmountSdCard();
    return false;
  }

  Serial.println("[SD] Test file write OK");

  FsFile readFile = sdCard.open("/recordings/sd_test.txt", O_RDONLY);
  if (!readFile) {
    reportSdOperationError(
        "Could not open /recordings/sd_test.txt for reading");
    unmountSdCard();
    return false;
  }

  char readBuffer[64] = {};
  const int bytesRead = readFile.read(
      readBuffer,
      sizeof(readBuffer) - 1);
  readFile.close();

  if (bytesRead < 0 ||
      static_cast<size_t>(bytesRead) != testTextLength ||
      strcmp(readBuffer, testText) != 0) {
    reportSdOperationError("Test file read-back content mismatch");
    unmountSdCard();
    return false;
  }

  Serial.println("[SD] Test file read OK");
  Serial.printf("[SD] Read back: %s\n", readBuffer);
  Serial.println("[SD] SD TEST PASSED");
  sdCardReady = true;
  sdReadyMillis = millis();
  Serial.printf("[TIME] SD ready millis: %lu\n", sdReadyMillis);
  return true;
}

bool runSdCardTestWithRetries() {
  sdTestAttemptFailed = false;

  if (!startSdCheckRequired && (sdCardReady || millis() < nextSdRetryMillis)) {
    return sdCardReady;
  }

  if (millis() < nextSdRetryMillis) {
    return sdCardReady;
  }

  if (runSdCardTest()) {
    nextSdRetryMillis = 0;
    startSdCheckRequired = false;
    return true;
  }

  sdTestAttemptFailed = true;
  nextSdRetryMillis = millis() + SD_RETRY_INTERVAL_MS;
  return false;
}

void reportRecordingError(const char *message) {
  Serial.printf("[REC] ERROR: %s\n", message);
  sdCard.errorPrint(&Serial);
}

bool isFixedDigitString(const char *value, size_t length) {
  if (strlen(value) != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (value[i] < '0' || value[i] > '9') {
      return false;
    }
  }

  return true;
}

bool parseTimedCommand(const uint8_t *incomingData,
                       int length,
                       const char *expectedEvent,
                       char *dateOut,
                       char *timeOut) {
  if (length <= 0 || length >= 64) {
    return false;
  }

  char packet[64] = {};
  memcpy(packet, incomingData, static_cast<size_t>(length));
  packet[length] = '\0';

  char *firstComma = strchr(packet, ',');
  if (firstComma == nullptr) {
    return false;
  }
  *firstComma = '\0';

  char *secondComma = strchr(firstComma + 1, ',');
  if (secondComma == nullptr) {
    return false;
  }
  *secondComma = '\0';

  const char *date = firstComma + 1;
  const char *time = secondComma + 1;
  if (strcmp(packet, expectedEvent) != 0 ||
      !isFixedDigitString(date, 8) ||
      !isFixedDigitString(time, 6)) {
    return false;
  }

  memcpy(dateOut, date, 9);
  memcpy(timeOut, time, 7);
  return true;
}

bool queuePendingCommand(PendingCommandType type,
                         const char *date,
                         const char *time,
                         const uint8_t *senderMac,
                         unsigned long receivedMillis) {
  bool queuedWhileSdNotReady =
      type == PENDING_START && !sdCardReady;

  portENTER_CRITICAL(&pendingCommandMux);

  // Keep an early START until the SD retry loop can process it.  A duplicate
  // START must not replace or execute the already queued request.
  if (pendingCommandReady &&
      (pendingCommand.type == PENDING_START || type == PENDING_START)) {
    portEXIT_CRITICAL(&pendingCommandMux);
    return false;
  }

  pendingCommand.type = type;
  snprintf(pendingCommand.command,
           sizeof(pendingCommand.command),
           "%s,%s,%s",
           type == PENDING_START ? "START" : "STOP",
           date,
           time);
  memcpy(pendingCommand.date, date, sizeof(pendingCommand.date));
  memcpy(pendingCommand.time, time, sizeof(pendingCommand.time));
  memcpy(pendingCommand.senderMac, senderMac, sizeof(pendingCommand.senderMac));
  pendingCommand.receivedMillis = receivedMillis;
  pendingCommand.queuedWhileSdNotReady = queuedWhileSdNotReady;
  pendingCommand.sdRetryFailures = 0;
  pendingCommandReady = true;
  portEXIT_CRITICAL(&pendingCommandMux);
  return true;
}

bool peekPendingCommand(PendingCommand &command) {
  portENTER_CRITICAL(&pendingCommandMux);
  if (!pendingCommandReady) {
    portEXIT_CRITICAL(&pendingCommandMux);
    return false;
  }

  command = pendingCommand;
  portEXIT_CRITICAL(&pendingCommandMux);
  return true;
}

bool takePendingCommand(PendingCommand &command) {
  portENTER_CRITICAL(&pendingCommandMux);
  if (!pendingCommandReady) {
    portEXIT_CRITICAL(&pendingCommandMux);
    return false;
  }

  command = pendingCommand;
  pendingCommandReady = false;
  portEXIT_CRITICAL(&pendingCommandMux);
  return true;
}

bool writeEventRecord(const char *eventName,
                      const char *date,
                      const char *time,
                      unsigned long eventMillis) {
  if (!eventsFile.isOpen()) {
    reportRecordingError("Events file is not open");
    return false;
  }

  char line[96] = {};
  const int lineLength = snprintf(
      line,
      sizeof(line),
      "%s,%s,%s,%lu\n",
      eventName,
      date,
      time,
      eventMillis);

  if (lineLength <= 0 || static_cast<size_t>(lineLength) >= sizeof(line)) {
    reportRecordingError("Event record formatting failed");
    return false;
  }

  const size_t bytesWritten = eventsFile.print(line);
  if (bytesWritten != static_cast<size_t>(lineLength) ||
      !eventsFile.sync()) {
    reportRecordingError("Event record write failed");
    return false;
  }

  return true;
}

bool beginRecording(const char *date, const char *time) {
  if (!sdCardReady) {
    Serial.println("[REC] ERROR: SD card is not mounted");
    return false;
  }

  if (!audioReady) {
    Serial.println("[I2S] Microphone is not ready; recording not started");
    return false;
  }

  const RecordingsDirectoryStatus directoryStatus =
      ensureRecordingsDirectory();
  if (directoryStatus == RECORDINGS_IO_ERROR) {
    markSdCardNeedsRemount("/recordings I/O failure");
    return false;
  }
  if (directoryStatus == RECORDINGS_NOT_A_DIRECTORY) {
    markSdCardNeedsRemount("/recordings is not a directory");
    return false;
  }

  recordingBaseName = String(date) + "_" + String(time);
  Serial.printf("[TIME] WAV filename datetime: %s %s\n", date, time);
  Serial.printf("[TIME] SD ready millis: %lu\n", sdReadyMillis);
  const String wavPath =
      String("/recordings/") + recordingBaseName + ".wav";
  const String eventsPath =
      String("/recordings/") + recordingBaseName + "_events.csv";

  recordingWavFile = sdCard.open(
      wavPath.c_str(),
      O_WRONLY | O_CREAT | O_TRUNC);
  if (!recordingWavFile) {
    reportRecordingError("Could not open WAV file");
    markSdCardNeedsRemount("WAV open failure");
    return false;
  }

  if (!wavWriter.begin(recordingWavFile,
                       SAMPLE_RATE,
                       CHANNELS,
                       BITS_PER_SAMPLE)) {
    reportRecordingError("Could not write WAV header");
    markSdCardNeedsRemount("WAV header write failure");
    return false;
  }

  eventsFile = sdCard.open(
      eventsPath.c_str(),
      O_WRONLY | O_CREAT | O_TRUNC);
  if (!eventsFile) {
    reportRecordingError("Could not open events CSV file");
    markSdCardNeedsRemount("events CSV open failure");
    return false;
  }

  const char *header = "event,date,time,millis\n";
  if (eventsFile.print(header) != strlen(header) ||
      !eventsFile.sync()) {
    reportRecordingError("Could not write events CSV header");
    markSdCardNeedsRemount("events CSV header write failure");
    return false;
  }

  const unsigned long startMillis = millis();
  if (!writeEventRecord("START", date, time, startMillis)) {
    markSdCardNeedsRemount("START event write failure");
    return false;
  }

  memcpy(recordingStartDate, date, sizeof(recordingStartDate));
  memcpy(recordingStartTime, time, sizeof(recordingStartTime));
  recordingStartMillis = startMillis;
  runningStartMillis = startMillis;
  Serial.printf("[TIME] Actual recording start millis: %lu\n",
                recordingStartMillis);
  Serial.printf("[REC] Started: %s\n", wavPath.c_str());
  Serial.printf("[REC] Events: %s\n", eventsPath.c_str());
  return true;
}

bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

int daysInMonth(int year, int month) {
  static const int days[] = {
      0, 31, 28, 31, 30, 31, 30,
      31, 31, 30, 31, 30, 31};
  if (month == 2 && isLeapYear(year)) {
    return 29;
  }
  return days[month];
}

void makeEndTimestamp(char *dateOut, char *timeOut, unsigned long eventMillis) {
  int year = atoi(String(recordingStartDate).substring(0, 4).c_str());
  int month = atoi(String(recordingStartDate).substring(4, 6).c_str());
  int day = atoi(String(recordingStartDate).substring(6, 8).c_str());
  int hour = atoi(String(recordingStartTime).substring(0, 2).c_str());
  int minute = atoi(String(recordingStartTime).substring(2, 4).c_str());
  int second = atoi(String(recordingStartTime).substring(4, 6).c_str());

  unsigned long elapsedSeconds =
      (eventMillis - recordingStartMillis) / 1000UL;
  second += static_cast<int>(elapsedSeconds % 60UL);
  minute += static_cast<int>((elapsedSeconds / 60UL) % 60UL);
  hour += static_cast<int>((elapsedSeconds / 3600UL) % 24UL);

  while (second >= 60) {
    second -= 60;
    ++minute;
  }
  while (minute >= 60) {
    minute -= 60;
    ++hour;
  }
  while (hour >= 24) {
    hour -= 24;
    ++day;
  }
  while (month >= 1 && month <= 12 && day > daysInMonth(year, month)) {
    day -= daysInMonth(year, month);
    ++month;
    if (month > 12) {
      month = 1;
      ++year;
    }
  }

  snprintf(dateOut, 9, "%04d%02d%02d", year, month, day);
  snprintf(timeOut, 7, "%02d%02d%02d", hour, minute, second);
}

bool verifyRecordingFiles() {
  const String wavPath =
      String("/recordings/") + recordingBaseName + ".wav";
  const String eventsPath =
      String("/recordings/") + recordingBaseName + "_events.csv";

  FsFile wavFile = sdCard.open(wavPath.c_str(), O_RDONLY);
  const bool wavOk = wavFile && wavFile.size() >= sizeof(WavHeader);
  const uint32_t wavSize = wavFile ? wavFile.size() : 0;
  if (wavFile) {
    wavFile.close();
  }

  FsFile eventFile = sdCard.open(eventsPath.c_str(), O_RDONLY);
  const bool eventsOk = eventFile && eventFile.size() > 0;
  const uint32_t eventsSize = eventFile ? eventFile.size() : 0;
  if (eventFile) {
    eventFile.close();
  }

  Serial.printf("[REC] File verify: WAV=%s (%lu bytes), EVENTS=%s (%lu bytes)\n",
                wavOk ? "OK" : "FAIL",
                static_cast<unsigned long>(wavSize),
                eventsOk ? "OK" : "FAIL",
                static_cast<unsigned long>(eventsSize));
  return wavOk && eventsOk;
}

void beepRecordingSuccess() {
  tone(BUZZER_PIN, 2000);
  delay(300);
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

bool finishRecording(const char *eventName,
                     const char *date,
                     const char *time,
                     unsigned long eventMillis) {
  if (!writeEventRecord(eventName, date, time, eventMillis)) {
    markSdCardNeedsRemount("final event write failure");
    return false;
  }

  if (!wavWriter.finalize(recordingWavFile)) {
    reportRecordingError("Could not finalize WAV header");
    markSdCardNeedsRemount("WAV finalize failure");
    return false;
  }
  recordingWavFile.close();

  if (eventsFile.isOpen()) {
    if (!eventsFile.sync()) {
      reportRecordingError("Could not sync events CSV");
      markSdCardNeedsRemount("events CSV sync failure");
      return false;
    }
    eventsFile.close();
  }

  if (!verifyRecordingFiles()) {
    reportRecordingError("Recording file verification failed");
    markSdCardNeedsRemount("recording file verification failure");
    return false;
  }

  beepRecordingSuccess();

  const unsigned long wavCompleteMillis = millis();
  char wavCompleteDate[9] = {};
  char wavCompleteTime[7] = {};
  makeEndTimestamp(wavCompleteDate, wavCompleteTime, wavCompleteMillis);
  Serial.printf("[TIME] WAV complete millis: %lu\n", wavCompleteMillis);
  Serial.printf("[TIME] WAV complete datetime: %s %s\n",
                wavCompleteDate, wavCompleteTime);

  Serial.printf("[REC] Recording stopped: %s\n", eventName);
  recordingBaseName = "";
  return true;
}

void printTimestampPrefix() {
  const unsigned long now = millis();
  const unsigned long minutes = now / 60000UL;
  const unsigned long seconds = (now / 1000UL) % 60UL;
  const unsigned long milliseconds = now % 1000UL;
  Serial.printf("[%02lu:%02lu.%03lu | %lu ms] ",
                minutes, seconds, milliseconds, now);
}

void printMacLine(const char *label, const uint8_t *macAddress) {
  Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n",
                label,
                macAddress[0], macAddress[1], macAddress[2],
                macAddress[3], macAddress[4], macAddress[5]);
}

void printPacketDetails(const uint8_t *data, size_t length) {
  Serial.printf("Length: %u\n", static_cast<unsigned>(length));
  Serial.print("Data: ");
  for (size_t i = 0; i < length; ++i) {
    const uint8_t value = data[i];
    Serial.write((value >= 32 && value <= 126) ? value : '.');
  }
  Serial.println();

  Serial.print("Raw:");
  for (size_t i = 0; i < length; ++i) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
}

bool recreatePeerForStartSender(const uint8_t *senderMac) {
  if (senderMac == nullptr) {
    return false;
  }

  printTimestampPrefix();
  Serial.println("START received: refreshing M5 peer");
  printMacLine("START source MAC: ", senderMac);

  if (hasStartSenderMac &&
      memcmp(startSenderMac, senderMac, sizeof(startSenderMac)) != 0 &&
      esp_now_is_peer_exist(startSenderMac)) {
    const esp_err_t oldPeerResult = esp_now_del_peer(startSenderMac);
    Serial.printf("Old M5 peer delete result: %d\n", oldPeerResult);
  }

  if (esp_now_is_peer_exist(senderMac)) {
    const esp_err_t existingPeerResult = esp_now_del_peer(senderMac);
    Serial.printf("Existing M5 peer delete result: %d\n",
                  existingPeerResult);
    if (existingPeerResult != ESP_OK &&
        existingPeerResult != ESP_ERR_ESPNOW_NOT_FOUND) {
      return false;
    }
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, senderMac, sizeof(peerInfo.peer_addr));
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  Serial.printf("M5 peer settings: channel=%d ifidx=WIFI_IF_STA encrypt=false\n",
                peerInfo.channel);
  const esp_err_t peerResult = esp_now_add_peer(&peerInfo);
  Serial.printf("New M5 peer add result: %d\n", peerResult);
  if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
    return false;
  }

  memcpy(startSenderMac, senderMac, sizeof(startSenderMac));
  hasStartSenderMac = true;
  return esp_now_is_peer_exist(senderMac);
}

bool ensurePeer(const uint8_t *targetMac) {
  if (esp_now_is_peer_exist(targetMac)) {
    return true;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, targetMac, sizeof(peerInfo.peer_addr));
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  const esp_err_t peerResult = esp_now_add_peer(&peerInfo);
  Serial.printf("Peer add result: %d\n", peerResult);
  return (peerResult == ESP_OK || peerResult == ESP_ERR_ESPNOW_EXIST) &&
         esp_now_is_peer_exist(targetMac);
}

const char *sendStatusName(bool success) {
  return success ? "SUCCESS" : "FAIL";
}

bool sendPacketWithCallback(const char *packetName,
                            const uint8_t *targetMac,
                            const uint8_t *payload,
                            size_t payloadLength,
                            uint8_t retryLimit) {
  if (targetMac == nullptr || !ensurePeer(targetMac)) {
    printTimestampPrefix();
    Serial.printf("[ESPNOW] %s send failed: peer unavailable\n", packetName);
    return false;
  }

  for (uint8_t attempt = 1; attempt <= retryLimit; ++attempt) {
    portENTER_CRITICAL(&sendCallbackMux);
    sendCallbackReceived = false;
    sendCallbackStatus = ESP_NOW_SEND_FAIL;
    memset((void *)sendCallbackMac, 0, sizeof(sendCallbackMac));
    portEXIT_CRITICAL(&sendCallbackMux);

    printTimestampPrefix();
    Serial.printf("[ESPNOW] packet=%s attempt=%u/%u\n",
                  packetName, attempt, retryLimit);
    printMacLine("Target MAC: ", targetMac);
    Serial.printf("Channel: %d\n", ESPNOW_CHANNEL);
    printPacketDetails(payload, payloadLength);

    const esp_err_t sendResult = esp_now_send(
        targetMac,
        payload,
        payloadLength);
    Serial.printf("esp_now_send return: %d\n", sendResult);

    bool callbackReceived = false;
    esp_now_send_status_t callbackStatus = ESP_NOW_SEND_FAIL;
    unsigned long waitStartMillis = millis();
    while (millis() - waitStartMillis < ESPNOW_CALLBACK_TIMEOUT_MS) {
      uint8_t callbackMac[6] = {};
      portENTER_CRITICAL(&sendCallbackMux);
      callbackReceived = sendCallbackReceived;
      if (callbackReceived) {
        callbackStatus = sendCallbackStatus;
        memcpy(callbackMac, (const void *)sendCallbackMac, sizeof(callbackMac));
      }
      portEXIT_CRITICAL(&sendCallbackMux);
      if (callbackReceived && memcmp(callbackMac, targetMac, sizeof(callbackMac)) != 0) {
        callbackReceived = false;
        portENTER_CRITICAL(&sendCallbackMux);
        sendCallbackReceived = false;
        portEXIT_CRITICAL(&sendCallbackMux);
      }
      if (callbackReceived) {
        break;
      }
      yield();
    }

    if (sendResult != ESP_OK) {
      Serial.printf("callback status: FAIL (send return=%d)\n", sendResult);
    } else if (!callbackReceived) {
      Serial.println("callback status: FAIL (timeout)");
    } else {
      Serial.printf("callback status: %s\n",
                    sendStatusName(callbackStatus == ESP_NOW_SEND_SUCCESS));
    }

    if (sendResult == ESP_OK && callbackReceived &&
        callbackStatus == ESP_NOW_SEND_SUCCESS) {
      printTimestampPrefix();
      Serial.printf("[ESPNOW] %s send confirmed SUCCESS\n", packetName);
      return true;
    }

    if (attempt < retryLimit) {
      Serial.printf("[ESPNOW] %s retrying after failed callback\n",
                    packetName);
    }
  }

  printTimestampPrefix();
  Serial.printf("[ESPNOW] %s send failed after %u attempts\n",
                packetName, retryLimit);
  return false;
}

bool sendErrorToSender(const uint8_t *senderMac) {
  if (senderMac == nullptr) {
    printTimestampPrefix();
    Serial.println("ERROR send failed: START sender MAC unavailable");
    return false;
  }

  const char *errorMessage = "ERROR";
  return sendPacketWithCallback(
      "ERROR",
      senderMac,
      reinterpret_cast<const uint8_t *>(errorMessage),
      strlen(errorMessage),
      ESPNOW_RETRY_LIMIT);
}

bool recordPendingStartRetry(PendingCommand &exhaustedCommand) {
  bool retryRecorded = false;
  bool retryLimitReached = false;
  uint8_t retryCount = 0;

  portENTER_CRITICAL(&pendingCommandMux);
  if (pendingCommandReady &&
      pendingCommand.type == PENDING_START &&
      !sdCardReady) {
    ++pendingCommand.sdRetryFailures;
    retryCount = pendingCommand.sdRetryFailures;
    retryRecorded = true;
    if (pendingCommand.sdRetryFailures >= SD_START_RETRY_LIMIT) {
      exhaustedCommand = pendingCommand;
      pendingCommandReady = false;
      retryLimitReached = true;
    }
  }
  portEXIT_CRITICAL(&pendingCommandMux);

  if (retryRecorded) {
    Serial.printf("[REC] START retry %u/%u\n",
                  retryCount,
                  SD_START_RETRY_LIMIT);
  }

  return retryLimitReached;
}

void failQueuedStartAfterSdRetries() {
  if (!sdTestAttemptFailed) {
    return;
  }

  PendingCommand command;
  if (recordPendingStartRetry(command)) {
    startSdCheckRequired = false;
    printTimestampPrefix();
    Serial.println("START failed: SD card initialization retries exhausted");
    sendErrorToSender(command.senderMac);
  }
}

void sendUnicastTest() {
  if (!hasStartSenderMac) {
    printTimestampPrefix();
    Serial.println("TEST send failed: START sender MAC unavailable");
    return;
  }

  const char *unicastMessage = "TEST";
  sendPacketWithCallback(
      "TEST",
      startSenderMac,
      reinterpret_cast<const uint8_t *>(unicastMessage),
      strlen(unicastMessage),
      1);
}

void OnDataSent(const uint8_t *macAddress, esp_now_send_status_t status) {
  portENTER_CRITICAL(&sendCallbackMux);
  memcpy((void *)sendCallbackMac, macAddress, sizeof(sendCallbackMac));
  sendCallbackStatus = status;
  sendCallbackReceived = true;
  portEXIT_CRITICAL(&sendCallbackMux);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  char eventDate[9] = {};
  char eventTime[7] = {};
  const unsigned long receivedMillis = millis();

  if (parseTimedCommand(incomingData,
                        len,
                        "START",
                        eventDate,
                        eventTime)) {
    Serial.printf("[TIME] START received ESP32 millis: %lu\n",
                  receivedMillis);
    Serial.printf("[TIME] M5 START datetime: %s %s\n",
                  eventDate, eventTime);
    const bool queued =
        queuePendingCommand(PENDING_START,
                            eventDate,
                            eventTime,
                            mac,
                            receivedMillis);
    if (queued) {
      startPeerConfigured = false;
      startSdCheckRequired = true;
      if (!sdCardReady) {
        Serial.println("START queued: SD card not ready");
      }
    }
  } else if (parseTimedCommand(incomingData,
                               len,
                               "STOP",
                               eventDate,
                               eventTime)) {
    queuePendingCommand(PENDING_STOP,
                        eventDate,
                        eventTime,
                        mac,
                        receivedMillis);
  }
}

void processPendingCommand() {
  PendingCommand command;
  if (!peekPendingCommand(command)) {
    return;
  }

  if (command.type == PENDING_START) {
    if (currentState != STANDBY) {
      takePendingCommand(command);
      printTimestampPrefix();
      Serial.println("START ignored: already RUNNING");
      return;
    }

    if (!startPeerConfigured) {
      if (!recreatePeerForStartSender(command.senderMac)) {
        printTimestampPrefix();
        Serial.println("START failed: could not configure M5 peer");
        return;
      }
      startPeerConfigured = true;
    }

    // Leave the complete START command in the queue until the background SD
    // retry has succeeded.  It is consumed exactly once below.
    if (startSdCheckRequired || !sdCardReady) {
      return;
    }

    if (command.queuedWhileSdNotReady) {
      printTimestampPrefix();
      Serial.println("Processing queued START");
    }

    memcpy(startSenderMac, command.senderMac, sizeof(startSenderMac));
    hasStartSenderMac = true;
    Serial.printf("[TIME] START received ESP32 millis: %lu\n",
                  command.receivedMillis);
    Serial.printf("[TIME] SD ready millis: %lu\n", sdReadyMillis);
    Serial.printf("[TIME] M5 START datetime retained: %s %s\n",
                  command.date, command.time);
    Serial.println("[REC] SD card ready");
    Serial.println("[REC] /recordings directory ready");
    Serial.printf("[REC] START retry count: %u\n",
                  command.sdRetryFailures);

    if (!beginRecording(command.date, command.time)) {
      if (!sdCardReady) {
        startSdCheckRequired = true;
        sdTestAttemptFailed = false;
        PendingCommand exhaustedCommand;
        if (recordPendingStartRetry(exhaustedCommand)) {
          startSdCheckRequired = false;
          printTimestampPrefix();
          Serial.println("START failed: SD card initialization retries exhausted");
          sendErrorToSender(exhaustedCommand.senderMac);
        }
        return;
      }

      startSdCheckRequired = false;
      if (!takePendingCommand(command)) {
        return;
      }
      printTimestampPrefix();
      Serial.println("START failed");
      sendErrorToSender(command.senderMac);
      hasStartSenderMac = false;
      startPeerConfigured = false;
      return;
    }

    if (!takePendingCommand(command)) {
      return;
    }

    currentState = RUNNING;
    pendingUnicastTest = true;
    printTimestampPrefix();
    Serial.println("Recording started");
    printTimestampPrefix();
    Serial.println("START received");
    printTimestampPrefix();
    Serial.print("START sender MAC: ");
    printMacLine("", command.senderMac);
    printTimestampPrefix();
    Serial.println("RX START recognized");
    Serial.println("State -> RUNNING");
    printTimestampPrefix();
    Serial.println("pending TEST set");
    return;
  }

  if (command.type == PENDING_STOP) {
    if (!takePendingCommand(command)) {
      return;
    }

    if (currentState != RUNNING) {
      printTimestampPrefix();
      Serial.println("STOP ignored: already STANDBY");
      return;
    }

    const unsigned long stopMillis = millis();
    char endDate[9] = {};
    char endTime[7] = {};
    makeEndTimestamp(endDate, endTime, stopMillis);
    uint8_t responseMac[6] = {};
    const bool responseMacAvailable = hasStartSenderMac;
    if (responseMacAvailable) {
      memcpy(responseMac, startSenderMac, sizeof(responseMac));
    }
    const bool recordingFinished =
        finishRecording("STOP", endDate, endTime, stopMillis);
    currentState = STANDBY;
    printTimestampPrefix();
    Serial.println("RX STOP recognized");
    Serial.println("State -> STANDBY");
    if (!recordingFinished && responseMacAvailable) {
      sendErrorToSender(responseMac);
    } else if (!recordingFinished) {
      Serial.println("Recording stopped with SD error; remount required");
    }
    hasStartSenderMac = false;
    startPeerConfigured = false;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  audioReady = i2s_up();
  if (!audioReady) {
    Serial.println("[I2S] Audio recording unavailable");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  const esp_err_t powerSaveResult = esp_wifi_set_ps(WIFI_PS_NONE);
  printTimestampPrefix();
  Serial.printf("WiFi sleep disabled result: %d\n", powerSaveResult);
  esp_err_t channelResult =
      esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  printTimestampPrefix();
  Serial.printf("WiFi channel set result: %d\n", channelResult);
  printTimestampPrefix();
  Serial.printf("WiFi Channel: %d\n", WiFi.channel());

  String sensorMac = WiFi.macAddress();
  sensorMac.toUpperCase();
  printTimestampPrefix();
  Serial.printf("ESP32 STA MAC: %s\n", sensorMac.c_str());
  printTimestampPrefix();
  Serial.printf("ESP32 WiFi Channel: %d\n", WiFi.channel());

  printTimestampPrefix();
  Serial.println("ESP32 Sensor Ready");
  printTimestampPrefix();
  Serial.printf("ESP32 Sensor MAC: %s\n", sensorMac.c_str());

  if (esp_now_init() == ESP_OK) {
    printTimestampPrefix();
    Serial.println("ESP-NOW init OK");
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    printTimestampPrefix();
    Serial.println("ESP-NOW Receiver Ready");
    printTimestampPrefix();
    Serial.println("Waiting for command...");
  } else {
    printTimestampPrefix();
    Serial.println("ESP-NOW Init Failed");
  }
}

void loop() {
  runSdCardTestWithRetries();
  failQueuedStartAfterSdRetries();

  processPendingCommand();

  if (pendingUnicastTest) {
    pendingUnicastTest = false;
    printTimestampPrefix();
    Serial.println("loop: sending TEST");
    sendUnicastTest();
  }

  if (currentState == RUNNING) {
    unsigned long now = millis();

    if (audioReady && recordingWavFile.isOpen()) {
      const size_t bytesRead = i2s_read_samples(audioBuffer, FRAME_BYTES);
      if (bytesRead > 0) {
        const size_t samplesRead = bytesRead / sizeof(int16_t);
        if (!wavWriter.writeData(recordingWavFile,
                                 audioBuffer,
                                 samplesRead)) {
          uint8_t responseMac[6] = {};
          const bool responseMacAvailable = hasStartSenderMac;
          if (responseMacAvailable) {
            memcpy(responseMac, startSenderMac, sizeof(responseMac));
          }
          reportRecordingError("WAV audio data write failed");
          markSdCardNeedsRemount("WAV audio data write failure");
          if (responseMacAvailable) {
            sendErrorToSender(responseMac);
          }
          return;
        }
      }
    }

    if (now - runningStartMillis >= RUNNING_DURATION_MS) {
      char endDate[9] = {};
      char endTime[7] = {};
      makeEndTimestamp(endDate, endTime, now);
      uint8_t responseMac[6] = {};
      const bool responseMacAvailable = hasStartSenderMac;
      if (responseMacAvailable) {
        memcpy(responseMac, startSenderMac, sizeof(responseMac));
      }
      const bool recordingFinished =
          finishRecording("END", endDate, endTime, now);
      currentState = STANDBY;
      printTimestampPrefix();
      Serial.println("60 seconds completed");
      printTimestampPrefix();
      Serial.println(recordingFinished
                         ? "Recording finished"
                         : "Recording stopped with SD error");
      Serial.println("State -> STANDBY");
      if (!recordingFinished && responseMacAvailable) {
        sendErrorToSender(responseMac);
      }
      hasStartSenderMac = false;
      startPeerConfigured = false;
    }
  }

  unsigned long now = millis();
  if (now - lastWaitingMessageMillis >= WAITING_MESSAGE_INTERVAL_MS) {
    lastWaitingMessageMillis = now;
    String sensorMac = WiFi.macAddress();
    sensorMac.toUpperCase();
    printTimestampPrefix();
    Serial.println("Waiting for command...");
    printTimestampPrefix();
    Serial.printf("ESP32 Sensor MAC: %s\n", sensorMac.c_str());
  }
}
