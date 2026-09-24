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
  char date[9] = {};
  char time[7] = {};
  uint8_t senderMac[6] = {};
};

PendingCommand pendingCommand;
volatile bool pendingCommandReady = false;
portMUX_TYPE pendingCommandMux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint8_t SD_SCK_PIN = 14;
constexpr uint8_t SD_MISO_PIN = 12;
constexpr uint8_t SD_MOSI_PIN = 15;
constexpr uint8_t SD_CS_PIN = 16;
const unsigned long SD_RETRY_INTERVAL_MS = 2000;

bool sdCardReady = false;
bool sdCardBeginAttempted = false;
bool sdSpiStarted = false;
unsigned long nextSdRetryMillis = 0;
bool audioReady = false;

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
  return true;
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

void queuePendingCommand(PendingCommandType type,
                         const char *date,
                         const char *time,
                         const uint8_t *senderMac) {
  portENTER_CRITICAL(&pendingCommandMux);
  pendingCommand.type = type;
  memcpy(pendingCommand.date, date, sizeof(pendingCommand.date));
  memcpy(pendingCommand.time, time, sizeof(pendingCommand.time));
  memcpy(pendingCommand.senderMac, senderMac, sizeof(pendingCommand.senderMac));
  pendingCommandReady = true;
  portEXIT_CRITICAL(&pendingCommandMux);
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
    if (millis() >= nextSdRetryMillis && runSdCardTest()) {
      nextSdRetryMillis = 0;
    } else {
      nextSdRetryMillis = millis() + SD_RETRY_INTERVAL_MS;
    }
  }

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
    return false;
  }

  recordingBaseName = String(date) + "_" + String(time);
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

void sendDoneToStartSender() {
  if (!hasStartSenderMac) {
    printTimestampPrefix();
    Serial.println("DONE send failed: START sender MAC unavailable");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, startSenderMac, 6);
  peerInfo.channel = 1;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  printTimestampPrefix();
  Serial.println("TX DONE");
  Serial.printf("Elapsed since START: %lu ms\n", millis() - runningStartMillis);
  printMacLine("Target MAC: ", startSenderMac);
  printPacketDetails(reinterpret_cast<const uint8_t *>("DONE"), 4);
  Serial.printf("Channel: %d\n", peerInfo.channel);

  if (!esp_now_is_peer_exist(startSenderMac)) {
    const esp_err_t peerResult = esp_now_add_peer(&peerInfo);
    Serial.printf("Peer add result: %d\n", peerResult);
    if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
      printTimestampPrefix();
      Serial.printf("DONE send failed: peer add error: %d\n", peerResult);
      return;
    }
  } else {
    Serial.println("Peer already exists; reusing peer");
  }

  Serial.printf("Peer exists: %s\n",
                esp_now_is_peer_exist(startSenderMac) ? "YES" : "NO");

  const char *doneMessage = "DONE";
  printTimestampPrefix();
  Serial.println("Sending DONE...");

  esp_err_t sendResult = esp_now_send(
      startSenderMac,
      reinterpret_cast<const uint8_t *>(doneMessage),
      strlen(doneMessage));

  Serial.printf("esp_now_send return: %d\n", sendResult);

  if (sendResult == ESP_OK) {
    printTimestampPrefix();
    Serial.println("DONE sent successfully");
  } else {
    printTimestampPrefix();
    Serial.printf("DONE send failed: %d\n", sendResult);
  }
}

void sendUnicastTest() {
  if (!hasStartSenderMac) {
    printTimestampPrefix();
    Serial.println("TEST send failed: START sender MAC unavailable");
    return;
  }

  const char *unicastMessage = "TEST";
  constexpr size_t unicastMessageLength = 4;

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, startSenderMac, 6);
  peerInfo.channel = 1;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  printTimestampPrefix();
  Serial.println("Unicast test");
  printMacLine("Target MAC: ", startSenderMac);
  Serial.printf("Channel: %d\n", peerInfo.channel);
  printPacketDetails(reinterpret_cast<const uint8_t *>(unicastMessage),
                     unicastMessageLength);

  if (!esp_now_is_peer_exist(startSenderMac)) {
    const esp_err_t peerResult = esp_now_add_peer(&peerInfo);
    Serial.printf("Unicast peer add result: %d\n", peerResult);
    if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
      printTimestampPrefix();
      Serial.printf("Unicast send failed: peer add error: %d\n", peerResult);
      return;
    }
  } else {
    Serial.println("Unicast peer already exists; reusing peer");
  }

  Serial.printf("Unicast peer exists: %s\n",
                esp_now_is_peer_exist(startSenderMac) ? "YES" : "NO");

  const esp_err_t sendResult = esp_now_send(
      startSenderMac,
      reinterpret_cast<const uint8_t *>(unicastMessage),
      unicastMessageLength);
  Serial.printf("esp_now_send return: %d\n", sendResult);
}

void OnDataSent(const uint8_t *macAddress, esp_now_send_status_t status) {
  printTimestampPrefix();
  if (memcmp(macAddress, startSenderMac, 6) == 0) {
    Serial.printf("TEST send callback Status: %s\n",
                  status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
  } else {
    Serial.println("TX DONE callback");
    Serial.printf("DONE send callback Status: %s\n",
                  status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  char eventDate[9] = {};
  char eventTime[7] = {};

  if (parseTimedCommand(incomingData,
                        len,
                        "START",
                        eventDate,
                        eventTime)) {
    queuePendingCommand(PENDING_START, eventDate, eventTime, mac);
  } else if (parseTimedCommand(incomingData,
                               len,
                               "STOP",
                               eventDate,
                               eventTime)) {
    queuePendingCommand(PENDING_STOP, eventDate, eventTime, mac);
  }
}

void processPendingCommand() {
  PendingCommand command;
  if (!takePendingCommand(command)) {
    return;
  }

  if (command.type == PENDING_START) {
    if (currentState != STANDBY) {
      printTimestampPrefix();
      Serial.println("START ignored: already RUNNING");
      return;
    }

    if (!beginRecording(command.date, command.time)) {
      printTimestampPrefix();
      Serial.println("START rejected: recording initialization failed");
      return;
    }

    memcpy(startSenderMac, command.senderMac, sizeof(startSenderMac));
    hasStartSenderMac = true;
    currentState = RUNNING;
    pendingUnicastTest = true;
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
    if (currentState != RUNNING) {
      printTimestampPrefix();
      Serial.println("STOP ignored: already STANDBY");
      return;
    }

    const unsigned long stopMillis = millis();
    const bool recordingFinished =
        finishRecording("STOP", command.date, command.time, stopMillis);
    currentState = STANDBY;
    hasStartSenderMac = false;
    printTimestampPrefix();
    Serial.println("RX STOP recognized");
    Serial.println("State -> STANDBY");
    if (!recordingFinished) {
      Serial.println("Recording stopped with SD error; remount required");
    }
  }
}

void setup() {
  Serial.begin(115200);

  audioReady = i2s_up();
  if (!audioReady) {
    Serial.println("[I2S] Audio recording unavailable");
  }

  WiFi.mode(WIFI_STA);
  esp_err_t channelResult =
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
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
  if (!sdCardReady && millis() >= nextSdRetryMillis) {
    if (runSdCardTest()) {
      nextSdRetryMillis = 0;
    } else {
      nextSdRetryMillis = millis() + SD_RETRY_INTERVAL_MS;
    }
  }

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
          reportRecordingError("WAV audio data write failed");
          markSdCardNeedsRemount("WAV audio data write failure");
          return;
        }
      }
    }

    if (now - runningStartMillis >= RUNNING_DURATION_MS) {
      char endDate[9] = {};
      char endTime[7] = {};
      makeEndTimestamp(endDate, endTime, now);
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
      if (recordingFinished) {
        sendDoneToStartSender();
      }
      hasStartSenderMac = false;
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
