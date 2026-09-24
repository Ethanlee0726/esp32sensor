#include "wav_writer.h"

void WavWriter::createHeader(WavHeader &header) {
  memcpy(header.riff, "RIFF", 4);
  header.fileSize = dataSize_ + sizeof(WavHeader) - 8;
  memcpy(header.wave, "WAVE", 4);
  memcpy(header.fmt, "fmt ", 4);
  header.fmtSize = 16;
  header.audioFormat = 1;
  header.numChannels = channels_;
  header.sampleRate = sampleRate_;
  header.byteRate = sampleRate_ * channels_ * (bitsPerSample_ / 8);
  header.blockAlign = channels_ * (bitsPerSample_ / 8);
  header.bitsPerSample = bitsPerSample_;
  memcpy(header.data, "data", 4);
  header.dataSize = dataSize_;
}

bool WavWriter::begin(FsFile &file,
                      uint32_t sampleRate,
                      uint16_t channels,
                      uint16_t bitsPerSample) {
  if (!file.isOpen()) {
    return false;
  }

  sampleRate_ = sampleRate;
  channels_ = channels;
  bitsPerSample_ = bitsPerSample;
  dataSize_ = 0;

  WavHeader header;
  createHeader(header);

  const size_t written = file.write(
      reinterpret_cast<const uint8_t *>(&header),
      sizeof(WavHeader));
  if (written != sizeof(WavHeader)) {
    Serial.printf("[WAV] Header write failed: %u/%u\n",
                  static_cast<unsigned>(written),
                  static_cast<unsigned>(sizeof(WavHeader)));
    return false;
  }

  if (!file.sync()) {
    Serial.println("[WAV] Header sync failed");
    return false;
  }

  Serial.printf("[WAV] Header written: %uHz, %uch, %ubit\n",
                static_cast<unsigned>(sampleRate_),
                static_cast<unsigned>(channels_),
                static_cast<unsigned>(bitsPerSample_));
  return true;
}

bool WavWriter::writeData(FsFile &file,
                          const int16_t *buffer,
                          size_t samples) {
  if (!file.isOpen()) {
    return false;
  }

  const size_t bytes = samples * sizeof(int16_t);
  const size_t written = file.write(
      reinterpret_cast<const uint8_t *>(buffer),
      bytes);

  if (written != bytes) {
    Serial.printf("[WAV] Write error: %u/%u bytes\n",
                  static_cast<unsigned>(written),
                  static_cast<unsigned>(bytes));
    return false;
  }

  dataSize_ += written;
  if (!file.sync()) {
    Serial.println("[WAV] Audio data sync failed");
    return false;
  }
  return true;
}

bool WavWriter::updateHeader(FsFile &file) {
  if (!file.isOpen() || !file.seekSet(0)) {
    return false;
  }

  WavHeader header;
  createHeader(header);
  const size_t written = file.write(
      reinterpret_cast<const uint8_t *>(&header),
      sizeof(WavHeader));
  return written == sizeof(WavHeader);
}

bool WavWriter::finalize(FsFile &file) {
  if (!file.isOpen()) {
    return false;
  }

  Serial.printf("[WAV] Finalizing: %u bytes audio data\n",
                static_cast<unsigned>(dataSize_));

  if (!updateHeader(file)) {
    return false;
  }

  if (!file.sync() || !file.seek(file.size()) || !file.sync()) {
    Serial.println("[WAV] Final sync failed");
    return false;
  }
  Serial.printf("[WAV] Finalized: total %u bytes\n",
                static_cast<unsigned>(sizeof(WavHeader) + dataSize_));
  return true;
}
