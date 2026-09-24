#pragma once

#include <Arduino.h>
#include <SdFat.h>

#include "audio_config.h"

#pragma pack(push, 1)
struct WavHeader {
  char riff[4];
  uint32_t fileSize;
  char wave[4];
  char fmt[4];
  uint32_t fmtSize;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  char data[4];
  uint32_t dataSize;
};
#pragma pack(pop)

class WavWriter {
public:
  bool begin(FsFile &file,
             uint32_t sampleRate,
             uint16_t channels,
             uint16_t bitsPerSample);
  bool writeData(FsFile &file, const int16_t *buffer, size_t samples);
  bool finalize(FsFile &file);

private:
  uint32_t sampleRate_ = 0;
  uint16_t channels_ = 0;
  uint16_t bitsPerSample_ = 0;
  uint32_t dataSize_ = 0;

  void createHeader(WavHeader &header);
  bool updateHeader(FsFile &file);
};
