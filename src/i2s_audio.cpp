#include "i2s_audio.h"

#include "audio_config.h"
#include "driver/i2s.h"

namespace {
bool microphoneReady = false;
}

bool i2s_up() {
  const i2s_config_t config = {
      .mode = static_cast<i2s_mode_t>(
          I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 6,
      .dma_buf_len = 240,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0,
      .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
  };

  esp_err_t error = i2s_driver_install(I2S_NUM_0, &config, 0, nullptr);
  if (error != ESP_OK) {
    Serial.printf("[I2S] Driver install failed: %s\n",
                  esp_err_to_name(error));
    return false;
  }

  const i2s_pin_config_t pinConfig = {
      .mck_io_num = I2S_PIN_NO_CHANGE,
      .bck_io_num = PDM_CLK_PIN,
      .ws_io_num = I2S_PIN_NO_CHANGE,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = PDM_DATA_PIN,
  };

  error = i2s_set_pin(I2S_NUM_0, &pinConfig);
  if (error == ESP_OK) {
    error = i2s_set_pdm_rx_down_sample(I2S_NUM_0, I2S_PDM_DSR_16S);
  }

  if (error != ESP_OK) {
    Serial.printf("[I2S] PDM init failed: %s\n",
                  esp_err_to_name(error));
    i2s_driver_uninstall(I2S_NUM_0);
    return false;
  }

  microphoneReady = true;
  Serial.printf("[I2S] PDM microphone ready: CLK=%d DATA=%d %dHz\n",
                PDM_CLK_PIN, PDM_DATA_PIN, SAMPLE_RATE);
  return true;
}

size_t i2s_read_samples(void *buffer, size_t sizeBytes) {
  if (!microphoneReady) {
    return 0;
  }

  size_t bytesRead = 0;
  const esp_err_t error = i2s_read(
      I2S_NUM_0,
      buffer,
      sizeBytes,
      &bytesRead,
      pdMS_TO_TICKS(1000));

  if (error != ESP_OK && error != ESP_ERR_TIMEOUT) {
    Serial.printf("[I2S] Read failed: %s\n", esp_err_to_name(error));
  }

  return bytesRead;
}
