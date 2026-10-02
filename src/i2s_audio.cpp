#include "i2s_audio.h"

#include "audio_config.h"
#include "driver/i2s_pdm.h"

namespace {
i2s_chan_handle_t microphoneChannel = nullptr;
}

bool i2s_up() {
  i2s_chan_config_t channelConfig =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 6;
  channelConfig.dma_frame_num = 240;

  esp_err_t error =
      i2s_new_channel(&channelConfig, nullptr, &microphoneChannel);
  if (error != ESP_OK) {
    Serial.printf("[I2S] Channel create failed: %s\n",
                  esp_err_to_name(error));
    return false;
  }

  const i2s_pdm_rx_config_t config = {
      .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .clk = PDM_CLK_PIN,
          .din = PDM_DATA_PIN,
          .invert_flags = {.clk_inv = false},
      },
  };

  error = i2s_channel_init_pdm_rx_mode(microphoneChannel, &config);
  if (error == ESP_OK) {
    error = i2s_channel_enable(microphoneChannel);
  }
  if (error != ESP_OK) {
    Serial.printf("[I2S] PDM init failed: %s\n", esp_err_to_name(error));
    i2s_del_channel(microphoneChannel);
    microphoneChannel = nullptr;
    return false;
  }

  Serial.printf("[I2S] PDM microphone ready: CLK=%d DATA=%d %dHz\n",
                PDM_CLK_PIN, PDM_DATA_PIN, SAMPLE_RATE);
  return true;
}

size_t i2s_read_samples(void *buffer, size_t sizeBytes) {
  if (microphoneChannel == nullptr) {
    return 0;
  }

  size_t bytesRead = 0;
  const esp_err_t error = i2s_channel_read(
      microphoneChannel, buffer, sizeBytes, &bytesRead, pdMS_TO_TICKS(1000));
  if (error != ESP_OK && error != ESP_ERR_TIMEOUT) {
    Serial.printf("[I2S] Read failed: %s\n", esp_err_to_name(error));
  }
  return bytesRead;
}
