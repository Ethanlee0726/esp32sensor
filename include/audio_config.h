#pragma once

#include <Arduino.h>
#include "driver/gpio.h"

#define PDM_CLK_PIN GPIO_NUM_1
#define PDM_DATA_PIN GPIO_NUM_41
#define BUZZER_PIN GPIO_NUM_42

#define SAMPLE_RATE 16000
#define CHANNELS 1
#define BITS_PER_SAMPLE 16

#define FRAME_MS 20
#define FRAME_SAMPLES (SAMPLE_RATE * FRAME_MS / 1000)
#define FRAME_BYTES (FRAME_SAMPLES * sizeof(int16_t))
