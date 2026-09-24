#pragma once

#include <Arduino.h>

bool i2s_up();
size_t i2s_read_samples(void *buffer, size_t sizeBytes);
