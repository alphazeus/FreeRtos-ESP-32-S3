
#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t IOLEDInitialize(void);
esp_err_t control_light(bool ledon, int red, int green, int blue);