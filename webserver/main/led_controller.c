
#include "led_controller.h"
#include "driver/gpio.h"

#define LED_GPIO 2

esp_err_t IOLEDInitialize(void){
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    return ESP_OK;
}

esp_err_t control_light(bool ledon, int red, int green, int blue){
    ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, ledon ? 1 : 0));
    return ESP_OK;
}