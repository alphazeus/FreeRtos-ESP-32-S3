
#include "led_controller.h"
#include "driver/gpio.h"
#include "led_strip.h"

#define LED_GPIO 48
#define LED_STRIP_LENGTH 1

static led_strip_handle_t led_strip;

esp_err_t IOLEDInitialize(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_STRIP_LENGTH,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
        return err;
    }

    if (!led_strip) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t control_light(bool ledon, int red, int green, int blue) {
    if (!ledon) {
        return led_strip_clear(led_strip);
    }

    esp_err_t err = led_strip_set_pixel(led_strip, 0, red, green, blue);
    if (err != ESP_OK) {
        return err;
    }
    return led_strip_refresh(led_strip);
}