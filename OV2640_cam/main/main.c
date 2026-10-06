#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_sccb_i2c.h"
#include "esp_sccb_intf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAM_I2C_PORT             I2C_NUM_0
#define CAM_SCCB_SDA_GPIO        4
#define CAM_SCCB_SCL_GPIO        5
#define CAM_XCLK_GPIO            15
#define CAM_XCLK_HZ              20000000

#define CAM_D0_GPIO              11
#define CAM_D1_GPIO              9
#define CAM_D2_GPIO              8
#define CAM_D3_GPIO              10
#define CAM_D4_GPIO              12
#define CAM_D5_GPIO              18
#define CAM_D6_GPIO              17
#define CAM_D7_GPIO              16
#define CAM_VSYNC_GPIO           6
#define CAM_HREF_GPIO            7
#define CAM_PCLK_GPIO            13

#define CAM_WIDTH                640
#define CAM_HEIGHT               480
#define CAM_FRAME_BYTES          (CAM_WIDTH * CAM_HEIGHT * 2)
#define CAM_FORMAT_NAME          "DVP_8bit_20Minput_YUV422_YUYV_640x480_6fps"
#define CAM_CAPTURE_TIMEOUT_MS   10000

static const char *TAG = "camera_test";

#define CAM_SCCB_FREQ_HZ         100000

typedef struct {
    uint8_t *frames[2];
    size_t frame_size;
    unsigned int next_frame;
    TaskHandle_t capture_task;
    void * volatile completed_buffer;
    volatile size_t completed_size;
} camera_frame_pool_t;

static camera_frame_pool_t s_frame_pool;
static i2c_master_bus_handle_t s_i2c_bus;
static esp_cam_sensor_device_t *s_sensor;

static esp_err_t initialize_sensor(void)
{
    const i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CAM_SCCB_SDA_GPIO,
        .scl_io_num = CAM_SCCB_SCL_GPIO,
        .i2c_port = CAM_I2C_PORT,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_config, &s_i2c_bus),
                        TAG, "failed to initialize SCCB I2C bus");

    esp_cam_sensor_detect_fn_t *detect_start = NULL;
    esp_cam_sensor_detect_fn_t *detect_end = NULL;
    esp_cam_sensor_detect_get_array(&detect_start, &detect_end);

    esp_cam_sensor_config_t sensor_config = {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = CAM_XCLK_GPIO,
        .sensor_port = ESP_CAM_SENSOR_DVP,
    };
    for (esp_cam_sensor_detect_fn_t *candidate = detect_start;
         candidate < detect_end; candidate++) {
        if (candidate->port != ESP_CAM_SENSOR_DVP) {
            continue;
        }

        const sccb_i2c_config_t sccb_config = {
            .scl_speed_hz = CAM_SCCB_FREQ_HZ,
            .device_address = candidate->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        ESP_RETURN_ON_ERROR(
            sccb_new_i2c_io(s_i2c_bus, &sccb_config, &sensor_config.sccb_handle),
            TAG, "failed to initialize SCCB device");

        s_sensor = candidate->detect(&sensor_config);
        if (s_sensor != NULL) {
            break;
        }

        ESP_RETURN_ON_ERROR(esp_sccb_del_i2c_io(sensor_config.sccb_handle),
                            TAG, "failed to release unmatched SCCB device");
        sensor_config.sccb_handle = NULL;
    }

    if (s_sensor == NULL) {
        ESP_LOGE(TAG, "no DVP camera sensor detected on SCCB");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Detected camera sensor: %s",
             esp_cam_sensor_get_name(s_sensor));

    esp_cam_sensor_format_array_t formats = {0};
    ESP_RETURN_ON_ERROR(esp_cam_sensor_query_format(s_sensor, &formats),
                        TAG, "failed to query camera formats");

    const esp_cam_sensor_format_t *selected_format = NULL;
    for (uint32_t i = 0; i < formats.count; i++) {
        if (strcmp(formats.format_array[i].name, CAM_FORMAT_NAME) == 0) {
            selected_format = &formats.format_array[i];
            break;
        }
    }
    if (selected_format == NULL) {
        ESP_LOGE(TAG, "sensor does not support format %s", CAM_FORMAT_NAME);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_RETURN_ON_ERROR(esp_cam_sensor_set_format(s_sensor, selected_format),
                        TAG, "failed to configure camera format");

    int stream_enabled = 1;
    ESP_RETURN_ON_ERROR(
        esp_cam_sensor_ioctl(s_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_enabled),
        TAG, "failed to start camera sensor stream");
    return ESP_OK;
}

static bool IRAM_ATTR camera_get_new_frame(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data)
{
    (void)handle;
    camera_frame_pool_t *pool = user_data;

    trans->buffer = pool->frames[pool->next_frame];
    trans->buflen = pool->frame_size;
    pool->next_frame ^= 1U;
    return false;
}

static bool IRAM_ATTR camera_frame_finished(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data)
{
    (void)handle;
    camera_frame_pool_t *pool = user_data;
    BaseType_t higher_priority_task_woken = pdFALSE;

    pool->completed_buffer = trans->buffer;
    pool->completed_size = trans->received_size;
    vTaskNotifyGiveFromISR(pool->capture_task, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static uint32_t frame_checksum(const uint8_t *data, size_t length)
{
    uint32_t hash = 2166136261U;

    for (size_t i = 0; i < length; i++) {
        hash = (hash ^ data[i]) * 16777619U;
    }
    return hash;
}

void app_main(void)
{
    const esp_cam_ctlr_dvp_pin_config_t pin_config = {
        .data_width = 8,
        .data_io = {
            CAM_D0_GPIO, CAM_D1_GPIO, CAM_D2_GPIO, CAM_D3_GPIO,
            CAM_D4_GPIO, CAM_D5_GPIO, CAM_D6_GPIO, CAM_D7_GPIO,
        },
        .vsync_io = CAM_VSYNC_GPIO,
        .de_io = CAM_HREF_GPIO,
        .pclk_io = CAM_PCLK_GPIO,
        .xclk_io = CAM_XCLK_GPIO,
    };
    const esp_cam_ctlr_dvp_config_t controller_config = {
        .ctlr_id = 0,
        .clk_src = CAM_CLK_SRC_DEFAULT,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
        .input_data_color_type = CAM_CTLR_COLOR_YUV422_YUYV,
        .output_data_color_type = CAM_CTLR_COLOR_YUV422_YUYV,
        .dma_burst_size = 64,
        .pin = &pin_config,
        .bk_buffer_dis = 1,
        .xclk_freq = CAM_XCLK_HZ,
        .cam_data_width = 8,
    };

    esp_cam_ctlr_handle_t camera = NULL;
    ESP_ERROR_CHECK(esp_cam_new_dvp_ctlr(&controller_config, &camera));
    ESP_LOGI(TAG, "DVP controller initialized for %dx%d", CAM_WIDTH, CAM_HEIGHT);

    s_frame_pool.frame_size = CAM_FRAME_BYTES;
    s_frame_pool.frames[0] = esp_cam_ctlr_alloc_buffer(
        camera, CAM_FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    s_frame_pool.frames[1] = esp_cam_ctlr_alloc_buffer(
        camera, CAM_FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (s_frame_pool.frames[0] == NULL || s_frame_pool.frames[1] == NULL) {
        ESP_LOGE(TAG, "Could not allocate two DMA-capable PSRAM frame buffers");
        return;
    }

    ESP_ERROR_CHECK(initialize_sensor());
    ESP_LOGI(TAG, "Camera sensor initialized using %s", CAM_FORMAT_NAME);

    const esp_cam_ctlr_evt_cbs_t callbacks = {
        .on_get_new_trans = camera_get_new_frame,
        .on_trans_finished = camera_frame_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(
        camera, &callbacks, &s_frame_pool));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(camera));
    s_frame_pool.capture_task = xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK(esp_cam_ctlr_start(camera));
    ESP_LOGI(TAG, "Camera capture started; waiting for a frame");

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAM_CAPTURE_TIMEOUT_MS)) == 0) {
        ESP_LOGE(TAG, "Timed out waiting for a camera frame");
        ESP_ERROR_CHECK(esp_cam_ctlr_stop(camera));
        return;
    }

    ESP_ERROR_CHECK(esp_cam_ctlr_stop(camera));
    if (s_frame_pool.completed_size != s_frame_pool.frame_size) {
        ESP_LOGE(TAG, "Incomplete frame: received %zu of %zu bytes",
                 s_frame_pool.completed_size, s_frame_pool.frame_size);
        return;
    }

    const uint32_t checksum = frame_checksum(
        s_frame_pool.completed_buffer, s_frame_pool.completed_size);
    ESP_LOGI(TAG,
             "PASS: camera is running; captured %zu-byte %dx%d YUV422 frame "
             "(FNV-1a checksum 0x%08" PRIx32 ")",
             s_frame_pool.completed_size, CAM_WIDTH, CAM_HEIGHT, checksum);
    ESP_LOGI(TAG, "The captured frame is held in PSRAM until reset.");
}
