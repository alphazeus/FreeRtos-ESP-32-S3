# ESP32-S3 camera capture check

This ESP-IDF example initializes the Freenove ESP32-S3 WROOM-1 CAM board's
OV2640 sensor through SCCB, starts the `esp_driver_cam` DVP controller, and
captures frames for one second. It converts each completed 640x480 YUV422 frame
to RGB and reports the separate mean red, green, and blue values across all
processed pixels to the serial monitor. Two DMA-capable PSRAM frame buffers
are reused as capture continues; frames are not saved.

The pin assignments and sensor format match Espressif's ESP32-S3 OV2640 DVP
test configuration. Build from this folder with the ESP-IDF environment set:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

For this repository, initialize the bundled ESP-IDF environment from the
repository root first:

```sh
source esp-idf/export.sh
cd OV2640_cam
```

The board uses octal PSRAM. If startup reports that no camera sensor was
detected, check the camera module connection and verify the board revision's
SCCB and DVP wiring against the pin definitions in `main.c`.

The RGB means are calculated from YUYV using the standard limited-range
Y'CbCr-to-RGB conversion. They describe the average color across every pixel
in every complete frame processed during the one-second capture interval.

This project does not depend on ESP-IDF's camera example components. The
camera controller and I2C drivers are built into ESP-IDF; `esp_cam_sensor` is
the external component used to detect and configure the OV2640.
