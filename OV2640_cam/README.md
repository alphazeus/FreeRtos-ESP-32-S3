# ESP32-S3 camera capture check

This ESP-IDF example initializes the Freenove ESP32-S3 WROOM-1 CAM board's
OV2640 sensor through SCCB, starts the `esp_driver_cam` DVP controller, and
captures one 640x480 YUV422 frame into DMA-capable PSRAM. A successful capture
prints `PASS` and a frame checksum to the serial monitor. The captured frame is
kept in RAM; this diagnostic does not write an image file.

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
cd esp32-cam
```

The board uses octal PSRAM. If startup reports that no camera sensor was
detected, check the camera module connection and verify the board revision's
SCCB and DVP wiring against the pin definitions in `main.c`.
