# ESP32

Command cheatsheet for this codebase: 

### Set the terminal environment variables
`source esp-idf/export.sh`

### Set the target for the hardware
`idf.py set-target esp32s3`

### Builds the code
`idf.py build`

### Builds and flashes
`idf.py -p PORT flash`

### Builds, flashes, and opens the serial monitor over UART
`idf.py -p PORT flash monitor`
