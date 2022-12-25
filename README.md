# Treadmill Sensor

### Creates two BLE Services:
     FTMS Service                - Speed and Incline
     Speed and Cadence Service   - Speed and Cadense

### Provide following Characteristics:
     Speed in mps(meter per second)
     Cadence
     Inclide in Percentage
     Inclide in Degree
     Elevation gain

### Sensors used:
     MPU6050 - for Cadence calculations
     VL53L0X - for Incline calculations
     A3144E  - for Speed calculations

### Integrated LED statuses:
     LOW             - Device is trying to make initial setup.
     HIGH            - All checks passed and ready to work
     blink 2 times   - Cannot configure mpu6050 sensor
     blink 3 times   - Cannot configure vl53lox sensor

### ESP32 Connections
I2C(SDA=21, SCL=22) pins - for `MPU6050` and `VL53L0X`
26 pin - Hall sensor A3144E
GND pin - on treadmill

### Configure for use
Project is using PlatformIO + VisualStudio.
Regular workflow: build + deploy

### ZWIFT view
![Zwift](./img/zwift_example.jpg?raw=true)

### Strava chart
![Strava](./img/strava_example.png?raw=true)

### Hardware instalation
A3144E Hall sensor should be near drive roller, check magnet on side.
 ![A3144E](./img/IMG_20221222_124400444.jpg?raw=true)

VL53L0X - incline sensor neear motor that responsible for incline.
 ![VL53L0X](./img/IMG_20221222_124449129.jpg?raw=true)

MPU6050 - cadence sensor somewhere inside, so vibration from steps transfer to sensor.
 ![MPU6050](./img/IMG_20221222_124423789.jpg?raw=true)

### Notes
Wires could be affected by DC Motor, especially Hall Sensor. Use shielded wires, or connect GND contact of ESP32 to treadmill.

Inspired by projects:
    https://github.com/lefty01/ESP32_TTGO_FTMS
    https://github.com/imwitti/FootpodMimic
