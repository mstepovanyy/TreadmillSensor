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

### Configure for use
TBD

### ZWIFT view
TBD

### Strava chart
TBD

### Hardware instalation
TBD