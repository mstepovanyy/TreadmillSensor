/**
 * Creates two BLE Services:
 *      FTMS Service                - Speed and Incline
 *      Speed and Cadence Service   - Speed and Cadense
 * 
 * Provide following Characteristics:
 *      Speed in mps(meter per second)
 *      Cadence
 *      Inclide in Percentage
 *      Inclide in Degree
 *      Elevation gain
 *
 * Sensors used:
 *      MPU6050 - for Cadence calculations
 *      VL53L0X - for Incline calculations
 *      A3144E  - for Speed calculations
 * 
 * Integrated LED statuses:
 *      LOW             - Device is trying to make initial setup.
 *      HIGH            - All checks passed and ready to work
 *      blink 2 times   - Cannot configure mpu6050 sensor
 *      blink 3 times   - Cannot configure vl53lox sensor
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <RunningMedian.h>

// Incline sensor
#include <VL53L0X.h>
VL53L0X tofSensor;

// Cadence sensor
Adafruit_MPU6050 mpu;
RunningMedian cadenceFilter(20);

// Speed sensor
#define HALL_SENSOR_PIN 26
// Default I2C pins for SDA=21, SCL=22, used for VL53L0X, MPU6050


// Configure tredmill length from VL53L0X sensor location to back 
#define TREADMILL_LENGTH 1450           // in milimeters
#define TREADMILL_CYLINDER_LENGTH 0.174 // in meters
// INITIAL_INCLINE_DEGREE 0.95 == 24mm for ter
// TOF_SENSOR_HEIGTH = min_sensor_height - (TREADMILL_LENGTH * sin(radians(MIN_INCLINE_IN_DEGREE))) = 170 - 24 = 150

#define TOF_SENSOR_HEIGTH 89
// #define INITIAL_INCLINE_PERCENT 1.65
// #define INITIAL_INCLINE_DEGREE 0.95

//cadence
// max cadence 280 per minute, or 4.6 per second, or 1 step per 0.218 seconds
#define CADENCE_DEBOUNCE_TIME 218
#define MAX_STEP_DURATION 500

//#define DEBUG


#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));

#define FTMSService BLEUUID((uint16_t)0x1826)
BLECharacteristic TreadmillDataCharacteristics(BLEUUID((uint16_t)0x2ACD), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor FTMSDescriptor(BLEUUID((uint16_t)0x2901));

bool bluetoothClientConected = false;
unsigned long bleNotifyTime = millis();
volatile unsigned int rotationCount = 0;

void rotationIncrement() {
    rotationCount++;
}


class TreadmillData {
    
    public:
        double mps = 0;
        uint8_t cadence = 0;               // Unit is 1/min
        int16_t inclinePercent = 0;        // Unit is 1/10 of a percent
        int16_t inclineDegree = 0;         // Unit is 1/10 of a degree
        uint16_t elevationGain = 0;        // Elevation gain from begining of training session, Unit is 1/10 of a meter
        uint32_t totalDistance = 0;        // Total distance of running over all time

        uint8_t rscDataArray[10] = {};
        uint8_t treadmillDataArray[34] = {};

    public:
        uint8_t* getRscData();
        uint8_t* getTreadmillData();
        void procedSpeed();
        void procedIncline();
        void procedCadence();
    private:
        // speed
        unsigned long startRotationTime = 0;
        float rotationLength = TREADMILL_CYLINDER_LENGTH;

        // incline
        unsigned long startInclineTime = 0;
        unsigned long startReadValueTime = 0;
        uint16_t inclineMax = 0;

        // cadence
        unsigned long startStepTime = 0;
        unsigned long startCadenceTime = 0;

        // total distance time
        unsigned long startTotalDistanceTime = 0;
};

void TreadmillData::procedSpeed() {
    unsigned long endRotationTime = millis();
    if (endRotationTime - startRotationTime  > 3000) {
        if (rotationCount == 0) {
            mps = 0;
        } else {
            mps = rotationCount * rotationLength * 1000 / (endRotationTime - startRotationTime) ;
        }
#ifdef DEBUG
        Serial.print("Current speed: ");
        Serial.print(mps);
        Serial.print("Rotation: ");
        Serial.println(rotationCount);
#endif
        startRotationTime = endRotationTime;
        rotationCount = 0;
    }
    
}

void TreadmillData::procedIncline() {
    unsigned long endInclineTime = millis();

    if (endInclineTime - startInclineTime > 3000) {
        uint16_t inclineInMilimiter = inclineMax - TOF_SENSOR_HEIGTH;
        if (inclineInMilimiter > 1000) inclineInMilimiter = 0;
        uint16_t length_at_zero_level = sqrt(TREADMILL_LENGTH * TREADMILL_LENGTH - inclineInMilimiter * inclineInMilimiter);
        if (inclineInMilimiter == 0) {
            inclinePercent = 0;
            inclineDegree = 0;
        } else {
            inclinePercent = inclineInMilimiter * 100 * 10 / length_at_zero_level;                  // 100% and 0.1 resolution
            inclineDegree = degrees(atan(inclineInMilimiter/(float)length_at_zero_level)) * 10;     // 0.1 resolution
        }

        unsigned long duration = endInclineTime - startInclineTime;
        if (mps == 0 || inclinePercent == 0) {
            elevationGain = 0;
        } else {
            // duration in millis, 1s = 1000ms, and elevation resolution 1/10.
            elevationGain += mps * duration / 100 * inclinePercent / 1000;
        }
        startInclineTime = endInclineTime;
#ifdef DEBUG
        Serial.print("Incline: ");
        Serial.print(inclineInMilimiter);
        Serial.print(", length_at_zero_level: ");
        Serial.print(length_at_zero_level);
        Serial.print(", percent: ");
        Serial.print(inclinePercent);
        Serial.print(", degree: ");
        Serial.print(inclineDegree);
        Serial.print(", elevation: ");
        Serial.println(elevationGain);
#endif
        inclineMax = 0;
    } else if (endInclineTime - startReadValueTime > 700) {
        // Read incline each 700ms and pick max value
        inclineMax = max(inclineMax, tofSensor.readRangeSingleMillimeters());
        startReadValueTime = endInclineTime;
    }
}

void TreadmillData::procedCadence() {
    if(mpu.getMotionInterruptStatus()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // pick acceleration in range 11 - 20
        if (a.acceleration.z < 20 && a.acceleration.z > 11 && a.timestamp - startStepTime > CADENCE_DEBOUNCE_TIME) {
            if (a.timestamp - startStepTime < MAX_STEP_DURATION) {
                // Steps that do not lay in range CADENCE_DEBOUNCE_TIME - MAX_STEP_DURATION are ignored.
                // 218 ms - 275 steps per minute
                // 500 ms - 120 steps per minute
                cadenceFilter.add(60000 / (a.timestamp - startStepTime));
            }
            startStepTime = a.timestamp;
            
            if (a.timestamp - startCadenceTime > 3000) {
                // all array 20 values, pick only middle 10 to get cadence
                cadence = cadenceFilter.getAverage(10);     
                startCadenceTime = a.timestamp;
#ifdef DEBUG
                Serial.print("Cadence: ");
                Serial.println(cadence);
#endif
            }
        }
#ifdef DEBUG
        Serial.print("Time: ");
        Serial.print(millis());
        Serial.print(" AccelZ:");
        Serial.println(a.acceleration.z);
#endif
    }

    // reset cadence in case no new steps made for 5 seconds
    if (millis() - startStepTime > 5000) { 
        cadence = 0;
    }
}

uint8_t* TreadmillData::getRscData() {
    uint16_t speedInUnits = mps * 256;                  // Unit is 1/256th of a m/s
    uint16_t strideLength = 0;                          // Unit is 1/100 m
    if (mps != 0 && cadence != 0) {
        strideLength = mps / cadence * 2 * 100;    
    }

    // It had to be value overall time, but for now it is just for online time.
    unsigned long currentTime = millis();
    totalDistance += mps * (currentTime - startTotalDistanceTime) / 1000;
    startTotalDistanceTime = currentTime;
 
    rscDataArray[0] = 3;
    // '00000011'
    //         0 - Instantaneous Stride Length Present
    //        1 - Total Distance Present
    //       2 - Walking or Running Status: 0 = Walking, 1 = Running

    rscDataArray[1] = (uint8_t)(speedInUnits & 0xFF);
    rscDataArray[2] = (uint8_t)(speedInUnits >> 8);

    rscDataArray[3] = (uint8_t)(cadence);

    rscDataArray[4] = (uint8_t)(strideLength & 0xFF);
    rscDataArray[5] = (uint8_t)(strideLength >> 8);

    rscDataArray[6] = (uint8_t)(totalDistance & 0xFF);
    rscDataArray[7] = (uint8_t)((totalDistance >> 8) & 0xFF);
    rscDataArray[8] = (uint8_t)((totalDistance >> 16) & 0xFF);
    rscDataArray[9] = (uint8_t)(totalDistance >> 24);

    return rscDataArray;
}

uint8_t* TreadmillData::getTreadmillData() {
    uint16_t FTMSFlags = 0x0018;    
    // '000000011000'
    //             0 - Instantaneous Speed, uint16, Present if Flags field is set to 0, Unit is 1/100 of a kilometer per hour
    //            1 - Average Speed, uint16, Unit is 1/100 of a kilometer per hour
    //           2 - Total Distance, uint24, Unit metr
    //          3 - Inclination, sint16, Unit is 1/10 of a percent
    //              Ramp Angle Setting, sint16, Unit is 1/10 of a degree
    //         4 - Positive Elevation Gain, uint16, Unit is 1/10 of a meter, since the training session has started.
    //             Negative Elevation Gain, uint16, Unit is 1/10 of a meter, since the training session has started.
    // More: GATT_Specification_Supplement_v7.pdf


    uint16_t speedInKmph = mps * 360;                       // kilometers per hour with a resolution of 0.01
    uint16_t positiveElevationGain = elevationGain * 10;    // elevation from begining of the training session, in meters, resolution 0.1

    treadmillDataArray[0] = (uint8_t)(FTMSFlags & 0xFF);
    treadmillDataArray[1] = (uint8_t)(FTMSFlags >> 8);

    treadmillDataArray[2] = (uint8_t)(speedInKmph & 0xFF);
    treadmillDataArray[3] = (uint8_t)(speedInKmph >> 8);

    treadmillDataArray[4] = (uint8_t)(inclinePercent & 0xFF);
    treadmillDataArray[5] = (uint8_t)(inclinePercent >> 8);
    
    treadmillDataArray[6] = (uint8_t)(inclineDegree & 0xFF);
    treadmillDataArray[7] = (uint8_t)(inclineDegree >> 8);

    treadmillDataArray[8] = (uint8_t)(positiveElevationGain & 0xFF);
    treadmillDataArray[9] = (uint8_t)(positiveElevationGain >> 8);

    treadmillDataArray[10] = 0;
    treadmillDataArray[11] = 0;

    return treadmillDataArray;
}

TreadmillData treadmillData;

class ServerStatusCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        Serial.println("BLE Connected");
        bluetoothClientConected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        Serial.println("BLE Disconnected");
        bluetoothClientConected = false;
        pServer->getAdvertising()->start();        
    }
};

void mpu6050SensorSetup() {
    Serial.println("Adafruit MPU6050 test");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            // blink two times in case of mpu6050 cannot start
            delay(1000);
            for (u_int8_t i = 0; i < 2; i++) {
                digitalWrite(LED_BUILTIN, HIGH);
                delay(200);
                digitalWrite(LED_BUILTIN, LOW);
                delay(200);
            }
        }
    }
    Serial.println("MPU6050 Found!");

    // setup motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(2);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it lanched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
}

void hallSensorSetup() {
    Serial.println("Configure Hall sensor");
    pinMode(HALL_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), rotationIncrement, RISING);
}

void vl53loxSensorSetup() {
    Serial.println("Configure VL53LOX sensor");
    if (!tofSensor.init()) {
        while (1) {
            // blink three times in case of vl53lox cannot start
            delay(1000);
            for (u_int8_t i = 0; i < 3; i++) {
                digitalWrite(LED_BUILTIN, HIGH);
                delay(200);
                digitalWrite(LED_BUILTIN, LOW);
                delay(200);
            }
        }
    };
}

void bleSetup() {
    Serial.println("BLE configuration");   
    BLEDevice::init("TreadmillBLE");
    
    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerStatusCallbacks());

    // Create the BLE Service for Speed and Cadence
    Serial.println("BLE Speed and Cadence configuration"); 
    BLEService *pRSC = pServer->createService(RSCService);
    pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
    RSCDescriptor.setValue("Speed and Cadence descriptor");
    RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
    RSCMeasurementCharacteristics.addDescriptor(new BLE2902());
    pServer->getAdvertising()->addServiceUUID(RSCService);

    // Create the BLE service for Treadmill
    Serial.println("BLE Treadmill configuration"); 
    BLEService *pFTMS = pServer->createService(FTMSService);
    pFTMS->addCharacteristic(&TreadmillDataCharacteristics);
    FTMSDescriptor.setValue("FTMS Treadmill Descriptor");
    TreadmillDataCharacteristics.addDescriptor(&FTMSDescriptor);
    TreadmillDataCharacteristics.addDescriptor(new BLE2902());
    pServer->getAdvertising()->addServiceUUID(FTMSService);

    pRSC->start();
    pFTMS->start();

    Serial.println("BLE start Advertising");   
    pServer->getAdvertising()->start();
    bleNotifyTime = millis();
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    while(!Serial){
        delay(10);
    }

    Serial.println("Initial setup");
    
    hallSensorSetup();
    Wire.begin();
    
    vl53loxSensorSetup();
    mpu6050SensorSetup();
    bleSetup();
    
    Serial.println("Setup done.");
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
    treadmillData.procedSpeed();
    
    treadmillData.procedIncline();

    treadmillData.procedCadence();

    if (millis() - bleNotifyTime > 1000) {
        RSCMeasurementCharacteristics.setValue(treadmillData.getRscData(), 10);
        RSCMeasurementCharacteristics.notify();
        TreadmillDataCharacteristics.setValue(treadmillData.getTreadmillData(), 34);
        TreadmillDataCharacteristics.notify();
        bleNotifyTime = millis();
    }

    delay(10);
}
