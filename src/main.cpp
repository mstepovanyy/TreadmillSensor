//#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// incline sensor
#include <VL53L0X.h>
VL53L0X tofSensor;

// acceleration & gyro
Adafruit_MPU6050 mpu;

#define HALL_SENSOR_PIN 26
// Default pins for SDA=21, SCL=22
#define TREADMILL_LENGTH 1450
// TOF_SENSOR_HEIGTH = min_sensor_height - (TREADMILL_LENGTH * sin(radians(MIN_INCLINE_IN_DEGREE))) = 170 - 24 = 150
#define TOF_SENSOR_HEIGTH 150
// #define INITIAL_INCLINE_PERCENT 1.65
// #define INITIAL_INCLINE_DEGREE 0.95

#define ACCELERATION_Z 9.8

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));

#define FTMSService BLEUUID((uint16_t)0x1826)
BLECharacteristic TreadmillDataCharacteristics(BLEUUID((uint16_t)0x2ACD), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor FTMSDescriptor(BLEUUID((uint16_t)0x2901));

bool bluetoothClientConected = false;
unsigned long bleNotifyTime = millis();

class TreadmillData {
    public:
        float mps = 0;
        uint8_t cadence = 0;                // Unit is 1/min
        int16_t inclinePercent = 0;        // Unit is 1/10 of a percent
        int16_t inclineDegree = 0;         // Unit is 1/10 of a degree
        uint16_t elevationGain = 0;        // Elevation gain from begining of training session, Unit is 1/10 of a meter
        uint32_t totalDistance = 0;        // Total distance of running over all time

        uint8_t rscDataArray[10] = {};
        uint8_t treadmillDataArray[34] = {};

    public:
        uint8_t* getRscData();
        uint8_t* getTreadmillData();
        void procedSpeedData();
        void procedIncline();
        void procedCadence();
        void rotationIncrement() { rotationCount++; };
    private:
        unsigned long startRotationTimeFrame = 0;
        unsigned int rotationCount = 0;
        float rotationLength = 0.14;

        unsigned long startInclineTimeFrame = 0;
        unsigned long startReadValueTime = 0;
        uint16_t inclineMax = 0;

        uint16_t stepCount = 0;
        bool stepDown = false;
        bool stepUp = false;
        unsigned long startCadenceTimeFrame = 0;
};

void TreadmillData::procedSpeedData() {
    unsigned long endRotationTimeFrame = millis();
    if (endRotationTimeFrame - startRotationTimeFrame  > 1000) {
        mps = rotationCount * rotationLength / (endRotationTimeFrame - startRotationTimeFrame) * 1000;
        startRotationTimeFrame = endRotationTimeFrame;
        rotationCount = 0;
        Serial.print("Current speed: ");
        Serial.println(mps);
    }
}

void TreadmillData::procedIncline() {
    unsigned long end_incline_time = millis();

    if (end_incline_time - startInclineTimeFrame > 3000) {
        uint16_t incline_in_mm = inclineMax - TOF_SENSOR_HEIGTH;
        if (incline_in_mm > 1000) incline_in_mm = 1;
        uint16_t length_at_zero_level = sqrt(TREADMILL_LENGTH * TREADMILL_LENGTH - incline_in_mm * incline_in_mm);
        inclinePercent = incline_in_mm * 100 * 10 / length_at_zero_level;  // 100% and 0.1 resolution
        inclineDegree = degrees(atan(incline_in_mm/(float)length_at_zero_level)) * 10;     // 0.1 resolution

        unsigned long duration = end_incline_time - startInclineTimeFrame;
        elevationGain += mps * duration / 100 * inclinePercent / 1000;    // duration in millis 1s = 1000ms, and elevation resolution 1/10.
        startInclineTimeFrame = end_incline_time;
        Serial.print("Incline: ");
        Serial.print(incline_in_mm);
        Serial.print(", length_at_zero_level: ");
        Serial.print(length_at_zero_level);
        Serial.print(", percent: ");
        Serial.print(inclinePercent);
        Serial.print(", degree: ");
        Serial.print(inclineDegree);
        Serial.print(", elevation: ");
        Serial.println(elevationGain);
        inclineMax = 0;
    } else if (end_incline_time - startReadValueTime > 200) {  // Read incline each 200ms and pick max value
        inclineMax = max(inclineMax, tofSensor.readRangeSingleMillimeters());
        startReadValueTime = end_incline_time;
    }
}

uint8_t* TreadmillData::getRscData() {
    uint16_t speedInUnits = mps * 256;                  // Unit is 1/256th of a m/s
    uint16_t strideLength = mps / cadence * 2 * 100;    // Unit is 1/100 m
    totalDistance += mps * 10;                          // TODO: calculate this data. Unit is 1/10 m, distance over time
 
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


    uint16_t speedInKmph = mps * 360;                       // kilometer per hour with a resolution of 0.01
    int16_t inclinePercentInUnits = inclinePercent * 10;    // percent with a resolution of 0.1
    int16_t inclineDegreeInUnits = inclineDegree * 10;      // degree with a resolution of 0.1
    uint16_t positiveElevationGain = elevationGain * 10;    // elevation from begining of the training session, in meters, resolution 0.1

    treadmillDataArray[0] = (uint8_t)(FTMSFlags & 0xFF);
    treadmillDataArray[1] = (uint8_t)(FTMSFlags >> 8);

    treadmillDataArray[2] = (uint8_t)(speedInKmph & 0xFF);
    treadmillDataArray[3] = (uint8_t)(speedInKmph >> 8);

    treadmillDataArray[4] = (uint8_t)(inclinePercentInUnits & 0xFF);
    treadmillDataArray[5] = (uint8_t)(inclinePercentInUnits >> 8);
    
    treadmillDataArray[6] = (uint8_t)(inclineDegreeInUnits & 0xFF);
    treadmillDataArray[7] = (uint8_t)(inclineDegreeInUnits >> 8);

    treadmillDataArray[8] = (uint8_t)(positiveElevationGain & 0xFF);
    treadmillDataArray[9] = (uint8_t)(positiveElevationGain >> 8);

    treadmillDataArray[10] = 0;
    treadmillDataArray[11] = 0;

    return treadmillDataArray;
}

void TreadmillData::procedCadence() {
    if(mpu.getMotionInterruptStatus()) {
        /* Get new sensor events with the readings */
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        if (a.acceleration.z < 0) {
            stepDown = true;
        }
        
        if (a.acceleration.z > ACCELERATION_Z) {
            stepUp = true;
        }

        if (stepDown && stepUp) {
            stepCount++;
            stepDown = false;
            stepUp = false;
        }

        /* Print out the values */
        Serial.print("AccelX:");
        Serial.print(a.acceleration.x);
        Serial.print(",");
        Serial.print("AccelY:");
        Serial.print(a.acceleration.y);
        Serial.print(",");
        Serial.print("AccelZ:");
        Serial.print(a.acceleration.z);
        Serial.print(", ");
        Serial.print("GyroX:");
        Serial.print(g.gyro.x);
        Serial.print(",");
        Serial.print("GyroY:");
        Serial.print(g.gyro.y);
        Serial.print(",");
        Serial.print("GyroZ:");
        Serial.print(g.gyro.z);
        Serial.println("");
    }

    unsigned long endCadenceTimeFrame = millis();
    if (endCadenceTimeFrame - startCadenceTimeFrame  > 1000) {
        if (stepCount == 0) {
            cadence = 0;
        } else {
            cadence = stepCount * 1000 / (endCadenceTimeFrame - startCadenceTimeFrame);
        }
        startCadenceTimeFrame = endCadenceTimeFrame;
        stepCount = 0;
        Serial.print("Current cadence: ");
        Serial.println(cadence);
    }
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

void mpu6050Setup() {
    Serial.println("Adafruit MPU6050 test!");

    // Try to initialize!
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(30);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    Serial.println("");
}

void hallSensorSetup() {
    Serial.println("Configure Hall sensor");
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), []() { treadmillData.rotationIncrement(); }, RISING);
}

void vl53loxSensorSetup() {
    Serial.println("Configure VL53LOX sensor");
    tofSensor.init();
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
    Serial.begin(115200);
    while(!Serial){
        delay(10);
    }

    Serial.println("Initial setup");
    
    Wire.begin();
    
    hallSensorSetup();
    vl53loxSensorSetup();
    mpu6050Setup();
    bleSetup();
    
    Serial.println("Setup done.");
}

void loop()
{
    treadmillData.procedSpeedData();
    
    treadmillData.procedIncline();

    treadmillData.procedCadence();

    if (millis() - bleNotifyTime > 1000) { // Notify BLE each second
        RSCMeasurementCharacteristics.setValue(treadmillData.getRscData(), 10);
        RSCMeasurementCharacteristics.notify();
        TreadmillDataCharacteristics.setValue(treadmillData.getTreadmillData(), 34);
        TreadmillDataCharacteristics.notify();
        bleNotifyTime = millis();
    }

    delay(10);
}
