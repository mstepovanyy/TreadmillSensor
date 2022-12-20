#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define HALL_SENSOR
#define VL53LOX
#define D_MPU6050


#ifdef VL53LOX
// incline sensor
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X tofSensor;
#endif

#ifdef D_MPU6050
// acceleration & gyro
#include <MPU6050.h>
MPU6050 mpu;
#define INTERRUPT_PIN 35
#endif

unsigned long ble_notify_time = millis();


#define HALL_SENSOR_PIN 26
#define TREADMILL_LENGTH 1450
// TOF_SENSOR_HEIGTH = min_sensor_height - (TREADMILL_LENGTH * sin(radians(MIN_INCLINE_IN_DEGREE))) = 170 - 24 = 150
#define TOF_SENSOR_HEIGTH 150
// #define INITIAL_INCLINE_PERCENT 1.65
// #define INITIAL_INCLINE_DEGREE 0.95




class TreadmillData {
    public:
        float mps = 0;
        uint8_t cadence = 0;                // Unit is 1/min
        int16_t incline_percent = 0;        // Unit is 1/10 of a percent
        int16_t incline_degree = 0;         // Unit is 1/10 of a degree
        uint16_t elevation_gain = 0;        // Elevation gain from begining of training session, Unit is 1/10 of a meter
        uint32_t total_distance = 0;        // Total distance of running over all time

        uint8_t rsc_data_array[10] = {};
        uint8_t treadmill_array[34] = {};

    public:
        uint8_t* getRscData();
        uint8_t* getTreadmillData();
        void procedSpeedData();
        void procedIncline();
    private:
        unsigned long start_rotation_time = 0;
        unsigned int rotation_count = 0;
        float rotation_length = 0.14;

        unsigned long start_incline_time = 0;
        unsigned long start_read_value_time = 0;
        uint16_t incline_max = 0;
};

void TreadmillData::procedSpeedData() {
    rotation_count++;
    Serial.println("Rotation");

    if (rotation_count >= 20) {
        unsigned long end_rotation_time = millis();
        mps = rotation_count * rotation_length / (end_rotation_time - start_rotation_time) * 1000;
        start_rotation_time = end_rotation_time;
        rotation_count = 0;
    }
}

void TreadmillData::procedIncline() {
    unsigned long end_incline_time = millis();

    if (end_incline_time - start_incline_time > 3000) {
        uint16_t incline_in_mm = incline_max - TOF_SENSOR_HEIGTH;
        if (incline_in_mm > 1000) incline_in_mm = 1;
        uint16_t length_at_zero_level = sqrt(TREADMILL_LENGTH * TREADMILL_LENGTH - incline_in_mm * incline_in_mm);
        incline_percent = incline_in_mm * 100 * 10 / length_at_zero_level;  // 100% and 0.1 resolution
        incline_degree = degrees(atan(incline_in_mm/(float)length_at_zero_level)) * 10;     // 0.1 resolution

        
        unsigned long duration = end_incline_time - start_incline_time;
        elevation_gain += mps * duration / 100 * incline_percent / 1000;    // duration in millis 1s = 1000ms, and elevation resolution 1/10.
        start_incline_time = end_incline_time;
        /*Serial.print("Incline: ");
        Serial.print(incline_in_mm);
        Serial.print(", length_at_zero_level: ");
        Serial.print(length_at_zero_level);
        Serial.print(", percent: ");
        Serial.print(incline_percent);
        Serial.print(", degree: ");
        Serial.print(incline_degree);
        Serial.print(", elevation: ");
        Serial.println(elevation_gain);*/
        incline_max = 0;
    } else if (end_incline_time - start_read_value_time > 200) {  // Read incline each 200ms and pick max value
#ifdef VL53LOX
        incline_max = max(incline_max, tofSensor.readRangeSingleMillimeters());
        start_read_value_time = end_incline_time;
#endif
    }
}


uint8_t* TreadmillData::getRscData() {
    float speed_in_meters_per_sec = mps;
    uint16_t speed_in_units = speed_in_meters_per_sec * 256;                // 1 km=1000m, 1 hour=3600 sec(60*60) , Unit is 1/256th of a m/s
    uint16_t stride_length = speed_in_meters_per_sec / cadence * 2 * 100;   // Unit is 1/100 m
    total_distance += speed_in_meters_per_sec * 10;                         // TODO: calculate this data. Unit is 1/10 m, 
 
    rsc_data_array[0] = 3;
    // speed
    rsc_data_array[1] = (uint8_t)(speed_in_units & 0xFF);
    rsc_data_array[2] = (uint8_t)(speed_in_units >> 8);
    // cadence
    rsc_data_array[3] = (uint8_t)(cadence);
    // stride length
    rsc_data_array[4] = (uint8_t)(stride_length & 0xFF);
    rsc_data_array[5] = (uint8_t)(stride_length >> 8);
    // total distance
    rsc_data_array[6] = (uint8_t)(total_distance & 0xFF);
    rsc_data_array[7] = (uint8_t)((total_distance >> 8) & 0xFF);
    rsc_data_array[8] = (uint8_t)((total_distance >> 16) & 0xFF);
    rsc_data_array[9] = (uint8_t)(total_distance >> 24);

    return rsc_data_array;
}

uint8_t* TreadmillData::getTreadmillData() {
    uint16_t flags = 0x0018;    
    // '000000011000'
    //                             0 - Instantaneous Speed, uint16, Present if Flags field is set to 0, Unit is 1/100 of a kilometer per hour
    //                             1 - Average Speed, uint16, Unit is 1/100 of a kilometer per hour
    //                             2 - Total Distance, uint24, Unit metr
    //                             3 - Inclination, sint16, Unit is 1/10 of a percent
    //                                 Ramp Angle Setting, sint16, Unit is 1/10 of a degree
    //                             4 - Positive Elevation Gain, uint16, Unit is 1/10 of a meter, since the training session has started.
    //                                 Negative Elevation Gain, uint16, Unit is 1/10 of a meter, since the training session has started.
    //                             More: GATT_Specification_Supplement_v7.pdf


    uint16_t speed_in_kmph = mps * 360;     // kilometer per hour with a resolution of 0.01
    int16_t incline_percent_in_units = incline_percent * 10; // percent with a resolution of 0.1
    int16_t incline_degree_in_units = incline_degree * 10;
    uint16_t positive_elevation_gain = elevation_gain * 10;

    // treadmillData[0,1] -> flags
    treadmill_array[0] = (uint8_t)(flags & 0xFF);
    treadmill_array[1] = (uint8_t)(flags >> 8);

    // speed with resolution 0.01
    treadmill_array[2] = (uint8_t)(speed_in_kmph & 0xFF);
    treadmill_array[3] = (uint8_t)(speed_in_kmph >> 8);

    // incline percentage with resolution 0.1
    treadmill_array[4] = (uint8_t)(incline_percent_in_units & 0xFF);
    treadmill_array[5] = (uint8_t)(incline_percent_in_units >> 8);
    // incline degree with resolution 0.1
    treadmill_array[6] = (uint8_t)(incline_degree_in_units & 0xFF);
    treadmill_array[7] = (uint8_t)(incline_degree_in_units >> 8);

    // Positive Elevation Gain 16 Meters with a resolution of 0.1
    treadmill_array[8] = (uint8_t)(positive_elevation_gain & 0xFF);
    treadmill_array[9] = (uint8_t)(positive_elevation_gain >> 8);

    // Positive Elevation Gain 16 Meters with a resolution of 0.1
    treadmill_array[10] = 0;
    treadmill_array[11] = 0;

    return treadmill_array;
}

TreadmillData treadmillData;

void onSpeedDataRise() { treadmillData.procedSpeedData(); };
void step_done() {Serial.print("Motion detected"); };
bool clientConected = false;

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));

#define FTMSService BLEUUID((uint16_t)0x1826)
BLECharacteristic TreadmillDataCharacteristics(BLEUUID((uint16_t)0x2ACD), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor TreadmillDescriptor(BLEUUID((uint16_t)0x2901));

class ServerStatusCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        clientConected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        clientConected = false;
    }
};

void setup()
{
    Serial.begin(115200);
    while(!Serial){
        delay(10);
    }
    Serial.println("Initial setup");
    Wire.begin();
    
    Serial.println("Configure Hall sensor");
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), onSpeedDataRise, RISING);

#ifdef VL53LOX
    Serial.println("Configure VL53LOX sensor");
    tofSensor.init();
#endif

#ifdef D_MPU6050
    Serial.println("Setup MPU6050");
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println("Setup MPU6050: setInterruptMode()");
    mpu.setInterruptMode(true);         // включаем режим прерываний
    mpu.setIntMotionEnabled(true);      // включаем прерывания движения
    mpu.setMotionDetectionThreshold(2); // порог 0..255
    mpu.setMotionDetectionDuration(2);  // таймаут 0..255
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), step_done, RISING);    // прерывание
    Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
#endif

    Serial.println("BLE configuration");   
    BLEDevice::init("Treadmill_and_Speed_Cadence");
    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerStatusCallbacks());

    Serial.println("BLE speed and cadence configuration");   
    // Create the BLE Service for Speed and Cadence
    BLEService *pRSC = pServer->createService(RSCService);
    pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
    RSCDescriptor.setValue("Speed and Cadence descriptor");
    RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
    RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

    pServer->getAdvertising()->addServiceUUID(RSCService);

    Serial.println("BLE Treadmill configuration");   
    // Create the BLE service for Treadmill
    BLEService *pService = pServer->createService(FTMSService);
    pService->addCharacteristic(&TreadmillDataCharacteristics);
    TreadmillDescriptor.setValue("Treadmill Descriptor");
    TreadmillDataCharacteristics.addDescriptor(&TreadmillDescriptor);
    TreadmillDataCharacteristics.addDescriptor(new BLE2902());

    pServer->getAdvertising()->addServiceUUID(FTMSService);

    pRSC->start();
    pService->start();

    Serial.println("BLE start Advertising");   
    // Start advertising
    pServer->getAdvertising()->start();
    Serial.print("Setup done.");
    ble_notify_time = millis();
}



void loop()
{
    treadmillData.procedIncline();

    if (millis() - ble_notify_time > 1000) { // Notify BLE each second
        RSCMeasurementCharacteristics.setValue(treadmillData.getRscData(), 10);
        RSCMeasurementCharacteristics.notify();
        TreadmillDataCharacteristics.setValue(treadmillData.getTreadmillData(), 34);
        TreadmillDataCharacteristics.notify();
        ble_notify_time = millis();

        Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
    }

    delay(100);
}
