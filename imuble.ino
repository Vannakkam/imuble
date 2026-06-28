#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

// --- Configuration ---
#define SDA_PIN 6
#define SCL_PIN 7
#define SAMPLING_RATE 200 // Hz
#define INTERVAL_MS (1000 / SAMPLING_RATE)
#define BATCH_SIZE 5      // Number of samples per BLE packet (5 * 20 bytes = 100 bytes)

// --- BLE UUIDs ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_CMD  "beb5483e-36e1-4688-b7f5-ea07361b26a8" 
#define CHARACTERISTIC_DATA "0000ffe1-0000-1000-8000-00805f9b34fb" 

// --- MPU6050 Registers ---
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1   0x6B

// --- Global Variables ---
BLEServer *pServer = NULL;
BLECharacteristic *pCmdCharacteristic = NULL;
BLECharacteristic *pDataCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool isRecording = false;

unsigned long lastSampleTime = 0;
uint16_t seqNum = 0; 

// --- Batching Structure ---
struct IMUSample {
  uint16_t seq;
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
};

IMUSample sampleBuffer[BATCH_SIZE];
uint8_t sampleIndex = 0;

// Command Callback
class CmdCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      if (value.length() > 0) {
        if (value == "START") {
          isRecording = true;
          seqNum = 0;
          sampleIndex = 0; 
          Serial.println("Recording Started");
        } else if (value == "STOP") {
          isRecording = false;
          Serial.println("Recording Stopped");
        } else if (value == "RESET") {
          seqNum = 0;
          sampleIndex = 0;
          isRecording = false;
          Serial.println("Sequence & State Reset");
        }
      }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

void setupMPU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); 
  Wire.endTransmission(true);
  delay(100);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, true);

  sampleBuffer[sampleIndex].ax = Wire.read() << 8 | Wire.read();
  sampleBuffer[sampleIndex].ay = Wire.read() << 8 | Wire.read();
  sampleBuffer[sampleIndex].az = Wire.read() << 8 | Wire.read();
  sampleBuffer[sampleIndex].temp = Wire.read() << 8 | Wire.read();
  sampleBuffer[sampleIndex].gx = Wire.read() << 8 | Wire.read();
  sampleBuffer[sampleIndex].gy = Wire.read() << 8 | Wire.read();
  sampleBuffer[sampleIndex].gz = Wire.read() << 8 | Wire.read();
}

void sendBatch() {
  uint8_t payload[BATCH_SIZE * 20]; // 100 bytes total
  
  // Loop through the batch and pack the data
  for (int i = 0; i < BATCH_SIZE; i++) {
    int offset = i * 20;
    memcpy(payload + offset, &sampleBuffer[i].seq, 2);
    memcpy(payload + offset + 2, &sampleBuffer[i].timestamp, 4);
    memcpy(payload + offset + 6, &sampleBuffer[i].ax, 2);
    memcpy(payload + offset + 8, &sampleBuffer[i].ay, 2);
    memcpy(payload + offset + 10, &sampleBuffer[i].az, 2);
    memcpy(payload + offset + 12, &sampleBuffer[i].temp, 2);
    memcpy(payload + offset + 14, &sampleBuffer[i].gx, 2);
    memcpy(payload + offset + 16, &sampleBuffer[i].gy, 2);
    memcpy(payload + offset + 18, &sampleBuffer[i].gz, 2);
  }

  pDataCharacteristic->setValue(payload, BATCH_SIZE * 20);
  pDataCharacteristic->notify();
}

void setup() {
  Serial.begin(115200);
  setupMPU();

  BLEDevice::init("XIAO_C3_IMU");
  
  // --- RANGE & THROUGHPUT OPTIMIZATIONS ---
  BLEDevice::setPower(ESP_PWR_LVL_P9); // Max TX Power (+9dBm)
  BLEDevice::setMTU(128);              // Request 128 MTU (Allows 125 bytes payload)
  // Note: Connection parameters are automatically negotiated by the central device (phone/PC)

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCmdCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_CMD, BLECharacteristic::PROPERTY_WRITE);
  pCmdCharacteristic->setCallbacks(new CmdCallback());

  pDataCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_DATA, BLECharacteristic::PROPERTY_NOTIFY);
  pDataCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  
  Serial.println("Waiting for Web Bluetooth connection...");
}

void loop() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  unsigned long now = millis();
  if (now - lastSampleTime >= INTERVAL_MS) {
    lastSampleTime += INTERVAL_MS;
    
    readMPU(); 
    
    sampleBuffer[sampleIndex].seq = seqNum++;
    sampleBuffer[sampleIndex].timestamp = now;
    
    sampleIndex++;

    // When the buffer is full, pack and send the batch
    if (sampleIndex == BATCH_SIZE) {
      sampleIndex = 0;
      if (isRecording && deviceConnected) {
        sendBatch();
      }
    }
  }
}
