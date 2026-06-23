#include <Arduino.h>
#include <NimBLEDevice.h>

#define TX_TO_BOARD_B 17
#define RX_FROM_BOARD_B 16
#define UART_BAUD_RATE 115200

#define BH_SERVICE_UUID     "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BH_CHAR_MOTOR       "6e400002-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_SERIAL      "6e400003-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_CONFIG      "6e400005-b5a3-f393-e0a9-e50e24dcca9e"
#define BH_CHAR_VERSION     "6e400007-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_BATTERY     "6e400008-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_STABLE      "6e40000a-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_MONITOR     "6e40000b-b5a3-f393-e0a9-e50e24dcca9e"
#define BH_CHAR_ATH         "6e40000c-b5a3-f393-e0a9-e50e24dcca9e"

uint8_t currentMotorState[40] = {0};
uint8_t bh_serial[10] = { 0xcf, 0xcb, 0x0d, 0x95, 0x5f, 0xf6, 0xee, 0x2c, 0xbd, 0x73 };

void sendStateToBoardB() {
    uint8_t frame[45];
    frame[0] = 0xAA; 
    frame[1] = 0xBB;
    uint8_t checksum = 0;
    
    for (int i = 0; i < 40; i++) {
        frame[2 + i] = currentMotorState[i];
        checksum ^= currentMotorState[i];
    }
    
    frame[42] = checksum; 
    frame[43] = 0xCC; 
    frame[44] = 0xDD;
    
    Serial2.write(frame, sizeof(frame));
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("[BLE] Connected to PC");
    }
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("[BLE] Disconnected from PC");
        NimBLEDevice::startAdvertising();
        memset(currentMotorState, 0, 40);
        sendStateToBoardB();
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        
        if (rxValue.length() == 20) {
            const uint8_t* payload = (const uint8_t*)rxValue.data();
            
            for (int i = 0; i < 20; i++) {
                uint8_t byte = payload[i];
                uint8_t m1 = (byte >> 4) & 0x0F;
                uint8_t m2 = byte & 0x0F;
                
                currentMotorState[i * 2]     = (m1 * 100) / 15;
                currentMotorState[i * 2 + 1] = (m2 * 100) / 15;
            }
            sendStateToBoardB();
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, RX_FROM_BOARD_B, TX_TO_BOARD_B);
    delay(500);

    Serial.println("\n--- BOARD A (senseshift) ---");

    NimBLEDevice::init("TactSuitX40");
    NimBLEDevice::setSecurityAuth(false, false, true);

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pDeviceInfoService = pServer->createService("180A");
    pDeviceInfoService->createCharacteristic("2A29", NIMBLE_PROPERTY::READ)->setValue("bhaptics");
    pDeviceInfoService->createCharacteristic("2A24", NIMBLE_PROPERTY::READ)->setValue("TactSuitX40");
    pDeviceInfoService->createCharacteristic("2A25", NIMBLE_PROPERTY::READ)->setValue(bh_serial, 10);
    pDeviceInfoService->createCharacteristic("2A26", NIMBLE_PROPERTY::READ)->setValue("1.0.0");
    pDeviceInfoService->createCharacteristic("2A27", NIMBLE_PROPERTY::READ)->setValue("1.0.0");
    pDeviceInfoService->start();

    NimBLEService *pBatteryService = pServer->createService("180F");
    NimBLECharacteristic *pBatChar = pBatteryService->createCharacteristic("2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uint8_t batLvl = 100;
    pBatChar->setValue(&batLvl, 1);
    pBatteryService->start();

    NimBLEService *pService = pServer->createService(BH_SERVICE_UUID);
    
    RxCallbacks* pRxCallbacks = new RxCallbacks();

    auto* motorChar = pService->createCharacteristic(BH_CHAR_MOTOR, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    motorChar->setCallbacks(pRxCallbacks);

    auto* motorCharStable = pService->createCharacteristic(BH_CHAR_STABLE, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    motorCharStable->setCallbacks(pRxCallbacks);

    auto* customBatChar = pService->createCharacteristic(BH_CHAR_BATTERY, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
    uint16_t customBatLvl = 100;
    customBatChar->setValue((uint8_t*)&customBatLvl, 2);

    pService->createCharacteristic(BH_CHAR_SERIAL, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE)->setValue(bh_serial, 10);

    uint16_t firmwareVersion = 0xFFFF;
    pService->createCharacteristic(BH_CHAR_VERSION, NIMBLE_PROPERTY::READ)->setValue((uint8_t*)&firmwareVersion, 2);

    pService->createCharacteristic(BH_CHAR_CONFIG, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pService->createCharacteristic(BH_CHAR_MONITOR, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::BROADCAST | NIMBLE_PROPERTY::INDICATE | NIMBLE_PROPERTY::WRITE_NR);
    pService->createCharacteristic(BH_CHAR_ATH, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setAppearance(509); 
    
    NimBLEAdvertisementData scanData;
    scanData.setName("TactSuitX40");
    scanData.setAppearance(509);
    pAdvertising->setScanResponseData(scanData);

    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Waiting for PC connection UwU");
}

void loop() {
    delay(100);
}