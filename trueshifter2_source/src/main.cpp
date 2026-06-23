#include <Arduino.h>
#include <NimBLEDevice.h>

#define TX_TO_BOARD_A 17 
#define RX_FROM_BOARD_A 16
#define UART_BAUD_RATE 115200

#define PIN_TEST_GND 22
#define PIN_TEST_BTN 23

bool lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 

unsigned long lastStatusCheckTime = 0;
const unsigned long STATUS_INTERVAL = 30000; 

static NimBLEUUID truegearServiceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID truegearRxUUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID truegearTxUUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e"); 

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

bool isConnectedToTrueGear = false;
bool isScanning = false;
NimBLEAddress* trueGearAddress = nullptr;
bool doConnect = false;

uint8_t currentMotorState[40] = {0};
uint8_t lastSentMotorState[40] = {0};
unsigned long lastSendTime = 0;
bool hasNewData = false;

const int FRONT_GROUPS[20] = { 1, 1, 4, 4, 1, 1, 4, 4, 1, 1, 4, 4, 1, 1, 4, 4, 1, 1, 4, 4 };
const int BACK_GROUPS[20] = { 2, 2, 3, 3, 2, 2, 3, 3, 2, 2, 3, 3, 2, 2, 3, 3, 2, 2, 3, 3 };
const uint16_t MASKS[20] = {
    0x8000, 0x0400, 0x8000, 0x0400, 0x4000, 0x0200, 0x4000, 0x0200,
    0x2000, 0x0100, 0x2000, 0x0100, 0x1000, 0x0080, 0x1000, 0x0080,
    0x0800, 0x0040, 0x0800, 0x0040
};

void translateAndSendToTrueGear() {
    if (pRemoteCharacteristic == nullptr || !isConnectedToTrueGear) return;

    uint16_t durationMillis = 150; 
    uint8_t frame[20] = {0};
    frame[0] = 0x68; frame[1] = 0x68;
    frame[2] = 0x01; frame[3] = 0x01;
    frame[4] = 0x00; frame[5] = 0x00; frame[6] = 0x00;
    frame[7] = (durationMillis >> 8) & 0xFF; frame[8] = durationMillis & 0xFF;

    int maxIntens = 0;
    uint16_t group1 = 0, group2 = 0, group3 = 0, group4 = 0;

    for (int i = 0; i < 40; i++) {
        int motorValue = currentMotorState[i];
        if (motorValue > maxIntens) maxIntens = motorValue;

        if (motorValue > 0) {
            if (i < 20) {
                if (FRONT_GROUPS[i] == 1) group1 |= MASKS[i];
                if (FRONT_GROUPS[i] == 4) group4 |= MASKS[i];
            } else {
                int idx = i - 20;
                if (BACK_GROUPS[idx] == 2) group2 |= MASKS[idx];
                if (BACK_GROUPS[idx] == 3) group3 |= MASKS[idx];
            }
        }
    }

    int intensity = (int)((maxIntens / 100.0) * 255.0);
    frame[9] = (uint8_t)intensity; frame[10] = (uint8_t)intensity;
    frame[11] = (group1 >> 8) & 0xFF; frame[12] = group1 & 0xFF;
    frame[13] = (group2 >> 8) & 0xFF; frame[14] = group2 & 0xFF;
    frame[15] = (group3 >> 8) & 0xFF; frame[16] = group3 & 0xFF;
    frame[17] = (group4 >> 8) & 0xFF; frame[18] = group4 & 0xFF;
    frame[19] = 0x16;

    pRemoteCharacteristic->writeValue(frame, 20, false);
}

void processUART() {
    if (Serial2.available() >= 45) {
        if (Serial2.read() == 0xAA) {
            if (Serial2.read() == 0xBB) {
                uint8_t tempBuf[40];
                Serial2.readBytes(tempBuf, 40);
                uint8_t checksum = Serial2.read();
                uint8_t footer1 = Serial2.read();
                uint8_t footer2 = Serial2.read();

                if (footer1 == 0xCC && footer2 == 0xDD) {
                    uint8_t calcCheck = 0;
                    for (int i = 0; i < 40; i++) calcCheck ^= tempBuf[i];
                    
                    if (calcCheck == checksum) {
                        memcpy(currentMotorState, tempBuf, 40);
                        hasNewData = true;
                    }
                }
            }
        }
    }
}

void processTestButton() {
    bool reading = digitalRead(PIN_TEST_BTN);

    if (reading != lastBtnState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        static bool isVibrating = false; 

        if (reading == LOW) {
            if (!isVibrating) {
                Serial.println("[TEST] Pins 22 & 23 shorted. Sending haptics...");
                isVibrating = true;
            }
            memset(currentMotorState, 100, 40);
            hasNewData = true;
        } 
        else {
            if (isVibrating) {
                Serial.println("[TEST] Pins opened. Stopping haptics.");
                isVibrating = false;
                memset(currentMotorState, 0, 40);
                hasNewData = true;
            }
        }
    }
    lastBtnState = reading;
}

class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pclient) {
        Serial.println("[BLE] TrueGear radio link established");
    }

    void onDisconnect(NimBLEClient* pclient, int reason) {
        Serial.print("[BLE] TrueGear disconnected. HCI Reason: 0x");
        Serial.println(reason, HEX);
        isConnectedToTrueGear = false;
    }
};

void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) { 
}

bool connectToTrueGear(NimBLEAddress address) {
    if (pClient == nullptr) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallbacks());
    }

    if (!pClient->connect(address)) {
        Serial.println("[BLE] Connection rejected.");
        return false;
    }

    pClient->updateConnParams(12, 24, 0, 500); 
    delay(1000);

    NimBLERemoteService* pRemoteService = pClient->getService(truegearServiceUUID);
    if (pRemoteService == nullptr) {
        pClient->disconnect();
        return false;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(truegearRxUUID);
    if (pRemoteCharacteristic == nullptr) {
        pClient->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic* pTxCharacteristic = pRemoteService->getCharacteristic(truegearTxUUID);
    if (pTxCharacteristic != nullptr && pTxCharacteristic->canNotify()) {
        pTxCharacteristic->subscribe(true, notifyCallback);
    }

    Serial.println("[BLE] SUCCESS! Bridge ready.");
    isConnectedToTrueGear = true;
    lastStatusCheckTime = millis();
    return true;
}

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        std::string name = advertisedDevice->getName();
        if (name.find("Truegear") != std::string::npos || name.find("truegear") != std::string::npos) {
            Serial.print("[BLE] Found TrueGear MAC: ");
            Serial.println(advertisedDevice->getAddress().toString().c_str());
            
            NimBLEDevice::getScan()->stop();
            isScanning = false;
            
            if (trueGearAddress != nullptr) {
                delete trueGearAddress;
            }
            trueGearAddress = new NimBLEAddress(advertisedDevice->getAddress());
            
            doConnect = true;
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, RX_FROM_BOARD_A, TX_TO_BOARD_A);

    pinMode(PIN_TEST_GND, OUTPUT);
    digitalWrite(PIN_TEST_GND, LOW); 
    pinMode(PIN_TEST_BTN, INPUT_PULLUP); 

    Serial.println("\n--- BOARD B (TrueGear Bridge) ---");

    NimBLEDevice::init("");
    NimBLEDevice::setMTU(512); 
    NimBLEDevice::setSecurityAuth(false, false, true); 

    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
    pBLEScan->setActiveScan(true);
    
    Serial.println("[BLE] Scanning for TrueGear...");
    pBLEScan->start(0, false);
    isScanning = true;
}

void loop() {
    processTestButton();

    if (doConnect == true && trueGearAddress != nullptr) {
        doConnect = false;
        delay(250); 
        connectToTrueGear(*trueGearAddress);
    }

    processUART();

    if (isConnectedToTrueGear && hasNewData) {
        unsigned long now = millis();
        
        if (now - lastSendTime >= 35) {
            
            bool stateChanged = false;
            bool isAllZero = true;

            for (int i = 0; i < 40; i++) {
                if (currentMotorState[i] != lastSentMotorState[i]) stateChanged = true;
                if (currentMotorState[i] > 0) isAllZero = false;
            }

            bool shouldSend = false;

            if (stateChanged) {
                shouldSend = true;
            } 
            else if (!isAllZero) {
                if (now - lastSendTime >= 120) {
                    shouldSend = true;
                }
            }

            if (shouldSend) {
                translateAndSendToTrueGear();
                memcpy(lastSentMotorState, currentMotorState, 40); 
                lastSendTime = now;
            }
        }
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastStatusCheckTime >= STATUS_INTERVAL) {
        lastStatusCheckTime = currentMillis;
        
        if (isConnectedToTrueGear) {
            Serial.println("[STATUS] TrueGear CONNECTED.");
        } else {
            Serial.println("[STATUS] TrueGear DISCONNECTED.");
            if (!isScanning && !doConnect) {
                isScanning = true;
                NimBLEDevice::getScan()->start(0, false);
            }
        }
    }

    if (!isConnectedToTrueGear && !isScanning && !doConnect) {
        isScanning = true;
        NimBLEDevice::getScan()->start(0, false);
    }
}