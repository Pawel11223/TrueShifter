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

const uint8_t BH_TO_TG_GROUP[40] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4
};

const uint16_t BH_TO_TG_MASK[40] = {
    0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040,
    0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040,
    0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040,
    0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040
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
        if (currentMotorState[i] > maxIntens) maxIntens = currentMotorState[i];

        if (currentMotorState[i] > 0) {
            uint8_t targetGroup = BH_TO_TG_GROUP[i];
            uint16_t targetMask = BH_TO_TG_MASK[i];

            if (targetGroup == 1) group1 |= targetMask;
            else if (targetGroup == 2) group2 |= targetMask;
            else if (targetGroup == 3) group3 |= targetMask;
            else if (targetGroup == 4) group4 |= targetMask;
        }
    }
 //------------------------------------
    int intensity = (int)((maxIntens / 100.0) * 65.0);  // !!!!!!
 //------------------------------------
    if (intensity > 255) intensity = 255;
    
    frame[9] = (uint8_t)intensity; frame[10] = (uint8_t)intensity;
    frame[11] = (group1 >> 8) & 0xFF; frame[12] = group1 & 0xFF;
    frame[13] = (group2 >> 8) & 0xFF; frame[14] = group2 & 0xFF;
    frame[15] = (group3 >> 8) & 0xFF; frame[16] = group3 & 0xFF;
    frame[17] = (group4 >> 8) & 0xFF; frame[18] = group4 & 0xFF;
    frame[19] = 0x16;

    pRemoteCharacteristic->writeValue(frame, 20, false);
}

void processUART() {
    static int uartState = 0;
    static int byteCount = 0;
    static uint8_t tempBuf[40];
    static uint8_t checksum = 0;

    while (Serial2.available() > 0) {
        uint8_t b = Serial2.read();

        if (uartState == 0) {
            if (b == 0xAA) uartState = 1;
        } else if (uartState == 1) {
            if (b == 0xBB) {
                uartState = 2;
                byteCount = 0;
            } else {
                uartState = 0;
            }
        } else if (uartState == 2) {
            tempBuf[byteCount++] = b;
            if (byteCount == 40) uartState = 3;
        } else if (uartState == 3) {
            checksum = b;
            uartState = 4;
        } else if (uartState == 4) {
            if (b == 0xCC) uartState = 5;
            else uartState = 0;
        } else if (uartState == 5) {
            if (b == 0xDD) {
                uint8_t calcCheck = 0;
                for (int i = 0; i < 40; i++) calcCheck ^= tempBuf[i];
                
                if (calcCheck == checksum) {
                    memcpy(currentMotorState, tempBuf, 40);
                    hasNewData = true;
                }
            }
            uartState = 0; 
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
            if (!isVibrating) isVibrating = true;
            memset(currentMotorState, 100, 40);
            hasNewData = true;
        } 
        else {
            if (isVibrating) {
                isVibrating = false;
                memset(currentMotorState, 0, 40);
                hasNewData = true;
            }
        }
    }
    lastBtnState = reading;
}

class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pclient) {}
    void onDisconnect(NimBLEClient* pclient, int reason) {
        isConnectedToTrueGear = false;
    }
};

void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {}

bool connectToTrueGear(NimBLEAddress address) {
    if (pClient == nullptr) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallbacks());
    }

    if (!pClient->connect(address)) return false;

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

    isConnectedToTrueGear = true;
    lastStatusCheckTime = millis();
    return true;
}

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        std::string name = advertisedDevice->getName();
        if (name.find("Truegear") != std::string::npos || name.find("truegear") != std::string::npos) {
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
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, RX_FROM_BOARD_A, TX_TO_BOARD_A);

    pinMode(PIN_TEST_GND, OUTPUT);
    digitalWrite(PIN_TEST_GND, LOW); 
    pinMode(PIN_TEST_BTN, INPUT_PULLUP); 

    NimBLEDevice::init("");
    NimBLEDevice::setMTU(512); 
    NimBLEDevice::setSecurityAuth(false, false, true); 

    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
    pBLEScan->setActiveScan(true);
    
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
        
        if (!isConnectedToTrueGear) {
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