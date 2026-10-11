#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_system.h>
#include <mutex>
#include <Preferences.h>


// MUTEKS
std::mutex motorMutex;   
std::mutex stateMutex;   
std::mutex configMutex;  


// DEFINITIONS-OTHER
#define TRUESHIFTER_VERSION "2.0.0"

// DEBOUNCE  & TIMERS

bool lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 
unsigned long lastStatusCheckTime = 0;
const unsigned long STATUS_INTERVAL = 15000; 


// UUIDs for BH Server, TG Client 

#define BH_SERVICE_UUID     "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BH_CHAR_MOTOR       "6e400002-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_SERIAL      "6e400003-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_CONFIG      "6e400005-b5a3-f393-e0a9-e50e24dcca9e"
#define BH_CHAR_VERSION     "6e400007-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_BATTERY     "6e400008-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_STABLE      "6e40000a-b5a3-f393-e0a9-e50e24dcca9e" 
#define BH_CHAR_MONITOR     "6e40000b-b5a3-f393-e0a9-e50e24dcca9e"
#define BH_CHAR_ATH         "6e40000c-b5a3-f393-e0a9-e50e24dcca9e"
#define BH_SERVICE_DFU      "0000fe59-0000-1000-8000-00805f9b34fb"
#define BH_CHAR_DFU_CTRL    "8ec90003-f315-4f60-9fb8-838830daea50"

static NimBLEUUID truegearServiceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID truegearRxUUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID truegearTxUUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e"); 


// GLOB VAR

uint8_t currentMotorState[40] = {};
uint8_t lastSentMotorState[40] = {};
unsigned long lastSendTime = 0;
bool hasNewData = false; 

uint8_t bh_serial[10] = { 0xcf, 0xcb, 0x0d, 0x95, 0x5f, 0xf6, 0xee, 0x2c, 0xbd, 0x73 };

NimBLECharacteristic* pBhapticsBatteryChar = nullptr;
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

bool isConnectedToTrueGear = false;
bool isScanning = false;
NimBLEAddress* trueGearAddress = nullptr;
bool doConnect = false;

uint8_t globalIntensity = 60;


// BH -> TG motor remap

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


// PC DATA CALLBACK

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        size_t len = rxValue.length();
        
        if (len > 0) {
            const uint8_t* payload = (const uint8_t*)rxValue.data();
            std::lock_guard<std::mutex> lock(motorMutex);
            memset(currentMotorState, 0, 40);  
            size_t bytesToProcess = (len < 20) ? len : 20;
            
            for (size_t i = 0; i < bytesToProcess; i++) {
                uint8_t byte = payload[i];
                uint8_t m1 = (byte >> 4) & 0x0F;
                uint8_t m2 = byte & 0x0F;
                
                currentMotorState[i * 2]     = (m1 * 100) / 15;
                currentMotorState[i * 2 + 1] = (m2 * 100) / 15;
            }
            hasNewData = true; 
        }
    }
};


//  INTENSITY CALLBACK 

class ConfigCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() >= 1) {
            uint8_t newIntensity = value[0];
            if (newIntensity > 100) newIntensity = 100;
            
            {
                std::lock_guard<std::mutex> lock(configMutex);
                globalIntensity = newIntensity;
            }
            
            Preferences preferences;
            preferences.begin("app", false);
            preferences.putUChar("intensity", globalIntensity);
            preferences.end();
            
            Serial.printf("NEW INTENSITY: %d%%\n", globalIntensity);
        } 
    }
};

// SERWER CALLBACK
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Connected to PC/VR/APP");
    }
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Disconnected. Restarting advertising...");
        NimBLEDevice::startAdvertising();
        {
            std::lock_guard<std::mutex> lock(motorMutex);
            memset(currentMotorState, 0, 40);
            hasNewData = true;
        }
    }
};

// SEND TO TG
void translateAndSendToTrueGear(const uint8_t* motorData) {
    if (pRemoteCharacteristic == nullptr) return;

    bool connected;
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        connected = isConnectedToTrueGear;
    }
    if (!connected) return;

    uint8_t localIntensity;
    {
        std::lock_guard<std::mutex> lock(configMutex);
        localIntensity = globalIntensity;
    }
    
    uint16_t durationMillis = 150; 
    uint8_t frame[20] = {0};
    frame[0] = 0x68; frame[1] = 0x68;
    frame[2] = 0x01; frame[3] = 0x01;
    frame[4] = 0x00; frame[5] = 0x00; frame[6] = 0x00;
    frame[7] = (durationMillis >> 8) & 0xFF; frame[8] = durationMillis & 0xFF;

    int maxIntens = 0;
    uint16_t group1 = 0, group2 = 0, group3 = 0, group4 = 0;

    for (int i = 0; i < 40; i++) {
        if (motorData[i] > maxIntens) maxIntens = motorData[i];
        if (motorData[i] > 0) {
            uint8_t targetGroup = BH_TO_TG_GROUP[i];
            uint16_t targetMask = BH_TO_TG_MASK[i];
            if (targetGroup == 1) group1 |= targetMask;
            else if (targetGroup == 2) group2 |= targetMask;
            else if (targetGroup == 3) group3 |= targetMask;
            else if (targetGroup == 4) group4 |= targetMask;
        }
    }

    float intensityFactor = localIntensity / 100.0f;
    float maxIntensScaled = maxIntens * intensityFactor;
    int intensity = (int)((maxIntensScaled / 100.0f) * 127.0f);
    if (intensity > 255) intensity = 255;
    if (intensity < 0) intensity = 0;
    
    frame[9] = (uint8_t)intensity; frame[10] = (uint8_t)intensity;
    frame[11] = (group1 >> 8) & 0xFF; frame[12] = group1 & 0xFF;
    frame[13] = (group2 >> 8) & 0xFF; frame[14] = group2 & 0xFF;
    frame[15] = (group3 >> 8) & 0xFF; frame[16] = group3 & 0xFF;
    frame[17] = (group4 >> 8) & 0xFF; frame[18] = group4 & 0xFF;
    frame[19] = 0x16;
    
    pRemoteCharacteristic->writeValue(frame, 20, false);
}

// CLIENT CALLBACK
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pclient) {
        Serial.println("[CLIENT] TrueGear connected");
    }
    void onDisconnect(NimBLEClient* pclient, int reason) {
        Serial.println("[CLIENT] TrueGear disconnected");
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            isConnectedToTrueGear = false;
        }
    }
};

// TG BATTERY
void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.printf("odebrano pakiet len=%d, isNotify=%d, dane: ", length, isNotify);
    for (int i=0; i<length; i++) Serial.printf("%02X ", pData[i]);
    Serial.println();

    if (length == 20 && pData[0] == 0x68 && pData[1] == 0x68 && pData[3] == 0x81) {
        int vestVoltage = (pData[9] << 8) | pData[10];
        
        int percentage = 0;
        if (vestVoltage >= 4150) percentage = 100;
        else if (vestVoltage <= 3300) percentage = 0;
        else percentage = (int)(((vestVoltage - 3300.0f) / (4150.0f - 3300.0f)) * 100.0f);

        if (percentage > 100) percentage = 100;
        if (percentage < 0) percentage = 0;

        Serial.printf(" BATTERY: %d%% (%d mV)\n", percentage, vestVoltage);

        if (pBhapticsBatteryChar) {
            uint16_t finalVal = (uint16_t)percentage;
            pBhapticsBatteryChar->setValue((uint8_t*)&finalVal, 2);
            pBhapticsBatteryChar->notify();
        }
    }
}

// TG CONNECTION

bool connectToTrueGear(NimBLEAddress address) {
    if (pClient == nullptr) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallbacks());
    }

    if (!pClient->connect(address)) return false;

    pClient->updateConnParams(24, 48, 0, 500); 
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
        pTxCharacteristic->registerForNotify(notifyCallback);
        Serial.println("[CLIENT] Subskrypcja TX zarejestrowana.");
        
        NimBLERemoteDescriptor* pDesc = pTxCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902));
        if (pDesc) {
            uint8_t val[2] = {0x01, 0x00};
            pDesc->writeValue(val, 2, true);
            Serial.println("[CLIENT] CCCD ustawiony ręcznie.");
        } else {
            Serial.println("[CLIENT] Nie znaleziono deskryptora CCCD!");
        }
    } else {
        Serial.println("[CLIENT] TX characteristic not found or cannot notify!");
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        isConnectedToTrueGear = true;
    }
    lastStatusCheckTime = millis();
    return true;
}

// SCAN FOR TG


class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        std::string name = advertisedDevice->getName();
        if (name.find("Truegear") != std::string::npos || name.find("truegear") != std::string::npos) {
            NimBLEDevice::getScan()->stop();
            {
                std::lock_guard<std::mutex> lock(stateMutex);
                isScanning = false;
                if (trueGearAddress != nullptr) delete trueGearAddress;
                trueGearAddress = new NimBLEAddress(advertisedDevice->getAddress());
                doConnect = true;
            }
        }
    }
};



// SETUP!

void setup() {
    Serial.begin(115200);

    delay(500);
    Serial.printf("\n TrueShifter V%s \n", TRUESHIFTER_VERSION);

    Preferences preferences;
    preferences.begin("app", false);
    globalIntensity = preferences.getUChar("intensity", 100);
    preferences.end();
    Serial.printf("Wczytano intensywność: %d%%\n", globalIntensity);

    uint8_t bhaptics_mac[6] = {0xE0, 0x5A, 0x1B, 0xA1, 0xB2, 0xC3};
    esp_base_mac_addr_set(bhaptics_mac);

    NimBLEDevice::init("TactSuitX40");
    NimBLEDevice::setMTU(128);


    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pDeviceInfoService = pServer->createService("180A");
    pDeviceInfoService->createCharacteristic("2A29", NIMBLE_PROPERTY::READ)->setValue("bhaptics");
    pDeviceInfoService->createCharacteristic("2A24", NIMBLE_PROPERTY::READ)->setValue("TactSuitX40");
    pDeviceInfoService->createCharacteristic("2A25", NIMBLE_PROPERTY::READ)->setValue(bh_serial, 10);
    pDeviceInfoService->createCharacteristic("2A26", NIMBLE_PROPERTY::READ)->setValue(TRUESHIFTER_VERSION);
    pDeviceInfoService->createCharacteristic("2A27", NIMBLE_PROPERTY::READ)->setValue("1.0.0");
    pDeviceInfoService->start();

    NimBLEService *pBatteryService = pServer->createService("180F");
    uint8_t batLvl = 100;
    pBatteryService->createCharacteristic("2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY)->setValue(&batLvl, 1);
    pBatteryService->start();

    NimBLEService *pService = pServer->createService(BH_SERVICE_UUID);
    
    RxCallbacks* pRxCallbacks = new RxCallbacks();
    pService->createCharacteristic(BH_CHAR_MOTOR, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)->setCallbacks(pRxCallbacks);
    pService->createCharacteristic(BH_CHAR_STABLE, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)->setCallbacks(pRxCallbacks);

    pBhapticsBatteryChar = pService->createCharacteristic(BH_CHAR_BATTERY, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
    uint16_t initialBat = 50;
    pBhapticsBatteryChar->setValue((uint8_t*)&initialBat, 2);

    pService->createCharacteristic(BH_CHAR_SERIAL, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE)->setValue(bh_serial, 10);
    

    uint16_t firmwareVersion = 0x0502; 
    pService->createCharacteristic(BH_CHAR_VERSION, NIMBLE_PROPERTY::READ)->setValue((uint8_t*)&firmwareVersion, 2);

    NimBLECharacteristic* pConfigChar = pService->createCharacteristic(
        BH_CHAR_CONFIG, 
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    );
    pConfigChar->setCallbacks(new ConfigCallbacks());
    uint8_t configVal[3] = {globalIntensity, 0, 0};
    pConfigChar->setValue(configVal, 3);

    uint8_t monitorVal = 0;
    pService->createCharacteristic(BH_CHAR_MONITOR, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::BROADCAST | NIMBLE_PROPERTY::INDICATE | NIMBLE_PROPERTY::WRITE_NR)->setValue(&monitorVal, 1);
    

    uint8_t athVal[20] = {0};
    pService->createCharacteristic(BH_CHAR_ATH, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE)->setValue(athVal, 20);

    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setAppearance(509); 
    
    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    std::string mData = "";
    mData += (char)0x9B; mData += (char)0x03; mData += (char)0x00; mData += (char)0x00;
    advData.setManufacturerData(mData);
    pAdvertising->setAdvertisementData(advData);

    NimBLEAdvertisementData scanData;
    scanData.setName("TactSuitX40");
    scanData.setAppearance(509);
    scanData.setCompleteServices(NimBLEUUID((uint16_t)0x1530));
    pAdvertising->setScanResponseData(scanData);
    pAdvertising->start();

    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(0, false);
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        isScanning = true;
    }
    
    Serial.println("[SYSTEM] Server & Client initialized. Starting main loop.");
}


// LOOP

void loop() {

    bool l_doConnect = false;
    NimBLEAddress l_addr;
    bool hasAddr = false;
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (doConnect && trueGearAddress != nullptr) {
            l_doConnect = true;
            l_addr = *trueGearAddress;
            hasAddr = true;
            doConnect = false;
        }
    }
    if (l_doConnect && hasAddr) {
        delay(250);
        connectToTrueGear(l_addr);
    }


    bool connected;
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        connected = isConnectedToTrueGear;
    }
    
    bool hasNew = false;
    {
        std::lock_guard<std::mutex> lock(motorMutex);
        hasNew = hasNewData;
    }
    
    if (connected && hasNew) {
        unsigned long now = millis();
        if (now - lastSendTime >= 35) {
            uint8_t localCopy[40];
            {
                std::lock_guard<std::mutex> lock(motorMutex);
                memcpy(localCopy, currentMotorState, 40);
            }
        
            bool stateChanged = false;
            bool isAllZero = true;
            for (int i = 0; i < 40; i++) {
                if (localCopy[i] != lastSentMotorState[i]) stateChanged = true;
                if (localCopy[i] > 0) isAllZero = false;
            }
            
            bool shouldSend = false;
            if (stateChanged) shouldSend = true;
            else if (!isAllZero && (now - lastSendTime >= 120)) shouldSend = true;

            if (shouldSend) {
                translateAndSendToTrueGear(localCopy);
                {
                    std::lock_guard<std::mutex> lock(motorMutex);
                    memcpy(lastSentMotorState, localCopy, 40);
                    hasNewData = false; 
                }
                lastSendTime = now;
            } else {
                std::lock_guard<std::mutex> lock(motorMutex);
                hasNewData = false;
            }
        }
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastStatusCheckTime >= STATUS_INTERVAL) {
        lastStatusCheckTime = currentMillis;
        bool shouldScan = false;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            if (!isConnectedToTrueGear && !isScanning && !doConnect) {
                shouldScan = true;
                isScanning = true;
            }
        }
        if (shouldScan) {
            NimBLEDevice::getScan()->start(0, false);
        }
    }

    bool shouldScanNow = false;
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (!isConnectedToTrueGear && !isScanning && !doConnect) {
            shouldScanNow = true;
            isScanning = true;
        }
    }
    if (shouldScanNow) {
        NimBLEDevice::getScan()->start(0, false);
    }
    delay(1);
}