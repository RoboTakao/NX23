#include <M5StickC.h>
#include "Hat_JoyC.h"
#include <BLEDevice.h>

#define LED 10

BLEUUID serviceUUID("1010");
BLEUUID charaUUID_RX("1011");
BLEUUID charaUUID_TX("1012");
BLERemoteCharacteristic* pRemoteCharacteristicRX;
BLERemoteCharacteristic* pRemoteCharacteristicTX;

BLEAdvertisedDevice* periphDevice;
BLEScanResults *scanCompleteCB;
boolean connecting = false;
boolean scanning = false;
bool connected = false;
char errCode='0';
String errStr="";

uint8_t joyStick[6];
volatile uint16_t loopCounter=1, loopCounter0=0;

char info[50];

TFT_eSprite canvas = TFT_eSprite(&M5.Lcd);
JoyC joyc;

class funcClientCallbacks: public BLEClientCallbacks {
    void onConnect(BLEClient* pClient) {
    };
    void onDisconnect(BLEClient* pClient) {
      Serial.println("Disconnected");
      pRemoteCharacteristicRX->registerForNotify(NULL);
      delete periphDevice;
      connected = false;
    }
};

class advertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Advertised Device found: ");
    Serial.println(advertisedDevice.getName().c_str());
    if (advertisedDevice.haveServiceUUID()) {
      BLEUUID service = advertisedDevice.getServiceUUID();
      if (service.equals(serviceUUID)) {
        BLEDevice::getScan()->stop();
        periphDevice = new BLEAdvertisedDevice(advertisedDevice);
        connecting = true;
        scanning = true;
      }
    }
    scanning =true;
  }
};

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                uint8_t* pData, size_t length, bool isNotify) {
    errCode=pData[0];
    errStr=(char*)pData;
    if (errCode!='0') {
      Serial.print("Error:");
      Serial.println((char*)pData);
    }
}

void setupBLE() {
    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new advertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(1, false);
}

bool connectBLE() {
    Serial.print("Forming a connection to ");
    Serial.println(periphDevice->getAddress().toString().c_str());
    BLEClient* pClient = BLEDevice::createClient();
    Serial.println(" - Created client");
    pClient->setClientCallbacks(new funcClientCallbacks());
    pClient->connect(periphDevice);
    Serial.println(" - Connected to server");
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.print("Failed to find serviceUUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println("Found ServiceUUID");
    pRemoteCharacteristicRX = pRemoteService->getCharacteristic(charaUUID_RX);
    if (pRemoteCharacteristicRX == nullptr) {
      Serial.print("Failed to find characteristicUUID: ");
      Serial.println(charaUUID_RX.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println("Found CharaUUID RX");
    //if(pRemoteCharacteristicRX->canNotify()) {
    //    pRemoteCharacteristicRX->registerForNotify(notifyCallback);
    //}
    pRemoteCharacteristicTX = pRemoteService->getCharacteristic(charaUUID_TX);
    if (pRemoteCharacteristicTX == nullptr) {
      Serial.print("Failed to find charaUUID_TX: ");
      Serial.println(charaUUID_TX.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println("Found CharaUUID TX");
    connected = true;
    return true;
}

void setup() {
    Serial.begin(115200);
    M5.begin();             // Initialize host device.
    pinMode(LED,OUTPUT);
    digitalWrite(LED,HIGH);
    M5.Lcd.setRotation(1);  // Rotation screen.
    canvas.createSprite(160,80);  // Create a 160*80 canvas.
    joyc.begin();  // Initialize JoyC.
    setupBLE();
}

void loop() {
    joyc.update();             // Update JoyC's data
    if (joyc.btn1 == 1) {
      canvas.fillSprite(BLUE);  // Fill the canvas with blue
    }
    else if (joyc.btn1 == 0) {
      canvas.fillSprite(GREEN);  // Fill the canvas with green
    }
    canvas.pushSprite(0, 0);

    joyStick[0]=joyc.x0;
    joyStick[1]=joyc.y0;
    joyStick[2]=joyc.x1;
    joyStick[3]=joyc.y1;
    joyStick[4]=joyc.btn0;
    joyStick[5]=joyc.btn1;

    if (connecting == true) {
      Serial.println("Connecting...");
      if (connectBLE()) {
        Serial.println("Connected to the BLE Server.");
        digitalWrite(LED,LOW);
      }
      else {
        Serial.println("Failed to connect BLE server.");
      }
      connecting = false;
    }
    if (connected) {
      pRemoteCharacteristicTX->writeValue(joyStick, 6, false); //do not expect response
    } 
    else if (scanning) {
      BLEDevice::getScan()->start(1, false);
    }
}