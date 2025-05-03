#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//#include "driver/ledc.h"
//#include "esp_err.h"

#include<Preferences.h>
Preferences stcPrefs;

//const int ledPin = 8; // Use the appropriate GPIO pin for your setup

// use 12 bit precission for LEDC timer
#define LEDC_TIMER_12_BIT  12

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     4000

// LED PIN
const int LED0_PIN   = 8;
const int LED1_PIN   = 9;
const int LED_COUNT  = 2;

uint8_t brightness[LED_COUNT] = {128, 255};
bool    ledOn[LED_COUNT] = {true, true};
uint8_t oldBrightness[LED_COUNT] = {1, 1};

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;     // Read-only
BLECharacteristic* pBrightnessCharacteristic = NULL; // Write-only
BLECharacteristic* pLedCharacteristic = NULL;        // Write-only

bool deviceConnected = false;
bool oldDeviceConnected = false;

unsigned long lastNotify = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID                   "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID     "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID        "19b10002-e8f2-537e-4f6c-d104768a1214"
#define BRIGHTNESS_CHARACTERISTIC_UUID "19b10003-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class LedOnOffCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 1) {
      int channel = static_cast<int>(value[0]);
      int receivedValue = static_cast<int>(value[1]);

      if (channel < 0 || channel >= LED_COUNT) {
        Serial.println("Invalid channel number. Ignoring the write request.");
        return;
      }

      Serial.print("LedOnOffCharacteristicCallbacks - value=");
      Serial.print(receivedValue); // Print the integer value
      Serial.print(", channel=");
      Serial.println(channel); // Print the integer value

      if (receivedValue) {
        ledOn[channel] = true;
      } else {
        ledOn[channel] = false;
      }
    }
  }
};

class LedBrightnessCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 1) {
      int channel = static_cast<int>(value[0]);
      int receivedValue = static_cast<int>(value[1]);
      
      if (channel < 0 || channel >= LED_COUNT) {
        Serial.println("Invalid channel number. Ignoring the write request.");
        return;
      }

      Serial.print("LedBrightnessCharacteristicCallbacks - value=");
      Serial.print(receivedValue); // Print the integer value
      Serial.print(", channel=");
      Serial.println(channel); // Print the integer value

      if (receivedValue > 0 && receivedValue <= 255) {
        if (receivedValue == 0) {
          ledOn[channel] = false;
        } else {
          ledOn[channel] = true;
        }
        brightness[channel] = receivedValue;
        lastNotify = millis();
      }
    }
  }
};

#define RW_MODE false
#define RO_MODE true
void initPrefs() {
  stcPrefs.begin("STCPrefs", RO_MODE);          // Open our namespace (or create it
                                                //  if it doesn't exist) in RO mode.

  bool tpInit = stcPrefs.isKey("nvsInit");      // Test for the existence
                                                // of the "already initialized" key.

  if (tpInit == false) {
    // If tpInit is 'false', the key "nvsInit" does not yet exist therefore this
    //  must be our first-time run. We need to set up our Preferences namespace keys. So...
    stcPrefs.end();                             // close the namespace in RO mode and...
    stcPrefs.begin("STCPrefs", RW_MODE);        //  reopen it in RW mode.


    // The .begin() method created the "STCPrefs" namespace and since this is our
    //  first-time run we will create
    //  our keys and store the initial "factory default" values.
    stcPrefs.putBytes("curBright", brightness, LED_COUNT); // Store the initial brightness values
    stcPrefs.putBool("nvsInit", true);          // Create the "already initialized"
                                                //  key and store a value.

    // The "factory defaults" are created and stored so...
    stcPrefs.end();                             // Close the namespace in RW mode and...
    stcPrefs.begin("STCPrefs", RO_MODE);        //  reopen it in RO mode so the setup code
                                                //  outside this first-time run 'if' block
                                                //  can retrieve the run-time values
                                                //  from the "STCPrefs" namespace.
  }

}

void updatePrefs() {
    stcPrefs.end();                             // close the namespace in RO mode and...
    stcPrefs.begin("STCPrefs", RW_MODE);        //  reopen it in RW mode.
    stcPrefs.putBytes("curBright", brightness, LED_COUNT); // Store the brightness values
    for(int i = 0; i < LED_COUNT; i++) {
      Serial.print("updatePrefs - brightness[");
      Serial.print(i);
      Serial.print("]=");
      Serial.println(brightness[i]);
    }
    stcPrefs.end();                             // Close the namespace in RW mode and...
    stcPrefs.begin("STCPrefs", RO_MODE);        //  reopen it in RO mode
}

// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(pin, duty);
}

void setup() {
  Serial.begin(115200);
  //pinMode(ledPin, OUTPUT);

  initPrefs();
  stcPrefs.getBytes("curBright", brightness, LED_COUNT);
  stcPrefs.getBytes("curBright", oldBrightness, LED_COUNT);

  // Create the BLE Device
  BLEDevice::init("Saturn V LUT");
  //BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the ON/OFF Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Create the SetBrightness Characteristic
  pBrightnessCharacteristic = pService->createCharacteristic(
                      BRIGHTNESS_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the ON button characteristic
  pBrightnessCharacteristic->setCallbacks(new LedBrightnessCharacteristicCallbacks());

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new LedOnOffCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());
  pBrightnessCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  
  // Setup timer and attach timer to a led pin
  ledcAttach(LED0_PIN, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttach(LED1_PIN, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
}

void loop() {
  uint8_t buf[2];

  // notify changed value
  if (deviceConnected) {
    unsigned long msec = millis();
    if (msec - lastNotify > 1000) {
      lastNotify = msec;

      pSensorCharacteristic->setValue(brightness, LED_COUNT);
      pSensorCharacteristic->notify();
       
      if (memcmp(oldBrightness, brightness, LED_COUNT) != 0) {
        memcpy(oldBrightness, brightness, LED_COUNT);
          
        for (int i = 0; i < LED_COUNT; i++) {
          Serial.print("value notified: ");
          Serial.print(brightness[i]);
          Serial.print(" channel=");
          Serial.print(i);
        }
        updatePrefs();
      }
      Serial.println();

      //delay(100); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }

  if (ledOn[0]) {    
    ledcAnalogWrite(LED0_PIN, brightness[0]); // set the brightness on LEDC channel
  } else {    
    ledcAnalogWrite(LED0_PIN, 0); // set the brightness on LEDC channel to 0
  }

  if (ledOn[1]) {    
    ledcAnalogWrite(LED1_PIN, brightness[1]); // set the brightness on LEDC channel
  } else {    
    ledcAnalogWrite(LED1_PIN, 0); // set the brightness on LEDC channel to 0
  }

  delay(50);
}

