#include <Arduino.h>

/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-web-bluetooth/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
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
#define LED_PIN            8


BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;     // Read-only
BLECharacteristic* pBrightnessCharacteristic = NULL; // Write-only
BLECharacteristic* pLedCharacteristic = NULL;        // Write-only

bool deviceConnected = false;
bool oldDeviceConnected = false;

unsigned long lastNotify = 0;

bool ledOn = true;
uint8_t brightness = 0;    // how bright the LED is
uint8_t oldBrightness = 0;    


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID                   "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID     "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID        "19b10002-e8f2-537e-4f6c-d104768a1214"
#define BRIGHTNESS_CHARACTERISTIC_UUID "19b10003-e8f2-537e-4f6c-d104768a1214"

// #define SERVICE_UUID               "555b0000-b2f0-48ee-ab74-3ce1af15f506"
// #define SENSOR_CHARACTERISTIC_UUID "555b0001-b2f0-48ee-ab74-3ce1af15f506"
// #define LED_CHARACTERISTIC_UUID    "555b0002-b2f0-48ee-ab74-3ce1af15f506"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class LedCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1 || receivedValue == '1') {
        ledOn = true;
      } else {
        ledOn = false;
      }
    }
  }
};

class BrightnessCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Brightness Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue > 0 && receivedValue <= 255) {
        brightness = receivedValue;
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
    stcPrefs.putUChar("curBright", 128);
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
    stcPrefs.putUChar("curBright", brightness);
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
  brightness = stcPrefs.getUChar("curBright");
  oldBrightness = brightness;

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
  pBrightnessCharacteristic->setCallbacks(new BrightnessCharacteristicCallbacks());

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new LedCharacteristicCallbacks());

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
  ledcAttach(LED_PIN, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);

}

void loop() {
  // notify changed value
  if (deviceConnected) {
    unsigned long msec = millis();
    if (msec - lastNotify > 1000) {
      lastNotify = msec;
      pSensorCharacteristic->setValue(String(brightness));
      pSensorCharacteristic->notify();
      Serial.print("New value notified: ");
      Serial.println(brightness);

      if (oldBrightness != brightness) {
        oldBrightness = brightness;
        updatePrefs();
      }
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

  if (ledOn) {
    // set the brightness on LEDC channel 0
    ledcAnalogWrite(LED_PIN, brightness);
  } else {
    // set the brightness on LEDC channel 0
    ledcAnalogWrite(LED_PIN, 0);
  }

  delay(30);
}

