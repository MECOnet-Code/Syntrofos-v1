/*
   ESP32 scans for BLE devices and sounds a buzzer
   if it finds one of known beacon MAC addresses

   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/


#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define BUZZ_PIN 23
// PWM channel 0 parameter
const int freq = 2000;      // Hz
const int pwmChannel = 0;
const int resolution = 8;   // 8-bit resolution

// MAC addresses to search
const String bejkon[3] = {"ac:23:3f:a8:ee:0a", "ac:23:3f:a8:ee:0b", "ac:23:3f:a8:ee:0c"};

int scanTime = 5;    // in seconds
BLEScan* pBLEScan;
String a = "";
uint32_t found_millis = 0;
uint32_t current_millis;
bool found_ind = false;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      a = advertisedDevice.toString().c_str();
      current_millis = millis();
      if (a.indexOf(bejkon[0]) > 0)
      {
        found_millis = millis();
        Serial.println("Pacient A");
        ledcWrite(pwmChannel, 100);
      }
      else if (a.indexOf(bejkon[1]) > 0)
      {
        found_millis = millis();
        Serial.println("Pacient B");
        ledcWrite(pwmChannel, 100);
      }
      else if (a.indexOf(bejkon[2]) > 0)
      {
        found_millis = millis();
        Serial.println("Pacient C");
        ledcWrite(pwmChannel, 100);
      }
      a = "";
      current_millis = millis();
      if ((abs(current_millis - found_millis)) > 25) // stop the beep after 25 ms to sound periodically
        ledcWrite(pwmChannel, 0);
    }
};

void setup()
{
  ledcSetup(pwmChannel, freq, resolution);  // configure the channel 0
  ledcAttachPin(BUZZ_PIN, pwmChannel);      // attach the channel 0 on the pin
  Serial.begin(115200);
  Serial.println("Scanning...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();  // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);    // active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);          // less or equal setInterval value
} // setup


void loop()
{
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  ledcWrite(pwmChannel, 0);
  Serial.println("Scan done!");
  pBLEScan->clearResults();     // delete results fromBLEScan buffer to release memory
  delay(2000);
} // loop
