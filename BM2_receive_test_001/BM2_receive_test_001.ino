
// Quick BM2 Broadcast receive test using ESP32

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5; //In seconds
BLEScan *pBLEScan;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  int i;
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      uint8_t *payLoad = advertisedDevice.getPayload();
      uint8_t len = advertisedDevice.getPayloadLength();

      if ((len> 15) && (payLoad[8] == 0xff) && (payLoad[12] == 0xeb) && (payLoad[13] == 0xcb)) // BM2 Advert Packet Scan Filter
      {
          Serial.print("BM2 Address: ");
          for(i=11;i<=16;i++)
              Serial.printf("%02x ", payLoad[i]);
          Serial.printf("  ADC Value: V-%02x%02x X-%02x%02x Y-%02x%02x Z-%02x%02x T-%02x -- ",payLoad[18],payLoad[17],payLoad[20],payLoad[19],payLoad[22],payLoad[21],payLoad[24],payLoad[23],payLoad[25]);
          Serial.printf("Voltage : %f", (double)(((double)(payLoad[17]+(payLoad[18] << 8)) / 304.16) -7.0682)); // Need to adjust values for proper calibration
          Serial.println("");  
      }
    }
};

void setup()
{
  Serial.begin(115200);
  Serial.println("Scanning...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value
}

void loop()
{
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory
}
