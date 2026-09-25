/*
 * Mopeka BLE MAC address scanner
 * (C) Justin M Dunlop 2026
 * ------------------------------------------------------------------
 * Scans for BLE advertisements and prints details for anything that
 * looks like a Mopeka sensor (manufacturer ID 0x0059), so you can
 * collect the MAC addresses of all your Mopeka devices.
 *
 * If you have MULTIPLE Mopeka sensors, this also prints each one's
 * Sensor ID (a unique per-device identifier baked into the sensor
 * itself) and RSSI (signal strength) — useful for telling which MAC
 * belongs to which physical tank: the sensor closest to the ESP32
 * will generally have the strongest (least negative) RSSI, or you
 * can temporarily cover/uncover one sensor's face and watch which
 * entry's raw level changes.
 *
 * Just flash this, open Serial Monitor at 115200, and let it run for
 * 10-20 seconds. Every distinct Mopeka MAC it sees gets logged.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define MOPEKA_MANUFACTURER_ID 0x0059

BLEScan* pBLEScan;

class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String manufacturerData = advertisedDevice.getManufacturerData();
    if (manufacturerData.length() < 10) return; // too short to be Mopeka

    uint8_t data[12] = {0};
    size_t n = min((size_t)12, manufacturerData.length());
    for (size_t i = 0; i < n; i++) data[i] = (uint8_t)manufacturerData[i];

    uint16_t manufacturerID = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if (manufacturerID != MOPEKA_MANUFACTURER_ID) return; // not a Mopeka device — skip

    String address = advertisedDevice.getAddress().toString().c_str();
    address.toLowerCase();

    uint8_t hardwareID = data[2];
    uint32_t sensorID = ((uint32_t)data[7] << 16) | ((uint32_t)data[8] << 8) | (uint32_t)data[9];
    int rssi = advertisedDevice.getRSSI();

    Serial.println();
    Serial.println("=== MOPEKA SENSOR FOUND ===");
    Serial.print("  MAC:        "); Serial.println(address);
    Serial.print("  Hardware ID: 0x"); Serial.println(hardwareID, HEX);
    Serial.print("  Sensor ID:   "); Serial.printf("%06lX\n", sensorID);
    Serial.print("  RSSI:        "); Serial.print(rssi); Serial.println(" dBm (closer to 0 = stronger)");
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Scanning for Mopeka sensors...");
  Serial.println("Leave this running 10-20s, or longer if you have several sensors.");
  Serial.println();

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);
  pBLEScan->start(0, false); // 0 = scan continuously
}

void loop() {
  delay(1000); // nothing to do here — onResult() fires asynchronously
}
