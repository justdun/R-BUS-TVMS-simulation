/*
 * RedArc TVMS Prime Tank Sennosr via TVMS sensor inputs via ESP32 + Mopeka Pro (per tank) + 2x MCP23017 + HY-M154 (per tank)
 * (C) Justin M Dunlop 2026
 *----------------------------------------------------------------------------
 * Drives RedARC's discrete 4-wire tank inputs (25/50/75/100%) via
 * optocouplers, from live Mopeka BLE sensor readings. Supports up to
 *  6 tanks, each independently linked to its own Mopeka sensor by MAC
 *  address, with its own empty/full calibration.
 *
 *  WHY TWO MCP23017 BOARDS
 *  ------------------------
 *  Each tank needs 4 output pins (one per discrete level) driving one
 *  4-channel opto/relay board (e.g. HY-M154) into that tank's 4
 *  physical RedARC input pins. One MCP23017 has 16 GPIO pins = 4
 *  tanks' worth. A second MCP23017, at a different I2C address on the
 *  SAME bus, is needed for tanks 5-6.
 *
 *  PIN ASSIGNMENT (fixed by the code below, not configurable per-tank)
 *  ---------------------------------------------------------------
 *    Tank 1: MCP23017 #1 (0x20), pins 0-3   (GPA0-GPA3)
 *    Tank 2: MCP23017 #1 (0x20), pins 4-7   (GPA4-GPA7)
 *    Tank 3: MCP23017 #1 (0x20), pins 8-11  (GPB0-GPB3)
 *    Tank 4: MCP23017 #1 (0x20), pins 12-15 (GPB4-GPB7)
 *    Tank 5: MCP23017 #2 (0x21), pins 0-3   (GPA0-GPA3)
 *    Tank 6: MCP23017 #2 (0x21), pins 4-7   (GPA4-GPA7)
 *
 * Within each tank's 4 pins, pin+0/1/2/3 = 25%/50%/75%/100% — wire
 * each to its own HY-M154 (or equivalent) channel, which then wires
 * to that tank's corresponding RedARC input pin.
 *
 * WIRING
 * ------
 * ESP32 I2C (shared by both MCP23017 boards):
 *   GPIO21 -> SDA
 *   GPIO22 -> SCL
 *
 * MCP23017 #1:
 *   VCC -> 3.3V, GND -> GND
 *   A0, A1, A2 -> GND               (address 0x20)
 *
 * MCP23017 #2:
 *   VCC -> 3.3V, GND -> GND
 *   A0 -> 3.3V, A1 -> GND, A2 -> GND (address 0x21)
 *
 * Per tank, per level (e.g. Tank 1, 25%):
 *   MCP23017 pin -> HY-M154 input channel -> HY-M154 output ->
 *   RedARC tank input pin for that level
 *   RedARC common -> HY-M154 output GND
 *
 * Fill in TANKS[] below with each sensor's MAC (run
 * mopeka_mac_scanner.ino to find them) and empty/full calibration.
 * Leave mac = "" for any tank you don't have a sensor for yet — its
 * outputs will simply stay off (0%).
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// =================================================================
// EDIT YOUR TANKS HERE
// =================================================================
struct TankConfig {
  const char* mac;   // lowercase MAC from mopeka_mac_scanner.ino, or "" for no sensor
  float empty_mm;    // Mopeka-measured height at empty
  float full_mm;     // Mopeka-measured height at full
};

static TankConfig TANKS[6] = {
  /* Tank 1 */ { "", 0.0, 290.0 },
  /* Tank 2 */ { "", 0.0, 370.0 },
  /* Tank 3 */ { "", 0.0, 370.0 },
  /* Tank 4 */ { "", 0.0, 370.0 },
  /* Tank 5 */ { "", 0.0, 370.0 },
  /* Tank 6 */ { "", 0.0, 370.0 },
};

#define BLE_SCAN_TIME_SEC 5     // how long each poll scans for
#define SCAN_DELAY_MS 15000     // how often to poll — tank level doesn't change fast

// Set false if you only have ONE MCP23017 board (tanks 1-4 only) and
// don't want the code to even attempt talking to a second one. Either
// way, if the second board doesn't respond, the code now logs a
// warning and continues with tanks 1-4 only, instead of halting.
#define USE_SECOND_MCP true
// =================================================================

#define SDA_PIN 21
#define SCL_PIN 22
#define MCP1_ADDRESS 0x20
#define MCP2_ADDRESS 0x21
#define MOPEKA_MANUFACTURER_ID 0x0059

Adafruit_MCP23X17 mcp1;
Adafruit_MCP23X17 mcp2;
bool mcp2Available = false; // set true in setup() only if it's actually found
BLEScan* pBLEScan;

int currentRedarcLevel[6] = { -1, -1, -1, -1, -1, -1 };
volatile bool newReadingAvailable[6] = { false, false, false, false, false, false };
int pendingRedarcLevel[6] = { 0, 0, 0, 0, 0, 0 };

// ---------------------------------------------------------------
// Which MCP chip (0 or 1) and base pin (within that chip) each
// tank's 4 output pins start at.
// ---------------------------------------------------------------
int tankMcpIndex(int tankIndex) { return tankIndex / 4; }
int tankBasePin(int tankIndex) { return (tankIndex % 4) * 4; }
Adafruit_MCP23X17* mcpForTank(int tankIndex) { return (tankMcpIndex(tankIndex) == 0) ? &mcp1 : &mcp2; }

// ---------------------------------------------------------------
// RedARC output control
// ---------------------------------------------------------------
// Returns false (and skips the hardware write) if this tank needs
// the second MCP23017 and it isn't available.
bool tankOutputReady(int tankIndex)
{
  if (tankMcpIndex(tankIndex) == 1 && !mcp2Available) {
    static bool warned[6] = {false, false, false, false, false, false};
    if (!warned[tankIndex]) {
      warned[tankIndex] = true;
      Serial.printf("Tank %d: reading received, but no second MCP23017 — output not driven.\n", tankIndex + 1);
    }
    return false;
  }
  return true;
}

void allLevelsOff(int tankIndex)
{
  if (!tankOutputReady(tankIndex)) return;
  Adafruit_MCP23X17* mcp = mcpForTank(tankIndex);
  int base = tankBasePin(tankIndex);
  for (int i = 0; i < 4; i++) mcp->digitalWrite(base + i, LOW);
}

void setRedarcLevel(int tankIndex, int level)
{
  if (!tankOutputReady(tankIndex)) return;
  if (level == currentRedarcLevel[tankIndex]) return;

  allLevelsOff(tankIndex);
  delay(20);

  Adafruit_MCP23X17* mcp = mcpForTank(tankIndex);
  int base = tankBasePin(tankIndex);

  switch (level)
  {
    case 0:   break; // all off = 0%
    case 25:  mcp->digitalWrite(base + 0, HIGH); break;
    case 50:  mcp->digitalWrite(base + 1, HIGH); break;
    case 75:  mcp->digitalWrite(base + 2, HIGH); break;
    case 100: mcp->digitalWrite(base + 3, HIGH); break;
  }

  currentRedarcLevel[tankIndex] = level;

  Serial.printf("Tank %d output set to %d%%\n", tankIndex + 1, level);
}

// ---------------------------------------------------------------
// Mopeka decoding
// ---------------------------------------------------------------
float mopekaRawToHeight(uint16_t rawLevel, uint8_t rawTemperature)
{
  double rawT = rawTemperature;
  double height = rawLevel * (0.573045 - (0.002822 * rawT) - (0.00000535 * rawT * rawT));
  return (float)height;
}

float heightToPercentage(float heightMM, float emptyMM, float fullMM)
{
  float percentage = ((heightMM - emptyMM) / (fullMM - emptyMM)) * 100.0;
  if (percentage < 0.0) percentage = 0.0;
  if (percentage > 100.0) percentage = 100.0;
  return percentage;
}

// RedARC's inputs are discrete (4 wires), so the continuous Mopeka
// percentage has to be bucketed to the nearest supported level.
int percentageToRedarcLevel(float percentage)
{
  if (percentage < 12.5) return 0;
  if (percentage < 37.5) return 25;
  if (percentage < 62.5) return 50;
  if (percentage < 87.5) return 75;
  return 100;
}

int findTankForMac(const String& address)
{
  for (int i = 0; i < 6; i++) {
    if (strlen(TANKS[i].mac) > 0 && address.equals(TANKS[i].mac)) return i;
  }
  return -1;
}

class MopekaCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    String address = advertisedDevice.getAddress().toString().c_str();
    address.toLowerCase();

    int tankIndex = findTankForMac(address);
    if (tankIndex < 0) return; // not one of our configured tanks

    String manufacturerData = advertisedDevice.getManufacturerData();
    if (manufacturerData.length() < 10) return;

    uint8_t data[12] = {0};
    size_t n = min((size_t)12, manufacturerData.length());
    for (size_t i = 0; i < n; i++) data[i] = (uint8_t)manufacturerData[i];

    uint16_t manufacturerID = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if (manufacturerID != MOPEKA_MANUFACTURER_ID) return;

    uint8_t rawTemperature = data[4] & 0x7F;
    uint16_t rawLevel = ((uint16_t)data[5] | ((uint16_t)data[6] << 8)) & 0x3FFF;

    float heightMM = mopekaRawToHeight(rawLevel, rawTemperature);
    float percentage = heightToPercentage(heightMM, TANKS[tankIndex].empty_mm, TANKS[tankIndex].full_mm);
    int level = percentageToRedarcLevel(percentage);

    pendingRedarcLevel[tankIndex] = level;
    newReadingAvailable[tankIndex] = true;

    Serial.printf("Tank %d: %.1f%% (height=%.1fmm, raw=%d) -> RedARC level %d%%\n",
      tankIndex + 1, percentage, heightMM, rawLevel, level);
  }
};

// ---------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32 MOPEKA -> REDARC TVMS INTERFACE");
  Serial.println("========================================");

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Starting MCP23017 #1 (0x20)...");
  if (!mcp1.begin_I2C(MCP1_ADDRESS)) {
    Serial.println("ERROR: MCP23017 #1 NOT FOUND at 0x20!");
    Serial.println("Check SDA->21, SCL->22, VCC->3.3V, GND->GND, A0/A1/A2->GND");
    while (1) delay(1000);
  }
  Serial.println("MCP23017 #1 found.");

  for (int pin = 0; pin < 16; pin++) {
    mcp1.pinMode(pin, OUTPUT);
    mcp1.digitalWrite(pin, LOW);
  }

  // Second board is optional — tanks 5-6 only. A missing/unwired
  // board here is a normal setup (4-tank system), not an error, so
  // this warns and continues instead of halting.
  if (USE_SECOND_MCP) {
    Serial.println("Starting MCP23017 #2 (0x21)...");
    if (mcp2.begin_I2C(MCP2_ADDRESS)) {
      mcp2Available = true;
      Serial.println("MCP23017 #2 found.");
      for (int pin = 0; pin < 16; pin++) {
        mcp2.pinMode(pin, OUTPUT);
        mcp2.digitalWrite(pin, LOW);
      }
    } else {
      Serial.println("WARNING: MCP23017 #2 not found at 0x21 — continuing with tanks 1-4 only.");
      Serial.println("(If you only have one board, set USE_SECOND_MCP to false to skip this check.)");
    }
  } else {
    Serial.println("USE_SECOND_MCP is false — skipping second board, tanks 1-4 only.");
  }

  Serial.println("Tank outputs configured.");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MopekaCallbacks(), true);
  pBLEScan->setActiveScan(false); // passive — Mopeka's data is in the advertisement itself
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);

  Serial.println();
  for (int i = 0; i < 6; i++) {
    bool needsMissingMcp = (tankMcpIndex(i) == 1 && !mcp2Available);
    if (strlen(TANKS[i].mac) > 0) {
      Serial.printf("Tank %d: live from %s (empty=%.0fmm full=%.0fmm) -> MCP#%d pins %d-%d%s\n",
        i + 1, TANKS[i].mac, TANKS[i].empty_mm, TANKS[i].full_mm,
        tankMcpIndex(i) + 1, tankBasePin(i), tankBasePin(i) + 3,
        needsMissingMcp ? "  [NO OUTPUT - 2nd MCP missing]" : "");
    } else {
      Serial.printf("Tank %d: no sensor configured (outputs stay off)\n", i + 1);
    }
  }
  Serial.println();
  Serial.println("System ready.");
  Serial.println();
}

void loop()
{
  Serial.println("Scanning for Mopeka sensors...");
  pBLEScan->start(BLE_SCAN_TIME_SEC, false);
  pBLEScan->clearResults();

  for (int i = 0; i < 6; i++) {
    if (newReadingAvailable[i]) {
      newReadingAvailable[i] = false;
      setRedarcLevel(i, pendingRedarcLevel[i]);
    }
  }

  delay(SCAN_DELAY_MS);
}
