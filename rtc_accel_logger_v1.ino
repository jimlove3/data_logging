/*
  DS3231 RTC + MPU6050 accelerometer -> timestamped serial output

  Libraries: RTClib by Adafruit (install via Library Manager)
  The MPU6050 is read directly over Wire, so no extra library is needed.

  Wiring (Uno/Nano: SDA = A4, SCL = A5 | Mega: SDA = 20, SCL = 21 | ESP32: SDA = 21, SCL = 22):
    DS3231  VCC -> 3.3V or 5V, GND -> GND, SDA -> SDA, SCL -> SCL
    MPU6050 VCC -> 3.3V (or 5V on a board with regulator), GND -> GND, SDA -> SDA, SCL -> SCL

  !! I2C ADDRESS CONFLICT !!
  Both the DS3231 and the MPU6050 default to address 0x68.
  Connect the MPU6050's AD0 pin to 3.3V so it moves to 0x69.
  (If you leave AD0 low, change MPU_ADDR below to 0x68 and one device will fail.)
*/

#include <Wire.h>
#include <RTClib.h>
#include "Arduino_LED_Matrix.h"

#define MPU_ADDR 0x69          // AD0 pulled HIGH. Use 0x68 if AD0 is low (conflicts with DS3231).
#define SAMPLE_INTERVAL_MS 500 // time between readings

// MPU6050 registers
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// +/-2g range -> 16384 LSB per g
#define ACCEL_SCALE 16384.0

ArduinoLEDMatrix matrix;
// 8 rows x 12 columns. 1 = LED on, 0 = LED off.
// The M is 8 columns wide, centered, with 2-pixel-thick strokes.
byte frame[8][12] = {
  { 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0 }
};

RTC_DS3231 rtc;

unsigned long lastSample = 0;

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6) return false;

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  ax = rawX / ACCEL_SCALE;
  ay = rawY / ACCEL_SCALE;
  az = rawZ / ACCEL_SCALE;
  return true;
}

void setup() {
  
  matrix.begin();
  matrix.renderBitmap(frame, 8, 12);

  Serial.begin(115200);
  while (!Serial) { ; }   // wait for serial on native-USB boards
  Wire.begin();

  // --- DS3231 ---
  if (!rtc.begin()) {
    Serial.println(F("ERROR: DS3231 not found. Check wiring."));
    while (true) delay(1000);
  }

  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power - setting time to compile time."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  // To force-set the clock to the compile time, uncomment once, upload, then re-comment and re-upload:
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // --- MPU6050 ---
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) {      // wake from sleep
    Serial.println(F("ERROR: MPU6050 not found. Check wiring and AD0/address."));
    while (true) delay(1000);
  }
  mpuWrite(REG_ACCEL_CONFIG, 0x00);           // +/-2g

  Serial.println(F("Timestamp,Ax_g,Ay_g,Az_g"));
}

void loop() {
  if (millis() - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = millis();

  DateTime now = rtc.now();
  float ax, ay, az;

  if (!mpuReadAccel(ax, ay, az)) {
    Serial.println(F("MPU6050 read failed"));
    return;
  }

  char ts[20];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  Serial.print(ts);
  Serial.print(',');
  Serial.print(ax, 3);
  Serial.print(',');
  Serial.print(ay, 3);
  Serial.print(',');
  Serial.println(az, 3);
}
