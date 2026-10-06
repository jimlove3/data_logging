/*
  DS3231 RTC + MPU6050 accelerometer -> serial monitor AND CSV file on SD card

  Libraries: RTClib by Adafruit (Library Manager). SD and Wire are built in.

  Wiring (Uno R4 WiFi / Uno: SDA = A4, SCL = A5):
    DS3231   VCC -> 3.3V or 5V, GND -> GND, SDA -> SDA, SCL -> SCL
    MPU6050  VCC -> 3.3V (or 5V on a board with regulator), GND -> GND, SDA -> SDA, SCL -> SCL
    SD card adapter (SPI):
             VCC -> 5V (module must have a regulator/level shifter; bare 3.3V-only modules -> 3.3V)
             GND -> GND, MOSI -> D11, MISO -> D12, SCK -> D13, CS -> D10 (see SD_CS)
    Format the card as FAT32.

  !! I2C ADDRESS CONFLICT !!
  The DS3231 and MPU6050 both default to 0x68. Connect the MPU6050's AD0 pin
  to 3.3V so it moves to 0x69 (MPU_ADDR below).

  FILES
  Each power-up creates a new file: LOG000.CSV, LOG001.CSV, ... (never overwrites).
  Every line is flushed to the card, so you can pull power without losing data.

  CALIBRATION
  Keep the board still at startup; send 'c' in the serial monitor to recalibrate.
    REMOVE_GRAVITY true  -> stationary reads ~0,0,0 in any orientation
    REMOVE_GRAVITY false -> bias removed only; stationary reads ~0,0,1 g (calibrate lying flat)
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>

#define SD_CS 10
#define MPU_ADDR 0x69
#define SAMPLE_INTERVAL_MS 500

#define REMOVE_GRAVITY true
#define CAL_SAMPLES    500
#define FILTER_ALPHA   0.3
#define DEADBAND_G     0.02

// MPU6050 registers
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define ACCEL_SCALE 16384.0    // +/-2g -> 16384 LSB per g

RTC_DS3231 rtc;
File logFile;

unsigned long lastSample = 0;
float offX = 0, offY = 0, offZ = 0;
float fx = 0, fy = 0, fz = 0;

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

void calibrate() {
  Serial.println(F("Calibrating - keep the board still..."));
  delay(1000);

  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    float ax, ay, az;
    if (mpuReadAccel(ax, ay, az)) {
      sx += ax; sy += ay; sz += az;
      n++;
    }
    delay(3);
  }

  if (n == 0) {
    Serial.println(F("Calibration failed - no MPU6050 data."));
    return;
  }

  offX = sx / n;
  offY = sy / n;
  offZ = sz / n;
  if (!REMOVE_GRAVITY) offZ -= 1.0;

  fx = 0; fy = 0;
  fz = REMOVE_GRAVITY ? 0 : 1.0;

  Serial.print(F("Offsets (g): "));
  Serial.print(offX, 4); Serial.print(F(", "));
  Serial.print(offY, 4); Serial.print(F(", "));
  Serial.println(offZ, 4);
}

float deadband(float v) {
  return (REMOVE_GRAVITY && fabs(v) < DEADBAND_G) ? 0.0 : v;
}

// Opens the next unused LOGnnn.CSV and writes the header row.
bool openLogFile() {
  char name[13];
  for (int i = 0; i < 1000; i++) {
    snprintf(name, sizeof(name), "LOG%03d.CSV", i);
    if (!SD.exists(name)) {
      logFile = SD.open(name, FILE_WRITE);
      if (!logFile) return false;
      logFile.println(F("Timestamp,Ax_g,Ay_g,Az_g"));
      logFile.flush();
      Serial.print(F("Logging to "));
      Serial.println(name);
      return true;
    }
  }
  return false;
}

// Writes one CSV line to any Print target (Serial or the SD file).
void writeLine(Print &out, const char *ts, float x, float y, float z) {
  out.print(ts);
  out.print(',');
  out.print(x, 3);
  out.print(',');
  out.print(y, 3);
  out.print(',');
  out.println(z, 3);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { ; }   // don't hang when running without a PC
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

  // --- MPU6050 ---
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) {
    Serial.println(F("ERROR: MPU6050 not found. Check wiring and AD0/address."));
    while (true) delay(1000);
  }
  mpuWrite(REG_ACCEL_CONFIG, 0x00);
  delay(100);

  // --- SD card ---
  if (!SD.begin(SD_CS)) {
    Serial.println(F("ERROR: SD card init failed. Check wiring, CS pin, and FAT32 format."));
    while (true) delay(1000);
  }
  if (!openLogFile()) {
    Serial.println(F("ERROR: could not create log file."));
    while (true) delay(1000);
  }

  calibrate();

  Serial.println(F("Timestamp,Ax_g,Ay_g,Az_g   (send 'c' to recalibrate)"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') calibrate();
  }

  // Sample continuously to keep the filter settled; log at SAMPLE_INTERVAL_MS
  float ax, ay, az;
  if (mpuReadAccel(ax, ay, az)) {
    ax -= offX; ay -= offY; az -= offZ;
    fx += FILTER_ALPHA * (ax - fx);
    fy += FILTER_ALPHA * (ay - fy);
    fz += FILTER_ALPHA * (az - fz);
  }
  delay(5);

  if (millis() - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = millis();

  DateTime now = rtc.now();
  char ts[20];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  float x = deadband(fx);
  float y = deadband(fy);
  float z = REMOVE_GRAVITY ? deadband(fz) : fz;

  writeLine(Serial, ts, x, y, z);

  if (logFile) {
    writeLine(logFile, ts, x, y, z);
    logFile.flush();   // commit to the card so nothing is lost on power-off
  }
}
