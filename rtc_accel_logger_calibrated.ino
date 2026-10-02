/*
  DS3231 RTC + MPU6050 accelerometer -> timestamped serial output (with calibration)

  Libraries: RTClib by Adafruit (install via Library Manager)
  The MPU6050 is read directly over Wire, so no extra library is needed.

  Wiring (Uno/Nano: SDA = A4, SCL = A5 | Mega: SDA = 20, SCL = 21 | ESP32: SDA = 21, SCL = 22):
    DS3231  VCC -> 3.3V or 5V, GND -> GND, SDA -> SDA, SCL -> SCL
    MPU6050 VCC -> 3.3V (or 5V on a board with regulator), GND -> GND, SDA -> SDA, SCL -> SCL

  !! I2C ADDRESS CONFLICT !!
  Both the DS3231 and the MPU6050 default to address 0x68.
  Connect the MPU6050's AD0 pin to 3.3V so it moves to 0x69.

  CALIBRATION
  At startup the board must be sitting still. The sketch averages CAL_SAMPLES
  readings and subtracts them. Send 'c' in the serial monitor to recalibrate.

    REMOVE_GRAVITY true  -> stationary reads ~0,0,0 in any orientation
                            (only valid while the board isn't rotated afterward)
    REMOVE_GRAVITY false -> removes sensor bias only; stationary reads ~0,0,1 g
                            (assumes the board is lying flat during calibration)
*/

#include <Wire.h>
#include <RTClib.h>

#define MPU_ADDR 0x69
#define SAMPLE_INTERVAL_MS 500

#define REMOVE_GRAVITY true
#define CAL_SAMPLES    500     // readings averaged during calibration
#define FILTER_ALPHA   0.3     // 0..1, lower = smoother (1 = no smoothing)
#define DEADBAND_G     0.02    // readings smaller than this print as 0 (only when REMOVE_GRAVITY)

// MPU6050 registers
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

#define ACCEL_SCALE 16384.0    // +/-2g range -> 16384 LSB per g

RTC_DS3231 rtc;

unsigned long lastSample = 0;
float offX = 0, offY = 0, offZ = 0;   // calibration offsets (g)
float fx = 0, fy = 0, fz = 0;         // filtered values (g)

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
  if (!REMOVE_GRAVITY) offZ -= 1.0;   // leave 1 g on Z

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

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
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

  calibrate();

  Serial.println(F("Timestamp,Ax_g,Ay_g,Az_g   (send 'c' to recalibrate)"));
}

void loop() {
  // Recalibrate on demand
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') calibrate();
  }

  // Sample fast to keep the filter settled, print at SAMPLE_INTERVAL_MS
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

  Serial.print(ts);
  Serial.print(',');
  Serial.print(deadband(fx), 3);
  Serial.print(',');
  Serial.print(deadband(fy), 3);
  Serial.print(',');
  Serial.println(REMOVE_GRAVITY ? deadband(fz) : fz, 3);
}
