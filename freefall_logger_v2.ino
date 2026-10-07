/*
  Free-Fall Acceleration Logger  -  Arduino UNO R4 WiFi
  MPU6050 accelerometer + DS3231 RTC + SD card + IR receiver + micro-servo

  Press a button on the IR remote -> the micro-servo releases the object
  and the MPU6050 logs acceleration to the SD card as it falls. Each log
  file is stamped with the date/time from the DS3231.

  Hardware (default pins)
    I2C bus (shared)  : SDA -> SDA, SCL -> SCL, 3.3V, GND
      MPU6050 (GY-521): AD0 -> 3.3V  => address 0x69
      DS3231 module   : address 0x68 (fixed)
      !! Both chips default to 0x68. AD0 on the MPU6050 MUST be tied
         high so the two devices don't collide.
    SD card adapter   : CS -> D10, MOSI -> D11, MISO -> D12, SCK -> D13,
                        VCC -> 5V (module with regulator), GND
    IR receiver       : OUT -> D2, VCC -> 5V, GND
    Micro-servo       : Signal -> D9, VCC -> 5V (separate supply
                        recommended, shared GND), GND

  Libraries (Library Manager)
    IRremote (v4.4.0 or newer, for R4 support)
    Servo, SD, SPI, Wire come with the Arduino IDE / R4 core
    (MPU6050 and DS3231 are accessed directly - no extra libraries)

  Operation
    ARMED    : servo holds the object. Any IR button -> run a drop.
    DROP     : timestamps the run, records a short baseline, releases
               the servo, keeps recording, then saves FALLxxx.CSV.
    RELEASED : servo stays open so you can reload. Press any IR button
               again to close the servo and re-arm.

  Samples are buffered in RAM during the drop and written to the SD card
  afterwards, so slow SD writes can never create gaps in the data.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <IRremote.hpp>

// ---------------- User settings ----------------
const uint8_t  IR_PIN          = 2;
const uint8_t  SERVO_PIN       = 9;
const uint8_t  SD_CS_PIN       = 10;

const int      HOLD_ANGLE      = 0;     // servo angle that holds the object
const int      RELEASE_ANGLE   = 90;    // servo angle that lets it go

const int16_t  TRIGGER_COMMAND = -1;    // -1 = any IR button; otherwise the
                                        // command code printed on Serial
const uint16_t PRE_TRIGGER_MS  = 150;   // baseline recorded before release
const uint16_t SAMPLE_RATE_HZ  = 1000;  // MPU6050 runs at 1 kHz (see begin)
const uint16_t MAX_SAMPLES     = 1800;  // ~1.8 s at 1 kHz (RAM limited)

// Set true to force the RTC to this sketch's compile time on every boot.
// Normally leave false: the RTC is only set if it has lost power.
const bool     FORCE_SET_RTC   = false;

// ---------------- MPU6050 ----------------
const uint8_t  MPU_ADDR        = 0x69;  // AD0 high (0x68 is taken by DS3231)
const uint8_t  MPU_SMPLRT_DIV  = 0x19;
const uint8_t  MPU_CONFIG      = 0x1A;
const uint8_t  MPU_ACCEL_CFG   = 0x1C;
const uint8_t  MPU_ACCEL_XOUT  = 0x3B;
const uint8_t  MPU_PWR_MGMT_1  = 0x6B;
const uint8_t  MPU_WHO_AM_I    = 0x75;
const float    G_PER_LSB       = 1.0f / 2048.0f;   // +/-16 g range

// ---------------- DS3231 ----------------
const uint8_t  RTC_ADDR        = 0x68;
const uint8_t  RTC_STATUS      = 0x0F;

struct DateTime {
  uint16_t year;
  uint8_t  month, day, hour, minute, second;
};

// ---------------- Globals ----------------
// 6 bytes per sample keeps the buffer small enough for the R4's 32 KB RAM.
// Time is not stored: samples are taken on a fixed schedule, so
// time = index * (1 / SAMPLE_RATE_HZ).
struct Sample {
  int16_t x, y, z;    // raw counts
};

Sample  samples[MAX_SAMPLES];
Servo   releaseServo;

enum State { ARMED, RELEASED };
State state = ARMED;

const uint32_t PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;

// ---------------- Generic I2C helpers ----------------
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ---------------- MPU6050 ----------------
bool mpuBegin() {
  if (!i2cPresent(MPU_ADDR)) return false;

  uint8_t who = i2cRead8(MPU_ADDR, MPU_WHO_AM_I);
  Serial.print(F("MPU WHO_AM_I = 0x")); Serial.println(who, HEX);
  // Genuine parts report 0x68; some clones report 0x70/0x72 - still fine.

  i2cWrite(MPU_ADDR, MPU_PWR_MGMT_1, 0x01);  // wake, PLL clock source
  delay(100);
  i2cWrite(MPU_ADDR, MPU_CONFIG, 0x01);      // DLPF 184 Hz -> 1 kHz output rate
  i2cWrite(MPU_ADDR, MPU_SMPLRT_DIV, 0x00);  // sample rate = 1 kHz
  i2cWrite(MPU_ADDR, MPU_ACCEL_CFG, 0x18);   // +/-16 g
  return true;
}

bool mpuReadXYZ(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  x = (int16_t)((b[0] << 8) | b[1]);         // MPU6050 is big-endian
  y = (int16_t)((b[2] << 8) | b[3]);
  z = (int16_t)((b[4] << 8) | b[5]);
  return true;
}

// ---------------- DS3231 ----------------
uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool rtcRead(DateTime &dt) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(RTC_ADDR, (uint8_t)7) != 7) return false;
  uint8_t s = Wire.read(), m = Wire.read(), h = Wire.read();
  Wire.read();                                // day of week (unused)
  uint8_t d = Wire.read(), mo = Wire.read(), y = Wire.read();
  dt.second = bcd2dec(s & 0x7F);
  dt.minute = bcd2dec(m);
  dt.hour   = bcd2dec(h & 0x3F);              // 24-hour mode
  dt.day    = bcd2dec(d);
  dt.month  = bcd2dec(mo & 0x1F);
  dt.year   = 2000 + bcd2dec(y);
  return true;
}

void rtcWrite(const DateTime &dt) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write(dec2bcd(dt.second));
  Wire.write(dec2bcd(dt.minute));
  Wire.write(dec2bcd(dt.hour));               // 24-hour mode
  Wire.write((uint8_t)1);                     // day of week (unused)
  Wire.write(dec2bcd(dt.day));
  Wire.write(dec2bcd(dt.month));
  Wire.write(dec2bcd(dt.year - 2000));
  Wire.endTransmission();
  // Clear the oscillator-stop flag (bit 7) now that the time is valid
  i2cWrite(RTC_ADDR, RTC_STATUS, i2cRead8(RTC_ADDR, RTC_STATUS) & 0x7F);
}

// Parse __DATE__ ("Oct  7 2026") and __TIME__ ("14:32:05")
DateTime compileTime() {
  const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
  DateTime dt;
  dt.month  = (strstr(months, mon) - months) / 3 + 1;
  dt.day    = atoi(__DATE__ + 4);
  dt.year   = atoi(__DATE__ + 7);
  dt.hour   = atoi(__TIME__);
  dt.minute = atoi(__TIME__ + 3);
  dt.second = atoi(__TIME__ + 6);
  return dt;
}

bool rtcBegin() {
  if (!i2cPresent(RTC_ADDR)) return false;
  bool lostPower = i2cRead8(RTC_ADDR, RTC_STATUS) & 0x80;
  if (lostPower || FORCE_SET_RTC) {
    Serial.println(F("RTC time not valid - setting to compile time."));
    rtcWrite(compileTime());
  }
  return true;
}

void formatTime(const DateTime &dt, char *out, size_t len) {
  snprintf(out, len, "%04u-%02u-%02u %02u:%02u:%02u",
           dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
}

// ---------------- Utility ----------------
void fatal(const char *msg) {
  Serial.println(msg);
  while (true) {                     // rapid blink = error
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
  }
}

void nextFileName(char *name, size_t len) {
  for (int i = 1; i < 1000; i++) {
    snprintf(name, len, "FALL%03d.CSV", i);
    if (!SD.exists(name)) return;
  }
  snprintf(name, len, "FALL999.CSV");
}

// ---------------- Record a drop ----------------
uint16_t readErrors = 0;

size_t recordDrop(size_t &releaseIdx) {
  size_t   n = 0;
  bool     released = false;
  int16_t  lx = 0, ly = 0, lz = 0;     // last good reading
  releaseIdx = 0;
  readErrors = 0;

  uint32_t start = micros();
  uint32_t next  = start;

  while (n < MAX_SAMPLES) {
    uint32_t now = micros();
    if ((int32_t)(now - next) < 0) continue;   // wait for next sample slot
    next += PERIOD_US;

    if (!released && (now - start) >= (uint32_t)PRE_TRIGGER_MS * 1000UL) {
      releaseServo.write(RELEASE_ANGLE);
      released   = true;
      releaseIdx = n;
    }

    int16_t x, y, z;
    if (mpuReadXYZ(x, y, z)) { lx = x; ly = y; lz = z; }
    else                     { readErrors++; }  // repeat last good value

    samples[n].x = lx;
    samples[n].y = ly;
    samples[n].z = lz;
    n++;
  }
  return n;
}

// ---------------- Save to SD ----------------
bool saveDrop(size_t n, size_t releaseIdx, const char *timestamp) {
  char name[16];
  nextFileName(name, sizeof(name));

  File f = SD.open(name, FILE_WRITE);
  if (!f) return false;

  f.print(F("# Free-fall log, "));      f.println(timestamp);
  f.print(F("# rate="));               f.print(SAMPLE_RATE_HZ);
  f.print(F(" Hz, release at sample ")); f.print(releaseIdx);
  f.print(F(", g = raw * "));          f.println(G_PER_LSB, 6);
  f.println(F("sample,time_ms,released,ax_g,ay_g,az_g,mag_g"));

  float minMag = 1e9f, maxMag = 0;
  for (size_t i = 0; i < n; i++) {
    float ax = samples[i].x * G_PER_LSB;
    float ay = samples[i].y * G_PER_LSB;
    float az = samples[i].z * G_PER_LSB;
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (i > releaseIdx) {                 // ignore baseline for summary
      if (mag < minMag) minMag = mag;
      if (mag > maxMag) maxMag = mag;
    }

    f.print(i);                             f.print(',');
    f.print(i * 1000.0f / SAMPLE_RATE_HZ, 3); f.print(',');
    f.print(i >= releaseIdx ? 1 : 0);       f.print(',');
    f.print(ax, 4);                         f.print(',');
    f.print(ay, 4);                         f.print(',');
    f.print(az, 4);                         f.print(',');
    f.println(mag, 4);
  }
  f.close();

  Serial.print(F("Saved "));  Serial.print(n);
  Serial.print(F(" samples to ")); Serial.println(name);
  Serial.print(F("Min |a| after release: ")); Serial.print(minMag, 3);
  Serial.print(F(" g   Peak |a|: "));         Serial.print(maxMag, 3);
  Serial.println(F(" g"));
  if (readErrors) { Serial.print(F("Warning: failed sensor reads: ")); Serial.println(readErrors); }
  return true;
}

// ---------------- IR ----------------
bool irTriggered() {
  if (!IrReceiver.decode()) return false;

  bool valid = (IrReceiver.decodedIRData.protocol != UNKNOWN) &&
               !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT);
  uint16_t cmd = IrReceiver.decodedIRData.command;
  IrReceiver.resume();

  if (!valid) return false;
  Serial.print(F("IR command received: 0x"));
  Serial.println(cmd, HEX);

  return (TRIGGER_COMMAND < 0) || (cmd == (uint16_t)TRIGGER_COMMAND);
}

// ---------------- Setup / loop ----------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // don't hang if no USB

  Wire.begin();
  Wire.setClock(400000);

  if (!mpuBegin()) fatal("ERROR: MPU6050 not found at 0x69. Is AD0 tied to 3.3V?");
  if (!rtcBegin()) fatal("ERROR: DS3231 not found at 0x68.");
  if (!SD.begin(SD_CS_PIN)) fatal("ERROR: SD card init failed.");

  releaseServo.attach(SERVO_PIN);
  releaseServo.write(HOLD_ANGLE);

  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);

  DateTime now;
  char ts[24];
  if (rtcRead(now)) { formatTime(now, ts, sizeof(ts)); Serial.print(F("RTC: ")); Serial.println(ts); }

  state = ARMED;
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println(F("Armed. Load the object, then press a button on the IR remote."));
}

void loop() {
  if (!irTriggered()) return;

  if (state == ARMED) {
    // Grab the timestamp BEFORE recording so RTC reads don't disturb sampling
    DateTime now;
    char ts[24] = "time unavailable";
    if (rtcRead(now)) formatTime(now, ts, sizeof(ts));

    Serial.print(F("Recording drop at ")); Serial.println(ts);
    digitalWrite(LED_BUILTIN, LOW);

    size_t releaseIdx;
    size_t n = recordDrop(releaseIdx);

    if (!saveDrop(n, releaseIdx, ts)) Serial.println(F("ERROR: could not write file."));

    state = RELEASED;
    Serial.println(F("Reload the object, then press IR button to re-arm."));
  } else {
    releaseServo.write(HOLD_ANGLE);
    state = ARMED;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println(F("Armed. Press IR button to drop."));
  }
}
