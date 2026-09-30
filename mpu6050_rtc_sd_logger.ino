// MPU6050 + DS3231 RTC + SD card data logger for Arduino Uno R4
//
// Libraries required (Library Manager):
//   - Adafruit MPU6050   (pulls in Adafruit Unified Sensor, Adafruit BusIO)
//   - RTClib             (by Adafruit)
//   - SD                 (built in to the Arduino IDE)
//
// Wiring:
//   MPU6050  VCC->3.3V/5V  GND->GND  SCL->SCL  SDA->SDA
//   DS3231   VCC->5V       GND->GND  SCL->SCL  SDA->SDA  (shares the I2C bus)
//   SD card  VCC->5V       GND->GND  MISO->12  MOSI->11  SCK->13  CS->10

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#define SD_CS_PIN 10
#define LOG_FILENAME "datalog.csv"

Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("Initializing I2C sensors...");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 found.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!rtc.begin()) {
    Serial.println("Failed to find DS3231. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("DS3231 found.");

  if (rtc.lostPower()) {
    // Sets the RTC to the date/time this sketch was compiled.
    // Comment this out after the first upload so it doesn't
    // keep resetting the clock every time you re-upload.
    Serial.println("RTC lost power, setting time to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card init failed. Check wiring/card!");
    while (1) delay(10);
  }
  Serial.println("SD card ready.");

  // Write a CSV header if the file doesn't exist yet
  if (!SD.exists(LOG_FILENAME)) {
    File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) {
      logFile.println("timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c");
      logFile.close();
    }
  }

  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  DateTime now = rtc.now();

  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());

  String row = String(timestamp) + "," +
               String(a.acceleration.x, 3) + "," +
               String(a.acceleration.y, 3) + "," +
               String(a.acceleration.z, 3) + "," +
               String(g.gyro.x, 3) + "," +
               String(g.gyro.y, 3) + "," +
               String(g.gyro.z, 3) + "," +
               String(temp.temperature, 2);

  // Write to SD
  File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  if (logFile) {
    logFile.println(row);
    logFile.close();
  } else {
    Serial.println("Error opening log file!");
  }

  // Mirror to Serial Monitor
  Serial.println(row);

  delay(500);
}
