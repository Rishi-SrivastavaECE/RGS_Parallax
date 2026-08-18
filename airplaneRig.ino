/*
  Aircraft Telemetry Node (A101) - Final Code
  
  Pin Connections:
    - OLED Display & MPU6050 (I2C):
        SDA -> Arduino A4
        SCL -> Arduino A5
        VCC -> 5V
        GND -> GND
    - HC-05 Bluetooth:
        TX  -> Arduino D10 (SoftwareSerial RX)
        RX  -> Arduino D11 (SoftwareSerial TX via voltage divider: 1k / 2k resistors)
        VCC -> 5V
        GND -> GND
*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// OLED & MPU6050 Instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MPU6050 mpu;

// Bluetooth HC-05 Setup
#define BT_RX 10
#define BT_TX 11
#define BT_BAUD 9600
SoftwareSerial bluetooth(BT_RX, BT_TX);

const char* AIRCRAFT_ID = "A101";

void setup() {
  Serial.begin(115200);
  bluetooth.begin(BT_BAUD);
  Wire.begin();

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[ERROR] OLED Init Failed!"));
    while (1) { delay(10); }
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 15);
  display.println(F("AIRCRAFT A101"));
  display.println(F("SYSTEM CHECK..."));
  display.display();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(F("[ERROR] MPU6050 Init Failed!"));
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println(F("MPU6050 FAIL!"));
    display.display();
    while (1) { delay(10); }
  }

  // MPU Configuration
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(1000);

  // System Ready Display
  display.clearDisplay();
  display.setCursor(0, 15);
  display.println(F("AIRCRAFT A101"));
  display.println(F("MPU6050 : OK"));
  display.println(F("HC-05   : READY"));
  display.display();

  Serial.println(F("=============================="));
  Serial.println(F("AIRCRAFT A101 READY"));
  Serial.println(F("=============================="));
  delay(1500);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate Pitch and Roll (Degrees)
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958;
  float roll  = atan2(a.acceleration.y, a.acceleration.z) * 57.2958;

  // 1. Update On-board OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("=== AIRCRAFT A101 ==="));
  display.print(F("PITCH : "));
  display.print(pitch, 1);
  display.println(F(" deg"));
  display.print(F("ROLL  : "));
  display.print(roll, 1);
  display.println(F(" deg"));
  display.println(F("--------------------"));

  if (abs(pitch) < 5 && abs(roll) < 5) {
    display.println(F("ATTITUDE: STABLE"));
  } else {
    display.println(F("ATTITUDE: MANEUVER"));
  }
  display.display();

  // 2. Send Bluetooth Telemetry to Raspberry Pi Dashboard (Format: A101,ROLL,PITCH)
  bluetooth.print(AIRCRAFT_ID);
  bluetooth.print(",");
  bluetooth.print(roll, 2);
  bluetooth.print(",");
  bluetooth.println(pitch, 2);

  // 3. Serial Monitor Debugging
  Serial.print(F("ROLL: "));
  Serial.print(roll, 2);
  Serial.print(F(" | PITCH: "));
  Serial.println(pitch, 2);

  delay(100); // 10 Hz Telemetry Rate
}