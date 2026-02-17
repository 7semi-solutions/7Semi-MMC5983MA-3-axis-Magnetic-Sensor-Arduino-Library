/**
 * 7Semi MMC5983MA Example (I2C) - Temperature + Magnetometer
 *
 * - Library : 7Semi MMC5983MA Arduino Library
 * - Bus     : I2C
 * - Prints  : Magnetometer (mG) and Temperature (°C)
 *
 * VCC            -> 3.3V
 * GND            -> GND
 * SDA            -> SDA
 * SCL            -> SCL
 *
 * UNO/Nano:
 * - SDA = A4
 * - SCL = A5
 *
 * ESP32 (typical):
 * - SDA = GPIO21
 * - SCL = GPIO22
 */
#include <7Semi_MMC5983MA.h>

MMC5983MA_7Semi mag;

void setup() {
  Serial.begin(115200);
  delay(100);

  /** - Start sensor using I2C (default address 0x30) */
  if (!mag.beginI2C(Wire, 0x30)) {
    // if (!mag.beginI2C(Wire, 0x30, 21, 22)) {
    Serial.println("MMC5983MA not found on I2C");
    while (1) delay(10);
  }

  /** - Recommended: Auto Set/Reset improves stability */
  mag.enableAutoSetReset(true, MMC5983MA_7Semi::MES_100);

  Serial.println("MMC5983MA OK");
}

void loop() {
  float x, y, z;

  /** - Read magnetometer in mG */
  if (mag.readMagnetometer(x, y, z)) {
    Serial.print("mG: ");
    Serial.print(x, 2);
    Serial.print(", ");
    Serial.print(y, 2);
    Serial.print(", ");
    Serial.println(z, 2);
  } else {
    Serial.println("Mag read failed");
  }

  float tC;

  /** - Read temperature in Celsius */
  if (mag.readTemperature(tC)) {
    Serial.print("Temp (C): ");
    Serial.println(tC, 2);
  } else {
    Serial.println("Temp read failed");
  }

  delay(250);
}
