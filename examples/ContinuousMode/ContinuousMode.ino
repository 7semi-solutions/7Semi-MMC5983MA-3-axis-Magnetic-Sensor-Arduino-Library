/**
 * 7Semi MMC5983MA Example - Continuous Mode + Bandwidth
 *
 * - Library : 7Semi MMC5983MA Arduino Library
 * - Bus     : I2C
 * - Shows   : How to enable continuous mode and set bandwidth
 *
 * VCC -> 3.3V
 * GND -> GND
 * SDA -> SDA 
 * SCL -> SCL
 *
 * Notes
 * - Continuous mode updates at the configured rate.
 * - Bandwidth affects noise vs response.
 */

#include <7Semi_MMC5983MA.h>

MMC5983MA_7Semi mag;

void setup()
{
  Serial.begin(115200);
  delay(100);

  Wire.begin();

  if (!mag.beginI2C(Wire))
  {
    Serial.println("MMC5983MA not found");
    while (1) delay(10);
  }

  mag.enableAutoSetReset(true, MMC5983MA_7Semi::MES_100);

  /** - Set bandwidth (your enum is 0..3; choose one) */
  mag.setBandwidth(MMC5983MA_7Semi::BW_200KHZ);

  /** - Enable continuous mode and set frequency */
  if (!mag.enableContinuousMode(true, MMC5983MA_7Semi::CM_50HZ))
  {
    Serial.println("Failed to enable continuous mode");
  }

  Serial.println("Continuous + Bandwidth configured");
}

void loop()
{
  float x, y, z;
  if (mag.readMagnetometer(x, y, z))
  {
    Serial.print("mG: ");
    Serial.print(x, 2); Serial.print(", ");
    Serial.print(y, 2); Serial.print(", ");
    Serial.println(z, 2);
  }

  delay(20);
}
