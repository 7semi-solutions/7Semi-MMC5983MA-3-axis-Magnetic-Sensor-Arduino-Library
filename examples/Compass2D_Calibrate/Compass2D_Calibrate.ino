/**
 * 7Semi MMC5983MA Example - 2D Heading + Tilt Heading Demo
 *
 * - Library : 7Semi MMC5983MA Arduino Library
 * - Prints  : 2D heading and tilt-compensated heading
 *
 * VCC -> 3.3V
 * GND -> GND
 * SDA -> SDA
 * SCL -> SCL
 *
 * Notes
 * - Tilt heading needs roll and pitch from an IMU (accelerometer/gyro fusion).
 * - This example uses placeholder roll/pitch values so you can see the API usage.
 */

#include <7Semi_MMC5983MA.h>

MMC5983MA_7Semi mag;

/** - Set your local declination (degrees) */
static const float DECLINATION_DEG = 0.0f;

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

  Serial.println("Heading demo started");
}

void loop()
{
  /** - 2D heading (device should be roughly level) */
  float h2d = mag.getHeading2D(DECLINATION_DEG);

  /** - Replace these with IMU fusion outputs (degrees) */
  float roll_deg  = 0.0f;
  float pitch_deg = 0.0f;

  /** - Tilt-compensated heading */
  float htilt = mag.getHeadingTilt(roll_deg, pitch_deg, DECLINATION_DEG);

  Serial.print("H2D: ");
  if (!isnan(h2d)) Serial.print(h2d, 1); else Serial.print("NAN");

  Serial.print("  HTilt: ");
  if (!isnan(htilt)) Serial.println(htilt, 1); else Serial.println("NAN");

  delay(100);
}
