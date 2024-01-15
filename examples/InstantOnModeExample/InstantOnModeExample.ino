#include "ADXL372.h"

ADXL372class ADXL372(10); // Pin 10 as CS pin of the accelerometer. Use the corresponding pin on your own board

float x = 0;
float y = 0;
float z = 0;

void setup()
{
  Serial.begin(9600);
  ADXL372.begin();
  ADXL372.setOperatingMode(STANDBY); // Set accelerometer in standby to update ceratin registers

  ADXL372.setOdr(ODR_3200Hz);
  ADXL372.setBandwidth(BW_1600Hz);

  ADXL372.setInstantOnThreshold(IOT_LOW_THRESH); // 10g ±5g threshold. This can be increased to 30g ±10g by using IOT_HIGH_THRESH instead.
  ADXL372.setOperatingMode(INSTANT_ON);
}

void loop()
{
  // Will now wait for an event that exceeds the threshold, and go into measurement mode.
  ADXL372.readAcceleration(x, y, z);

  Serial.println("Acceleration:");
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  delay(10); // Change this for faster/slower measurements
}