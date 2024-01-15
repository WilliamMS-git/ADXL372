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

  ADXL372.enableActivityDetection(true, true, true); // Enable activity detection on all axis
  ADXL372.setActivityThresholds(0b11, 0b11, 0b11);   // 300 mg activity threshold on all axis
  ADXL372.setActivityTimer(0b1111);                  // 99ms activity timer

  ADXL372.enableInactivityDetection(true, true, true); // Enable inactivity detection on all axis
  ADXL372.setInactivityThresholds(0b11, 0b11, 0b11);   // 300 mg inactivity threshold on all axis
  ADXL372.setInactivityTimer(0b1111);                  // 390 ms inactivity timer

  ADXL372.setLinkLoopActivityProcessing(DEFAULT); // Select how activity and inactivity processing are linked here. Default/Linked/Looped.

  ADXL372.enableAutosleep(true);    // Autosleep for getting the accelerometer back into wake-up mode.
  ADXL372.setWakeUpRate(WUR_104ms); // Wake-up rate will also determine the current draw during wake-up mode. Check the datasheet for more info

  ADXL372.setOperatingMode(WAKE_UP);
}

void loop()
{
  // Will now only read the acceleration when activity is detected.
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