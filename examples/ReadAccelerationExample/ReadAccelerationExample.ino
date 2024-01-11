#include "ADXL372.h"

ADXL372class ADXL372(10); //Pin 10 as CS pin of the accelerometer. Use the corresponding pin on your own board

float x = 0.0;
float y = 0.0;
float z = 0.0;

void setup() {
  Serial.begin(9600);
  ADXL372.begin(); 
  ADXL372.printDevice();
}

void loop() {
  ADXL372.readAcceleration(x, y, z);

  Serial.println("Acceleration:");
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  delay(100); //Change this for faster/slower measurements
}