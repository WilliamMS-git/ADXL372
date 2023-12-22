#include "ADXL372.h"

ADXL372class acc(10); //Pin 10 as CS pin of the accelerometer. Use the corresponding pin on your own board

float x = 0;
float y = 0;
float z = 0;


void setup() {
  Serial.begin(9600);
  acc.begin(); 
  acc.printDevice();
}

void loop() {
  acc.readAcceleration(x, y, z);

  Serial.println("Acceleration:");
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  delay(100); //Change this for faster/slower measurements
}