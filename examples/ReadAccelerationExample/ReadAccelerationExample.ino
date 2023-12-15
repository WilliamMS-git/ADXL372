#include "ADXL372.h"

ADXL372class acc;
float x = 0;
float y = 0;
float z = 0;


void setup() {
  acc.begin(10); //initialized with pin 10 as cs pin
}

void loop() {
  acc.readAcceleration(x, y, z);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);
  delay(100);
}
