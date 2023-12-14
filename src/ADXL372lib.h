#include <Arduino.h>
#include <SPI.h>

class ADXL372class
{
public:
    ADXL372class(); 
    virtual ~ADXL372class();


    int begin();
    void end();
    void readAcceleration(float& x, float& y, float& z);
    int accelerationAvailable(uint8_t csPin); //, uint16_t* data, size_t length
    byte SPIreadByte(uint8_t csPin, uint8_t subAddress);

private:
    void SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data);
    
    void readAccelerometerRegister(uint8_t csPin, uint16_t* data, size_t length);

};