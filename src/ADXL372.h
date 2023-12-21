#include "Arduino.h"
#include "SPI.h"

class ADXL372class
{
public:
    ADXL372class(int csPinInput); 
    virtual ~ADXL372class();

    void begin();
    void end();
    void readAcceleration(float& x, float& y, float& z);
    void printDevice();

private:
    int m_csPin;

    uint8_t readRegister(byte regAddress);
    void writeRegister(byte regAddress, uint8_t value);


};