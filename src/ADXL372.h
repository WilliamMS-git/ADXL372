#include "Arduino.h"
#include "SPI.h"

class ADXL372class
{
public:
    ADXL372class(int csPinInput); 
    virtual ~ADXL372class();

    int begin();
    void end();
    void readAcceleration(float& x, float& y, float& z);

private:
    int m_csPin;

    uint8_t readRegister(byte regAddress);
    void writeRegister(byte regAddress, uint8_t value);


};