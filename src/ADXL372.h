#include "Arduino.h"
#include "SPI.h"

enum Bandwidth {
    BW_200Hz = 0b000,
    BW_400Hz = 0b001,
    BW_800Hz = 0b010,
    BW_1600Hz = 0b011,
    BW_3200Hz = 0b100
};

enum Odr {
    ODR_400Hz = 0b000,
    ODR_800Hz = 0b001,
    ODR_1600Hz = 0b010,
    ODR_3200Hz = 0b011,
    ODR_6400Hz = 0b100
};

enum OperatingMode {
    STANDBY = 0b00,
    WAKE_UP_MODE = 0b01,
    INSTANT_ON_MODE = 0b10,
    FULL_BANDWIDTH_MODE = 0b11
};

class ADXL372class
{
public:
    ADXL372class(int csPinInput);
    virtual ~ADXL372class();

    void begin();
    void begin(Bandwidth bandwidth, Odr odr);
    void end();
    void printDevice();
    void readAcceleration(float& x, float& y, float& z, bool statusCheck);
    void setBandwidth(Bandwidth bandwidth);
    void setOdr(Odr odr);
    void setOperatingMode(OperatingMode opMode);

private:
    int m_csPin;

    uint8_t readRegister(byte regAddress);
    void writeRegister(byte regAddress, uint8_t value);


};