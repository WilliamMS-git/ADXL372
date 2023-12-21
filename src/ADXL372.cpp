#include "ADXL372.h"

//Data registers. Each axis data has a 12 bit value. Data is left justified, so MSBFIRST.
//register_H contains the eight most significant bits (MSBs), and register_L contains the four least significant bits (LSBs) of the 12-bit value
#define XDATA_H 0x08
#define XDATA_L 0x09
#define YDATA_H 0x0A
#define YDATA_L 0x0B
#define ZDATA_H 0x0C
#define ZDATA_L 0x0D

//ID registers
#define DEVID_PRODUCT 0xFA //372 in octal :) 
#define STATUS_REGISTER 0x04 //Page 33 in datasheet 0x04

//Accelerometer Constants
#define SPI_SPEED 10000000 //ADXL372 supports up to 10MHz in SCLK frequency
#define SCALE_FACTOR 10 //LSB/g


ADXL372class::ADXL372class(int csPinInput)
{
    m_csPin = csPinInput;
}

ADXL372class::~ADXL372class()
{
}

int ADXL372class::begin()
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
    pinMode(m_csPin, OUTPUT); 
    digitalWrite(m_csPin, HIGH); //SPI MODE is 0 so the CS pin goes from high to low when recieving data
    //Set some adresses here
    writeRegister(0x3F, 0b00000011); // Set accelerometer to FULL BW Measurement mode
    return 1;
}

void ADXL372class::end()
{
    //set some adresses here

    SPI.end();
}

uint8_t ADXL372class::readRegister(byte regAddress){
    digitalWrite(m_csPin, LOW);
    SPI.transfer(regAddress | 0x80);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(m_csPin, HIGH);
    return value;
}

void ADXL372class::writeRegister(byte regAddress, uint8_t value) {
    digitalWrite(m_csPin, LOW);
    SPI.transfer(regAddress);
    SPI.transfer(value);
    digitalWrite(m_csPin, HIGH);
}

void ADXL372class::readAcceleration(float& x, float& y, float& z) {
    float rawX = readRegister(XDATA_H) << 8 | readRegister(XDATA_L);
    float rawY = readRegister(YDATA_H) << 8 | readRegister(YDATA_L);
    float rawZ = readRegister(ZDATA_H) << 8 | readRegister(ZDATA_L);

    x = rawX / SCALE_FACTOR;
    y = rawY / SCALE_FACTOR;
    z = rawZ / SCALE_FACTOR;

}