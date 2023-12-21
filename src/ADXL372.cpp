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
#define SCALE_FACTOR 100 // mg/LSB

//Pins
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCLK_PIN 13


ADXL372class::ADXL372class(int csPinInput)
{
    m_csPin = csPinInput;
}

ADXL372class::~ADXL372class()
{
}

void ADXL372class::begin()
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
    pinMode(MOSI_PIN, OUTPUT); 
    pinMode(MISO_PIN, INPUT); 
    pinMode(SCLK_PIN, OUTPUT); 
    pinMode(m_csPin, OUTPUT); 
    digitalWrite(m_csPin, HIGH); //SPI MODE is 0 so the CS pin goes from high to low when recieving data

    //Set some adresses here
    writeRegister(0x3F, 0b00000011); // Set accelerometer to FULL BW Measurement mode

}

void ADXL372class::end()
{
    //set some adresses here

    SPI.end();
}

void ADXL372class::printDevice(){
    byte devidAd = readRegister(0x00);
    byte devidMst = readRegister(0x01);
    byte partId = readRegister(0x02);
    byte revId = readRegister(0x03);
    byte status = readRegister(0x04);

    Serial.print("DEVID_AD: 0x");
    Serial.println(devidAd, HEX);

    Serial.print("DEVID_MST: 0x");
    Serial.println(devidMst, HEX);

    Serial.print("PARTID: 0x");
    Serial.println(partId, HEX);

    Serial.print("REVID: 0x");
    Serial.println(revId, HEX);
    
    Serial.print("STATUS: 0x");
    Serial.println(status, HEX);
}

uint8_t ADXL372class::readRegister(byte regAddress){
    digitalWrite(m_csPin, LOW);
    regAddress = regAddress << 1 | 1;
    SPI.transfer(regAddress);
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
  byte status;
  do {
        status = readRegister(0x04);
    } while(status == 1);

    short rawX = readRegister(XDATA_H) << 8 | readRegister(XDATA_L);
    short rawY = readRegister(YDATA_H) << 8 | readRegister(YDATA_L);
    short rawZ = readRegister(ZDATA_H) << 8 | readRegister(ZDATA_L);

    rawX = rawX >> 4;
    rawY = rawY >> 4;
    rawZ = rawZ >> 4;

    x = (rawX * 100) / 1000;
    y = (rawY * 100) / 1000;
    z = (rawZ * 100) / 1000;

}