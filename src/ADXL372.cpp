#include "ADXL372.h"

#define DEVID_PRODUCT 0xFA //372 in octal :) 

// Data registers. Each axis data has a 12 bit value. Data is left justified, MSBFIRST.
// Register *_H contains the eight most significant bits (MSBs), and Register *_L contains the four least significant bits (LSBs) of the 12-bit value
#define XDATA_H 0x08
#define XDATA_L 0x09
#define YDATA_H 0x0A
#define YDATA_L 0x0B
#define ZDATA_H 0x0C
#define ZDATA_L 0x0D

// ID registers
#define DEVID_AD 0x00
#define DEVID_MST 0x01
#define PARTID 0x02
#define REVID 0x03
#define STATUS 0x04 // Page 33 in datasheet 0x04

// System registers
#define TIMING 0x3D // Timing control register
#define MEASURE 0x3E // Measurement control register
#define POWER_CTL 0x3F // Power control register

// Accelerometer Constants
#define SPI_SPEED 10000000 // ADXL372 supports up to 10MHz in SCLK frequency
#define SCALE_FACTOR 100 // mg per LSB
#define MG_TO_G 0.001 // g per mg

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
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0)); // CPHA = CPOL = 0
    pinMode(m_csPin, OUTPUT); // Setting chip select pin
    digitalWrite(m_csPin, HIGH); // Pin ready

}

void ADXL372class::begin(Bandwidth bandwidth, Odr odr)
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0)); // CPHA = CPOL = 0
    pinMode(m_csPin, OUTPUT); // Setting chip select pin
    digitalWrite(m_csPin, HIGH); // Pin ready

    setBandwidth(bandwidth);
    setOdr(odr);

}

void ADXL372class::end()
{
    //set some adresses here

    SPI.end();
}

void ADXL372class::printDevice(){
    byte devidAd = readRegister(DEVID_AD);
    byte devidMst = readRegister(DEVID_MST);
    byte partId = readRegister(PARTID);
    byte revId = readRegister(REVID);
    byte status = readRegister(STATUS);

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

void ADXL372class::readAcceleration(float& x, float& y, float& z, bool statusCheck) {
    
    if (statusCheck == true)
    {
        byte status;
        do {
            status = readRegister(0x04);
        } while((status & 0x01) == 0); // Waiting for status register
    }

    // The register is left justified. *DATA_H has bits 11:4 of the register. *DATA_L has bits 3:0.
    short rawX = readRegister(XDATA_H) << 8 | readRegister(XDATA_L); 
    short rawY = readRegister(YDATA_H) << 8 | readRegister(YDATA_L);
    short rawZ = readRegister(ZDATA_H) << 8 | readRegister(ZDATA_L);

    rawX = rawX >> 4; 
    rawY = rawY >> 4;
    rawZ = rawZ >> 4;

    // Converting raw axis data to acceleration in g unit
    x = rawX * SCALE_FACTOR * MG_TO_G;
    y = rawY * SCALE_FACTOR * MG_TO_G;
    z = rawZ * SCALE_FACTOR * MG_TO_G;
}

void ADXL372class::setBandwidth(Bandwidth bandwidth){
    writeRegister(MEASURE, bandwidth);
}

void ADXL372class::setOdr(Odr odr){
    writeRegister(TIMING, odr);
}

void ADXL372class::setOperatingMode(OperatingMode opMode){
    writeRegister(POWER_CTL, opMode);
}

uint8_t ADXL372class::readRegister(byte regAddress){
    digitalWrite(m_csPin, LOW);
    regAddress = regAddress << 1 | 1; // Reading from a register
    SPI.transfer(regAddress);
    uint8_t value = SPI.transfer(0x00); // Transfering dummy byte to recieve data from accelerometer
    digitalWrite(m_csPin, HIGH);
    return value;
}

void ADXL372class::writeRegister(byte regAddress, uint8_t value) {
    digitalWrite(m_csPin, LOW); 
    regAddress = regAddress << 1; // Writing to a register
    SPI.transfer(regAddress);
    SPI.transfer(value);
    digitalWrite(m_csPin, HIGH);
}

