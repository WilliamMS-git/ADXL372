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

//Constants
#define SPI_SPEED 10000000 //ADXL372 supports up to 10MHz in SCLK frequency


ADXL372class::ADXL372class()
{
}

ADXL372class::~ADXL372class()
{
}

int ADXL372class::begin(int csPinInput)
{
    csPin = csPinInput;
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
    pinMode(csPin, OUTPUT); 
    digitalWrite(csPin, HIGH); //SPI MODE is 0 so the CS pin goes from high to low when recieving data
    //Set some adresses here
    return 1;
}

void ADXL372class::end()
{
    //set some adresses here

    SPI.end();
}

void ADXL372class::readAcceleration(float& x, float& y, float& z)
{
    uint16_t data[3];
    
    //Read accelerometer value and insert data into array
    ADXL372class::readAccelerometerRegister(ACCEL_CS_PIN, data, sizeof(data));
    
    //Converting calculations to read in G's
    x = ((float)data[0] * 200.0)/32768.0; 
    y = ((float)data[1] * 200.0)/32768.0;
    z = ((float)data[2] * 200.0)/32768.0;
}

int ADXL372class::accelerationAvailable() 
{
    //check if there is any data, otherwise return 0
    digitalWrite(csPin, LOW);

    uint8_t status = SPI.transfer(STATUS_REGISTER);

	digitalWrite(csPin, HIGH);
	return (status & (1<<0)); //If the accelerometer has data ready, the first bit the the status register will be 1.

}

void ADXL372class::SPIwriteByte(uint8_t subAddress, uint8_t data)
{
    digitalWrite(csPin, LOW); //Start of SPI transfer
	
	// If write, bit 0 (MSB) should be 0
	// If single write, bit 1 should be 0
	SPI.transfer(subAddress /*& 0x3F*/); // Send Address
	SPI.transfer(data); // Send data
	
	digitalWrite(csPin, HIGH); //End of SPI transfer
}

byte ADXL372class::SPIreadByte(uint8_t subAddress)
{
    byte value = 0;
    digitalWrite(csPin, LOW); //Start of SPI transfer
	
	SPI.transfer(/*0x80 |*/ subAddress);			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	value = SPI.transfer(0);					// Read the value back. Send 0 to stop reading.
	
	digitalWrite(csPin, HIGH); //End of SPI transfer
    return value;
}

void ADXL372class::readAccelerometerRegister(uint16_t* data, size_t length)
{

    uint8_t transferData[6];
    //Gathering data from accelerometer. This might be wrong if it is not multibyte transfer
    digitalWrite(csPin, LOW); //Start of SPI transfer
    
    for(uint8_t i = 0; i < sizeof(transferData); i++)
    {
        //This might be incorrect, place the first transfer outside the loop, without the index.
        SPI.transfer(XDATA_H+i); //First the address is called and then its recieved.
        transferData[i] = SPI.transfer(0x00);
    }
    digitalWrite(csPin, HIGH); //End of SPI transfer

    //Formatting the data
    for(uint8_t i = 0; i < length; i++)
    {
        uint16_t formattedData;
        formattedData = transferData[i];

        //Bit 3:0 in the LSB axis are reserved, so they are bitshifted out of existence, and the remaining 4 bits are added
        formattedData = formattedData << 4;
        
        formattedData += (transferData[1+i] >> 4);

        data[i] = formattedData;
    }

}