# ADXL372
## Arduino library for the ADXL372 accelerometer
Available in Arduino Library manager!

Documentation: [ADXL372 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl372.pdf)

### How to install from Arduino IDE:
1. Open the Library Manager (Ctrl + Shift + I)
2. Search for ADXL372
3. Install the latest version

### How to use:
This library uses SPI as communication with the accelerometer.
* ```ADXL372class(csPin)``` Create an ADXL372 object with the CS (Chip select) pin
* ```printDevice()``` Print the device's Analog Devices ID, device ID, MEMS ID, revision ID and Device Status
* ```readAcceleration(float& x, float& y, float& z, bool statusCheck)``` Read the 3-axis acceleration in the unit of g. Use ```statusCheck``` to check if the register is ready to read new values
* ```setBandwidth(Bandwidth bandwidth)``` Sets the bandwidth of the accelerometer. Default is 200Hz. Please use up to half of the ODR to unsure the Nyquist criteria. Use ```BW_200Hz```, ```BW_400Hz```, ```BW_800Hz```, ```BW_1600Hz```, or ```BW_3200Hz```.
* ```setOdr(Odr odr)``` Sets the ODR. Default is 400Hz. Use ```ODR_400Hz```, ```ODR_800Hz```, ```ODR_1600Hz```, ```ODR_3200Hz```,  or ```ODR_6400Hz```.
* ```setOperatingMode(OperatingMode opMode)``` Sets the accelerometer operating mode. Use ```STANDBY```, ```WAKE_UP```, ```INSTANT_ON```, or ```FULL_BANDWIDTH```.

Check out the example [here](https://github.com/Fourier-git/ADXL372/blob/main/examples/ReadAccelerationExample/ReadAccelerationExample.ino)
