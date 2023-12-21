# ADXL372
## Arduino library for the ADXL372 accelerometer
Available in Arduino Library manager!

Documentation: [ADXL372 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl372.pdf)

### How to install from Arduino IDE:
1. Open the Library Manager (Ctrl + Shift + I)
2. Search for ADXL372
3. Install the latest version

### How to use:

* ```ADXL372class(csPin)``` Create an ADXL372 object with the CS (Chip select) pin
* ```printDevice()``` Print the device's Analog Devices ID, device ID, MEMS ID, revision ID and Device Status
* ```readAcceleration(float& x, float& y, float& z)``` Read the 3-axis acceleration

Check out the example [here](https://github.com/Fourier-git/ADXL372/blob/main/examples/ReadAccelerationExample/ReadAccelerationExample.ino)