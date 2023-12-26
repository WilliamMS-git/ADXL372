# ADXL372
## Arduino library for the ADXL372 accelerometer
Available in Arduino Library manager!

Documentation: [ADXL372 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl372.pdf)

### How to install from Arduino IDE:
1. Open the Library Manager (Ctrl + Shift + I).
2. Search for ADXL372.
3. Install the latest version.

### How to use:
This library uses SPI as communication with the accelerometer. I²C is supported, but has limited functionality. See the documentation for further information.

This accelerometer is packed with features, so check out the [simple example here](https://github.com/Fourier-git/ADXL372/blob/main/examples/ReadAccelerationExample/ReadAccelerationExample.ino) for the minimum requirements for continuously reading the 3-axis acceleration.

* ```ADXL372class(csPin)``` Create an ADXL372 object with the CS (Chip select) pin.
* ```printDevice()``` Print the device's Analog Devices ID, device ID, MEMS ID, revision ID and Device Status.
* ```setStatusCheck(bool isCheckingStatus)``` Sets if you want to check if the data is ready or not, before reading the acceleration data.
* ```readAcceleration(float& x, float& y, float& z)``` Read the 3-axis acceleration in g.

* ```setOdr(Odr odr)``` Sets the ODR. Default is 400Hz. 
    
    ODR options:
    1. ```ODR_400Hz```
    2. ```ODR_800Hz```
    3. ```ODR_1600Hz```
    4. ```ODR_3200Hz```
    5. ```ODR_6400Hz```

* ```setWakeUpRate(WakeUpRate wur)``` Sets the wake-up rate. Default is 52ms.
    
    Wake-up rate options:
    1. ```WUR_52ms```
    2. ```WUR_104ms```
    3. ```WUR_208ms```
    4. ```WUR_512ms```
    5. ```WUR_2048ms```
    6. ```WUR_4096ms```
    7. ```WUR_8192ms```
    8. ```WUR_24576ms```

* ```enableExternalClock(bool isEnabled)``` Enables external clock. When using this, apply a clock in the INT1 pin.
* ```enableExternalTrigger(bool isEnabled)``` Enables external triggers. When this enabled, use the INT2 pin as the sync trigger input.
* ```setBandwidth(Bandwidth bandwidth)``` Sets the bandwidth of the accelerometer. Default is 200Hz. Please use up to half of the ODR to unsure the Nyquist criteria. 
    
    Bandwidth options:
    1. ```BW_200Hz```
    2. ```BW_400Hz```
    3. ```BW_800Hz```
    4. ```BW_1600Hz```
    5. ```BW_3200Hz```

* ```enableLowNoiseOperation(bool isEnabled)``` Enables Low-noise operation when set to true.
* ```setLinkLoopActivityProcessing(LinkLoop activityProcessing)``` Sets the activity processing mode.

    Link-Loop options:
    1. ```DEFAULT```
    2. ```LINKED```
    3. ```LOOPED```

* ```enableAutosleep(bool isEnabled)``` Enables Autosleep mode when set to true. If activity processing is set to ```DEFAULT```, this bit will be ignored.

* ```setOperatingMode(OperatingMode opMode)``` Sets the accelerometer operating mode. 

    Operating mode options:
    1. ```STANDBY```
    2. ```WAKE_UP```
    3. ```INSTANT_ON```
    4. ```FULL_BANDWIDTH```

* ```disableHighPassFilter(bool isDisabled)``` Disables the digital high-pass filter.
* ```disableLowPassFilter(bool isDisabled)``` Disables the digital low-pass activity detect filter.
* ```setFilterSettling(FilterSettlingPeriod filterSettling)``` Sets the filter settling period. It is ideal to set this to 16ms when the high-pass filter and the low-pass activity
detect filter are disabled.

    Filter settling period options:
    1. ```FSP_370ms```
    2. ```FSP_16ms```

* ```setInstantOnThreshold(InstantOnThreshold threshold)``` Sets the instant on threshold.

    Intant on threshold options:
    1. ```IOT_LOW_THRESH```
    2. ```IOT_HIGH_THRESH```

