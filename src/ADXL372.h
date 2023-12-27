#include "Arduino.h"
#include "SPI.h"

enum FifoMode {
    FIFO_DISABLED = 0b00,
    STREAM = 0b01,
    TRIGGER = 0b10,
    OLDEST_SAVED = 0b11
};

enum FifoFormat {
    XYZ = 0b000,
    X = 0b001,
    Y = 0b010,
    XY = 0b011,
    Z = 0b100,
    XZ = 0b101,
    YZ = 0b110,
    XYZ_PEAK = 0b111
};

enum Odr {
    ODR_400Hz = 0b000,
    ODR_800Hz = 0b001,
    ODR_1600Hz = 0b010,
    ODR_3200Hz = 0b011,
    ODR_6400Hz = 0b100
};

enum WakeUpRate {
    WUR_52ms = 0b000,
    WUR_104ms = 0b001,
    WUR_208ms = 0b010,
    WUR_512ms = 0b011,
    WUR_2048ms = 0b100,
    WUR_4096ms = 0b101,
    WUR_8192ms = 0b110,
    WUR_24576ms = 0b111,
};

enum Bandwidth {
    BW_200Hz = 0b000,
    BW_400Hz = 0b001,
    BW_800Hz = 0b010,
    BW_1600Hz = 0b011,
    BW_3200Hz = 0b100
};

enum LinkLoop {
    DEFAULT = 0b00,
    LINKED = 0b01,
    LOOPED = 0b10
};

enum OperatingMode {
    STANDBY = 0b00,
    WAKE_UP = 0b01,
    INSTANT_ON = 0b10,
    FULL_BANDWIDTH = 0b11
};

enum FilterSettlingPeriod {
    FSP_370ms = 0,
    FSP_16ms = 1
};

enum InstantOnThreshold {
    IOT_LOW_THRESH = 0,
    IOT_HIGH_THRESH = 1
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
    void setStatusCheck(bool isCheckingStatus);

    void readAcceleration(float& x, float& y, float& z);

    void setFifoSamples(int sampleSize);
    void setFifoMode(FifoMode mode);
    void setFifoFormat(FifoFormat format);
    
    void setOdr(Odr odr);
    void setWakeUpRate(WakeUpRate wur);
    void enableExternalClock(bool isEnabled);
    void enableExternalTrigger(bool isEnabled);

    void setBandwidth(Bandwidth bandwidth);
    void enableLowNoiseOperation(bool isEnabled);
    void setLinkLoopActivityProcessing(LinkLoop activityProcessing);
    void enableAutosleep(bool isEnabled);

    void setOperatingMode(OperatingMode opMode);
    void disableHighPassFilter(bool isDisabled);
    void disableLowPassFilter(bool isDisabled);
    void setFilterSettling(FilterSettlingPeriod filterSettling);
    void setInstantOnThreshold(InstantOnThreshold threshold);
    
private:
    int m_csPin;
    int m_odr = 0;
    int m_bandwidth = 0;
    bool m_isCheckingStatus = true;

    uint8_t readRegister(byte regAddress);
    void writeRegister(byte regAddress, uint8_t value);
    void updateRegister(byte regAddress, uint8_t value, byte mask);


};