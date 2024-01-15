#include "Arduino.h"
#include "SPI.h"

enum OffsetTrims {
    OT_0 = 0b0000,
    OT_7_5 = 0b0001,
    OT_15 = 0b0010,
    OT_22_5 = 0b0011,
    OT_30 = 0b0100,
    OT_37_5 = 0b0101,
    OT_45 = 0b0110,
    OT_52_5 = 0b0111,
    OT_n60 = 0b1000,
    OT_n52_5 = 0b1001,
    OT_n45 = 0b1010,
    OT_n37_5 = 0b1011,
    OT_n30 = 0b1100,
    OT_n22_5 = 0b1101,
    OT_n15 = 0b1110,
    OT_n7_5 = 0b1111,
};

enum FifoMode
{
    FIFO_DISABLED = 0b00,
    STREAM = 0b01,
    TRIGGER = 0b10,
    OLDEST_SAVED = 0b11
};

enum FifoFormat
{
    XYZ = 0b000,
    X = 0b001,
    Y = 0b010,
    XY = 0b011,
    Z = 0b100,
    XZ = 0b101,
    YZ = 0b110,
    XYZ_PEAK = 0b111
};

enum Odr
{
    ODR_400Hz = 0b000,
    ODR_800Hz = 0b001,
    ODR_1600Hz = 0b010,
    ODR_3200Hz = 0b011,
    ODR_6400Hz = 0b100
};

enum WakeUpRate
{
    WUR_52ms = 0b000,
    WUR_104ms = 0b001,
    WUR_208ms = 0b010,
    WUR_512ms = 0b011,
    WUR_2048ms = 0b100,
    WUR_4096ms = 0b101,
    WUR_8192ms = 0b110,
    WUR_24576ms = 0b111,
};

enum Bandwidth
{
    BW_200Hz = 0b000,
    BW_400Hz = 0b001,
    BW_800Hz = 0b010,
    BW_1600Hz = 0b011,
    BW_3200Hz = 0b100
};

enum LinkLoop
{
    DEFAULT = 0b00,
    LINKED = 0b01,
    LOOPED = 0b10
};

enum OperatingMode
{
    STANDBY = 0b00,
    WAKE_UP = 0b01,
    INSTANT_ON = 0b10,
    FULL_BANDWIDTH = 0b11
};

enum FilterSettlingPeriod
{
    FSP_370ms = 0,
    FSP_16ms = 1
};

enum InstantOnThreshold
{
    IOT_LOW_THRESH = 0,
    IOT_HIGH_THRESH = 1
};

enum InterruptFunction {
    DATA_RDY = 0b00000000,
    FIFO_RDY = 0b00000010,
    FIFO_FULL = 0b00000100,
    FIFO_OVR = 0b00001000,
    INACT = 0b00010000,
    ACT = 0b00100000,
    ACT2 = 0b00100000,
    AWAKE = 0b01000000,
    INT_LOW = 0b10000000
};

class ADXL372class
{
public:
    ADXL372class(int csPinInput);
    virtual ~ADXL372class();

    void begin();
    void begin(uint32_t spiClockSpeed);
    void end();
    void printDevice();
    bool selfTest();

    void readAcceleration(float &x, float &y, float &z);
    void readPeakAcceleration(float &x, float &y, float &z);

    void setOffsetTrims(OffsetTrims xOffset, OffsetTrims yOffset, OffsetTrims zOffset);

    void setActivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold);
    void enableActivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ);
    void setReferencedActivityProcessing(bool isReferenced);
    void setActivityTimer(uint8_t timerPeriod);
    
    void setInactivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold);
    void enableInactivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ);
    void setReferencedInactivityProcessing(bool isReferenced);
    void setInactivityTimer(uint16_t timerPeriod);

    void setMotionWarningThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold);
    void enableMotionWarningDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ);
    void setReferencedMotionWarningProcessing(bool isReferenced);

    void readFifoData(uint16_t *fifoData);
    void setFifoSamples(int sampleSize);
    void setFifoMode(FifoMode mode);
    void setFifoFormat(FifoFormat format);

    void selectInt1Function(InterruptFunction function);
    void selectInt1Functions(uint8_t function);
    void selectInt2Function(InterruptFunction function);
    void selectInt2Functions(uint8_t function);

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
    int m_sampleSize;

    uint8_t formatThresholdValue(uint16_t thresholdValue);
    void checkStandbyMode();

    uint8_t readRegister(byte regAddress);
    void writeRegister(byte regAddress, uint8_t value);
    void updateRegister(byte regAddress, uint8_t value, byte mask);

};