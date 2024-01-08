#include "ADXL372.h"

#define DEVID_PRODUCT 0xFA // 372 in octal :)

// Data registers. Each axis data has a 12 bit value. Data is left justified, MSBFIRST.
// Register *_H contains the eight most significant bits (MSBs), and Register *_L contains the four least significant bits (LSBs) of the 12-bit value
#define XDATA_H 0x08
#define XDATA_L 0x09
#define YDATA_H 0x0A
#define YDATA_L 0x0B
#define ZDATA_H 0x0C
#define ZDATA_L 0x0D

// Peak Data registers.
#define MAXPEAK_X_H 0x15
#define MAXPEAK_X_L 0x16
#define MAXPEAK_Y_H 0x17
#define MAXPEAK_Y_L 0x18
#define MAXPEAK_Z_H 0x19
#define MAXPEAK_Z_L 0x1A

// ID registers
#define DEVID_AD 0x00
#define DEVID_MST 0x01
#define PARTID 0x02
#define REVID 0x03

// System registers
#define STATUS 0x04 // Status register

#define OFFSET_X 0x20
#define OFFSET_Y 0x21
#define OFFSET_Z 0x22

#define THRESH_ACT_X_H 0x23 // Activity threshold register
#define THRESH_ACT_X_L 0x24
#define THRESH_ACT_Y_H 0x25
#define THRESH_ACT_Y_L 0x26
#define THRESH_ACT_Z_H 0x27
#define THRESH_ACT_Z_L 0x28

#define TIME_ACT 0x29 // Activity time register

#define THRESH_INACT_X_H 0x2A // Inactivity threshold register
#define THRESH_INACT_X_L 0x2B
#define THRESH_INACT_Y_H 0x2C
#define THRESH_INACT_Y_L 0x2D
#define THRESH_INACT_Z_H 0x2E
#define THRESH_INACT_Z_L 0x2F

#define TIME_INACT_H 0x30 // Inactivity time register
#define TIME_INACT_L 0x31

#define THRESH_ACT2_X_H 0x32 // Motion Warning Threshold register
#define THRESH_ACT2_X_L 0x33
#define THRESH_ACT2_Y_H 0x34
#define THRESH_ACT2_Y_L 0x35
#define THRESH_ACT2_Z_H 0x36
#define THRESH_ACT2_Z_L 0x37

#define FIFO_SAMPLES 0x39 // FIFO samples register
#define FIFO_CTL 0x3A     // FIFO control register

#define INT1_MAP 0x3B // Interrupt 1 & 2 map register
#define INT2_MAP 0x3C

#define TIMING 0x3D    // Timing control register
#define MEASURE 0x3E   // Measurement control register
#define POWER_CTL 0x3F // Power control register

#define FIFO_DATA 0x42 // FIFO data register

// System bitmasks
#define THRESH_ACT_L_MASK 0x1F // Activity detection
#define ACT_EN_MASK 0xFE
#define ACT_REF_MASK 0xFD

#define THRESH_INACT_L_MASK 0x1F // Inactivity detection
#define INACT_EN_MASK 0xFE
#define INACT_REF_MASK 0xFD

#define THRESH_ACT2_L_MASK 0x1F // Motion Warning
#define ACT2_EN_MASK 0xFE
#define ACT2_REF_MASK 0xFD

#define FIFO_SAMPLES_8_MASK 0xFE // FIFO control
#define FIFO_MODE_MASK 0xF9
#define FIFO_FORMAT_MASK 0xC7

#define INT_MAP_MASK 0xFF // Interrupt 1 and 2

#define EXT_SYNC_MASK 0xFE // Timing
#define EXT_CLK_MASK 0xFD
#define WAKEUP_RATE_MASK 0xE3
#define ODR_MASK 0x1F

#define BANDWIDTH_MASK 0xF8 // Measure
#define LOW_NOISE_MASK 0xF7
#define LINKLOOP_MASK 0xCF
#define AUTOSLEEP_MASK 0xBF

#define MODE_MASK 0xFC // Power Control
#define HPF_DISABLE_MASK 0xFB
#define LPF_DISABLE_MASK 0xF7
#define FILTER_SETTLE_MASK 0xEF
#define INSTANT_ON_THRESH_MASK 0xDF

// Accelerometer Constants
#define SPI_SPEED 10000000 // ADXL372 supports up to 10MHz in SCLK frequency
#define SCALE_FACTOR 100   // mg per LSB
#define MG_TO_G 0.001      // g per mg

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
    pinMode(m_csPin, OUTPUT);                                          // Setting chip select pin
    digitalWrite(m_csPin, HIGH);                                       // Pin ready
}

void ADXL372class::begin(uint32_t spiClockSpeed)
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0)); // CPHA = CPOL = 0
    pinMode(m_csPin, OUTPUT);                                          // Setting chip select pin
    digitalWrite(m_csPin, HIGH);                                       // Pin ready
}

void ADXL372class::end()
{
    // set some adresses here

    SPI.end();
}

void ADXL372class::printDevice()
{
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

void ADXL372class::setStatusCheck(bool isCheckingStatus)
{
    m_isCheckingStatus = isCheckingStatus;
}

void ADXL372class::readAcceleration(float &x, float &y, float &z)
{

    if (m_isCheckingStatus == true)
    {
        byte status;
        do
        {
            status = readRegister(0x04);
        } while ((status & 0x01) == 0); // Waiting for status register
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

void ADXL372class::readPeakAcceleration(float &xPeak, float &yPeak, float &zPeak)
{
    if (m_isCheckingStatus == true)
    {
        byte status;
        do
        {
            status = readRegister(0x04);
        } while ((status & 0x01) == 0);
    }

    short rawX = readRegister(MAXPEAK_X_H) << 8 | readRegister(MAXPEAK_X_L);
    short rawY = readRegister(MAXPEAK_Y_H) << 8 | readRegister(MAXPEAK_Y_L);
    short rawZ = readRegister(MAXPEAK_Z_H) << 8 | readRegister(MAXPEAK_Z_L);

    rawX = rawX >> 4;
    rawY = rawY >> 4;
    rawZ = rawZ >> 4;

    // Converting raw axis data to acceleration in g unit
    xPeak = rawX * SCALE_FACTOR * MG_TO_G;
    yPeak = rawY * SCALE_FACTOR * MG_TO_G;
    zPeak = rawZ * SCALE_FACTOR * MG_TO_G;
}

void ADXL372class::setOffsetTrims(OffsetTrims xOffset, OffsetTrims yOffset, OffsetTrims zOffset)
{
    // No need to mask or bitshift for the offset registers
    writeRegister(OFFSET_X, xOffset);
    writeRegister(OFFSET_Y, yOffset);
    writeRegister(OFFSET_Z, zOffset);
}

uint8_t ADXL372class::formatThresholdValue(uint16_t thresholdValue)
{
    if (thresholdValue > 0x7FF) // The threshold value is an 11-bit value. So the max limit is 0x7FF.
    {
        Serial.println("WARNING: Threshold value limit is 2047");
    }
    return thresholdValue = thresholdValue >> 3; // Get 8 MSB
}

void ADXL372class::setActivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
{
    checkStandbyMode();
    uint8_t xThresh8Msb = formatThresholdValue(xThreshold);
    writeRegister(THRESH_ACT_X_H, xThresh8Msb);
    updateRegister(THRESH_ACT_X_L, (xThreshold << 5), THRESH_ACT_L_MASK);

    uint8_t yThresh8Msb = formatThresholdValue(yThreshold);
    writeRegister(THRESH_ACT_Y_H, yThresh8Msb);
    updateRegister(THRESH_ACT_Y_L, (yThreshold << 5), THRESH_ACT_L_MASK);

    uint8_t zThresh8Msb = formatThresholdValue(zThreshold);
    writeRegister(THRESH_ACT_Z_H, zThresh8Msb);
    updateRegister(THRESH_ACT_Z_L, (zThreshold << 5), THRESH_ACT_L_MASK);
}

void ADXL372class::enableActivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    checkStandbyMode();
    updateRegister(THRESH_ACT_X_L, isEnabledX, ACT_EN_MASK); // bit 1 in register
    updateRegister(THRESH_ACT_Y_L, isEnabledY, ACT_EN_MASK);
    updateRegister(THRESH_ACT_Z_L, isEnabledZ, ACT_EN_MASK);
}

void ADXL372class::setReferencedActivityProcessing(bool isReferenced)
{
    checkStandbyMode();
    updateRegister(THRESH_ACT_X_L, isReferenced << 1, ACT_REF_MASK); // bit 1 in register
}

void ADXL372class::setActivityTimer(uint8_t timerPeriod)
{
    checkStandbyMode();
    uint8_t currentOpMode = readRegister(POWER_CTL);
    currentOpMode &= 0b11; // Get only the MODE bits
    if (currentOpMode != FULL_BANDWIDTH)
    {
        Serial.println("WARNING: The activity timer is operational in measurement mode only");
    }
    writeRegister(TIME_ACT, timerPeriod);
}

void ADXL372class::setInactivityThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
{
    checkStandbyMode();
    uint8_t xThresh8Msb = formatThresholdValue(xThreshold);
    writeRegister(THRESH_INACT_X_H, xThresh8Msb);
    updateRegister(THRESH_INACT_X_L, (xThreshold << 5), THRESH_INACT_L_MASK);

    uint8_t yThresh8Msb = formatThresholdValue(yThreshold);
    writeRegister(THRESH_INACT_Y_H, yThresh8Msb);
    updateRegister(THRESH_INACT_Y_L, (yThreshold << 5), THRESH_INACT_L_MASK);

    uint8_t zThresh8Msb = formatThresholdValue(zThreshold);
    writeRegister(THRESH_INACT_Z_H, zThresh8Msb);
    updateRegister(THRESH_INACT_Z_L, (zThreshold << 5), THRESH_INACT_L_MASK);
}

void ADXL372class::enableInactivityDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    checkStandbyMode();
    updateRegister(THRESH_INACT_X_L, isEnabledX, INACT_EN_MASK);
    updateRegister(THRESH_INACT_Y_L, isEnabledY, INACT_EN_MASK);
    updateRegister(THRESH_INACT_Z_L, isEnabledZ, INACT_EN_MASK);
}

void ADXL372class::setReferencedInactivityProcessing(bool isReferenced)
{
    checkStandbyMode();
    updateRegister(THRESH_INACT_X_L, isReferenced << 1, INACT_REF_MASK);
}

void ADXL372class::setInactivityTimer(uint16_t timerPeriod)
{
    checkStandbyMode();
    uint8_t timerPeriodH = timerPeriod >> 8;
    uint8_t timerPeriodL = timerPeriod;

    writeRegister(TIME_INACT_H, timerPeriodH);
    writeRegister(TIME_INACT_L, timerPeriodL);
}

void ADXL372class::setMotionWarningThresholds(uint16_t xThreshold, uint16_t yThreshold, uint16_t zThreshold)
{
    uint8_t xThresh8Msb = formatThresholdValue(xThreshold);
    writeRegister(THRESH_ACT2_X_H, xThresh8Msb);
    updateRegister(THRESH_ACT2_X_L, (xThreshold << 5), THRESH_ACT2_L_MASK);

    uint8_t yThresh8Msb = formatThresholdValue(yThreshold);
    writeRegister(THRESH_ACT2_Y_H, yThresh8Msb);
    updateRegister(THRESH_ACT2_Y_L, (yThreshold << 5), THRESH_ACT2_L_MASK);

    uint8_t zThresh8Msb = formatThresholdValue(zThreshold);
    writeRegister(THRESH_ACT2_Z_H, zThresh8Msb);
    updateRegister(THRESH_ACT2_Z_L, (zThreshold << 5), THRESH_ACT2_L_MASK);
}
void ADXL372class::enableMotionWarningDetection(bool isEnabledX, bool isEnabledY, bool isEnabledZ)
{
    updateRegister(THRESH_ACT2_X_L, isEnabledX, ACT_EN_MASK); // bit 1 in register
    updateRegister(THRESH_ACT2_Y_L, isEnabledY, ACT_EN_MASK);
    updateRegister(THRESH_ACT2_Z_L, isEnabledZ, ACT_EN_MASK);
}
void ADXL372class::setReferencedMotionWarningProcessing(bool isReferenced)
{
    updateRegister(THRESH_ACT2_X_L, isReferenced << 1, ACT2_REF_MASK);
}

void ADXL372class::readFifoData(uint16_t *fifoData)
{
    if (m_isCheckingStatus == true)
    {
        byte status;
        do
        {
          status = readRegister(0x04);
        } while ((status & 0x04) == 0); // Waiting for FIFO full
    }
    digitalWrite(m_csPin, LOW);
    SPI.transfer(FIFO_DATA << 1 | 1);
    for (int i = 0; i < m_sampleSize; i++)
    {
      uint8_t msbFifoData = SPI.transfer(0x00);
      uint8_t lsbFifoData = SPI.transfer(0x00);
        fifoData[i] =  msbFifoData << 4 | lsbFifoData; // 12 bit data. 8 MSB and 4 LSB.
    }
    digitalWrite(m_csPin, HIGH);
}

void ADXL372class::setFifoSamples(int sampleSize)
{
    checkStandbyMode();
    if (sampleSize > 512)
    {
        Serial.println("WARNING: FIFO samples limit is 512");
        sampleSize = 512;
    }
    m_sampleSize = sampleSize;
    sampleSize -= 1;
    writeRegister(FIFO_SAMPLES, sampleSize & 0xFF); // Sending the 8 least significant bits in the samples
    updateRegister(FIFO_CTL, (sampleSize > 0xFF), FIFO_SAMPLES_8_MASK);
}

void ADXL372class::setFifoMode(FifoMode mode)
{
    checkStandbyMode();
    byte modeShifted = mode << 1; // starts from bit 1 in register
    updateRegister(FIFO_CTL, modeShifted, FIFO_MODE_MASK);
}

void ADXL372class::setFifoFormat(FifoFormat format)
{
    checkStandbyMode();
    byte formatShifted = format << 3; // starts from bit 3 in register
    updateRegister(FIFO_CTL, formatShifted, FIFO_FORMAT_MASK);
}

void ADXL372class::readFifoRegisters() {
    Serial.print("FIFO_CTL: ");
    Serial.println(readRegister(FIFO_CTL), BIN);
    Serial.print("FIFO_SAMPLES: ");
    Serial.println(readRegister(FIFO_SAMPLES), BIN);
}

void ADXL372class::selectInt1Function(InterruptFunction function)
{
    updateRegister(INT1_MAP, function, INT_MAP_MASK);
}

void ADXL372class::selectInt1Functions(uint8_t function)
{
    writeRegister(INT1_MAP, function);
}

void ADXL372class::selectInt2Function(InterruptFunction function)
{
    updateRegister(INT2_MAP, function, INT_MAP_MASK);
}

void ADXL372class::selectInt2Functions(uint8_t function)
{
    writeRegister(INT2_MAP, function);
}

void ADXL372class::setOdr(Odr odr)
{
    m_odr = (int)odr;
    if (m_odr < m_bandwidth)
    {
        Serial.println("WARNING: ODR must be at least double the bandwidth, to not violate the Nyquist criteria. Otherwise signal integrity will not be maintained");
    }
    byte odrShifted = m_odr << 5; // odr bits start from bit 5
    updateRegister(TIMING, odrShifted, ODR_MASK);
}

void ADXL372class::setWakeUpRate(WakeUpRate wur)
{
    byte wurShifted = wur << 2; // wur bits start from bit 2
    updateRegister(TIMING, wurShifted, WAKEUP_RATE_MASK);
}

void ADXL372class::enableExternalClock(bool isEnabled)
{
    byte valueShifted = isEnabled << 1; // bit 1 in register
    updateRegister(TIMING, valueShifted, EXT_CLK_MASK);
}

void ADXL372class::enableExternalTrigger(bool isEnabled)
{
    updateRegister(TIMING, isEnabled, EXT_SYNC_MASK);
}

void ADXL372class::setBandwidth(Bandwidth bandwidth)
{
    m_bandwidth = (int)bandwidth;
    if (m_bandwidth > m_odr)
    {
        Serial.println("WARNING: Bandwidth must be no greater than half the ODR, to not violate the Nyquist criteria. Otherwise signal integrity will not be maintained");
    }
    updateRegister(MEASURE, bandwidth, BANDWIDTH_MASK);
}

void ADXL372class::enableLowNoiseOperation(bool isEnabled)
{
    byte valueShifted = isEnabled << 3; // bit 3 in register
    updateRegister(MEASURE, valueShifted, LOW_NOISE_MASK);
}

void ADXL372class::setLinkLoopActivityProcessing(LinkLoop activityProcessing)
{
    checkStandbyMode();
    if (activityProcessing == LINKED | LOOPED)
    {
        // Check if Activity and Inactivity detection is enabled
        if ((readRegister(THRESH_ACT_X_L) & ACT_EN_MASK) == false | (readRegister(THRESH_ACT_Y_L) & ACT_EN_MASK) == false | (readRegister(THRESH_ACT_Z_L) & ACT_EN_MASK) == false |
            (readRegister(THRESH_INACT_X_L) & ACT_EN_MASK) == false | (readRegister(THRESH_INACT_Y_L) & ACT_EN_MASK) == false | (readRegister(THRESH_INACT_Z_L) & ACT_EN_MASK) == false)
        {
            Serial.println("WARNING: Activity and Inactivity detection must be enabled");
        }
    }
    byte valueShifted = activityProcessing << 4; // bit 4 in register
    updateRegister(MEASURE, valueShifted, LINKLOOP_MASK);
}

void ADXL372class::enableAutosleep(bool isEnabled)
{
    byte valueShifted = isEnabled << 6; // bit 6 in register
    updateRegister(MEASURE, valueShifted, AUTOSLEEP_MASK);
}

void ADXL372class::setOperatingMode(OperatingMode opMode)
{
    updateRegister(POWER_CTL, opMode, MODE_MASK);
}

void ADXL372class::disableHighPassFilter(bool isDisabled)
{
    byte valueShifted = isDisabled << 2; // bit 2 in register
    updateRegister(POWER_CTL, valueShifted, HPF_DISABLE_MASK);
}

void ADXL372class::disableLowPassFilter(bool isDisabled)
{
    byte valueShifted = isDisabled << 3; // bit 3 in register
    updateRegister(POWER_CTL, valueShifted, LPF_DISABLE_MASK);
}

void ADXL372class::setFilterSettling(FilterSettlingPeriod filterSettling)
{
    byte valueShifted = filterSettling << 4; // bit 4 in register
    updateRegister(POWER_CTL, valueShifted, LPF_DISABLE_MASK);
}

void ADXL372class::setInstantOnThreshold(InstantOnThreshold threshold)
{
    byte valueShifted = threshold << 5; // bit 5 in register
    updateRegister(POWER_CTL, valueShifted, LPF_DISABLE_MASK);
}

void ADXL372class::checkStandbyMode() {
   byte mode = readRegister(POWER_CTL);
   mode &= MODE_MASK;
   if(mode != STANDBY) {
        Serial.println("WARNING: Activity, Inactivity and FIFO can only be set while in standy mode");
   }
}

uint8_t ADXL372class::readRegister(byte regAddress)
{
    digitalWrite(m_csPin, LOW);
    regAddress = regAddress << 1 | 1; // Reading from a register
    SPI.transfer(regAddress);
    uint8_t value = SPI.transfer(0x00); // Transfering dummy byte to recieve data from accelerometer
    digitalWrite(m_csPin, HIGH);
    return value;
}

void ADXL372class::writeRegister(byte regAddress, uint8_t value)
{
    digitalWrite(m_csPin, LOW);
    regAddress = regAddress << 1; // Writing to a register
    SPI.transfer(regAddress);
    SPI.transfer(value);
    digitalWrite(m_csPin, HIGH);
}

void ADXL372class::updateRegister(byte regAddress, uint8_t value, byte mask)
{
    // Need to use bitmasks to only change the desired bits in the registers
    byte registerState = readRegister(regAddress);
    registerState &= mask;
    value |= registerState;
    writeRegister(regAddress, value);
}

void ADXL372class::testRegister(byte regAddress, uint8_t value, byte mask){
    updateRegister(regAddress, value, mask);
    delay(100); // making sure register is set
    Serial.println(readRegister(regAddress), BIN);
}