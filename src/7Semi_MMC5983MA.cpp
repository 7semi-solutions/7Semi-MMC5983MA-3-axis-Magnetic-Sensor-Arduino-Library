/**
 * 7Semi MMC5983MA Magnetometer Driver - Arduino Library
 * - Copyright (c) 2026 7Semi
 * - License: MIT
 *
 * - Device  : MMC5983MA (3-axis magnetometer + temperature)
 * - Buses   : I2C and SPI (Mode 3)
 * - Output  : Raw 18-bit magnetometer data converted to mG (milli-Gauss)
 * - Extras  : Optional smoothing (low-pass) filter, 2D calibration, background 3D auto-cal,
 *             heading + tilt-compensated heading
 *
 * Notes
 * - By default, single-shot reads are used unless continuous mode is enabled.
 * - For heading, set your local magnetic declination (degrees).
 * - Keep sensor away from magnetic interference during calibration.
 */

#include "7Semi_MMC5983MA.h"

MMC5983MA_7Semi::MMC5983MA_7Semi()
{
}

/**
 * - Initialize the sensor using I2C
 * - Optionally supports custom SDA/SCL pins on ESP32/ESP8266
 * - Probes the device address before running common begin()
 */
bool MMC5983MA_7Semi::beginI2C(TwoWire &wire, uint8_t i2cAddress, uint32_t clockHz, uint8_t i2cSDA, uint8_t i2cSCL)
{
    bus = Bus::I2C;
    i2c = &wire;
    address = i2cAddress;

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    if (i2cSDA != 255 && i2cSCL != 255)
    {
        i2c->begin(i2cSDA, i2cSCL);
    }
    else
    {
        i2c->begin();
    }
#else
    i2c->begin();
#endif

    i2c->setClock(clockHz);

    i2c->beginTransmission(address);
    if (i2c->endTransmission() != 0)
        return false;

    return begin();
}

/**
 * - Initialize the sensor using SPI (Mode 3, MSB first)
 * - Requires a valid CS pin (csPin must not be 255)
 * - Calls common begin() after bus setup
 */
bool MMC5983MA_7Semi::beginSPI(SPIClass &spiBus, uint8_t csPin, uint32_t clockHz, uint8_t spiSCK, uint8_t spiMOSI, uint8_t spiMISO)
{
    bus = Bus::SPI;
    spi = &spiBus;
    cs_pin = csPin;

    if (cs_pin == 255)
        return false;

    spiSettings = SPISettings(clockHz, MSBFIRST, SPI_MODE3);

    pinMode(cs_pin, OUTPUT);

    /** - Keep CS high when idle */
    digitalWrite(cs_pin, HIGH);
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    if (spiSCK != 255 && spiMOSI != 255 && spiMISO != 255)
    {
        spi->begin(spiSCK, spiMISO, spiMOSI, cs_pin);
    }
    else
        spi->begin();
#else
    spi->begin();
#endif

    return begin();
}

/**
 * - Common init for both I2C and SPI
 * - Validates chip ID, resets the sensor, then applies default configuration
 * - Enables Auto Set/Reset by default for improved stability
 */
bool MMC5983MA_7Semi::begin()
{
    if (bus == Bus::None)
        return false;

    uint8_t id = 0;
    if (!getProductID(id))
        return false;

    if (!sensorReset())
        return false;

    if (!writeReg(REG_CTRL0, (1u << 5)))
        return false;

    /** - Default CTRL1/CTRL2 */
    if (!writeReg(REG_CTRL1, 0x00))
        return false;
    if (!writeReg(REG_CTRL2, 0x00))
        return false;

    magFiltInit = false;
    return true;
}

/**
 * - Reads REG_PRODUCTID and validates the expected device ID
 * - Returns false if I/O fails or ID mismatch
 */
bool MMC5983MA_7Semi::getProductID(uint8_t &id)
{
    if (!readReg(REG_PRODUCTID, id))
        return false;
    if (id != MMC5983MA_EXPECTED_ID)
        return false;
    return true;
}

/* ---------------- Public API ---------------- */

/**
 * - Reads raw signed magnetometer data (centered around 0)
 * - If continuous mode is OFF, performs a single-shot trigger and waits ready
 * - If continuous mode is ON, waits for fresh data (short timeout)
 */
bool MMC5983MA_7Semi::readMagnetometerRaw(int32_t &x, int32_t &y, int32_t &z)
{
    bool ready = false;
    getbit(REG_CTRL2, 3, ready);

    uint8_t v = 0;
    if (!readReg(REG_CTRL2, v))
        return false;

    /** - CTRL2 bit3 indicates continuous measurement enable */
    bool continuous_mode = (v & 0x08) != 0;

    if (!continuous_mode)
    {
        if (!initMagMeasurement())
            return false;

        /** - STATUS bit0 becomes 1 when magnetometer data is ready */
        if (!waitStatusBit(MEAS_M, 25))
            return false;
    }
    else
    {
        /** - In continuous mode, wait only if not ready */
        bool rdy = false;
        if (!getbit(REG_STATUS, MEAS_M, rdy))
            return false;

        if (!rdy)
        {
            if (!waitStatusBit(MEAS_M, 25))
                return false;
        }
    }

    /** - 18-bit per axis packed across 7 bytes */
    uint8_t buf[7];
    if (!readNReg(REG_XOUT0, buf, 7))
        return false;

    uint32_t Xraw = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((buf[6] >> 6) & 0x03);
    uint32_t Yraw = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[6] >> 4) & 0x03);
    uint32_t Zraw = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[6] >> 2) & 0x03);

    uint8_t ctrl1 = 0;
    if (!readReg(REG_CTRL1, ctrl1))
        return false;

    uint8_t bwBits = ctrl1 & 0x03;
    int32_t midScale = 131072;

    if (bwBits == 0)
    {
        Xraw >>= 2;
        Yraw >>= 2;
        Zraw >>= 2;
        midScale = 32768;
    }

    x = (int32_t)Xraw - midScale;
    y = (int32_t)Yraw - midScale;
    z = (int32_t)Zraw - midScale;

    return true;
}

/**
 * - Converts raw readings into mG using LSB scale
 * - Applies user offsets/scales (calibration)
 * - Optional autoCal3D updates offsets/scales gradually over time
 */
bool MMC5983MA_7Semi::readMagnetometer(float &x_mG, float &y_mG, float &z_mG)
{
    int32_t xr, yr, zr;
    if (!readMagnetometerRaw(xr, yr, zr))
        return false;

    /** - Select scale based on BW bits (BW=00 -> 16-bit, else 18-bit) */
    uint8_t bwBits = 0;
    uint8_t ctrl1 = 0;
    if (!readReg(REG_CTRL1, ctrl1))
        return false;
    bwBits = ctrl1 & 0x03;

    const float lsb_mG = ((bwBits & 0x03) == 0) ? 0.25f : 0.0625f;

    float mx = (xr * lsb_mG - offsetX) * scaleX;
    float my = (yr * lsb_mG - offsetY) * scaleY;
    float mz = (zr * lsb_mG - offsetZ) * scaleZ;

    if (autoCal3D)
        autoCalUpdate3D(mx, my, mz);

    /** - Output smoothing on axes */
    if (smoothEnable)
    {
        fx = smoothUpdate(sx, mx);
        fy = smoothUpdate(sy, my);
        fz = smoothUpdate(sz, mz);
    }
    else
    {
        fx = mx;
        fy = my;
        fz = mz;
    }

    x_mG = fx;
    y_mG = fy;
    z_mG = fz;
    return true;
}

/**
 * - Triggers temperature measurement and waits for ready
 * - Converts TOUT register to degrees C using:
 *   - tempC = Tout * 0.8 - 75
 * - Returns NAN on failure
 */
bool MMC5983MA_7Semi::readTemperature(float &tempC)
{
    if (!initTempMeasurement())
    {
        tempC = NAN;
        return false;
    }

    const uint32_t tStart = millis();
    while (true)
    {
        bool ready = false;
        if (!getbit(REG_STATUS, 1, ready))
        {
            tempC = NAN;
            return false;
        }
        if (ready)
            break;

        if ((millis() - tStart) > 25)
        {
            tempC = NAN;
            return false;
        }
        delay(1);
    }

    uint8_t Tout = 0;
    if (!readReg(REG_TOUT, Tout))
    {
        tempC = NAN;
        return false;
    }

    tempC = ((float)Tout) * 0.8f - 75.0f;
    return true;
}

/**
 * - Triggers a magnetometer measurement in single-shot mode
 * - Uses CTRL0 bit0 (self-clearing)
 */
bool MMC5983MA_7Semi::initMagMeasurement()
{
    return setbit(REG_CTRL0, 0);
}

/**
 * - Triggers a temperature measurement
 * - Uses CTRL0 bit1 (self-clearing)
 */
bool MMC5983MA_7Semi::initTempMeasurement()
{
    return setbit(REG_CTRL0, 1);
}

/**
 * - Reads STATUS bit0 to indicate magnetometer ready state
 */
bool MMC5983MA_7Semi::isMagReady(bool &ready)
{
    return getbit(REG_STATUS, 0, ready);
}

/**
 * - Reads STATUS bit1 to indicate temperature ready state
 */
bool MMC5983MA_7Semi::isTempReady(bool &ready)
{
    return getbit(REG_STATUS, 1, ready);
}

/** - Tilt-compensated heading
 *  - roll_deg, pitch_deg are expected in degrees
 *  - declination_deg is added to heading (degrees)
 *  - Uses magnetometer X/Y/Z and compensates for roll/pitch
 *  - Returns heading in degrees, normalized to 0..360
 */
float MMC5983MA_7Semi::getHeadingTilt(float roll_deg, float pitch_deg, float declination_deg)
{
    int32_t xi = 0;
    int32_t yi = 0;
    int32_t zi = 0;

    if (!readMagnetometerRaw(xi, yi, zi))
        return NAN;

    float x = (float)xi;
    float y = (float)yi;
    float z = (float)zi;


    float roll = roll_deg * (float)(M_PI / 180.0);
    float pitch = pitch_deg * (float)(M_PI / 180.0);

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    /** - Common tilt-compensation formula (assumes:
     *  - roll about X axis, pitch about Y axis
     *  - magnetometer axes aligned with roll/pitch frame)
     */
    float Xh = (x * cp) + (z * sp);
    float Yh = (x * sr * sp) + (y * cr) - (z * sr * cp);

    float heading = atan2f(Yh, Xh) * (float)(180.0 / M_PI);

    /** - Convert -180..180 to 0..360 */
    if (heading < 0.0f)
        heading += 360.0f;

    /** - Apply declination and re-normalize */
    heading += declination_deg;
    while (heading >= 360.0f)
        heading -= 360.0f;
    while (heading < 0.0f)
        heading += 360.0f;

    return heading;
}
/**
 * - Polls a STATUS bit until it becomes 1 or timeout expires
 * - Used by both magnetometer and temperature paths
 */
bool MMC5983MA_7Semi::waitStatusBit(uint8_t bit, uint32_t timeoutMs)
{
    uint32_t t0 = millis();
    while ((millis() - t0) < timeoutMs)
    {
        bool ready = false;
        if (!getbit(REG_STATUS, bit, ready))
            return false;
        if (ready)
            return true;
        delay(1);
    }
    return false;
}

/**
 * - Enables or disables continuous measurement mode
 * - CTRL2 bit3: continuous enable
 * - CTRL2 bits[2:0]: output frequency selection
 */
bool MMC5983MA_7Semi::enableContinuousMode(bool enable, Frequency freq)
{
    uint8_t val = 0;
    if (!readReg(REG_CTRL2, val))
        return false;

    if (enable)
    {
        val |= 0x08;
        val = (val & 0xF8) | (freq & 0x07);
    }
    else
    {
        val &= ~0x08;
        val = (val & 0xF8);
    }
    return writeReg(REG_CTRL2, val);
}

/**
 * - Sets digital filter bandwidth option
 * - Lower bandwidth reduces noise but increases latency
 */
bool MMC5983MA_7Semi::setBandwidth(Bandwidth bw)
{
    uint8_t val = 0;
    if (!readReg(REG_CTRL1, val))
        return false;

    val = (val & 0xFC) | (bw & 0x03);
    return writeReg(REG_CTRL1, val);
}

/**
 * - Performs a software reset using CTRL1 bit7
 * - Uses a short delay for internal restart
 */
bool MMC5983MA_7Semi::sensorReset()
{
    if (!writeReg(REG_CTRL1, 0x80))
        return false;
    delay(200);
    return true;
}

/**
 * - Enables/disables Auto Set/Reset
 * - CTRL0 bit5 controls Auto Set/Reset enable
 * - CTRL2 bits[6:4] configure Auto Set/Reset frequency (ASR)
 */
bool MMC5983MA_7Semi::enableAutoSetReset(bool enable, autoSet as)
{
    uint8_t val;

    if (!readReg(REG_CTRL0, val))
        return false;

    if (enable)
        val |= (1u << 5);
    else
        val &= ~(1u << 5);

    if (!writeReg(REG_CTRL0, val))
        return false;

    if (enable)
    {
        if (!readReg(REG_CTRL2, val))
            return false;

        val &= ~0x70;
        val |= ((uint8_t)as << 4);

        if (!writeReg(REG_CTRL2, val))
            return false;
    }

    return true;
}

/**
 * - Enables/disables periodic SET feature
 * - CTRL2 bit7 : En_prd_set
 * - CTRL2 bits[6:4] : Prd_set[2:0]
 * - Requires Auto_SR_en (CTRL0 bit5) and continuous mode (CTRL2 bit3) enabled
 */
bool MMC5983MA_7Semi::enablePeriodicSet(bool enable, autoSet period)
{
    if (enable)
    {
        /** - Ensure Auto Set/Reset is enabled first */
        if (!enableAutoSetReset(true, period))
            return false;

        uint8_t ctrl2 = 0;
        if (!readReg(REG_CTRL2, ctrl2))
            return false;

        /** - Ensure continuous mode enabled with a non-zero frequency */
        uint8_t freq = (ctrl2 & 0x07);
        if (freq == 0)
            freq = (uint8_t)CM_10Hz;

        ctrl2 |= 0x08;                                   /** - Cmm_en */
        ctrl2 = (ctrl2 & 0x8F) | ((uint8_t)period << 4); /** - Prd_set[2:0] */
        ctrl2 = (ctrl2 & 0xF8) | (freq & 0x07);          /** - Cm_freq[2:0] */
        ctrl2 |= 0x80;                                   /** - En_prd_set */

        return writeReg(REG_CTRL2, ctrl2);
    }
    else
    {
        uint8_t ctrl2 = 0;
        if (!readReg(REG_CTRL2, ctrl2))
            return false;

        ctrl2 &= ~0x80; /** - Disable periodic set */
        return writeReg(REG_CTRL2, ctrl2);
    }
}

/**
 * - Enables/disables INT output on completed measurements
 * - CTRL0 bit2 : INT_meas_done_en
 */
bool MMC5983MA_7Semi::enableMeasurementDoneInterrupt(bool enable)
{
    uint8_t ctrl0 = 0;
    if (!readReg(REG_CTRL0, ctrl0))
        return false;

    if (enable)
        ctrl0 |= (1u << 2);
    else
        ctrl0 &= ~(1u << 2);

    return writeReg(REG_CTRL0, ctrl0);
}

/**
 * - Clears measurement-done interrupt flags
 * - Datasheet: writing '1' to STATUS Meas_*_Done clears the corresponding interrupt
 */
bool MMC5983MA_7Semi::clearMeasurementDoneInterrupt(bool clearMag, bool clearTemp)
{
    uint8_t v = 0;
    if (clearMag)
        v |= (1u << 0);
    if (clearTemp)
        v |= (1u << 1);

    return writeReg(REG_STATUS, v);
}

/**
 * - Low power idle helper
 * - Disables continuous mode and periodic set (chip will only run on-demand single-shot)
 */
bool MMC5983MA_7Semi::setPowerDown(bool enable)
{
    if (!enable)
        return true;

    /** - Disable periodic set first */
    if (!enablePeriodicSet(false))
        return false;

    /** - Disable continuous mode */
    if (!enableContinuousMode(false, CM_OFF))
        return false;

    return true;
}

/**
 * - Issues a SET operation (self-clearing bit)
 * - Useful when manually forcing set/reset cycles
 */
bool MMC5983MA_7Semi::opSet()
{
    return setbit(REG_CTRL0, 3);
}

/**
 * - Issues a RESET operation (self-clearing bit)
 * - Useful when manually forcing set/reset cycles
 */
bool MMC5983MA_7Semi::opReset()
{
    return setbit(REG_CTRL0, 4);
}

/**
 * - Enables/disables output smoothing on XYZ readings
 * - When disabled, readMagnetometer() returns raw calibrated values (no smoothing state)
 */
void MMC5983MA_7Semi::enableSmoothing(bool enable)
{
    smoothEnable = enable;

    /** - Reset smoothing state when toggling */
    sx.init = false;
    sy.init = false;
    sz.init = false;
}

/**
 * - Returns true if output smoothing is enabled
 */
bool MMC5983MA_7Semi::isSmoothingEnabled() const
{
    return smoothEnable;
}

/**
 * - Selects how heading is computed from X/Y (board orientation handling)
 * - Resets heading smoothing state so the next reading initializes again
 */
void MMC5983MA_7Semi::setAxisMap(AxisMap map)
{
    axisMap = map;
    headingInit = false;
}

/**
 * - Sets hard-iron offsets in mG (applied to converted values)
 */
void MMC5983MA_7Semi::setOffset(float x_off_mG, float y_off_mG, float z_off_mG)
{
    offsetX = x_off_mG;
    offsetY = y_off_mG;
    offsetZ = z_off_mG;
}

/**
 * - Sets soft-iron scale factors (1.0 means no scaling)
 */
void MMC5983MA_7Semi::setScale(float x_scale, float y_scale, float z_scale)
{
    scaleX = x_scale;
    scaleY = y_scale;
    scaleZ = z_scale;
}

/**
 * - Resets 2D calibration capture state
 * - Call update2DCalibration() repeatedly while rotating the device, then finish2DCalibration()
 */
void MMC5983MA_7Semi::calibrationReset2D()
{
    minX = 1e9f;
    maxX = -1e9f;
    minY = 1e9f;
    maxY = -1e9f;
    calOffX = 0;
    calOffY = 0;
    calScX = 1.0f;
    calScY = 1.0f;
    calDone2D = false;
}

/**
 * - Updates min/max capture for 2D calibration
 * - Best results when keeping the sensor level and rotating through full turns
 */
void MMC5983MA_7Semi::update2DCalibration()
{
    float mx, my, mz;
    if (!readMagnetometer(mx, my, mz))
        return;

    if (mx < minX)
        minX = mx;
    if (mx > maxX)
        maxX = mx;
    if (my < minY)
        minY = my;
    if (my > maxY)
        maxY = my;
}

/**
 * - Computes 2D calibration offsets and scale factors from captured min/max
 * - After calling this, heading functions apply 2D correction automatically
 */
void MMC5983MA_7Semi::finish2DCalibration()
{
    calOffX = (maxX + minX) * 0.5f;
    calOffY = (maxY + minY) * 0.5f;

    float radX = (maxX - minX) * 0.5f;
    float radY = (maxY - minY) * 0.5f;

    if (radX < 1e-6f)
        radX = 1.0f;
    if (radY < 1e-6f)
        radY = 1.0f;

    float avg = (radX + radY) * 0.5f;
    calScX = avg / radX;
    calScY = avg / radY;

    calDone2D = true;
    headingInit = false;
}
/**
 * - Returns true if 2D calibration has been completed and is ready to apply correction
 */
bool MMC5983MA_7Semi::calibrationReady2D() const
{
    return calDone2D;
}

/**
 * - Enables background 3D auto-cal (best effort)
 * - Updates min/max on all axes over time to estimate offsets and scale
 */
void MMC5983MA_7Semi::enableAutoCal3D(bool enable)
{
    autoCal3D = enable;
    if (enable)
    {
        minX3 = minY3 = minZ3 = 1e9f;
        maxX3 = maxY3 = maxZ3 = -1e9f;
    }
}

/**
 * - Returns 2D heading in degrees [0..360)
 * - Applies 2D calibration (if done), axis mapping, declination, and smoothing
 */
float MMC5983MA_7Semi::getHeading2D(float declination_deg)
{
    float mx, my, mz;
    if (!readMagnetometer(mx, my, mz))
        return NAN;

    if (calDone2D)
    {
        mx = (mx - calOffX) * calScX;
        my = (my - calOffY) * calScY;
    }

    float hdg = applyAxisMapHeading(mx, my);
    hdg = wrap360(hdg + declination_deg);

    if (!headingInit)
    {
        fHeading = hdg;
        headingInit = true;
    }
    else
    {
        float d = angleDiffDeg(hdg, fHeading);
        fHeading = wrap360(fHeading + d);
    }

    return fHeading;
}

/* ================= I2C helpers ================= */

bool MMC5983MA_7Semi::i2cRead(uint8_t reg, uint8_t &val)
{
    return i2cNRead(reg, &val, 1);
}

/**
 * - Reads one register byte using the selected bus (I2C or SPI)
 */
bool MMC5983MA_7Semi::readReg(uint8_t reg, uint8_t &val)
{
    if (bus == Bus::I2C)
        return i2cRead(reg, val);
    else if (bus == Bus::SPI)
        return spiRead(reg, val);

    return false;
}

/**
 * - Reads multiple bytes starting at reg using the selected bus (I2C or SPI)
 */
bool MMC5983MA_7Semi::readNReg(uint8_t reg, uint8_t *buf, size_t len)
{
    if (bus == Bus::I2C)
        return i2cNRead(reg, buf, len);
    else if (bus == Bus::SPI)
        return spiNRead(reg, buf, len);

    return false;
}

/**
 * - Writes one register byte using the selected bus (I2C or SPI)
 */
bool MMC5983MA_7Semi::writeReg(uint8_t reg, uint8_t val)
{
    if (bus == Bus::I2C)
        return i2cWrite(reg, val);
    else if (bus == Bus::SPI)
        return spiWrite(reg, val);

    return false;
}

/**
 * - Reads len bytes starting from reg over I2C
 * - Caller must provide a valid buffer and non-zero length
 */
bool MMC5983MA_7Semi::i2cNRead(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!buf || len == 0)
        return false;

    i2c->beginTransmission(address);
    i2c->write(reg);
    if (i2c->endTransmission(false) != 0)
        return false;

    if (i2c->requestFrom(address, (uint8_t)len) != len)
        return false;

    size_t idx = 0;

    for (size_t timeout = 0; timeout < 1000; timeout++)
    {
        while (i2c->available() && idx < len)
        {
            buf[idx++] = i2c->read();
        }

        if (idx == len)
            return true;
        delay(1);
    }

    // Timeout before all bytes arrived
    return false;
}

/**
 * - Writes one byte to a register over I2C (reg then val)
 */
bool MMC5983MA_7Semi::i2cWrite(uint8_t reg, uint8_t val)
{
    i2c->beginTransmission(address);
    i2c->write(reg);
    i2c->write(val);
    return (i2c->endTransmission() == 0);
}

/* ---------------- Bit helpers ---------------- */

/**
 * - Read-modify-write: set a single bit in a register
 */
bool MMC5983MA_7Semi::setbit(uint8_t reg, uint8_t bit)
{
    uint8_t val = 0;
    if (!readReg(reg, val))
        return false;
    val |= (1u << bit);
    return writeReg(reg, val);
}

/**
 * - Read-modify-write: clear a single bit in a register
 */
bool MMC5983MA_7Semi::clearbit(uint8_t reg, uint8_t bit)
{
    uint8_t val = 0;
    if (!readReg(reg, val))
        return false;
    val &= ~(1u << bit);
    return writeReg(reg, val);
}

/**
 * - Reads a register and returns the state of a single bit
 */
bool MMC5983MA_7Semi::getbit(uint8_t reg, uint8_t bit, bool &state)
{
    uint8_t val = 0;
    if (!readReg(reg, val))
        return false;
    state = ((val & (1u << bit)) != 0);
    return true;
}

/* ================= SPI helpers ================= */

/**
 * - Reads one register byte over SPI
 * - Command byte format:
 *   - bit7: 1 = read
 *   - bits[5:0]: register address
 */
bool MMC5983MA_7Semi::spiRead(uint8_t reg, uint8_t &val)
{
    uint8_t cmd = (1u << 7) | (reg & 0x3Fu);

    spiSelect();
    spi->transfer(cmd);
    val = spi->transfer(0x00);
    spiDeselect();

    return true;
}

/**
 * - Reads multiple bytes starting at reg over SPI
 */
bool MMC5983MA_7Semi::spiNRead(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!buf || len == 0)
        return false;

    uint8_t cmd = (1u << 7) | (reg & 0x3Fu);

    spiSelect();
    spi->transfer(cmd);
    for (size_t i = 0; i < len; i++)
        buf[i] = spi->transfer(0x00);
    spiDeselect();

    return true;
}

/**
 * - Writes one register byte over SPI
 * - Command byte format:
 *   - bit7: 0 = write
 *   - bits[5:0]: register address
 */
bool MMC5983MA_7Semi::spiWrite(uint8_t reg, uint8_t val)
{
    uint8_t cmd = (0u << 7) | (reg & 0x3Fu);

    spiSelect();
    spi->transfer(cmd);
    spi->transfer(val);
    spiDeselect();

    return true;
}

/**
 * - Pulls CS low and begins an SPI transaction
 */
void MMC5983MA_7Semi::spiSelect()
{
    spi->beginTransaction(spiSettings);
    digitalWrite(cs_pin, LOW);
}

/**
 * - Ends an SPI transaction and releases CS high
 */
void MMC5983MA_7Semi::spiDeselect()
{
    spi->endTransaction();
    digitalWrite(cs_pin, HIGH);
}

/* ---------------- Math helpers ---------------- */

/**
 * - Wraps an angle into range [0 .. 360)
 */
float MMC5983MA_7Semi::wrap360(float deg)
{
    while (deg < 0)
        deg += 360.0f;
    while (deg >= 360.0f)
        deg -= 360.0f;
    return deg;
}

/**
 * - Returns smallest signed difference between two angles in degrees
 * - Output range is [-180 .. 180]
 */
float MMC5983MA_7Semi::angleDiffDeg(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

/**
 * - Computes heading angle from x/y using selected axis mapping
 * - Output is in degrees (not wrapped to 0..360 here)
 */
float MMC5983MA_7Semi::applyAxisMapHeading(float x, float y) const
{
    float rad = 0.0f;
    switch (axisMap)
    {
    case MAP_ATAN2_Y_X:
        rad = atan2f(y, x);
        break;
    case MAP_ATAN2_X_Y:
        rad = atan2f(x, y);
        break;
    case MAP_ATAN2_NEGY_X:
        rad = atan2f(-y, x);
        break;
    case MAP_ATAN2_Y_NEGX:
        rad = atan2f(y, -x);
        break;
    default:
        rad = atan2f(y, x);
        break;
    }
    return rad * (180.0f / PI);
}

/* ---------------- Auto-cal (best effort) ---------------- */

/**
 * - Best-effort background 3D calibration
 * - Updates offsets/scales using min/max capture on all axes
 * - Rejects readings that are too small or too large (likely interference)
 */
void MMC5983MA_7Semi::autoCalUpdate3D(float mx, float my, float mz)
{
    float mag = sqrtf(mx * mx + my * my + mz * mz);

    /** - Reject very low magnitude readings */
    if (mag < 50.0f)
        return;

    /** - Reject unrealistic spikes caused by interference */
    if (mag > 2000.0f)
        return;

    if (mx < minX3)
        minX3 = mx;
    if (mx > maxX3)
        maxX3 = mx;
    if (my < minY3)
        minY3 = my;
    if (my > maxY3)
        maxY3 = my;
    if (mz < minZ3)
        minZ3 = mz;
    if (mz > maxZ3)
        maxZ3 = mz;

    offsetX = (maxX3 + minX3) * 0.5f;
    offsetY = (maxY3 + minY3) * 0.5f;
    offsetZ = (maxZ3 + minZ3) * 0.5f;

    float rx = (maxX3 - minX3) * 0.5f;
    float ry = (maxY3 - minY3) * 0.5f;
    float rz = (maxZ3 - minZ3) * 0.5f;

    /** - Avoid division by very small radii */
    if (rx < 1e-3f || ry < 1e-3f || rz < 1e-3f)
        return;

    float avg = (rx + ry + rz) / 3.0f;
    scaleX = avg / rx;
    scaleY = avg / ry;
    scaleZ = avg / rz;
}

float MMC5983MA_7Semi::smoothUpdate(Smooth1D &s, float measurement)
{
    if (!s.init)
    {
        s.x = measurement;
        s.init = true;
        return s.x;
    }

    s.p = s.p + s.q;

    float k = s.p / (s.p + s.r);

    s.x = s.x + k * (measurement - s.x);
    s.p = (1.0f - k) * s.p;

    return s.x;
}
