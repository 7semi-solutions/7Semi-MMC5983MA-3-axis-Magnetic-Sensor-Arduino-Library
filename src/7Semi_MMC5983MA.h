/**
 * 7Semi MMC5983MA Magnetometer Driver - Arduino Library
 * - Copyright (c) 2026 7Semi
 * - SPDX-License-Identifier: MIT
 * 
 * - Supports I2C communication
 * - Provides magnetometer readings, temperature, and heading calculations
 * - Includes optional EMA filtering and calibration routines
 * - Sensor-only driver:
 *   - Reads magnetometer XYZ (18-bit)
 *   - Reads temperature
 *   - Supports single-shot + continuous mode
 *   - Supports SET / RESET + Auto Set/Reset
 * - Compass helpers:
 *   - 2D heading (works when device is roughly level)
 *   - Tilt-compensated heading (requires external roll/pitch)
 *
 * Notes
 * - Roll/Pitch cannot be computed from magnetometer alone.
 * - For accurate heading when tilted, provide roll & pitch from an accelerometer/gyro fusion.
 */
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

/** - MMC5983MA Product ID expected value (REG_PRODUCTID should read this) */
#define MMC5983MA_EXPECTED_ID  0x30

/**
 * - Magnetometer scale factor in milli-Gauss per LSB for 18-bit output
 * - Used to convert raw signed counts to mG in readMagnetometer()
 */
#define MMC5983MA_LSB_mG  0.0625f

/** - Output register base address for X/Y/Z data block */
#define REG_XOUT0      0x00

/** - Packed XYZ upper bits register (used when reading 18-bit values) */
#define REG_XYZOUT2    0x06

/** - Temperature output register */
#define REG_TOUT       0x07

/** - Status register (measurement-ready flags) */
#define REG_STATUS     0x08

/** - Control register 0 (triggers + set/reset control) */
#define REG_CTRL0      0x09

/** - Control register 1 (reset + bandwidth, etc.) */
#define REG_CTRL1      0x0A

/** - Control register 2 (continuous mode + rate + ASR config) */
#define REG_CTRL2      0x0B

/** - Control register 3 (device-specific features, if used) */
#define REG_CTRL3      0x0C

/** - Product ID register */
#define REG_PRODUCTID  0x2F

class MMC5983MA_7Semi
{
public:
    /** - Default I2C address for MMC5983MA */
    static constexpr uint8_t DEFAULT_ADDRESS = 0x30;

    /**
     * - Axis mapping options for heading calculation
     * - Use these if your board orientation swaps or flips X/Y axes
     */
    enum AxisMap : uint8_t
    {
        /** - Heading = atan2(Y, X) */
        MAP_ATAN2_Y_X = 0,
        /** - Heading = atan2(X, Y) */
        MAP_ATAN2_X_Y,
        /** - Heading = atan2(-Y, X) */
        MAP_ATAN2_NEGY_X,
        /** - Heading = atan2(Y, -X) */
        MAP_ATAN2_Y_NEGX
    };

    /**
     * - Bandwidth selection for internal digital filtering
     * - Lower bandwidth typically reduces noise but increases response lag
     */
enum Bandwidth : uint8_t
{
    /** - Approx. 100 kHz output data bandwidth, Lowest noise filtering */
    BW_100KHZ = 0,

    /** - Approx. 200 kHz output data bandwidth,  Moderate noise filtering */
    BW_200KHZ,

    /** - Approx. 400 kHz output data bandwidth, Faster response, slightly higher noise */
    BW_400KHZ,

    /** - Approx. 800 kHz output data bandwidth, Maximum speed, highest noise */
    BW_800KHZ
};

    /**
     * - Continuous mode output rate selections
     * - Written into CTRL2 bits[2:0] when continuous is enabled
     */
    enum Frequency : uint8_t
    {
        /** - Continuous mode disabled (single-shot mode) */
        CM_OFF = 0,
        /** - Continuous mode at 1 Hz */
        CM_1Hz = 1,
        /** - Continuous mode at 10 Hz */
        CM_10Hz = 2,
        /** - Continuous mode at 20 Hz */
        CM_20HZ = 3,
        /** - Continuous mode at 50 Hz */
        CM_50HZ = 4,
        /** - Continuous mode at 100 Hz */
        CM_100HZ = 5,
        /** - Continuous mode at 200 Hz */
        CM_200HZ = 6,
        /** - Continuous mode at 1000 Hz */
        CM_1000HZ = 7
    };

    /**
     * - Auto Set/Reset periodicity selections
     * - Written into CTRL2 ASR bits[6:4] when Auto Set/Reset is enabled
     */
    enum autoSet : uint8_t
    {
        /** - Auto Set/Reset every 1 measurement */
        MES_1 = 0,
        /** - Auto Set/Reset every 25 measurements */
        MES_25 = 1,
        /** - Auto Set/Reset every 75 measurements */
        MES_75 = 2,
        /** - Auto Set/Reset every 100 measurements */
        MES_100 = 3,
        /** - Auto Set/Reset every 250 measurements */
        MES_250 = 4,
        /** - Auto Set/Reset every 500 measurements */
        MES_500 = 5,
        /** - Auto Set/Reset every 1000 measurements */
        MES_1000 = 6,
        /** - Auto Set/Reset every 2000 measurements */
        MES_2000 = 7
    };

    /** - Constructor (does not start bus; call beginI2C() or beginSPI()) */
    MMC5983MA_7Semi();

    /**
     * - Start the driver in I2C mode
     * - wire      : Wire instance (default Wire)
     * - address   : I2C address (default 0x30)
     * - clockHz   : I2C bus speed (default 400 kHz)
     * - i2cSDA/SCL: Optional custom pins for ESP32/ESP8266 (use 255 for default pins)
     */
    bool beginI2C(TwoWire& wire = Wire, uint8_t address = DEFAULT_ADDRESS, uint32_t clockHz = 400000, uint8_t i2cSDA = 255, uint8_t i2cSCL = 255);

    /**
     * - Start the driver in SPI mode (SPI Mode 3, MSB first)
     * - csPin   : Chip select pin
     * - clockHz : SPI clock speed (typical up to 10 MHz depending on platform/wiring)
     */
    bool beginSPI(SPIClass& spi = SPI, uint8_t csPin = -1, uint32_t clockHz = 10000000, uint8_t spiSCK = 255, uint8_t spiMOSI = 255, uint8_t spiMISO = 255);

    /**
     * - Common start routine after bus is selected
     * - Verifies product ID, applies reset, writes default config
     */
    bool begin();

    /** - Reads product ID from REG_PRODUCTID and validates expected value */
    bool getProductID(uint8_t &id);

    /** - Reads magnetometer in milli-Gauss (applies offset/scale and EMA filter if enabled) */
    bool readMagnetometer(float& x_mG, float& y_mG, float& z_mG);

    /** - Reads raw signed magnetometer counts (no scaling, centered around 0) */
    bool readMagnetometerRaw(int32_t& x, int32_t& y, int32_t& z);

    /** - Reads temperature in Celsius */
    bool readTemperature(float& tempC);

    /** - Triggers a single magnetometer measurement (TM_M) */
    bool initMagMeasurement();

    /** - Triggers a single temperature measurement (TM_T) */
    bool initTempMeasurement();

    /** - Reads STATUS flag for magnetometer data ready */
    bool isMagReady(bool& ready);

    /** - Reads STATUS flag for temperature data ready */
    bool isTempReady(bool& ready);

    /** - Tilt-compensated heading
     *  - roll_deg, pitch_deg are expected in degrees
     *  - declination_deg is added to heading (degrees)
     *  - Uses magnetometer X/Y/Z and compensates for roll/pitch
     *  - Returns heading in degrees, normalized to 0..360
     */
    float getHeadingTilt(float roll_deg, float pitch_deg, float declination_deg);


    /**
     * - Enable/disable continuous measurement mode
     * - freq selects the update rate when enabled
     */
    bool enableContinuousMode(bool enable, Frequency freq);

    /** - Set bandwidth option (written to CTRL1 BW bits) */
    bool setBandwidth(Bandwidth bw);

    /** - Software reset (CTRL1 SW_RST) */
    bool sensorReset();

    /**
     * - Enable/disable Auto Set/Reset (recommended for stability)
     * - as selects how often the device performs set/reset internally
     */
    bool enableAutoSetReset(bool enable, autoSet as = MES_100);

    /**
     * - Enable/disable periodic SET feature (En_prd_set)
     * - Datasheet requirement:
     *   - Auto_SR_en = 1
     *   - Cmm_en     = 1 (continuous mode enabled)
     * - period selects how often a SET operation occurs automatically
     */
    bool enablePeriodicSet(bool enable, autoSet period = MES_100);

    /**
     * - Enable/disable the INT pin for completed measurements (mag or temperature)
     * - CTRL0 bit2 = INT_meas_done_en
     */
    bool enableMeasurementDoneInterrupt(bool enable);

    /**
     * - Clear the measurement-done interrupt(s)
     * - Datasheet: writing '1' to STATUS Meas_*_Done clears the corresponding interrupt
     */
    bool clearMeasurementDoneInterrupt(bool clearMag = true, bool clearTemp = true);

    /**
     * - Put device into lowest-power idle
     * - Practical behavior:
     *   - Disables continuous mode and periodic set
     *   - Leaves the chip ready for single-shot measurements (TM_M / TM_T)
     */
    bool setPowerDown(bool enable = true);


    /** - Manual SET operation (self-clearing control bit) */
    bool opSet();

    /** - Manual RESET operation (self-clearing control bit) */
    bool opReset();

    /** - Enable/disable output smoothing on XYZ readings */
    void enableSmoothing(bool enable);

    /** - Returns true if output smoothing is enabled */
    bool isSmoothingEnabled() const;
    /** - Sets the axis mapping used for heading calculation */
    void setAxisMap(AxisMap map);

    /** - Sets hard-iron offsets in mG */
    void setOffset(float x_off_mG, float y_off_mG, float z_off_mG);

    /** - Sets soft-iron scale factors (1.0 = no scaling) */
    void setScale(float x_scale, float y_scale, float z_scale);

    /**
     * - 2D calibration helpers (best for level compass use)
     * - Call reset, then update repeatedly while rotating, then finish
     */
    void calibrationReset2D();
    void update2DCalibration();
    void finish2DCalibration();

    /** - Returns true when 2D calibration data has been finalized */
    bool calibrationReady2D() const;

    /**
     * - Enables background 3D auto-cal (best effort)
     * - Improves offsets/scales over time as the device moves through orientations
     */
    void enableAutoCal3D(bool enable);

    /** - Returns 2D heading in degrees [0..360), adds declination_deg */
    float getHeading2D(float declination_deg = 0.0f);
private:
    /** - Interface selection state for routing register I/O */
    enum class Bus : uint8_t { None, I2C, SPI };

    /**
     * - STATUS bits used by waitStatusBit()
     * - MEAS_M: magnetometer measurement complete
     * - MEAS_T: temperature measurement complete
     */
    enum StatusBit : uint8_t
    {
        MEAS_M = 0,
        MEAS_T = 1,
    };

    /** - Current active bus type */
    Bus bus = Bus::None;

    /** - I2C handle and device address */
    TwoWire* i2c = nullptr;
    uint8_t address = DEFAULT_ADDRESS;

    /** - SPI handle, chip select pin, and transaction settings */
    SPIClass* spi = nullptr;
    uint8_t cs_pin = 10;
    SPISettings spiSettings = SPISettings(10000000, MSBFIRST, SPI_MODE3);

    bool  magFiltInit = false;
    bool  headingInit = false;
    float fx = 0, fy = 0, fz = 0;
    float fHeading = 0;

    /** - Hard-iron offsets (mG) and soft-iron scale factors */
    float offsetX = 0, offsetY = 0, offsetZ = 0;
    float scaleX  = 1.0f, scaleY  = 1.0f, scaleZ  = 1.0f;

    /** - 2D calibration min/max capture and derived correction values */
    float minX =  1e9f, maxX = -1e9f;
    float minY =  1e9f, maxY = -1e9f;
    float calOffX = 0, calOffY = 0;
    float calScX  = 1.0f, calScY  = 1.0f;
    bool  calDone2D = false;

    /** - Auto 3D calibration state (min/max capture per axis) */
    bool  autoCal3D = false;
    float minX3 =  1e9f, maxX3 = -1e9f;
    float minY3 =  1e9f, maxY3 = -1e9f;
    float minZ3 =  1e9f, maxZ3 = -1e9f;

    /** - Selected axis mapping for heading calculation */
    AxisMap axisMap = MAP_ATAN2_Y_X;

    /**
     * - Internal smoothing state (per axis)
     * - x: estimated value
     * - p: estimation covariance
     * - q: response tuning
     * - r: noise tuning
     */
    struct Smooth1D
    {
        float x = 0.0f;
        float p = 1.0f;
        float q = 0.001f;
        float r = 0.02f;
        bool  init = false;
    };

    bool smoothEnable = true;

    Smooth1D sx;
    Smooth1D sy;
    Smooth1D sz;

private:
    /** - Single-register read (bus-routed) */
    bool readReg(uint8_t reg, uint8_t& value);

    /** - Single-register write (bus-routed) */
    bool writeReg(uint8_t reg, uint8_t value);

    /** - Multi-register read starting at startReg (bus-routed) */
    bool readNReg(uint8_t startReg, uint8_t* buf, size_t len);

    /** - Register bit set helper (read-modify-write) */
    bool setbit(uint8_t reg, uint8_t bit);

    /** - Register bit clear helper (read-modify-write) */
    bool clearbit(uint8_t reg, uint8_t bit);

    /** - Register bit read helper */
    bool getbit(uint8_t reg, uint8_t bit, bool& state);

    /** - I2C single-byte read helper (repeated-start style) */
    bool i2cRead(uint8_t reg, uint8_t &val);

    /** - I2C multi-byte read helper */
    bool i2cNRead(uint8_t reg, uint8_t* buf, size_t len);

    /** - I2C single-byte write helper */
    bool i2cWrite(uint8_t reg, uint8_t value);

    /** - SPI select (CS low + begin transaction) */
    void spiSelect();

    /** - SPI deselect (end transaction + CS high) */
    void spiDeselect();

    /** - SPI single-byte read helper */
    bool spiRead(uint8_t reg, uint8_t &val);

    /** - SPI multi-byte read helper */
    bool spiNRead(uint8_t reg, uint8_t* buf, size_t len);

    /** - SPI single-byte write helper */
    bool spiWrite(uint8_t reg, uint8_t value);

    /** - Polls a STATUS bit until ready or timeout */
    bool waitStatusBit(uint8_t bit, uint32_t timeoutMs);

    /** - Wraps degrees into [0..360) */
    static float wrap360(float deg);

    /** - Smallest signed angle difference in degrees [-180..180] */
    static float angleDiffDeg(float a, float b);

    /** - Applies axis mapping and returns heading angle in degrees (not wrapped) */
    float applyAxisMapHeading(float x, float y) const;

    /** - Updates best-effort auto 3D calibration state */
    void autoCalUpdate3D(float mx, float my, float mz);

    /** - Internal smoothing update */
    float smoothUpdate(Smooth1D &s, float measurement);
};
