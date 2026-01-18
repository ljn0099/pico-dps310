#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sensor/dps310.h"
#include <stdio.h>
#include <string.h>

bool dps310_init_struct(dps310_t *dps310, uint8_t i2cAddress, i2c_inst_t *i2c) {
    if (!dps310)
        return false;
    dps310->i2cAddress = i2cAddress;
    dps310->i2c = i2c;

    dps310->tempCoef.read = false;
    dps310->presCoef.read = false;

    dps310->tempOversampleSf = DPS310_OVERSAMPLING_1_TIME_SF;
    dps310->presOversampleSf = DPS310_OVERSAMPLING_1_TIME_SF;

    dps310->lastTempRawSc = 0;
    dps310->lastTempInit = false;

    return true;
}

bool dps310_correct_temp(const dps310_t *dps310) {
    if (!dps310)
        return false;

    if (!dps310_write_register(dps310, 0x0E, 0xA5))
        return false;
    if (!dps310_write_register(dps310, 0x0F, 0x96))
        return false;
    if (!dps310_write_register(dps310, 0x62, 0x02))
        return false;
    if (!dps310_write_register(dps310, 0x0E, 0x00))
        return false;
    if (!dps310_write_register(dps310, 0x0F, 0x00))
        return false;

    return true;
}

bool dps310_read_temp(dps310_t *dps310, float *temp) {
    if (!dps310 || !dps310->tempCoef.read)
        return false;

    int32_t tempRaw;

    float tempRawSc;

    uint8_t bufferTemp[DPS310_TEMP_REG_QNTY];

    // Read raw temperature data
    if (!dps310_read_block(dps310, DPS310_TEMP_HB, bufferTemp, DPS310_TEMP_REG_QNTY))
        return false;

    // Convert 3 bytes to signed 24-bit raw values
    tempRaw = dps310_read_s24(&bufferTemp[DPS310_TEMP_HB_I]);

    // Scale raw values
    tempRawSc = (float)tempRaw / (float)dps310->tempOversampleSf;

    // Store the measurement into the struct for the pressure calculation
    dps310->lastTempRawSc = tempRawSc;
    dps310->lastTempInit = true;

    // Apply compensation
    if (temp) {
        float tempComp = dps310_compensate_temp(dps310, tempRawSc);
        *temp = tempComp;
    }

    return true;
}

bool dps310_read_pres(dps310_t *dps310, float *pres, bool readNewTemp) {
    if (!dps310 || !dps310->presCoef.read)
        return false;

    // Check if a temp meas was made
    if (readNewTemp || !dps310->lastTempInit) {
        if (!dps310_read_temp(dps310, NULL))
            return false;
    }

    int32_t presRaw;

    float presRawSc;

    uint8_t bufferPres[DPS310_PRES_REG_QNTY];

    // Read raw pressure and temperature data
    if (!dps310_read_block(dps310, DPS310_PRES_HB, bufferPres, DPS310_PRES_REG_QNTY))
        return false;

    // Convert 3 bytes to signed 24-bit raw values
    presRaw = dps310_read_s24(&bufferPres[DPS310_PRES_HB_I]);

    // Scale raw values
    presRawSc = (float)presRaw / (float)dps310->presOversampleSf;

    // Apply compensation
    if (pres) {
        float presComp = dps310_compensate_pres(dps310, presRawSc, dps310->lastTempRawSc);
        *pres = presComp;
    }

    return true;
}

bool dps310_fifo_read(dps310_t *dps310, float *value, dps310_MeasType_t *measType) {
    if (!dps310 || !dps310->presCoef.read || !dps310->tempCoef.read)
        return false;

    uint8_t buffer[DPS310_PRES_REG_QNTY];
    int32_t rawValue;

    // The fifo measurement are always in the pressure registers
    if (!dps310_read_block(dps310, DPS310_PRES_HB, buffer, DPS310_PRES_REG_QNTY))
        return false;

    rawValue = dps310_read_s24(buffer);

    if (rawValue == DPS310_FIFO_EMPTY_VALUE_SIGNED)
        return false;

    // Presure measurement
    if (buffer[DPS310_PRES_LB_I] & DPS310_MEAS_TYPE_MASK) {
        // Without any temperature measurement we cant calculate pressure
        if (!dps310->lastTempInit)
            return false;

        float presRawSc = (float)rawValue / (float)dps310->presOversampleSf;
        float presComp = dps310_compensate_pres(dps310, presRawSc, dps310->lastTempRawSc);
        if (value)
            *value = presComp;
        if (measType)
            *measType = DPS310_MEAS_TYPE_PRES;
    }
    // Temperature measurement
    else {
        float tempRawSc = (float)rawValue / (float)dps310->tempOversampleSf;

        dps310->lastTempRawSc = tempRawSc;
        dps310->lastTempInit = true;

        float tempComp = dps310_compensate_temp(dps310, tempRawSc);
        if (value)
            *value = tempComp;
        if (measType)
            *measType = DPS310_MEAS_TYPE_TEMP;
    }

    return true;
}

bool dps310_read_pres_calibration(dps310_t *dps310) {
    if (!dps310)
        return false;

    if (dps310->presCoef.read)
        return true;

    uint8_t buffer[DPS310_PRES_COEF_REG_QNTY];

    if (!dps310_read_block(dps310, DPS310_C00_HB, buffer, DPS310_PRES_COEF_REG_QNTY))
        return false;

    uint32_t c00, c10;
    uint16_t c01, c11, c20, c21, c30;

    c00 = ((uint32_t)buffer[DPS310_C00_HB_I] << 12);
    c00 |= ((uint32_t)buffer[DPS310_C00_MB_I] << 4);
    c00 |= ((buffer[DPS310_C00_LB_C10_HB_I] & DPS310_C00_LB_MASK) >> 4);
    c00 = dps310_sign_extend(c00, 20);

    c10 = ((uint32_t)buffer[DPS310_C00_LB_C10_HB_I] & DPS310_C10_HB_MASK) << 16;
    c10 |= ((uint32_t)buffer[DPS310_C10_MB_I] << 8);
    c10 |= buffer[DPS310_C10_LB_I];
    c10 = dps310_sign_extend(c10, 20);

    c01 = ((uint16_t)buffer[DPS310_C01_HB_I] << 8);
    c01 |= buffer[DPS310_C01_LB_I];

    c11 = ((uint16_t)buffer[DPS310_C11_HB_I] << 8);
    c11 |= buffer[DPS310_C11_LB_I];

    c20 = ((uint16_t)buffer[DPS310_C20_HB_I] << 8);
    c20 |= buffer[DPS310_C20_LB_I];

    c21 = ((uint16_t)buffer[DPS310_C21_HB_I] << 8);
    c21 |= buffer[DPS310_C21_LB_I];

    c30 = ((uint16_t)buffer[DPS310_C30_HB_I] << 8);
    c30 |= buffer[DPS310_C30_LB_I];

    dps310->presCoef.c00 = c00;
    dps310->presCoef.c10 = c10;
    dps310->presCoef.c01 = c01;
    dps310->presCoef.c11 = c11;
    dps310->presCoef.c20 = c20;
    dps310->presCoef.c30 = c30;
    dps310->presCoef.read = true;

    return true;
}

bool dps310_read_temp_calibration(dps310_t *dps310) {
    if (!dps310)
        return false;

    if (dps310->tempCoef.read)
        return true;

    uint8_t buffer[DPS310_TEMP_COEF_REG_QNTY];

    if (!dps310_read_block(dps310, DPS310_C0_HB, buffer, DPS310_TEMP_COEF_REG_QNTY))
        return false;

    uint16_t c0, c1;

    c0 = ((uint16_t)buffer[DPS310_C0_HB_I] << 4);
    c0 |= ((buffer[DPS310_C0_LB_C1_HB_I] & DPS310_C0_LB_MASK) >> 4);

    c1 = ((uint16_t)buffer[DPS310_C0_LB_C1_HB_I] & DPS310_C1_HB_MASK) << 8;
    c1 |= (buffer[DPS310_C1_LB_I]);

    c0 = dps310_sign_extend(c0, 12);

    c1 = dps310_sign_extend(c1, 12);

    dps310->tempCoef.c0Half = c0 / 2.0f;
    dps310->tempCoef.c1 = c1;
    dps310->tempCoef.read = true;

    return true;
}

// ---- Meas Mode
bool dps310_set_meas_mode(const dps310_t *dps310, dps310_Mode_t mode) {
    if (!dps310)
        return false;

    // Set idle before changing the mode
    if (!dps310_set_mask(dps310, DPS310_CFG_MEAS_REG, DPS310_MEAS_MODE_MASK, DPS310_MODE_IDLE))
        return false;

    if (!dps310_set_mask(dps310, DPS310_CFG_MEAS_REG, DPS310_MEAS_MODE_MASK, mode))
        return false;

    return true;
}

bool dps310_read_meas_mode(const dps310_t *dps310, dps310_Mode_t *mode) {
    if (!dps310)
        return false;

    uint8_t buffer;
    if (!dps310_read_mask(dps310, DPS310_CFG_MEAS_REG, DPS310_MEAS_MODE_MASK, &buffer))
        return false;

    *mode = (dps310_Mode_t)buffer;

    return true;
}

// ---- Pressure Meas Rate
bool dps310_set_pres_meas_rate(const dps310_t *dps310, dps310_MeasRate_t measRate) {
    if (!dps310)
        return false;

    if (!dps310_set_mask(dps310, DPS310_CFG_PRES_REG, DPS310_PRES_MEAS_RATE_MASK, measRate))
        return false;

    return true;
}

bool dps310_read_pres_meas_rate(const dps310_t *dps310, dps310_MeasRate_t *measRate) {
    if (!dps310 || !measRate)
        return false;

    uint8_t buffer;
    if (!dps310_read_mask(dps310, DPS310_CFG_PRES_REG, DPS310_PRES_MEAS_RATE_MASK, &buffer))
        return false;

    *measRate = (dps310_MeasRate_t)buffer;

    return true;
}

// ---- Pressure Oversampling
bool dps310_set_pres_oversampling(dps310_t *dps310, dps310_Oversampling_t oversample) {
    if (!dps310)
        return false;

    if (!dps310_set_mask(dps310, DPS310_CFG_PRES_REG, DPS310_PRES_OVERSAMPLING_MASK, oversample))
        return false;

    bool shift = (oversample > DPS310_OVERSAMPLING_8_TIMES);
    if (!dps310_pres_bit_shift(dps310, shift))
        return false;

    dps310->presOversampleSf = oversampling_to_sf(oversample);

    return true;
}

bool dps310_read_pres_oversampling(const dps310_t *dps310, dps310_Oversampling_t *oversample) {
    if (!dps310 || !oversample)
        return false;

    uint8_t buffer;
    if (!dps310_read_mask(dps310, DPS310_CFG_PRES_REG, DPS310_PRES_OVERSAMPLING_MASK, &buffer))
        return false;

    *oversample = (dps310_Oversampling_t)buffer;

    return true;
}

// ---- Temperature Meas Rate
bool dps310_set_temp_meas_rate(const dps310_t *dps310, dps310_MeasRate_t measRate) {
    if (!dps310)
        return false;

    if (!dps310_set_mask(dps310, DPS310_CFG_TEMP_REG, DPS310_TEMP_MEAS_RATE_MASK, measRate))
        return false;

    return true;
}

bool dps310_read_temp_meas_rate(const dps310_t *dps310, dps310_MeasRate_t *measRate) {
    if (!dps310 || !measRate)
        return false;

    uint8_t buffer;
    if (!dps310_read_mask(dps310, DPS310_CFG_TEMP_REG, DPS310_TEMP_MEAS_RATE_MASK, &buffer))
        return false;

    *measRate = (dps310_MeasRate_t)buffer;

    return true;
}

// ---- Temperature Oversampling
bool dps310_set_temp_oversampling(dps310_t *dps310, dps310_Oversampling_t oversample) {
    if (!dps310)
        return false;

    if (!dps310_set_mask(dps310, DPS310_CFG_TEMP_REG, DPS310_TEMP_OVERSAMPLING_MASK, oversample))
        return false;

    bool shift = (oversample > DPS310_OVERSAMPLING_8_TIMES);
    if (!dps310_temp_bit_shift(dps310, shift))
        return false;

    dps310->tempOversampleSf = oversampling_to_sf(oversample);

    return true;
}

bool dps310_read_temp_oversampling(const dps310_t *dps310, dps310_Oversampling_t *oversample) {
    if (!dps310 || !oversample)
        return false;

    uint8_t buffer;
    if (!dps310_read_mask(dps310, DPS310_CFG_TEMP_REG, DPS310_TEMP_OVERSAMPLING_MASK, &buffer))
        return false;

    *oversample = (dps310_Oversampling_t)buffer;

    return true;
}

// ---- Sensor status
bool dps310_read_status(const dps310_t *dps310, bool *coefReady, bool *sensorReady) {
    if (!dps310)
        return false;

    uint8_t buffer;
    if (!dps310_read_register(dps310, DPS310_CFG_MEAS_REG, &buffer))
        return false;

    if (coefReady) {
        if (buffer & DPS310_MEAS_COEF_READY_MASK)
            *coefReady = true;
        else
            *coefReady = false;
    }

    if (sensorReady) {
        if (buffer & DPS310_MEAS_SENSOR_READY_MASK)
            *sensorReady = true;
        else
            *sensorReady = false;
    }

    return true;
}

bool dps310_wait_until_ready(const dps310_t *dps310, bool waitCoef, bool waitSensor) {
    if (!dps310)
        return false;

    uint32_t elapsed = 0;

    while (elapsed < 1000) { // Timer timeout to 1s
        bool coefReady = false;
        bool sensorReady = false;

        if (!dps310_read_status(dps310, &coefReady, &sensorReady))
            return false;

        if ((!waitCoef || coefReady) && (!waitSensor || sensorReady))
            return true;

        sleep_ms(5); // Wait 5 ms
        elapsed += 5;
    }

    return false;
}

bool dps310_read_meas_status(const dps310_t *dps310, bool *tempReady, bool *presReady) {
    if (!dps310)
        return false;

    uint8_t buffer;
    if (!dps310_read_register(dps310, DPS310_CFG_MEAS_REG, &buffer))
        return false;

    if (tempReady) {
        if (buffer & DPS310_MEAS_TEMP_MEAS_READY_MASK)
            *tempReady = true;
        else
            *tempReady = false;
    }

    if (presReady) {
        if (buffer & DPS310_MEAS_PRES_MEAS_READY_MASK)
            *presReady = true;
        else
            *presReady = false;
    }

    return true;
}

bool dps310_wait_until_meas_ready(const dps310_t *dps310, bool waitTemp, bool waitPres) {
    if (!dps310)
        return false;

    uint32_t elapsed = 0;

    while (elapsed < 1000) { // Timeout 1s
        bool tempReady = false;
        bool presReady = false;

        if (!dps310_read_meas_status(dps310, &tempReady, &presReady))
            return false;

        if ((!waitTemp || tempReady) && (!waitPres || presReady))
            return true;

        sleep_ms(5); // Wait 5 ms
        elapsed += 5;
    }

    return false;
}

bool dps310_read_fifo_status(const dps310_t *dps310, bool *fifoFull, bool *fifoEmpty) {
    if (!dps310)
        return false;

    uint8_t buffer;
    if (!dps310_read_register(dps310, DPS310_FIFO_STATUS_REG, &buffer))
        return false;

    if (fifoFull) {
        if (buffer & DPS310_FIFO_STATUS_FULL_MASK)
            *fifoFull = true;
        else
            *fifoFull = false;
    }
    
    if (fifoEmpty) {
        if (buffer & DPS310_FIFO_STATUS_EMPTY_MASK)
            *fifoEmpty = true;
        else
            *fifoEmpty = false;
    }

    return true;
}

// ----- Interrupts
bool dps310_set_interrupt_source(const dps310_t *dps310, dps310_IntSrcMasks_t intSrc, bool enable) {
    if (!dps310)
        return false;

    if (!dps310_set_bit(dps310, DPS310_CFG_REG, intSrc, enable))
        return false;

    return true;
}

bool dps310_read_interrupt_source(const dps310_t *dps310, dps310_IntSrcMasks_t intSrc, bool *enable) {
    if (!dps310 || !enable)
        return false;

    if (!dps310_read_bit(dps310, DPS310_CFG_REG, intSrc, enable))
        return false;

    return true;
}

// This register is cleared on read
bool dps310_read_interrupt_status(const dps310_t *dps310, bool *fifoFull, bool *tempMeas, bool *presMeas) {
    if (!dps310)
        return false;

    uint8_t buffer;
    if (!dps310_read_register(dps310, DPS310_INT_STATUS_REG, &buffer))
        return false;

    if (fifoFull) {
        if (buffer & DPS310_INT_STATUS_FIFO_FULL_MASK)
            *fifoFull = true;
        else
            *fifoFull = false;
    }

    if (tempMeas) {
        if (buffer & DPS310_INT_STATUS_TEMP_MASK)
            *tempMeas = true;
        else
            *tempMeas = false;
    }

    if (presMeas) {
        if (buffer & DPS310_INT_STATUS_PRES_MASK)
            *presMeas = true;
        else
            *presMeas = false;
    }

    return true;
}

// ----- FIFO
bool dps310_enable_fifo(const dps310_t *dps310, bool enable) {
    if (!dps310)
        return false;

    if (!dps310_set_bit(dps310, DPS310_CFG_REG, DPS310_CFG_ENABLE_FIFO_MASK, enable))
        return false;

    return true;
}

bool dps310_is_fifo_enabled(const dps310_t *dps310, bool *enabled) {
    if (!dps310 || !enabled)
        return false;

    if (!dps310_read_bit(dps310, DPS310_CFG_REG, DPS310_CFG_ENABLE_FIFO_MASK, enabled))
        return false;

    return true;
}

bool dps310_fifo_flush(const dps310_t *dps310) {
    if (!dps310)
        return false;

    if (!dps310_write_register(dps310, DPS310_RESET_REG, DPS310_RESET_FIFO_FLUSH_MASK))
        return false;

    return true;
}

// ------ Shifting
// Oversampling shifting, the change oversample functions call this automatically
bool dps310_pres_bit_shift(const dps310_t *dps310, bool shift) {
    if (!dps310)
        return false;

    if (!dps310_set_bit(dps310, DPS310_CFG_REG, DPS310_CFG_PRES_SHIFT_MASK, shift))
        return false;

    return true;
}

bool dps310_temp_bit_shift(const dps310_t *dps310, bool shift) {
    if (!dps310)
        return false;

    if (!dps310_set_bit(dps310, DPS310_CFG_REG, DPS310_CFG_TEMP_SHIFT_MASK, shift))
        return false;

    return true;
}

// ----- Temperature sensors used
bool dps310_set_temp_meas_sensor(const dps310_t *dps310, dps310_TempSrc_t source) {
    if (!dps310)
        return false;

    if (!dps310_set_mask(dps310, DPS310_CFG_TEMP_REG, DPS310_TEMP_SENSOR_MASK, source))
        return false;

    return true;
}

bool dps310_read_temp_meas_sensor(const dps310_t *dps310, dps310_TempSrc_t *source) {
    if (!dps310 || !source)
        return false;

    uint8_t buffer;
    if (!dps310_read_mask(dps310, DPS310_CFG_TEMP_REG, DPS310_TEMP_SENSOR_MASK, &buffer))
        return false;

    if (buffer & DPS310_TEMP_COEF_SRC_MASK)
        *source = DPS310_TEMP_COEF_SRC_MEMS;
    else
        *source = DPS310_TEMP_COEF_SRC_ASIC;

    return true;
}

// ------- Read for what temperature sensor are the calibrations coeficients
bool dps310_read_temp_coef_source(const dps310_t *dps310, dps310_TempSrc_t *source) {
    if (!dps310)
        return false;

    uint8_t buffer;
    if (!dps310_read_register(dps310, DPS310_TEMP_COEF_SRC_REG, &buffer))
        return false;

    if (buffer & DPS310_TEMP_COEF_SRC_MASK)
        *source = DPS310_TEMP_COEF_SRC_MEMS;
    else
        *source = DPS310_TEMP_COEF_SRC_ASIC;

    return true;
}

bool dps310_auto_set_temp_source(const dps310_t *dps310) {
    dps310_TempSrc_t source;

    if (!dps310_read_temp_coef_source(dps310, &source))
        return false;
    if (!dps310_set_temp_meas_sensor(dps310, source))
        return false;

    return true;
}

// ----- Id
bool dps310_read_full_id(const dps310_t *dps310, uint8_t *id) {
    if (!dps310 || !id)
        return false;

    uint8_t buffer;
    if (!dps310_read_register(dps310, DPS310_ID_REG, &buffer))
        return false;

    *id = buffer;

    return true;
}

bool dps310_read_id(const dps310_t *dps310, uint8_t *revId, uint8_t *productId) {
    if (!dps310)
        return false;

    uint8_t buffer;
    if (!dps310_read_register(dps310, DPS310_ID_REG, &buffer))
        return false;

    if (productId)
        *productId = (buffer & DPS310_ID_PRODUCT_MASK) >> 4;
    if (revId)
        *revId = (buffer & DPS310_ID_REV_MASK);

    return true;
}

// ----- Soft reset
bool dps310_soft_reset(dps310_t *dps310) {
    if (!dps310)
        return false;

    if (!dps310_write_register(dps310, DPS310_RESET_REG, DPS310_RESET_COMMAND))
        return false;

    dps310->tempOversampleSf = DPS310_OVERSAMPLING_1_TIME_SF;
    dps310->presOversampleSf = DPS310_OVERSAMPLING_1_TIME_SF;

    dps310->lastTempRawSc = 0;
    dps310->lastTempInit = false;

    sleep_ms(20);
    
    return true;
}

// ----- Internal
bool dps310_write_register(const dps310_t *dps310, uint8_t reg, uint8_t data) {
    if (!dps310)
        return false;

    uint8_t buffer[2] = {reg, data};
    if (i2c_write_blocking(dps310->i2c, dps310->i2cAddress, buffer, 2, false) != 2)
        return false;

    return true;
}

bool dps310_read_register(const dps310_t *dps310, uint8_t reg, uint8_t *data) {
    if (!dps310 || !data)
        return false;

    uint8_t buffer;

    if (i2c_write_blocking(dps310->i2c, dps310->i2cAddress, &reg, 1, true) != 1)
        return false;

    if (i2c_read_blocking(dps310->i2c, dps310->i2cAddress, &buffer, 1, false) != 1)
        return false;

    *data = buffer;

    return true;
}

bool dps310_write_block(const dps310_t *dps310, uint8_t startReg, uint8_t data[], size_t len) {
    if (!dps310 || !data)
        return false;

    uint8_t buffer[len + 1];
    buffer[0] = startReg;
    memcpy(&buffer[1], data, len);

    if (i2c_write_blocking(dps310->i2c, dps310->i2cAddress, buffer, len + 1, false) != len + 1)
        return false;

    return true;
}

bool dps310_read_block(const dps310_t *dps310, uint8_t startReg, uint8_t data[], size_t len) {
    if (!dps310 || !data)
        return false;

    uint8_t buffer[len];

    if (i2c_write_blocking(dps310->i2c, dps310->i2cAddress, &startReg, 1, true) != 1)
        return false;

    if (i2c_read_blocking(dps310->i2c, dps310->i2cAddress, buffer, len, false) != len)
        return false;

    memcpy(data, buffer, len);

    return true;
}

static inline bool dps310_set_mask(const dps310_t *dps310, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t buffer;

    if (!dps310_read_register(dps310, reg, &buffer))
        return false;

    buffer &= ~mask;
    buffer |= (value & mask);

    if (!dps310_write_register(dps310, reg, buffer))
        return false;

    return true;
}

static inline bool dps310_read_mask(const dps310_t *dps310, uint8_t reg, uint8_t mask, uint8_t *value) {
    uint8_t buffer;

    if (!dps310_read_register(dps310, reg, &buffer))
        return false;

    *value = buffer & mask;
    return true;
}

static inline bool dps310_set_bit(const dps310_t *dps310, uint8_t reg, uint8_t mask, bool value) {
    uint8_t buffer;

    if (!dps310_read_register(dps310, reg, &buffer))
        return false;

    buffer &= ~mask;
    if (value)
        buffer |= mask;

    if (!dps310_write_register(dps310, reg, buffer))
        return false;

    return true;
}

static inline bool dps310_read_bit(const dps310_t *dps310, uint8_t reg, uint8_t mask, bool *value) {
    uint8_t buffer;

    if (!dps310_read_register(dps310, reg, &buffer))
        return false;

    if (buffer & mask)
        *value = true;
    else
        *value = false;

    return true;
}

static inline int32_t dps310_read_s24(const uint8_t *buf) {
    uint32_t raw;

    raw = ((uint32_t)buf[0] << 16);
    raw |= ((uint32_t)buf[1] << 8);
    raw |= ((uint32_t)buf[2]);

    raw = dps310_sign_extend(raw, 24);

    return (int32_t)raw;
}

static inline float dps310_compensate_pres(const dps310_t *dps310, float presRawSc,
                                           float tempRawSc) {
    return dps310->presCoef.c00 +
           presRawSc * (dps310->presCoef.c10 +
                        presRawSc * (dps310->presCoef.c20 + presRawSc * dps310->presCoef.c30)) +
           tempRawSc * dps310->presCoef.c01 +
           tempRawSc * presRawSc * (dps310->presCoef.c11 + presRawSc * dps310->presCoef.c21);
}

static inline float dps310_compensate_temp(const dps310_t *dps310, float tempRawSc) {
    return dps310->tempCoef.c0Half + dps310->tempCoef.c1 * tempRawSc;
}

static inline dps310_OversamplingSf_t oversampling_to_sf(dps310_Oversampling_t oversample) {
    switch (oversample) {
        case DPS310_OVERSAMPLING_1_TIME:
            return DPS310_OVERSAMPLING_1_TIME_SF;
        case DPS310_OVERSAMPLING_2_TIMES:
            return DPS310_OVERSAMPLING_2_TIMES_SF;
        case DPS310_OVERSAMPLING_4_TIMES:
            return DPS310_OVERSAMPLING_4_TIMES_SF;
        case DPS310_OVERSAMPLING_8_TIMES:
            return DPS310_OVERSAMPLING_8_TIMES_SF;
        case DPS310_OVERSAMPLING_16_TIMES:
            return DPS310_OVERSAMPLING_16_TIMES_SF;
        case DPS310_OVERSAMPLING_32_TIMES:
            return DPS310_OVERSAMPLING_32_TIMES_SF;
        case DPS310_OVERSAMPLING_64_TIMES:
            return DPS310_OVERSAMPLING_64_TIMES_SF;
        case DPS310_OVERSAMPLING_128_TIMES:
            return DPS310_OVERSAMPLING_128_TIMES_SF;
        default:
            // Return a safe default value if an invalid oversampling is passed
            return DPS310_OVERSAMPLING_1_TIME_SF;
    }
}

static inline int dps310_sign_extend(int value, int bits) {
    if (value & (1 << (bits - 1)))
        value -= (1 << bits);
    return value;
}
