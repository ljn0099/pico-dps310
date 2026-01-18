/**
 * @file dps310.h
 * @brief DPS310 I2C Library for the Raspberry Pi pico-sdk
 *
 * @author ljn0099
 *
 * @license MIT License
 * Copyright (c) 2025 ljn0099
 *
 * See LICENSE file for details.
 */
#ifndef DPS310_H
#define DPS310_H

#include "hardware/i2c.h"

#define DPS310_PRES_HB 0x00
#define DPS310_PRES_MB 0x01
#define DPS310_PRES_LB 0x02
#define DPS310_PRES_REG_QNTY 3

#define DPS310_MEAS_TYPE_MASK (1 << 0)

typedef enum {
    DPS310_PRES_HB_I = 0,
    DPS310_PRES_MB_I = 1,
    DPS310_PRES_LB_I = 2
} dps310_PresIndex_t;

#define DPS310_FIFO_EMPTY_VALUE 0x800000
#define DPS310_FIFO_EMPTY_VALUE_SIGNED (-8388608) // 0x800000 sign-extended

#define DPS310_TEMP_HB 0x03
#define DPS310_TEMP_MB 0x04
#define DPS310_TEMP_LB 0x05
#define DPS310_TEMP_REG_QNTY 3

typedef enum {
    DPS310_TEMP_HB_I = 0,
    DPS310_TEMP_MB_I = 1,
    DPS310_TEMP_LB_I = 2
} dps310_TempIndex_t;

#define DPS310_CFG_PRES_REG 0x06
#define DPS310_CFG_TEMP_REG 0x07
#define DPS310_CFG_MEAS_REG 0x08
#define DPS310_CFG_REG 0x09

#define DPS310_INT_STATUS_REG 0x0A
#define DPS310_FIFO_STATUS_REG 0x0B

#define DPS310_RESET_REG 0x0C
#define DPS310_RESET_COMMAND 0b00001001

#define DPS310_ID_REG 0x0D

// Temperature coeficients
#define DPS310_TEMP_COEF_REG_QNTY 3
#define DPS310_C0_HB 0x10
#define DPS310_C0_LB_C1_HB 0x11
#define DPS310_C1_LB 0x12

typedef enum {
    DPS310_C0_HB_I = 0,
    DPS310_C0_LB_C1_HB_I = 1,
    DPS310_C1_LB_I = 2
} dps310_TempCoefIndex_t;

// Pressure coeficients
#define DPS310_PRES_COEF_REG_QNTY 15
#define DPS310_C00_HB 0x13
#define DPS310_C00_MB 0x14
#define DPS310_C00_LB_C10_HB 0x15
#define DPS310_C10_MB 0x16
#define DPS310_C10_LB 0x17
#define DPS310_C01_HB 0x18
#define DPS310_C01_LB 0x19
#define DPS310_C11_HB 0x1A
#define DPS310_C11_LB 0x1B
#define DPS310_C20_HB 0x1C
#define DPS310_C20_LB 0x1D
#define DPS310_C21_HB 0x1E
#define DPS310_C21_LB 0x1F
#define DPS310_C30_HB 0x20
#define DPS310_C30_LB 0x21

typedef enum {
    DPS310_C00_HB_I = 0,
    DPS310_C00_MB_I = 1,
    DPS310_C00_LB_C10_HB_I = 2,
    DPS310_C10_MB_I = 3,
    DPS310_C10_LB_I = 4,
    DPS310_C01_HB_I = 5,
    DPS310_C01_LB_I = 6,
    DPS310_C11_HB_I = 7,
    DPS310_C11_LB_I = 8,
    DPS310_C20_HB_I = 9,
    DPS310_C20_LB_I = 10,
    DPS310_C21_HB_I = 11,
    DPS310_C21_LB_I = 12,
    DPS310_C30_HB_I = 13,
    DPS310_C30_LB_I = 14
} dps310_PresCoefIndex_t;

#define DPS310_TEMP_COEF_SRC_REG 0x28

typedef enum {
    DPS310_TEMP_COEF_SRC_ASIC = (0 << 7),
    DPS310_TEMP_COEF_SRC_MEMS = (1 << 7)
} dps310_TempSrc_t;

#define DPS310_C0_LB_MASK 0b11110000
#define DPS310_C1_HB_MASK 0b00001111
#define DPS310_C00_LB_MASK 0b11110000
#define DPS310_C10_HB_MASK 0b00001111

typedef enum {
    DPS310_PRES_MEAS_RATE_MASK = 0b01110000,
    DPS310_PRES_OVERSAMPLING_MASK = 0b00000111
} dps310_PresMasks_t;

typedef enum {
    DPS310_TEMP_SENSOR_MASK = (1 << 7),
    DPS310_TEMP_MEAS_RATE_MASK = 0b01110000,
    DPS310_TEMP_OVERSAMPLING_MASK = 0b00000111
} dps310_TempMasks_t;

typedef enum {
    DPS310_MEAS_COEF_READY_MASK = (1 << 7),
    DPS310_MEAS_SENSOR_READY_MASK = (1 << 6),
    DPS310_MEAS_TEMP_MEAS_READY_MASK = (1 << 5),
    DPS310_MEAS_PRES_MEAS_READY_MASK = (1 << 4),
    DPS310_MEAS_MODE_MASK = 0b00000111
} dps310_MeasMasks_t;

typedef enum {
    DPS310_CFG_INT_LOGIC_LEVEL_MASK = (1 << 7),
    DPS310_CFG_INT_FIFO_FULL_MASK = (1 << 6),
    DPS310_CFG_INT_NEW_TEMP_MEAS_MASK = (1 << 5),
    DPS310_CFG_INT_NEW_PRES_MEAS_MASK = (1 << 4),
    DPS310_CFG_TEMP_SHIFT_MASK = (1 << 3),
    DPS310_CFG_PRES_SHIFT_MASK = (1 << 2),
    DPS310_CFG_ENABLE_FIFO_MASK = (1 << 1),
    DPS310_CFG_SPI_MODE_MASK = (1 << 0)
} dps310_CfgMasks_t;

typedef enum {
    DPS310_INT_SRC_FIFO_FULL_MASK = (1 << 6),
    DPS310_INT_SRC_TEMP_READY_MASK = (1 << 5),
    DPS310_INT_SRC_PRES_READY_MASK = (1 << 4)
} dps310_IntSrcMasks_t;

typedef enum {
    DPS310_INT_STATUS_FIFO_FULL_MASK = (1 << 2),
    DPS310_INT_STATUS_TEMP_MASK = (1 << 1),
    DPS310_INT_STATUS_PRES_MASK = (1 << 0)
} dps310_IntStatusMasks_t;

typedef enum {
    DPS310_FIFO_STATUS_FULL_MASK = (1 << 1),
    DPS310_FIFO_STATUS_EMPTY_MASK = (1 << 0)
} dps310_FifoStatusMasks_t;

typedef enum {
    DPS310_RESET_FIFO_FLUSH_MASK = (1 << 7),
    DPS310_RESET_SOFT_MASK = 0b00001111
} dps310_ResetMasks_t;

typedef enum {
    DPS310_ID_REV_MASK = 0b11110000,
    DPS310_ID_PRODUCT_MASK = 0b00001111
} dps310_IdMasks_t;

typedef enum {
    DPS310_MEAS_PER_SEC_1 = (0 << 4),
    DPS310_MEAS_PER_SEC_2 = (1 << 4),
    DPS310_MEAS_PER_SEC_4 = (2 << 4),
    DPS310_MEAS_PER_SEC_8 = (3 << 4),
    DPS310_MEAS_PER_SEC_16 = (4 << 4),
    DPS310_MEAS_PER_SEC_32 = (5 << 4),
    DPS310_MEAS_PER_SEC_64 = (6 << 4),
    DPS310_MEAS_PER_SEC_128 = (7 << 4),
} dps310_MeasRate_t;

typedef enum {
    DPS310_OVERSAMPLING_1_TIME = 0,
    DPS310_OVERSAMPLING_2_TIMES = 1,
    DPS310_OVERSAMPLING_4_TIMES = 2,
    DPS310_OVERSAMPLING_8_TIMES = 3,
    DPS310_OVERSAMPLING_16_TIMES = 4,
    DPS310_OVERSAMPLING_32_TIMES = 5,
    DPS310_OVERSAMPLING_64_TIMES = 6,
    DPS310_OVERSAMPLING_128_TIMES = 7,
} dps310_Oversampling_t;

typedef enum {
    DPS310_OVERSAMPLING_1_TIME_SF = 524288,
    DPS310_OVERSAMPLING_2_TIMES_SF = 1572864,
    DPS310_OVERSAMPLING_4_TIMES_SF = 3670016,
    DPS310_OVERSAMPLING_8_TIMES_SF = 7864320,
    DPS310_OVERSAMPLING_16_TIMES_SF = 253952,
    DPS310_OVERSAMPLING_32_TIMES_SF = 516096,
    DPS310_OVERSAMPLING_64_TIMES_SF = 1040384,
    DPS310_OVERSAMPLING_128_TIMES_SF = 2088960
} dps310_OversamplingSf_t;

typedef enum {
    DPS310_MODE_IDLE = 0,
    DPS310_MODE_MANUAL_PRES_MEAS = 1,
    DPS310_MODE_MANUAL_TEMP_MEAS = 2,
    DPS310_MODE_CONT_PRES_MEAS = 5,
    DPS310_MODE_CONT_TEMP_MEAS = 6,
    DPS310_MODE_CONT_PRES_TEMP_MEAS = 7
} dps310_Mode_t;

#define DPS310_TEMP_COEF_SRC_MASK (1 << 7)

typedef struct {
    float c0Half;
    int16_t c1;
    bool read;
} dps310_TempCoef_t;

typedef struct {
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
    bool read;
} dps310_PresCoef_t;

typedef enum {
    DPS310_MEAS_TYPE_PRES = 0,
    DPS310_MEAS_TYPE_TEMP
} dps310_MeasType_t;

typedef struct {
    uint8_t i2cAddress;
    i2c_inst_t *i2c;
    dps310_OversamplingSf_t tempOversampleSf;
    dps310_OversamplingSf_t presOversampleSf;
    dps310_TempCoef_t tempCoef;
    dps310_PresCoef_t presCoef;
    float lastTempRawSc;
    bool lastTempInit;
} dps310_t;

bool dps310_init_struct(dps310_t *dps310, uint8_t i2cAddress, i2c_inst_t *i2c);

bool dps310_correct_temp(const dps310_t *dps310);

bool dps310_read_temp(dps310_t *dps310, float *temp);
bool dps310_read_pres(dps310_t *dps310, float *pres, bool readNewTemp);
bool dps310_fifo_read(dps310_t *dps310, float *value, dps310_MeasType_t *measType);

bool dps310_read_pres_calibration(dps310_t *dps310);
bool dps310_read_temp_calibration(dps310_t *dps310);

bool dps310_set_meas_mode(const dps310_t *dps310, dps310_Mode_t mode);
bool dps310_read_meas_mode(const dps310_t *dps310, dps310_Mode_t *mode);

bool dps310_set_pres_meas_rate(const dps310_t *dps310, dps310_MeasRate_t measRate);
bool dps310_read_pres_meas_rate(const dps310_t *dps310, dps310_MeasRate_t *measRate);

bool dps310_set_pres_oversampling(dps310_t *dps310, dps310_Oversampling_t oversample);
bool dps310_read_pres_oversampling(const dps310_t *dps310, dps310_Oversampling_t *oversample);

bool dps310_set_temp_meas_rate(const dps310_t *dps310, dps310_MeasRate_t measRate);
bool dps310_read_temp_meas_rate(const dps310_t *dps310, dps310_MeasRate_t *measRate);

bool dps310_set_temp_oversampling(dps310_t *dps310, dps310_Oversampling_t oversample);
bool dps310_read_temp_oversampling(const dps310_t *dps310, dps310_Oversampling_t *oversample);

bool dps310_read_status(const dps310_t *dps310, bool *coefReady, bool *sensorReady);
bool dps310_wait_until_ready(const dps310_t *dps310, bool waitCoef, bool waitSensor);

bool dps310_read_meas_status(const dps310_t *dps310, bool *tempReady, bool *presReady);
bool dps310_wait_until_meas_ready(const dps310_t *dps310, bool waitTemp, bool waitPres);

bool dps310_read_fifo_status(const dps310_t *dps310, bool *fifoFull, bool *fifoEmpty);

bool dps310_set_interrupt_source(const dps310_t *dps310, dps310_IntSrcMasks_t intSrc, bool enable);
bool dps310_read_interrupt_source(const dps310_t *dps310, dps310_IntSrcMasks_t intSrc, bool *enable);

bool dps310_read_interrupt_status(const dps310_t *dps310, bool *fifoFull, bool *tempMeas, bool *presMeas);

bool dps310_enable_fifo(const dps310_t *dps310, bool enable);
bool dps310_is_fifo_enabled(const dps310_t *dps310, bool *enabled);
bool dps310_fifo_flush(const dps310_t *dps310);

bool dps310_auto_set_temp_source(const dps310_t *dps310);

bool dps310_set_temp_meas_sensor(const dps310_t *dps310, dps310_TempSrc_t source);
bool dps310_read_temp_meas_sensor(const dps310_t *dps310, dps310_TempSrc_t *source);
bool dps310_read_temp_coef_source(const dps310_t *dps310, dps310_TempSrc_t *source);

bool dps310_read_full_id(const dps310_t *dps310, uint8_t *id);
bool dps310_read_id(const dps310_t *dps310, uint8_t *revId, uint8_t *productId);

bool dps310_soft_reset(dps310_t *dps310);

// Low level

bool dps310_write_register(const dps310_t *dps310, uint8_t reg, uint8_t data);
bool dps310_read_register(const dps310_t *dps310, uint8_t reg, uint8_t *data);

bool dps310_write_block(const dps310_t *dps310, uint8_t startReg, uint8_t data[], size_t len);
bool dps310_read_block(const dps310_t *dps310, uint8_t startReg, uint8_t data[], size_t len);

bool dps310_set_mask(const dps310_t *dps310, uint8_t reg, uint8_t mask, uint8_t value);
bool dps310_read_mask(const dps310_t *dps310, uint8_t reg, uint8_t mask, uint8_t *value);

bool dps310_temp_bit_shift(const dps310_t *dps310, bool shift);
bool dps310_pres_bit_shift(const dps310_t *dps310, bool shift);

bool dps310_set_bit(const dps310_t *dps310, uint8_t reg, uint8_t mask, bool value);
bool dps310_read_bit(const dps310_t *dps310, uint8_t reg, uint8_t mask, bool *value);

int32_t dps310_read_s24(const uint8_t *buf);

float dps310_compensate_pres(const dps310_t *dps310, float presRawSc,
                                           float tempRawSc);

float dps310_compensate_temp(const dps310_t *dps310, float tempRawSc);

dps310_OversamplingSf_t oversampling_to_sf(dps310_Oversampling_t oversample);

int32_t dps310_sign_extend(uint32_t value, int bits);

#endif
