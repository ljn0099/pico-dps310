#include "pico/stdlib.h"
#include "unity.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/i2c.h"
#include "sensor/dps310.h"

#define I2C_BUS i2c1
#define I2C_SDA 18
#define I2C_SCL 19
#define I2C_ADDRESS 0x77
#define I2C_BAUDRATE 100000

dps310_t dps310;

void setUp(void) {
    TEST_ASSERT_TRUE(dps310_init_struct(&dps310, I2C_ADDRESS, I2C_BUS));
    TEST_ASSERT_TRUE(dps310_wait_until_ready(&dps310, true, true));

    TEST_ASSERT_TRUE(dps310_soft_reset(&dps310));
    TEST_ASSERT_TRUE(dps310_wait_until_ready(&dps310, true, true));

    TEST_ASSERT_TRUE(dps310_read_temp_calibration(&dps310));
    TEST_ASSERT_TRUE(dps310_read_pres_calibration(&dps310));

    TEST_ASSERT_TRUE(dps310_auto_set_temp_source(&dps310));

    TEST_ASSERT_TRUE(dps310_correct_temp(&dps310));
}

void tearDown(void) {
}

void test_soft_reset(void) {
    TEST_ASSERT_FALSE(dps310_soft_reset(NULL));
    TEST_ASSERT_TRUE(dps310_soft_reset(&dps310));
    TEST_ASSERT_TRUE(dps310_wait_until_ready(&dps310, true, true));
}

void test_meas_mode(void) {
    dps310_Mode_t modeRead;

    TEST_ASSERT_FALSE(dps310_set_meas_mode(NULL, DPS310_MODE_CONT_PRES_TEMP_MEAS));
    TEST_ASSERT_FALSE(dps310_read_meas_mode(NULL, &modeRead));

    TEST_ASSERT_TRUE(dps310_set_meas_mode(&dps310, DPS310_MODE_CONT_PRES_MEAS));
    TEST_ASSERT_TRUE(dps310_read_meas_mode(&dps310, &modeRead));
    TEST_ASSERT_EQUAL_INT(DPS310_MODE_CONT_PRES_MEAS, modeRead);

    TEST_ASSERT_TRUE(dps310_set_meas_mode(&dps310, DPS310_MODE_CONT_PRES_TEMP_MEAS));
    TEST_ASSERT_TRUE(dps310_read_meas_mode(&dps310, &modeRead));
    TEST_ASSERT_EQUAL_INT(DPS310_MODE_CONT_PRES_TEMP_MEAS, modeRead);
}

void test_temp_pres_read(void) {
    float temp = 0;
    float pres = 0;

    TEST_ASSERT_TRUE(dps310_set_meas_mode(&dps310, DPS310_MODE_CONT_PRES_TEMP_MEAS));
    TEST_ASSERT_TRUE(dps310_wait_until_meas_ready(&dps310, true, true));

    TEST_ASSERT_TRUE(dps310_read_temp(&dps310, &temp));
    TEST_ASSERT_TRUE(dps310_read_pres(&dps310, &pres, false));

    printf("Temperature: %f\n", temp);
    printf("Pressure: %f\n", pres);

    TEST_ASSERT(temp >= -50 && temp <= 100);
    TEST_ASSERT(pres >= 62000 && pres <= 105000);
}

void test_temp_oversampling(void) {
    dps310_Oversampling_t osValues[] = {
        DPS310_OVERSAMPLING_1_TIME,   DPS310_OVERSAMPLING_2_TIMES,  DPS310_OVERSAMPLING_4_TIMES,
        DPS310_OVERSAMPLING_8_TIMES,  DPS310_OVERSAMPLING_16_TIMES, DPS310_OVERSAMPLING_32_TIMES,
        DPS310_OVERSAMPLING_64_TIMES, DPS310_OVERSAMPLING_128_TIMES};

    dps310_Oversampling_t osRead;

    for (size_t i = 0; i < sizeof(osValues) / sizeof(osValues[0]); i++) {
        TEST_ASSERT_TRUE(dps310_set_temp_oversampling(&dps310, osValues[i]));
        TEST_ASSERT_TRUE(dps310_read_temp_oversampling(&dps310, &osRead));
        TEST_ASSERT_EQUAL_INT(osValues[i], osRead);
    }
}

void test_pres_oversampling(void) {
    dps310_Oversampling_t osValues[] = {
        DPS310_OVERSAMPLING_1_TIME,   DPS310_OVERSAMPLING_2_TIMES,  DPS310_OVERSAMPLING_4_TIMES,
        DPS310_OVERSAMPLING_8_TIMES,  DPS310_OVERSAMPLING_16_TIMES, DPS310_OVERSAMPLING_32_TIMES,
        DPS310_OVERSAMPLING_64_TIMES, DPS310_OVERSAMPLING_128_TIMES};

    dps310_Oversampling_t osRead;

    for (size_t i = 0; i < sizeof(osValues) / sizeof(osValues[0]); i++) {
        TEST_ASSERT_TRUE(dps310_set_pres_oversampling(&dps310, osValues[i]));
        TEST_ASSERT_TRUE(dps310_read_pres_oversampling(&dps310, &osRead));
        TEST_ASSERT_EQUAL_INT(osValues[i], osRead);
    }
}

void test_temp_meas_rate(void) {
    dps310_MeasRate_t rates[] = {DPS310_MEAS_PER_SEC_1,  DPS310_MEAS_PER_SEC_2,
                                 DPS310_MEAS_PER_SEC_4,  DPS310_MEAS_PER_SEC_8,
                                 DPS310_MEAS_PER_SEC_16, DPS310_MEAS_PER_SEC_32,
                                 DPS310_MEAS_PER_SEC_64, DPS310_MEAS_PER_SEC_128};
    dps310_MeasRate_t rateRead;

    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        TEST_ASSERT_TRUE(dps310_set_temp_meas_rate(&dps310, rates[i]));
        TEST_ASSERT_TRUE(dps310_read_temp_meas_rate(&dps310, &rateRead));
        TEST_ASSERT_EQUAL_INT(rates[i], rateRead);
    }
}

void test_pres_meas_rate(void) {
    dps310_MeasRate_t rates[] = {DPS310_MEAS_PER_SEC_1,  DPS310_MEAS_PER_SEC_2,
                                 DPS310_MEAS_PER_SEC_4,  DPS310_MEAS_PER_SEC_8,
                                 DPS310_MEAS_PER_SEC_16, DPS310_MEAS_PER_SEC_32,
                                 DPS310_MEAS_PER_SEC_64, DPS310_MEAS_PER_SEC_128};
    dps310_MeasRate_t rateRead;

    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        TEST_ASSERT_TRUE(dps310_set_pres_meas_rate(&dps310, rates[i]));
        TEST_ASSERT_TRUE(dps310_read_pres_meas_rate(&dps310, &rateRead));
        TEST_ASSERT_EQUAL_INT(rates[i], rateRead);
    }
}

void test_status(void) {
    bool coefReady, sensorReady, tempReady, presReady, fifoFull, fifoEmpty;

    TEST_ASSERT_TRUE(dps310_read_status(&dps310, &coefReady, &sensorReady));
    TEST_ASSERT_TRUE(coefReady || !coefReady);
    TEST_ASSERT_TRUE(sensorReady || !sensorReady);

    TEST_ASSERT_TRUE(dps310_read_meas_status(&dps310, &tempReady, &presReady));
    TEST_ASSERT_TRUE(tempReady || !tempReady);
    TEST_ASSERT_TRUE(presReady || !presReady);

    TEST_ASSERT_TRUE(dps310_read_fifo_status(&dps310, &fifoFull, &fifoEmpty));
    TEST_ASSERT_TRUE(fifoFull || !fifoFull);
    TEST_ASSERT_TRUE(fifoEmpty || !fifoEmpty);
}

void test_interrupts(void) {
    bool enabled;
    dps310_IntSrcMasks_t intMask = DPS310_INT_SRC_FIFO_FULL_MASK;

    TEST_ASSERT_FALSE(dps310_set_interrupt_source(NULL, intMask, true));
    TEST_ASSERT_FALSE(dps310_read_interrupt_source(NULL, intMask, &enabled));

    TEST_ASSERT_TRUE(dps310_set_interrupt_source(&dps310, intMask, true));
    TEST_ASSERT_TRUE(dps310_read_interrupt_source(&dps310, intMask, &enabled));
    TEST_ASSERT_TRUE(enabled);

    TEST_ASSERT_TRUE(dps310_set_interrupt_source(&dps310, intMask, false));
    TEST_ASSERT_TRUE(dps310_read_interrupt_source(&dps310, intMask, &enabled));
    TEST_ASSERT_FALSE(enabled);
}

void test_fifo(void) {
    bool enabled;
    float value;
    dps310_MeasType_t type;

    TEST_ASSERT_TRUE(dps310_enable_fifo(&dps310, true));
    TEST_ASSERT_TRUE(dps310_is_fifo_enabled(&dps310, &enabled));
    TEST_ASSERT_TRUE(enabled);

    TEST_ASSERT_TRUE(dps310_fifo_flush(&dps310));

    TEST_ASSERT_TRUE(dps310_enable_fifo(&dps310, false));
    TEST_ASSERT_TRUE(dps310_is_fifo_enabled(&dps310, &enabled));
    TEST_ASSERT_FALSE(enabled);
}

void test_fifo_read(void) {
    const int target_readings = 10;   // number of samples to read
    const int timeout_ms = 11000;      // maximum total wait time (11 seconds) The default measurements are one per sec
    const int poll_interval = 500;    // time between checks (milliseconds)

    bool enabled;
    float value;
    dps310_MeasType_t type;
    int readCount = 0;
    int elapsed = 0;

    // Perform a manual temperature measurement for calibration
    TEST_ASSERT_TRUE(dps310_set_meas_mode(&dps310, DPS310_MODE_MANUAL_TEMP_MEAS));
    TEST_ASSERT_TRUE(dps310_wait_until_meas_ready(&dps310, true, false));
    TEST_ASSERT_TRUE(dps310_read_temp(&dps310, NULL));

    // Set continuous pressure measurement mode and enable FIFO
    TEST_ASSERT_TRUE(dps310_set_meas_mode(&dps310, DPS310_MODE_CONT_PRES_TEMP_MEAS));
    TEST_ASSERT_TRUE(dps310_enable_fifo(&dps310, true));
    TEST_ASSERT_TRUE(dps310_is_fifo_enabled(&dps310, &enabled));
    TEST_ASSERT_TRUE(enabled);

    // Clear FIFO before starting
    TEST_ASSERT_TRUE(dps310_fifo_flush(&dps310));

    // Wait and read until the desired number of samples are obtained or timeout occurs
    while (readCount < target_readings && elapsed < timeout_ms) {
        if (dps310_fifo_read(&dps310, &value, &type)) {
            readCount++;
            TEST_ASSERT(type == DPS310_MEAS_TYPE_TEMP || type == DPS310_MEAS_TYPE_PRES);

            if (type == DPS310_MEAS_TYPE_TEMP) {
                printf("FIFO temperature: %f\n", value);
                TEST_ASSERT(value >= -50 && value <= 100);  // reasonable temperature range
            } else if (type == DPS310_MEAS_TYPE_PRES) {
                printf("FIFO pressure: %f\n", value);
                TEST_ASSERT(value >= 62000 && value <= 105000); // reasonable pressure range (Pa)
            }
        } else {
            sleep_ms(poll_interval);
            elapsed += poll_interval;
        }
    }

    printf("FIFO readings obtained: %d\n", readCount);
    TEST_ASSERT_MESSAGE(readCount >= target_readings, "Did not get enough FIFO samples before timeout");

    // Disable FIFO after the test
    TEST_ASSERT_TRUE(dps310_enable_fifo(&dps310, false));
    TEST_ASSERT_TRUE(dps310_is_fifo_enabled(&dps310, &enabled));
    TEST_ASSERT_FALSE(enabled);
}


void test_id(void) {
    uint8_t fullId, revId, productId;

    TEST_ASSERT_FALSE(dps310_read_full_id(NULL, &fullId));
    TEST_ASSERT_FALSE(dps310_read_full_id(&dps310, NULL));
    TEST_ASSERT_FALSE(dps310_read_id(NULL, &revId, &productId));

    TEST_ASSERT_TRUE(dps310_read_full_id(&dps310, &fullId));
    TEST_ASSERT_TRUE(dps310_read_id(&dps310, &revId, &productId));

    printf("Full ID: 0x%X, Rev: 0x%X, Product: 0x%X\n", fullId, revId, productId);
}

int main(void) {
    stdio_init_all();

    i2c_init(I2C_BUS, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    UNITY_BEGIN();
    RUN_TEST(test_soft_reset);
    RUN_TEST(test_meas_mode);
    RUN_TEST(test_temp_pres_read);
    RUN_TEST(test_temp_oversampling);
    RUN_TEST(test_pres_oversampling);
    RUN_TEST(test_temp_meas_rate);
    RUN_TEST(test_pres_meas_rate);
    RUN_TEST(test_status);
    RUN_TEST(test_interrupts);
    RUN_TEST(test_fifo);
    RUN_TEST(test_fifo_read);
    RUN_TEST(test_id);
    return UNITY_END();
}
