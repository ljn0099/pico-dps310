#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "hardware/i2c.h"
#include "sensor/dps310.h"

#define I2C_BUS i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define I2C_ADDRESS 0x77
#define I2C_BAUDRATE 100000

int main() {
    stdio_init_all();

    i2c_init(I2C_BUS, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    dps310_t dps310;

    if (!dps310_init_struct(&dps310, I2C_ADDRESS, I2C_BUS)) {
        printf("Error initializing the struct\n");
        return -1;
    }

    sleep_ms(500);

    if (!dps310_read_temp_calibration(&dps310)) {
        printf("Error reading temp calibration values\n");
        return -1;
    }

    if (!dps310_read_pres_calibration(&dps310)) {
        printf("Error reading pres calibration values\n");
        return -1;
    }

    if (!dps310_auto_set_temp_source(&dps310)) {
        printf("Error setting temp source\n");
        return -1;
    }

    if (!dps310_correct_temp(&dps310)) {
        printf("Error initializing the struct\n");
        return -1;
    }

    printf("c0: %f, c1:%d\n", dps310.tempCoef.c0Half*2.0f, dps310.tempCoef.c1);

    float pres = 0;

    float temp = 0;

    if (!dps310_set_meas_mode(&dps310, DPS310_MODE_CONT_PRES_TEMP_MEAS)) {
        printf("Error setting mode");
        return -1;
    }

    while (1) {

        sleep_ms(500);

        if (!dps310_read_temp(&dps310, &temp)) {
            printf("Error reading\n");
            return -1;
        }

        if (!dps310_read_pres(&dps310, &pres, false)) {
            printf("Error reading\n");
            return -1;
        }

        printf("Pressure: %f\n", pres);
        printf("Temperature: %f\n\n", temp);
    }
}
