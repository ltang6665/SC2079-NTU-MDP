#ifndef INC_GYRO_SAFE_H_
#define INC_GYRO_SAFE_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    I2C_HandleTypeDef *i2c;
    float bias_raw;
} GyroSafe;

/* Reset, identify, and configure the ICM-20948. */
bool GyroSafe_Init(GyroSafe *gyro, I2C_HandleTypeDef *i2c);

/* Reinitialize the STM32 I2C peripheral, then initialize the sensor. */
bool GyroSafe_Recover(GyroSafe *gyro, I2C_HandleTypeDef *i2c);

/* Robot must remain still. Calibration succeeds only if every read succeeds. */
bool GyroSafe_Calibrate(GyroSafe *gyro, uint16_t samples, uint32_t period_ms);

/* Read bias-corrected Z rate in degrees/s. */
bool GyroSafe_ReadRateDps(GyroSafe *gyro, float scale_trim, float *rate_dps);

#endif /* INC_GYRO_SAFE_H_ */
