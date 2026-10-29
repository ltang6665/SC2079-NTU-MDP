#include "gyro_safe.h"

#include "cmsis_os.h"
#include <stddef.h>

#define IMU_ADDRESS_7BIT       0x68U
#define IMU_WHO_AM_I_REG       0x00U
#define IMU_WHO_AM_I_VALUE     0xEAU
#define IMU_BANK_SEL_REG       0x7FU
#define IMU_PWR_MGMT_1_REG     0x06U
#define IMU_PWR_MGMT_2_REG     0x07U
#define IMU_LP_CONFIG_REG      0x05U
#define IMU_GYRO_ZOUT_H_REG    0x37U
#define IMU_GYRO_SMPLRT_REG    0x00U
#define IMU_GYRO_CONFIG_1_REG  0x01U

#define IMU_I2C_TIMEOUT_MS     3U
#define IMU_LSB_PER_DPS        16.4f

/* Board mapping from HAL_I2C_MspInit: I2C2 on PB10 (SCL), PB11 (SDA).
 * Keep the existing external pull-ups; do not add pull-ups to another rail.
 * Recovery is called only by gyroTask, with the scheduler running and no
 * other task/ISR using this I2C peripheral. It is not an ISR-safe function.
 */
#define BUS_PORT               GPIOB
#define BUS_SCL_PIN            GPIO_PIN_10
#define BUS_SDA_PIN            GPIO_PIN_11
#define BUS_PINS               (BUS_SCL_PIN | BUS_SDA_PIN)
#define BUS_LINE_TIMEOUT_MS    5U
#define BUS_EDGE_DELAY_MS      2U
#define BUS_CLEAR_CLOCKS       9U


static bool bus_pins_valid(void)
{
    /* Refuse to pulse unrelated pins if the CubeMX mapping has changed.
     * PB10/PB11 must currently be alternate-function, open-drain, AF4.
     * MODER has two bits per pin; AFR[1] covers pins 8 through 15.
     */
    const uint32_t pin_mode_mask = (3UL << 20) | (3UL << 22);
    const uint32_t pin_mode_af = (2UL << 20) | (2UL << 22);
    const uint32_t pin_af_mask = (15UL << 8) | (15UL << 12);
    const uint32_t pin_af_i2c2 = ((uint32_t)GPIO_AF4_I2C2 << 8) |
                             ((uint32_t)GPIO_AF4_I2C2 << 12);

    return __HAL_RCC_GPIOB_IS_CLK_ENABLED() &&
           (BUS_PORT->MODER & pin_mode_mask) == pin_mode_af &&
           (BUS_PORT->OTYPER & BUS_PINS) == BUS_PINS &&
           (BUS_PORT->AFR[1] & pin_af_mask) == pin_af_i2c2;
}


static bool wait_line_released(uint16_t pin)
{
    const uint32_t start_tick = HAL_GetTick();

    while (HAL_GPIO_ReadPin(BUS_PORT, pin) != GPIO_PIN_SET)
    {
        /* Unsigned elapsed time still works across a tick-counter wrap. */
        if ((uint32_t)(HAL_GetTick() - start_tick) >= BUS_LINE_TIMEOUT_MS)
        {
            return false;
        }
        osDelay(1U);
    }

    return true;
}


static bool clear_i2c_lines(void)
{
    GPIO_InitTypeDef pins = {0};
    bool lines_released = false;

    /* HAL_I2C_DeInit has disabled I2C2 and detached its pins by this point.
     * Preload released levels before enabling GPIO output mode.
     * In open-drain mode SET releases the wire; it does not drive it high.
     */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(BUS_PORT, BUS_PINS, GPIO_PIN_SET);
    pins.Pin = BUS_PINS;
    pins.Mode = GPIO_MODE_OUTPUT_OD;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUS_PORT, &pins);
    osDelay(BUS_EDGE_DELAY_MS);

    /* If SCL stays low, stop recovery once the line timeout expires. */
    if (!wait_line_released(BUS_SCL_PIN))
    {
        goto release_lines;
    }

    /* Let an interrupted target finish its byte and release SDA.
     * Stop early if SDA is already released. These are recovery clocks,
     * not normal sensor traffic; the slow edges allow other tasks to run.
     */
    for (uint32_t pulse = 0U;
         pulse < BUS_CLEAR_CLOCKS &&
         HAL_GPIO_ReadPin(BUS_PORT, BUS_SDA_PIN) == GPIO_PIN_RESET;
         ++pulse)
    {
        HAL_GPIO_WritePin(BUS_PORT, BUS_SCL_PIN, GPIO_PIN_RESET);
        osDelay(BUS_EDGE_DELAY_MS);
        HAL_GPIO_WritePin(BUS_PORT, BUS_SCL_PIN, GPIO_PIN_SET);
        if (!wait_line_released(BUS_SCL_PIN))
        {
            goto release_lines;
        }
        osDelay(BUS_EDGE_DELAY_MS);
    }

    if (!wait_line_released(BUS_SDA_PIN))
    {
        goto release_lines;
    }

    /* Generate STOP: prepare SDA low while SCL is low, release SCL,
     * then release SDA while SCL is high. Never actively drive either high.
     */
    HAL_GPIO_WritePin(BUS_PORT, BUS_SCL_PIN, GPIO_PIN_RESET);
    osDelay(BUS_EDGE_DELAY_MS);
    HAL_GPIO_WritePin(BUS_PORT, BUS_SDA_PIN, GPIO_PIN_RESET);
    osDelay(BUS_EDGE_DELAY_MS);
    HAL_GPIO_WritePin(BUS_PORT, BUS_SCL_PIN, GPIO_PIN_SET);
    if (!wait_line_released(BUS_SCL_PIN))
    {
        goto release_lines;
    }
    osDelay(BUS_EDGE_DELAY_MS);
    HAL_GPIO_WritePin(BUS_PORT, BUS_SDA_PIN, GPIO_PIN_SET);
    osDelay(BUS_EDGE_DELAY_MS);

    lines_released = wait_line_released(BUS_SDA_PIN) &&
              HAL_GPIO_ReadPin(BUS_PORT, BUS_SCL_PIN) == GPIO_PIN_SET;

release_lines:
    /* Release both lines on exit; recovery puts the pins back in AF mode. */
    HAL_GPIO_WritePin(BUS_PORT, BUS_PINS, GPIO_PIN_SET);
    return lines_released;
}


static bool sensor_write(GyroSafe *gyro, uint8_t address, uint8_t value)
{
    if (gyro == NULL || gyro->i2c == NULL)
    {
        return false;
    }

    return HAL_I2C_Mem_Write(
               gyro->i2c,
               IMU_ADDRESS_7BIT << 1,
               address,
               I2C_MEMADD_SIZE_8BIT,
               &value,
               1U,
               IMU_I2C_TIMEOUT_MS
           ) == HAL_OK;
}


static bool sensor_read(GyroSafe *gyro, uint8_t address, uint8_t *data, uint16_t size)
{
    if (gyro == NULL || gyro->i2c == NULL || data == NULL || size == 0U)
    {
        return false;
    }

    return HAL_I2C_Mem_Read(
               gyro->i2c,
               IMU_ADDRESS_7BIT << 1,
               address,
               I2C_MEMADD_SIZE_8BIT,
               data,
               size,
               IMU_I2C_TIMEOUT_MS
           ) == HAL_OK;
}


static bool sensor_bank(GyroSafe *gyro, uint8_t bank)
{
    return sensor_write(gyro, IMU_BANK_SEL_REG, (uint8_t)(bank << 4));
}


static bool read_z_axis(GyroSafe *gyro, int16_t *z_sample)
{
    uint8_t bytes[2];

    if (z_sample == NULL || !sensor_read(gyro, IMU_GYRO_ZOUT_H_REG, bytes, 2U))
    {
        return false;
    }

    *z_sample = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    return true;
}


bool GyroSafe_Init(GyroSafe *gyro, I2C_HandleTypeDef *i2c)
{
    uint8_t identity = 0U;

    if (gyro == NULL || i2c == NULL)
    {
        return false;
    }

    gyro->i2c = i2c;
    gyro->bias_raw = 0.0f;

    /* Select bank 0, request a reset, and wait for the sensor to restart. */
    if (!sensor_bank(gyro, 0U) ||
        !sensor_write(gyro, IMU_PWR_MGMT_1_REG, 0x80U))
    {
        return false;
    }
    osDelay(100U);

    if (!sensor_bank(gyro, 0U) ||
        !sensor_read(gyro, IMU_WHO_AM_I_REG, &identity, 1U) ||
        identity != IMU_WHO_AM_I_VALUE)
    {
        return false;
    }

    /* Use the PLL clock with sleep/cycling off and every accel/gyro axis enabled. */
    if (!sensor_write(gyro, IMU_PWR_MGMT_1_REG, 0x01U) ||
        !sensor_write(gyro, IMU_PWR_MGMT_2_REG, 0x00U) ||
        !sensor_write(gyro, IMU_LP_CONFIG_REG, 0x00U))
    {
        return false;
    }

    /* Configure bank 2 with the existing 16.4 LSB/(degree/s) scale and filter. */
    if (!sensor_bank(gyro, 2U) ||
        !sensor_write(gyro, IMU_GYRO_SMPLRT_REG, 0x11U) ||
        !sensor_write(gyro, IMU_GYRO_CONFIG_1_REG, 0x2FU) ||
        !sensor_bank(gyro, 0U))
    {
        return false;
    }

    osDelay(20U);
    return true;
}


bool GyroSafe_Recover(GyroSafe *gyro, I2C_HandleTypeDef *i2c)
{
    if (gyro == NULL || i2c == NULL || i2c->Instance != I2C2)
    {
        return false;
    }

    if (!bus_pins_valid())
    {
        return false;
    }

    if (HAL_I2C_DeInit(i2c) != HAL_OK)
    {
        return false;
    }

    const bool bus_cleared = clear_i2c_lines();

    /* Reset the STM32 peripheral separately from clearing the physical bus.
     * RCC reset does not reset or remove power from the external gyro.
     */
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C2_FORCE_RESET();
    osDelay(1U);
    __HAL_RCC_I2C2_RELEASE_RESET();
    osDelay(1U);

    /* DeInit set the HAL state to RESET, so Init calls HAL_I2C_MspInit and
     * restores PB10/PB11 to AF4 open-drain, even after a failed bus clear.
     * Do not short-circuit this call on bus_cleared: pins must be restored.
     */
    if (HAL_I2C_Init(i2c) != HAL_OK)
    {
        return false;
    }

    if (!bus_cleared)
    {
        return false;
    }

    /* The existing sensor soft reset, WHO_AM_I check and setup follow.
     * gyroTask performs its existing stationary calibration afterwards.
     */
    return GyroSafe_Init(gyro, i2c);
}


bool GyroSafe_Calibrate(GyroSafe *gyro, uint16_t samples, uint32_t period_ms)
{
    int64_t sum = 0;

    if (gyro == NULL || samples == 0U)
    {
        return false;
    }

    for (uint16_t index = 0U; index < samples; ++index)
    {
        int16_t z_sample;

        osDelay(period_ms);
        if (!read_z_axis(gyro, &z_sample))
        {
            return false;
        }
        sum += z_sample;
    }

    gyro->bias_raw = (float)sum / (float)samples;
    return true;
}


bool GyroSafe_ReadRateDps(GyroSafe *gyro, float rate_trim, float *rate_dps)
{
    int16_t z_sample;

    if (rate_dps == NULL || !read_z_axis(gyro, &z_sample))
    {
        return false;
    }

    *rate_dps = (((float)z_sample - gyro->bias_raw) / IMU_LSB_PER_DPS) *
                rate_trim;
    return true;
}
