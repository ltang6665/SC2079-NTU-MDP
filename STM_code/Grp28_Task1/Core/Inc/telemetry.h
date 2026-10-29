#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"
#include <stdint.h>

void Telemetry_Init(UART_HandleTypeDef *uart);
void Telemetry_StartCommand(const char *command_name, int value);

/* Heading gains are sent with each new command. */
void Telemetry_SetStraightPidGains(float kp, float ki, float kd);

/* Records a fault for EncoderTask to transmit; safe to call from another task. */
void Telemetry_RecordFault(const char *fault_code);

/* MotorTask only publishes a RAM snapshot: no formatting or UART waits here. */
void Telemetry_PublishWheelPi(
    uint32_t control_tick_ms, uint32_t control_dt_ms,
    float cps_a_filtered, float cps_b_filtered, int32_t speed_error,
    int32_t i_acc, int32_t i_limit, int applied_off, int drive_percent,
    int left_pwm, int right_pwm
);
void Telemetry_ClearWheelPi(void);
/* EncoderTask transmits at most one snapshot every 40 ms (25 Hz). */
void Telemetry_SendWheelPi(uint8_t straight_active);

/* Called by EncoderTask. sample_dt_ms is the real encoder measurement window. */
void Telemetry_SendEncoder(
    int16_t motor_a,
    int16_t motor_b,
    uint8_t command_active,
    uint16_t sample_dt_ms
);

void Telemetry_SendTurn(
    float yaw_deg,
    float target_deg,
    float yaw_rate_dps,
    float error_deg,
    float effort,
    int left_pwm,
    int right_pwm,
    uint8_t phase,
    uint8_t turn_active
);

/* Straight heading and steering diagnostics. */
void Telemetry_SendStraight(
    float yaw_deg,
    float target_deg,
    float yaw_rate_dps,
    float error_deg,
    float steer_p_percent,
    float steer_i_percent,
    float steer_d_percent,
    float steer_correction_percent,
    int servo_ccr,
    int servo_center_ccr,
    float steer_percent,
    int left_pwm,
    int right_pwm,
    uint8_t straight_active
);
#endif /* INC_TELEMETRY_H_ */
