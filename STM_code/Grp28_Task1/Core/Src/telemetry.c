#include "telemetry.h"

#include <stdio.h>
#include <string.h>

#define COMMAND_NAME_SIZE 16
#define FAULT_CODE_SIZE   20
#define WHEEL_PI_SEND_MS 40U
#define WHEEL_PI_STALE_MS 100U

static UART_HandleTypeDef *log_uart = NULL;

static volatile uint32_t active_command_id = 0;
static volatile uint8_t command_waiting = 0;
static char next_command_name[COMMAND_NAME_SIZE];
static volatile int next_command_value = 0;
static volatile uint32_t next_command_tick = 0;

static volatile uint8_t fault_waiting = 0;
static volatile uint32_t queued_fault_tick = 0;
static char fault_code_text[FAULT_CODE_SIZE];

/* Send the heading gains once when each command starts. */
static volatile float heading_kp = 0.0f;
static volatile float heading_ki = 0.0f;
static volatile float heading_kd = 0.0f;

static volatile uint8_t heading_gains_set = 0U;

/* Store all fields from a controller update together. */
typedef struct
{
    uint32_t command_id;
    uint32_t control_tick_ms;
    uint32_t control_dt_ms;
    float cps_a_filtered;
    float cps_b_filtered;
    int32_t speed_error;
    int32_t i_acc;
    int32_t i_limit;
    int applied_off;
    int drive_percent;
    int left_pwm;
    int right_pwm;
} WheelPiRecord;

static volatile WheelPiRecord wheel_pi_record;
static volatile uint8_t wheel_pi_ready = 0U;
static uint8_t wheel_pi_was_sent = 0U;
static uint32_t wheel_pi_send_tick = 0U;


static int32_t scaled_milli(float value)
{
    float scaled = value * 1000.0f;
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int32_t)scaled;
}


static void send_log_line(char *buffer, size_t size, int length)
{
    if (log_uart == NULL || length <= 0 || length >= (int)size)
    {
        return;
    }

    (void)HAL_UART_Transmit(
        log_uart,
        (uint8_t *)buffer,
        (uint16_t)length,
        20 /* The longer WPI line gets a 20 ms transmit timeout. */
    );
}


void Telemetry_Init(UART_HandleTypeDef *uart)
{
    log_uart = uart;
    active_command_id = 0;
    command_waiting = 0;
    fault_waiting = 0;
    next_command_name[0] = '\0';
    fault_code_text[0] = '\0';
    wheel_pi_ready = 0U;
    wheel_pi_was_sent = 0U;
    wheel_pi_send_tick = 0U;

    heading_kp = 0.0f;
    heading_ki = 0.0f;
    heading_kd = 0.0f;

    heading_gains_set = 0U;
}

void Telemetry_SetStraightPidGains(float kp,float ki,float kd)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();

    heading_kp = kp;
    heading_ki = ki;
    heading_kd = kd;

    heading_gains_set = 1U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Telemetry_StartCommand(const char *command_name, int value)
{
    char temp_name[COMMAND_NAME_SIZE];
    uint32_t primask;

    if (command_name == NULL)
    {
        return;
    }

    strncpy(temp_name, command_name, sizeof(temp_name) - 1U);
    temp_name[sizeof(temp_name) - 1U] = '\0';

    primask = __get_PRIMASK();
    __disable_irq();

    active_command_id++;
    wheel_pi_ready = 0U;
    wheel_pi_was_sent = 0U;
    next_command_tick = HAL_GetTick();

    next_command_value = value;
    memcpy(next_command_name, temp_name, sizeof(next_command_name));
    command_waiting = 1U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}


void Telemetry_RecordFault(const char *fault_code)
{
    char temp_code[FAULT_CODE_SIZE];
    uint32_t primask;

    if (fault_code == NULL)
    {
        return;
    }

    strncpy(temp_code, fault_code, sizeof(temp_code) - 1U);
    temp_code[sizeof(temp_code) - 1U] = '\0';

    primask = __get_PRIMASK();
    __disable_irq();

    queued_fault_tick = HAL_GetTick();
    memcpy(fault_code_text, temp_code, sizeof(fault_code_text));
    fault_waiting = 1U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}


void Telemetry_ClearWheelPi(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    wheel_pi_ready = 0U;
    if (primask == 0U) __enable_irq();
}


void Telemetry_PublishWheelPi(
    uint32_t control_tick_ms, uint32_t control_dt_ms,
    float cps_a_filtered, float cps_b_filtered, int32_t speed_error,
    int32_t i_acc, int32_t i_limit, int applied_off, int drive_percent,
    int left_pwm, int right_pwm
)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    wheel_pi_record.command_id = active_command_id;
    wheel_pi_record.control_tick_ms = control_tick_ms;
    wheel_pi_record.control_dt_ms = control_dt_ms;
    wheel_pi_record.cps_a_filtered = cps_a_filtered;
    wheel_pi_record.cps_b_filtered = cps_b_filtered;
    wheel_pi_record.speed_error = speed_error;
    wheel_pi_record.i_acc = i_acc;
    wheel_pi_record.i_limit = i_limit;
    wheel_pi_record.applied_off = applied_off;
    wheel_pi_record.drive_percent = drive_percent;
    wheel_pi_record.left_pwm = left_pwm;
    wheel_pi_record.right_pwm = right_pwm;
    wheel_pi_ready = 1U;
    if (primask == 0U) __enable_irq();
}


void Telemetry_SendWheelPi(uint8_t straight_active)
{
    WheelPiRecord sample;
    const uint32_t now = HAL_GetTick();
    uint8_t valid;
    uint32_t active_id;
    uint32_t primask;

    if (log_uart == NULL || !straight_active) return;
    if (wheel_pi_was_sent &&
        (uint32_t)(now - wheel_pi_send_tick) < WHEEL_PI_SEND_MS)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    valid = wheel_pi_ready;
    active_id = active_command_id;
    sample = wheel_pi_record;
    if (primask == 0U) __enable_irq();

    if (!valid || sample.command_id == 0U || sample.command_id != active_id ||
        (uint32_t)(now - sample.control_tick_ms) > WHEEL_PI_STALE_MS)
        return;

    /* Send rates as counts/s times 1000, matching the STR angle scale. */
    char buffer[192];
    const int length = snprintf(
        buffer, sizeof(buffer),
        "WPI,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d\r\n",
        (unsigned long)now, (unsigned long)sample.command_id,
        (unsigned long)sample.control_tick_ms, (unsigned long)sample.control_dt_ms,
        (long)scaled_milli(sample.cps_a_filtered),
        (long)scaled_milli(sample.cps_b_filtered),
        (long)sample.speed_error, (long)sample.i_acc, (long)sample.i_limit,
        sample.applied_off, sample.drive_percent, sample.left_pwm, sample.right_pwm
    );
    send_log_line(buffer, sizeof(buffer), length);
    wheel_pi_send_tick = now;
    wheel_pi_was_sent = 1U;
}


void Telemetry_SendEncoder(
    int16_t motor_a,
    int16_t motor_b,
    uint8_t command_active,
    uint16_t sample_dt_ms
)
{
    uint32_t command_id;

    uint8_t send_command = 0U;
    uint8_t send_fault = 0U;
    uint8_t pid_configured = 0U;

    uint32_t command_tick = 0U;
    uint32_t fault_tick = 0U;

    int command_value = 0;

    float pid_kp = 0.0f;
    float pid_ki = 0.0f;
    float pid_kd = 0.0f;

    char command_name[COMMAND_NAME_SIZE] = {0};
    char fault_code[FAULT_CODE_SIZE] = {0};

    uint32_t primask;

    if (log_uart == NULL)
    {
        return;
    }

    /* Take a local copy, then restore interrupts before sending. */
    primask = __get_PRIMASK();
    __disable_irq();

    command_id = active_command_id;

    if (command_waiting)
    {
        send_command = 1U;

        command_tick = next_command_tick;
        command_value = next_command_value;

        memcpy(
            command_name,
            next_command_name,
            sizeof(command_name)
        );

        /* Save the gains alongside the queued command. */
        pid_configured = heading_gains_set;

        pid_kp = heading_kp;
        pid_ki = heading_ki;
        pid_kd = heading_kd;

        command_waiting = 0U;
    }

    if (fault_waiting)
    {
        send_fault = 1U;
        fault_tick = queued_fault_tick;

        memcpy(
            fault_code,
            fault_code_text,
            sizeof(fault_code)
        );

        fault_waiting = 0U;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    if (send_command)
    {
        char buffer[64];

        int length = snprintf(
            buffer,
            sizeof(buffer),

            "CMD,%lu,%lu,%s:%d\r\n",

            (unsigned long)command_id,
            (unsigned long)command_tick,
            command_name,
            command_value
        );

        send_log_line(
            buffer,
            sizeof(buffer),
            length
        );
    }

    /* Scale SPID gains by 1000, as with the other numeric fields. */
    if (send_command && pid_configured)
    {
        char buffer[96];

        int length = snprintf(
            buffer,
            sizeof(buffer),

            "SPID,%lu,%lu,%ld,%ld,%ld\r\n",

            (unsigned long)command_tick,
            (unsigned long)command_id,

            (long)scaled_milli(pid_kp),
            (long)scaled_milli(pid_ki),
            (long)scaled_milli(pid_kd)
        );

        send_log_line(
            buffer,
            sizeof(buffer),
            length
        );
    }

    if (send_fault)
    {
        char buffer[64];

        int length = snprintf(
            buffer,
            sizeof(buffer),

            "FLT,%lu,%lu,%s\r\n",

            (unsigned long)fault_tick,
            (unsigned long)command_id,
            fault_code
        );

        send_log_line(
            buffer,
            sizeof(buffer),
            length
        );
    }

    {
        uint32_t sample_command_id =
            command_active ? command_id : 0U;

        char buffer[72];

        int length = snprintf(
            buffer,
            sizeof(buffer),

            "ENC,%lu,%lu,%d,%d,%u\r\n",

            (unsigned long)HAL_GetTick(),
            (unsigned long)sample_command_id,

            (int)motor_a,
            (int)motor_b,

            (unsigned int)sample_dt_ms
        );

        send_log_line(
            buffer,
            sizeof(buffer),
            length
        );
    }
}


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
)
{
    uint32_t command_id;
    uint32_t primask;
    char buffer[160];
    int length;

    if (log_uart == NULL || !turn_active)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    command_id = active_command_id;
    if (primask == 0U)
    {
        __enable_irq();
    }

    length = snprintf(
        buffer,
        sizeof(buffer),
        "TRN,%lu,%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%u\r\n",
        (unsigned long)HAL_GetTick(),
        (unsigned long)command_id,
        (long)scaled_milli(yaw_deg),
        (long)scaled_milli(target_deg),
        (long)scaled_milli(yaw_rate_dps),
        (long)scaled_milli(error_deg),
        (int)(effort + 0.5f),
        left_pwm,
        right_pwm,
        (unsigned int)phase
    );
    send_log_line(buffer, sizeof(buffer), length);
}


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
)
{
    uint32_t command_id;
    uint32_t primask;

    char buffer[208];
    int length;

    if (log_uart == NULL ||
        !straight_active)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    command_id = active_command_id;

    if (primask == 0U)
    {
        __enable_irq();
    }

    length = snprintf(
        buffer,
        sizeof(buffer),

        "STR,%lu,%lu,"
        "%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,"
        "%d,%d,%ld,%d,%d\r\n",

        (unsigned long)HAL_GetTick(),
        (unsigned long)command_id,

        (long)scaled_milli(yaw_deg),
        (long)scaled_milli(target_deg),
        (long)scaled_milli(yaw_rate_dps),
        (long)scaled_milli(error_deg),
        (long)scaled_milli(steer_p_percent),
        (long)scaled_milli(steer_i_percent),
        (long)scaled_milli(steer_d_percent),
        (long)scaled_milli(steer_correction_percent),

        servo_ccr,
        servo_center_ccr,

        (long)scaled_milli(steer_percent),

        left_pwm,
        right_pwm
    );

    send_log_line(
        buffer,
        sizeof(buffer),
        length
    );
}
