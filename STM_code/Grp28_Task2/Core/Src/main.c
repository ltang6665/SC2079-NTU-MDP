/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "telemetry.h"
#include "manual_control.h"
#include "gyro_safe.h"
#include "stdbool.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId LED_TaskHandle;
osThreadId OLED_TaskHandle;
osThreadId Motor_TaskHandle;
osThreadId Encoder_TaskHandle;
osThreadId Gyro_TaskHandle;
osThreadId Ultrasound_TaskHandle;
osThreadId IR_TaskHandle;
/* USER CODE BEGIN PV */
volatile float tc1 = 0, tc2 = 0, echo = 0;
// Ultrasonic capture diagnostics
volatile uint32_t echo_debug = 0;
volatile float distance = 0;
volatile float ir_distance_cm = 0.0f;
volatile bool new_measurement_ready = false;
// FU2 uses these timestamps and flags to reject old echo readings.
static volatile uint32_t us_capture_seq = 0U;
static volatile uint32_t us_capture_tick = 0U;
static volatile uint32_t us_valid_echo_tick = 0U;
static volatile uint8_t us_capture_valid = 0U;
static volatile uint32_t us_raw_cm = 0U;
static volatile float us_raw_heading = 0.0f;
static uint32_t us_history[3] = {0, 0, 0};
static uint8_t us_history_pos = 0;
static uint8_t us_history_used = 0;

// Wheel encoder deltas
volatile int delta_a = 0;
volatile int delta_b = 0;

volatile uint8_t abort_now = 0; // Stop/reset request

volatile uint8_t rx_byte; // Byte received by the UART interrupt
volatile enum {
  DRIVE_IDLE,
  DRIVE_FORWARD,
  DRIVE_REVERSE,
  DRIVE_STOP,
  DRIVE_FR,
  DRIVE_FL,
  DRIVE_RR,
  DRIVE_RL,
  DRIVE_JOYSTICK
} uart_cmd;

static char rx_line[256];
static uint16_t rx_pos = 0;
static uint8_t rx_line_overflow = 0U;
static char waiting_line[256];
static volatile uint8_t waiting_line_ready = 0U;
#define RX_SCRIPT_SLOTS 4U
static char rx_script_lines[RX_SCRIPT_SLOTS][256];
static volatile uint8_t rx_script_read = 0U, rx_script_write = 0U;
static volatile uint32_t uart3_rx_error_flags = 0U;
static volatile uint32_t uart3_rx_error_count = 0U;
static uint8_t fu_arrow_confirmed = 0U;
static uint8_t fu_sequence_no = 0U;
static float arc_trace_heading_reference = 0.0f; // Trace reference only, separate from the motion target
static int32_t reverse_travel_counts = 0;
static uint16_t fu2_rev_clear_cm = 0U;
static int32_t fu2_rev_min_counts = 0;
static uint32_t fu2_rev_sample_seq = 0U;
static uint32_t fu2_rev_start_tick = 0U;
static uint8_t fu2_rev_clear_samples = 0U;
static uint32_t fu2_rev_last_usable_tick = 0U;
static uint32_t fu2_rev_last_range_cm = 0U;
static uint8_t fu2_rev_waiting_echo = 0U;
static uint32_t fu2_rev_reference_cm = 0U;
static int32_t fu2_rev_reference_counts = 0, fu2_rev_max_counts = 0;
static uint8_t fu2_rev_reacquire_samples = 0U;
static uint32_t fu2_rev_reacquire_low_cm = 0U, fu2_rev_reacquire_high_cm = 0U;
static uint16_t fu2_rev_reacquire_start_a = 0U, fu2_rev_reacquire_start_b = 0U;
static uint32_t fu2_fwd_sample_seq = 0U;
static uint32_t fu2_fwd_range_cm = 0U;
static uint32_t fu2_fwd_wait_tick = 0U;
static uint8_t fu2_fwd_near_samples = 0U;
static uint8_t fu2_fwd_waiting_echo = 0U;
static uint8_t fu2_approach_select_pending = 0U;
static uint16_t fu2_close_approach_cm = 28U;
static uint32_t fu2_approach_sample_seq = 0U;


volatile uint32_t next_start_tick = 0; // Earliest start time for the next queued step

/* -------- Queued motion steps -------- */
typedef enum
{
  STEP_NONE = 0,
  STEP_FORWARD_CM, // Positive distance in cm
  STEP_REVERSE_CM, // Positive distance in cm
  STEP_FR, // Turn angle in degrees (90..360)
  STEP_FL,
  STEP_RR,
  STEP_RL,
  STEP_PARK_PROFILE,
  STEP_SLIDE_RIGHT, // first obstacle
  STEP_SLIDE_LEFT, // first obstacle
  STEP_STOP,
  STEP_END,
  STEP_FIR, // forward infra right
  STEP_FIL, // forward infra left
  STEP_FIRO,
  STEP_FILO,
  STEP_FU, // forward forever until ultrasound <= threshold
  STEP_FX  // same as FU but no reverse on overshoot
} step_kind_t;

typedef struct
{
  step_kind_t type;
  int value; // Angle or distance, selected by the step kind
} motion_step_t;

#define STEP_QUEUE_SIZE 32 // One slot stays empty, leaving room for 31 steps.
static volatile motion_step_t step_queue[STEP_QUEUE_SIZE];
static volatile uint8_t queue_read = 0, queue_write = 0;

static inline int queue_empty(void) { return queue_read == queue_write; }
static inline int queue_full(void) { return (uint8_t)(queue_write + 1) % STEP_QUEUE_SIZE == queue_read; }

static int queue_add(motion_step_t it)
{
  uint8_t next = (uint8_t)(queue_write + 1) % STEP_QUEUE_SIZE;
  if (next == queue_read)
    return 0;
  step_queue[queue_write] = it;
  queue_write = next;
  return 1;
}

static int queue_take(motion_step_t *out)
{
  if (queue_empty())
    return 0;
  *out = step_queue[queue_read];
  queue_read = (uint8_t)(queue_read + 1) % STEP_QUEUE_SIZE;
  return 1;
}

static inline uint8_t queue_count(void)
{
  return (queue_write >= queue_read) ? (queue_write - queue_read) : (STEP_QUEUE_SIZE - queue_read + queue_write);
}

static inline void queue_clear(void)
{
  __disable_irq();
  queue_read = queue_write = 0;
  __enable_irq();
}

static int run_next_step(void);

/* Inspect the following motion across script boundaries without consuming it. */
static int step_is_forward_turn(step_kind_t type)
{
  return type == STEP_FR || type == STEP_FL;
}

static int step_uses_ir(step_kind_t type)
{
  return type == STEP_FIR || type == STEP_FIL || type == STEP_FIRO || type == STEP_FILO;
}

static step_kind_t peek_motion_kind(void)
{
  uint8_t index = queue_read;
  while (index != queue_write)
  {
    const step_kind_t type = step_queue[index].type;
    if (type != STEP_END && type != STEP_PARK_PROFILE) return type;
    index = (uint8_t)((index + 1U) % STEP_QUEUE_SIZE);
  }
  return STEP_NONE;
}

static int peek_motion_direction(void)
{
  uint8_t index = queue_read;
  while (index != queue_write)
  {
    const step_kind_t type = step_queue[index].type;
    if (type == STEP_END || type == STEP_PARK_PROFILE) { index = (uint8_t)((index + 1U) % STEP_QUEUE_SIZE); continue; }
    if (type == STEP_STOP || type == STEP_NONE) return 0;
    return (type == STEP_REVERSE_CM || type == STEP_RR || type == STEP_RL) ? -1 : 1;
  }
  return 0;
}

/* Command acknowledgements */

static volatile uint8_t reply_tx_busy = 0;
static volatile uint8_t script_ack_waiting = 0U;

/* MotorTask owns this queue. Copies remain valid until UART transmission ends. */
#define REPLY_QUEUE_SIZE 16U
#define REPLY_LINE_SIZE 48U
static char reply_queue[REPLY_QUEUE_SIZE][REPLY_LINE_SIZE];
static char reply_in_flight[REPLY_LINE_SIZE];
static uint8_t reply_read = 0U;
static uint8_t reply_write = 0U;
static uint8_t reply_count = 0U;

static void clear_replies(void)
{
  reply_read = reply_write = reply_count = 0U;
  // Do not overwrite reply_in_flight while an earlier transfer is in flight.
}

static int add_reply(const char *message)
{
  const size_t length = strlen(message);
  if (length == 0U || length >= REPLY_LINE_SIZE ||
      reply_count >= REPLY_QUEUE_SIZE)
    return 0;
  memcpy(reply_queue[reply_write], message, length + 1U);
  reply_write = (uint8_t)((reply_write + 1U) % REPLY_QUEUE_SIZE);
  ++reply_count;
  return 1;
}

static void queue_range_reply(const char *message)
{
  if (!add_reply(message))
  {
    Telemetry_RecordFault("STM_REPLY_QUEUE_FULL");
    abort_now = 1U;
  }
}

static void send_queued_reply(void)
{
  if (reply_tx_busy || reply_count == 0U)
    return;
  memcpy(reply_in_flight, reply_queue[reply_read], REPLY_LINE_SIZE);
  const uint16_t length = (uint16_t)strlen(reply_in_flight);
  reply_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)reply_in_flight, length) == HAL_OK)
  {
    reply_read = (uint8_t)((reply_read + 1U) % REPLY_QUEUE_SIZE);
    --reply_count;
  }
  else
  {
    // HAL_BUSY/HAL_ERROR must leave the message queued for the next loop.
    reply_tx_busy = 0U;
  }
}

/* A blocking RX error ends HAL reception. Keep the byte reader armed even
 * after an overrun during reset/reconnection. Only motorTask queues replies. */
static void check_uart_receiver(void)
{
  if (huart3.RxState == HAL_UART_STATE_READY)
    (void)HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1U);

  static uint32_t last_report_tick = 0U;
  const uint32_t now = HAL_GetTick();
  if (uart3_rx_error_flags != 0U &&
      (uint32_t)(now - last_report_tick) >= 1000U)
  {
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    const uint32_t flags = uart3_rx_error_flags;
    const uint32_t count = uart3_rx_error_count;
    uart3_rx_error_flags = 0U;
    __set_PRIMASK(saved);
    char message[48];
    int n = snprintf(message, sizeof(message), "UART_RX_RECOVER,%lu,%lu\n",
        (unsigned long)flags, (unsigned long)count);
    if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
    last_report_tick = now;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
    reply_tx_busy = 0;
}

static void ack_script(void)
{
  ++script_ack_waiting;
  /* A later confirmed script may already be queued. Do not erase it. */
}

static void check_script_status(void)
{
  if (script_ack_waiting && add_reply("OK\n"))
    --script_ack_waiting;
}

// Reply when a motion step finishes
static void ack_step(void)
{
  (void)add_reply("done\n");
}

// ---- Distance tracking ----
volatile uint16_t enc_start_a = 0, enc_start_b = 0;
volatile int32_t target_counts = 0; // Positive for forward; negative for reverse

#define WHEEL_DIAMETER_CM 5.7f
#define ENCODER_PULSES_PER_REV 330
#define ENCODER_MULTIPLIER 4
#define ENCODER_COUNTS_PER_REV (ENCODER_PULSES_PER_REV * ENCODER_MULTIPLIER)

#define WHEEL_CIRCUMFERENCE_CM (3.14159f * WHEEL_DIAMETER_CM)
#define ENCODER_COUNTS_PER_CM ((float)ENCODER_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_CM)

#define PWM_OFF 7199
#define PWM_CRUISE 2500
#define PWM_SLOW 6800
#define PWM_INNER_WHEEL 6000

//luther direction higher value = slower
#define FORWARD_LEFT_COMPARE_SCALE   1.18f
#define FORWARD_RIGHT_COMPARE_SCALE  1.000f

#define REVERSE_LEFT_COMPARE_SCALE   0.993f //lower value = slower
#define REVERSE_RIGHT_COMPARE_SCALE  1.000f

/* Straight profile tuning for finite scripted F/R distance commands.
 * EFFORT is an absolute PWM drive count: compare = PWM_OFF - effort.
 * Lower effort means less drive. These endpoints do not scale with PWM_CRUISE.
 * Wheel PI and launch trim can still adjust the actual wheel outputs.
 * Keep DECEL_CM > TERMINAL_CM > 0 and terminal effort <= decel effort.
 * Targets are capped at the wheel's cruise effort to avoid accelerating
 * during slowdown when a lower cruise setting is selected.
 */
#define LINE_ACCEL_RAMP_MS          350U
#define LINE_DECEL_CM                 22.0f // Start cruise -> decel-target ramp.
#define LINE_TERMINAL_CM              10.0f // Start decel -> terminal-target ramp.
#define LINE_DECEL_EXPONENT           1.0f // 1 = linear; 2 = gentle at the start.

// Deceleration and terminal drive limits, in PWM effort counts.
#define FORWARD_DECEL_EFFORT_L             1740.0f
#define FORWARD_DECEL_EFFORT_R             1880.0f
#define FORWARD_TERMINAL_EFFORT_L           435.0f
#define FORWARD_TERMINAL_EFFORT_R           470.0f
#define REVERSE_DECEL_EFFORT_L             1886.0f
#define REVERSE_DECEL_EFFORT_R             1880.0f
#define REVERSE_TERMINAL_EFFORT_L           472.0f
#define REVERSE_TERMINAL_EFFORT_R           470.0f

/* Only the obstacle-2 reverse-until-clear. Fixed effort ceilings initially
 * match about 40% of calibrated reverse cruise; normal R and FU40 are separate. */
#define FU2_REVERSE_CAP_ENABLE              1
#define FU2_REVERSE_MAX_EFFORT_L          1886.0f
#define FU2_REVERSE_MAX_EFFORT_R          1880.0f
// Separate reverse clearance from FU's forward approach threshold. The
// Wheel encoder deltas minimum remains range-derived; this is not an obstacle length.
#define FU2_REVERSE_CLEAR_CM                28U // Clearance required before entry90.
#define FU2_REVERSE_TRIGGER_CM              24U // Reverse when the confirmed entry range is at or below this.
#define FU2_REVERSE_CLEAR_CONFIRM_SAMPLES    2U
#define FU2_REVERSE_US_STALE_MS            500U
/* FU2 only: never drive from the median's slide-era history. A suspect
 * close echo brakes immediately; another new close echo permits reverse. */
#define FU2_FORWARD_US_WAIT_MS             300U
#define FU2_REVERSE_START_CONFIRM_SAMPLES    2U
#define FU2_REVERSE_EXTRA_TRAVEL_CM           6.0f
#define FU2_REVERSE_RANGE_TOLERANCE_CM       12.0f
/* Reconsider an inconsistent entry range only after confirming new echoes at
 * rest. This can shorten the remaining reverse, never extend its original cap. */
#define FU2_REVERSE_REACQUIRE_CONFIRM_SAMPLES  3U
#define FU2_REVERSE_REACQUIRE_SPREAD_CM        3U
#define FU2_REVERSE_REACQUIRE_MAX_MOVE_CM      0.5f
#define FU2_REVERSE_REACQUIRE_MAX_ERROR_DEG   10.0f
#define FU2_REVERSE_REACQUIRE_MAX_YAW_DPS     15.0f
#define US_VALID_MIN_CM                      2U
#define US_VALID_MAX_CM                    400U
/* Trigger-to-trigger spacing; missed echoes must not block this task. */
#define US_TRIGGER_PERIOD_MS                 60U
#define US_ECHO_TIMEOUT_MS                   30U
/* Final parking FX confirms range at rest, then tapers drive.
 * FX22 brakes at 26 cm, allowing 4 cm for braking. */
#define PARKING_FX_STOP_MARGIN_CM                4U
#define PARKING_FX_SAMPLE_MAX_AGE_MS           100U
#define PARKING_FX_US_STALE_MS                 350U // Confirmation wait is braked, including the first echo.
#define PARKING_FX_SAMPLE_MAX_ERROR_DEG         10.0f
#define PARKING_FX_CRUISE_EFFORT_SCALE            0.70f
#define PARKING_FX_MIN_EFFORT_SCALE               0.30f
#define PARKING_FX_DECEL_START_CM               45.0f
#define PARKING_FX_NEAR_BRAKE_ZONE_CM            8.0f
#define PARKING_FX_NEAR_MAX_EFFORT_SCALE          0.35f
#define PARKING_FX_CONFIRM_SAMPLES                3U
#define PARKING_FX_CONFIRM_SPREAD_CM              4U
#define PARKING_FX_CONFIRM_MAX_MOVE_CM            0.5f
#define PARKING_FX_CONFIRM_MAX_YAW_DPS           15.0f
#define PARKING_FX_RANGE_INCREASE_TOLERANCE_CM   12.0f
#define PARK_FINAL90_US_GUARD_WINDOW_DEG      20.0f
#define PARK_FINAL90_NEAR_WALL_HANDOFF_DEG     3.0f
#define PARKING_FX_TRACE_MS                    100U

/* FU2 can arrive while still driving forward. Ease into the first 90 using
 * measured ultrasound distance remaining to its FU threshold. This is not
 * a timed wait or an obstacle-length assumption. Reverse has its own caps. */
/* Use fresh forward-facing raw echoes during the FINAL slide's inward
 * phase, before FU2 begins. Other slides retain their calibrated shape. */
#define FU2_EARLY_US_ENABLE                     1
#define FU2_EARLY_US_SCAN_WINDOW_DEG           35.0f
#define FU2_EARLY_US_SAMPLE_MAX_ERROR_DEG      25.0f
#define FU2_EARLY_US_SAMPLE_MAX_AGE_MS          120U
#define FU2_EARLY_US_SLOW_RANGE_CM             45.0f
#define FU2_EARLY_US_NEAR_RANGE_CM             20.0f
#define FU2_EARLY_US_MIN_SCALE                  0.50f
#define FU2_EARLY_US_NO_SAMPLE_SCALE            0.65f
static uint8_t fu2_slide_us_announced = 0U;
#define FU2_APPROACH_DECEL_CM                14.0f
#define FU2_APPROACH_ENTRY_EFFORT_L          FORWARD_DECEL_EFFORT_L
#define FU2_APPROACH_ENTRY_EFFORT_R          FORWARD_DECEL_EFFORT_R
#define FU2_ADAPTIVE_APPROACH_ENABLE           1
#define FU2_FAR_MIN_RANGE_CM                  55U
#define FU2_FAR_APPROACH_CM                   40U
#define FU2_APPROACH_SAMPLE_MAX_AGE_MS       150U
#define FU2_APPROACH_SAMPLE_MAX_ERROR_DEG     10.0f

#define FORWARD_LAUNCH_TRIM_MS                        400U
#define FORWARD_LAUNCH_LEFT_EFFORT_TRIM_PERCENT       4.0f

#define REVERSE_LAUNCH_TRIM_MS                        400U
#define REVERSE_LAUNCH_LEFT_EFFORT_TRIM_PERCENT       0.0f
// Terminal anti-stall.
#define LINE_STALL_DETECT_MS           150U
#define LINE_STALL_PROGRESS_COUNTS       4
#define LINE_STALL_BOOST_STEP_EFFORT    140.0f
#define LINE_STALL_MAX_EFFORT          1130.0f

// Calculate one wheel's requested drive before the acceleration limit.
static float straight_slowdown_target(float remaining_cm, float cruise_effort,
                                      float decel_effort, float terminal_effort,
                                      float decel_distance_cm)
{
  cruise_effort = fminf((float)PWM_OFF, fmaxf(0.0f, cruise_effort));
  decel_effort = fminf(cruise_effort, fmaxf(0.0f, decel_effort));
  terminal_effort = fminf(decel_effort, fmaxf(0.0f, terminal_effort));
  remaining_cm = fmaxf(0.0f, remaining_cm);
  const float terminal_cm = fmaxf(0.0f, LINE_TERMINAL_CM);
  const float decel_cm = fmaxf(terminal_cm, decel_distance_cm);

  if (remaining_cm >= decel_cm)
    return cruise_effort;

  if (remaining_cm <= terminal_cm)
  {
    const float fraction = terminal_cm > 0.0f
        ? remaining_cm / terminal_cm : 0.0f;
    return terminal_effort + (decel_effort - terminal_effort) * fraction;
  }

  const float progress = 1.0f -
      (remaining_cm - terminal_cm) / (decel_cm - terminal_cm);
  const float exponent = fmaxf(0.01f, LINE_DECEL_EXPONENT);
  return cruise_effort - (cruise_effort - decel_effort) *
      powf(progress, exponent);
}

static float straight_boost_terminal(float terminal_effort,
                                     float decel_effort, float cruise_effort)
{
  const float limit = fmaxf(0.0f, fminf(decel_effort, cruise_effort));
  terminal_effort = fminf(limit, fmaxf(0.0f, terminal_effort));
  // A low stall cap never reduces the configured terminal floor.
  const float cap = fminf(limit,
      fmaxf(terminal_effort, LINE_STALL_MAX_EFFORT));
  return fminf(cap, terminal_effort +
      fmaxf(0.0f, LINE_STALL_BOOST_STEP_EFFORT));
}

/* --- Gyro-based turn control ------------------------------------- luther
 *
 * Motor PWM is inverted by the H-bridge wiring used below:
 *   compare == PWM_OFF -> brake / zero drive
 *   smaller compare    -> more drive
 *
 * For that reason the controller works in positive "effort counts" and only
 * converts effort to a timer compare value at the final output step.
 */
volatile float arc_target_angle = 0.0f;

#define ARC_KP_EFFORT_PER_DEG      130.0f
#define ARC_KI_EFFORT_PER_DEG_S      0.0f  // Leave at zero while tuning to avoid added overshoot
#define ARC_KD_EFFORT_PER_DPS        0.0f
#define ARC_I_LIMIT_DEG_S           40.0f

#define ARC_OUTER_EFFORT_MAX      6400.0f  // v45: entry90 retains its separate lower ceiling below.
#define ARC_OUTER_EFFORT_MIN      2000.0f
#define ARC_INNER_EFFORT_RATIO_R     0.15f
#define ARC_INNER_EFFORT_RATIO_L     0.35f

/* Exit limits for queued segment-2 forward turns and their following straight.
 * These are PWM-effort limits, not measured cm/s limits. No stop is inserted. */
#define SECTION2_EXIT_CAP_ENABLE                 1
#define SECTION2_EXIT_CAP_FORWARD_LEGS           0 // Return F runs at cruise until parking-entry slowdown.
#define SECTION2_EXIT_TURN_WINDOW_DEG          25.0f
#define SECTION2_EXIT_TURN_MAX_EFFORT         1400.0f // Fallback for non-image forward turns.
/* Image-entry90 keeps higher effort through its exit, then carries the
 * combined applied effort into its IR pass. Effort counts are not cm/s. */
#define FU2_ENTRY90_EXIT_MAX_EFFORT         3200.0f
/* Prepare for the opposite-direction180 before leaving the first90.
 * Unwind steering and wheel split together without adding a straight leg. */
#define FU2_ENTRY90_UNWIND_ENABLE               1
#define FU2_ENTRY90_UNWIND_START_DEG           18.0f
#define FU2_ENTRY90_UNWIND_MIN_FRACTION         0.35f
static float entry90_turn_fraction = 1.0f;
static uint8_t entry90_unwind_announced = 0U;
#define SECTION2_ENTRY_CARRY_ENABLE                1
#define SECTION2_ENTRY_CARRY_MAX_EFFORT_L        2800.0f
#define SECTION2_ENTRY_CARRY_MAX_EFFORT_R        3000.0f
#define SECTION2_ENTRY_CARRY_EFFORT_SCALE          1.20f
#define SECTION2_ENTRY_CARRY_BALANCE_HEADROOM     200.0f
#define SECTION2_180_ENTRY_OUTER_EFFORT_MAX      7500.0f
#define SECTION2_180_ENTRY_BLEND_MS                75U
static uint8_t section2_entry_carry_active = 0U;
static float section2_entry_carry_effort_l = 0.0f, section2_entry_carry_effort_r = 0.0f;
static float section2_180_entry_outer_effort = 0.0f;
/* Do not finish the180 launch blend while still rotating out of the preceding
 * opposite90. Keep rolling under a lower effort until the gyro releases it. */
#define SECTION2_180_OPPOSING_YAW_LIMIT_ENABLE       1
#define SECTION2_180_OPPOSING_YAW_MAX_DPS          10.0f
#define SECTION2_180_OPPOSING_YAW_MAX_EFFORT     2800.0f
static uint8_t section2_180_opposing_yaw_pending = 0U;
static uint32_t section2_180_entry_blend_tick = 0U;
#define SECTION2_REAR90_EXIT_TURN_MAX_EFFORT   6400.0f
#define SECTION2_REAR90_MIN_EFFORT            6400.0f
#define SECTION2_REAR90_MAX_EFFORT            6400.0f
#define SECTION2_REAR90_ENTRY_MAX_EFFORT      6400.0f
#define SECTION2_REAR90_ENTRY_BLEND_MS           60U
#define SECTION2_REAR90_ORDINARY_STEER_SETTLE_MS  80U
static float rear_turn_entry_outer_effort = 0.0f;
/* Rear-edge90 keeps its combined drive request; unwind near its end to
 * reduce residual yaw before the return without the old low-effort tail. */
#define SECTION2_REAR90_UNWIND_ENABLE             1
#define SECTION2_REAR90_UNWIND_START_DEG          30.0f
#define SECTION2_REAR90_UNWIND_MIN_FRACTION        0.25f // Keep yaw authority while finishing the last degrees.
static float rear_turn_turn_fraction = 1.0f;
static uint8_t rear_turn_unwind_announced = 0U;
#define SECTION2_EXIT_STRAIGHT_MAX_EFFORT_L    FORWARD_DECEL_EFFORT_L
#define SECTION2_EXIT_STRAIGHT_MAX_EFFORT_R    FORWARD_DECEL_EFFORT_R
#define SECTION2_EXIT_MIN_HOLD_MS              500U
#define SECTION2_EXIT_STABLE_MS                100U
#define SECTION2_EXIT_STABLE_YAW_DPS            15.0f
#define SECTION2_EXIT_STABLE_ERROR_DEG           3.0f
/* Retain the 180-degree arc's combined applied effort through the following
 * IR pass, redistributed to the calibrated straight-wheel ratio. */
#define SECTION2_REAR_CARRY_ENABLE               1
#define SECTION2_REAR_CARRY_EFFORT_SCALE          1.15f
#define SECTION2_REAR_STEADY_TURN_ENABLE          1
#define SECTION2_REAR_STEADY_OUTER_EFFORT      7500.0f // Scoped steady180 ceiling; ordinary turns retain theirs.
/* Ease only the steady180 arc: scale steering offset AND wheel split
 * together. 1.0 restores v40 curvature; smaller values request a wider arc.
 * This is a command fraction, not a calibrated radius multiplier. */
#define SECTION2_180_ARC_TURN_FRACTION             0.80f // Slightly wider curve; retain the existing combined drive effort.
#define SECTION2_REAR_HANDOFF_LOOKAHEAD_S         0.12f // Release arc early; v31 straight prediction uses the same horizon.
#define SECTION2_REAR_HANDOFF_MAX_DEG            20.0f // Bound the rate-based lead; not an obstacle-size allowance.
/* Begin straightening WHILE the steady180 is still executing. Keep a small
 * turn request until its gyro endpoint can hand off, avoiding a straight
 * stall short of the heading target. Both mirrors use the same fraction. */
#define SECTION2_180_PROGRESSIVE_UNWIND_ENABLE      1
#define SECTION2_180_UNWIND_START_DEG             40.0f
#define SECTION2_180_UNWIND_MIN_FRACTION           0.12f
/* Do not release to IR while the predictive lead still leaves a large
 * heading error. Keep the existing IR feedback prediction separately. */
#define SECTION2_180_EXIT_MAX_REMAIN_DEG           3.0f
static float section2_180_turn_fraction = 1.0f;
static uint8_t section2_180_unwind_announced = 0U;
/* Rear straight predicts the same yaw travel used to release the 180 early.
 * Otherwise heading P asks for MORE turn while the robot is still short of
 * the target, fighting the intended unwind. Bounded, mirrored, no brake. */
#define SECTION2_REAR_UNWIND_ENABLE                1
#define SECTION2_REAR_UNWIND_LOOKAHEAD_S           SECTION2_REAR_HANDOFF_LOOKAHEAD_S
#define SECTION2_REAR_UNWIND_MAX_DEG               SECTION2_REAR_HANDOFF_MAX_DEG
#define SECTION2_REAR_UNWIND_STABLE_MS           100U
static uint8_t rear_unwind_active = 0U, rear_unwind_seed_pending = 0U;
static uint32_t rear_unwind_stable_tick = 0U;
#define SECTION2_REAR_CARRY_BALANCE_HEADROOM    200.0f
static uint8_t section2_rear_steady_turn_active = 0U;
static uint8_t section2_rear_carry_active = 0U;
static float section2_rear_carry_effort_l = 0.0f, section2_rear_carry_effort_r = 0.0f;
static uint8_t section2_exit_turn_limited = 0U, section2_exit_straight_limited = 0U;
static uint32_t section2_exit_start_tick = 0U, section2_exit_stable_tick = 0U;

#define ARC_FINISH_TOL_DEG           0.1f //tighten the finish tolerance by decreasing
#define ARC_BRAKE_LOOKAHEAD_S        0.0855f // main parameter for consistent overshoot, Overshooting: increase look-ahead
#define ARC_SETTLE_MS               150U
#define ARC_RETRY_SERVO_MS           80U
#define ARC_MAX_APPROACHES            3U
#define ARC_TIMEOUT_BASE_MS        1500U
#define ARC_TIMEOUT_PER_DEG_MS       80U
#define ARC_TRACE_MS                250U
static uint32_t arc_trace_tick = 0U;
// Fail a commanded turn that makes too little yaw progress, rather than
// driving almost straight until the much longer full-angle timeout.
#define ARC_PROGRESS_CHECK_ENABLE       1
#define ARC_PROGRESS_MIN_DEG            3.0f
#define ARC_PROGRESS_TIMEOUT_MS       900U
#define ARC_PROGRESS_MIN_REMAIN_DEG    10.0f
static uint32_t turn_progress_tick = 0U;
static float turn_progress_heading = 0.0f;

#define IMU_PERIOD_MS                10U
#define IMU_RATE_DEADBAND_DPS         0.20f
#define IMU_STALE_TIMEOUT_MS         100U
#define IMU_RETRY_DELAY_MS          1000U
#define IMU_MAX_CONSECUTIVE_FAILURES   3U

#define STEP_GAP_MS 0
#define ARC_DELAY 0
#define ARC_DELAY_LEFT 0
#define ARC_DELAY_RIGHT 0
#define SLIDE_CMD_MS 350

/* SmoothFlow tuning; effort = PWM_OFF - compare. Reversal crosses zero drive. */
#define SMOOTH_ACCEL_EFFORT_PER_S       20000.0f
#define POST_IMAGE_ACCEL_EFFORT_PER_S   36000.0f
#define PARK_RETURN_ACCEL_EFFORT_PER_S  48000.0f // First finite PARK forward leg only.
#define SMOOTH_DECEL_EFFORT_PER_S       36000.0f
#define SMOOTH_TURN_HANDOFF_LOOKAHEAD_S     0.015f
#define SMOOTH_TURN_HANDOFF_TOL_DEG         0.5f
#define SMOOTH_TURN_EXIT_BALANCE             1
#define ROLLING_TURN_ENTRY_EFFORT_L         FORWARD_DECEL_EFFORT_L
#define ROLLING_TURN_ENTRY_EFFORT_R         FORWARD_DECEL_EFFORT_R
#define ROLLING_TURN_ENTRY_BLEND_MS         300U
#define ROLLING_TURN_OUTER_EFFORT_MAX      5500.0f
#define ROLLING_TURN_OUTER_EFFORT_MIN      1000.0f
#define ROLLING_TURN_INITIAL_OUTER_EFFORT  2000.0f
/* The final two PARK forward arcs use an independent profile. */
#define PARK_TURN_CARRY_SPEED_ENABLE          1 // Forward continuations: steady combined wheel effort.
#define PARK_TURN_OUTER_EFFORT_MAX         6900.0f
#define PARK_TURN_EXIT_EASE_START_DEG        15.0f
#define PARK_TURN_EXIT_MIN_EFFORT_SCALE       0.80f
#define PARK_TURN_OUTER_EFFORT_MIN         3200.0f
#define PARK_TURN_INITIAL_OUTER_EFFORT     3200.0f
#define PARK_TURN_ENTRY_BLEND_MS            30U
#define PARK_TURN_ENTRY_EFFORT_L           3200.0f
#define PARK_TURN_ENTRY_EFFORT_R           3400.0f
#define PARK_TURN_CARRY_ENTRY_MAX_EFFORT    6700.0f
static float parking_entry_outer_effort = PARK_TURN_INITIAL_OUTER_EFFORT;
#define PARK_TURN_UNWIND_ENABLE                  1
#define PARK_TURN_UNWIND_START_DEG              30.0f
#define PARK_TURN_UNWIND_MIN_FRACTION            0.25f
/* The first parking90 precedes the cross-bay correction. Its old near-centre
 * tail kept driving for several seconds while barely changing heading. */
#define PARK_FIRST90_UNWIND_START_DEG           30.0f
#define PARK_FIRST90_UNWIND_MIN_FRACTION          0.25f
static float park_turn_fraction = 1.0f;
static uint8_t parking_unwind_announced = 0U;
#define PARK_RETURN_ENTRY_MAX_EFFORT_L     4200.0f
#define PARK_RETURN_ENTRY_MAX_EFFORT_R     4500.0f
#define PARK_FORWARD_USE_OUTPUT_RAMP          1 // Cruise immediately through the existing output slew.
#define PARK_RETURN_CRUISE_EFFORT_SCALE       1.53f // Near calibrated PWM ceiling; first PARK leg only.
#define PARK_RETURN_SLOWDOWN_ENABLE             1 // Slow the return before PARK90 even with turn carry enabled.
#define PARK_RETURN_DECEL_CM                 30.0f // Return-only ramp toward PARK_TURN_ENTRY_EFFORT_L/R.
static uint8_t moving_turn_pending = 0U, moving_turn_active = 0U;
/* v26: dispatch the rear-edge FL90/FR90 through the ordinary turn path.
 * Set to0 only to reproduce the archived braked/direct rear90 behavior.
 * PARK entries remain independently selected below. */
#define REAR90_USE_ORDINARY_TURN          1
/* Legacy rear90 settings apply only when REAR90_USE_ORDINARY_TURN is0. */
#define REAR90_STEERING_SETTLE_MS       200U // v18 timing trial: allow more motor-braked steering time.
// Restore the successful v10 normal angle-based turn, with a modestly
// higher finish floor to reduce its slow tail. No constant-effort rear90 law.
#define REAR90_OUTER_EFFORT_MAX       5000.0f
#define REAR90_OUTER_EFFORT_MIN       1650.0f
#define FU2_ENTRY90_OUTER_EFFORT_MIN  3200.0f // Modest tail before the opposite-direction180.
#define FU2_ENTRY90_OUTER_EFFORT_MAX  5300.0f // Image-entry90 has its own ceiling.
static uint8_t fu2_entry90_active = 0U;
static uint8_t rear_turn_direct_active = 0U;
static uint8_t rear_turn_ordinary_active = 0U; // Context selects exit cap only; never selects direct drive.
// v27: both PARK turns use normal output slew with no timed braked entry.
// Their slower effort profile still applies; setting0 restores v26 entries.
#define PARK_CONTINUOUS_TURNS                 1
// Legacy entry times apply only when PARK_CONTINUOUS_TURNS is0.
#define PARK_FIRST90_STEERING_SETTLE_MS     100U
#define PARK_FINAL90_STEERING_SETTLE_MS      50U
static uint8_t parking_90_started = 0U, parking_first90_active = 0U, parking_final90_active = 0U;
static uint8_t turn_steering_entry_pending = 0U;
static uint8_t park_turn_profile = 0U;
static uint32_t turn_motion_start_tick = 0U;
static uint8_t turn_odom_active = 0U;
static uint16_t turn_odom_last_a = 0U, turn_odom_last_b = 0U;
static float turn_odom_last_heading = 0.0f;
static float turn_forward_cm = 0.0f, turn_left_cm = 0.0f;
static int32_t turn_wheel_counts_a = 0, turn_wheel_counts_b = 0;


// --- Steering mode (no angles; tokens like "fr", "fl") ---
typedef enum
{
  TURN_NO_SIDE = 0,
  TURN_TO_LEFT,
  TURN_TO_RIGHT
} turn_side_t;
volatile turn_side_t cmd_turn = TURN_NO_SIDE;

// luther CCR
#define STEER_CENTER_CCR 152            // Straight steering centre.
#define STEER_CENTER_AFTERLEFT_CCR 163 // Existing calibrated setting.
#define STEER_CENTER_AFTERRIGHT_CCR 150 // Existing calibrated setting.
#define STEER_RIGHT_CCR 240             // Forward-right endpoint.
#define STEER_LEFT_CCR 110              // Forward-left endpoint.
#define STEER_REVERSE_LEFT_CCR 110
#define STEER_REVERSE_RIGHT_CCR 240

/* Only the first image-recognition90 uses this modest steering increase.
 * The180 scales normal endpoints with its arc fraction; rear90 uses them
 * directly. PARK has its own endpoints below. */
#define FU2_ENTRY90_TIGHT_ARC_ENABLE       1
#define FU2_ENTRY90_SERVO_RIGHT_CCR      245
#define FU2_ENTRY90_SERVO_LEFT_CCR       105
/* Parking endpoints can be tuned separately from the ordinary turn profile. */
#define PARK_TIGHT_ARC_ENABLE           1 // Parking-only steering and wheel profile.
/* Use the endpoints that work for ordinary forward arcs. More steering
 * travel can load the linkage or scrub the tyres instead of tightening. */
#define PARK_SERVO_RIGHT_CCR          STEER_RIGHT_CCR
#define PARK_SERVO_LEFT_CCR           STEER_LEFT_CCR
#define PARK_INNER_EFFORT_RATIO_R     ARC_INNER_EFFORT_RATIO_R
#define PARK_INNER_EFFORT_RATIO_L     ARC_INNER_EFFORT_RATIO_L

/* Reverse straight-line centres depend on the last steering direction.
 * The after-right centre is calibrated separately from forward travel. */
#define STEER_REV_CENTER_AFTERLEFT_CCR  STEER_CENTER_AFTERLEFT_CCR
#define STEER_REV_CENTER_AFTERRIGHT_CCR 151

#define ARC_SERVO_ONLY_TEST       0 // for testing
#define ARC_SERVO_TEST_HOLD_MS    10000U //Steering hold time during the test

#define SPEED_FILTER_TAU 0.05f // Encoder-speed filter time constant: 50 ms

#define STEER_HOME_RIGHT_CCR              STEER_RIGHT_CCR // Homing endpoint before returning to centre
#define STEER_HOME_LEFT_CCR               STEER_LEFT_CCR // for testing
#define STEERING_HOME_BRAKE_SETTLE_MS     50U // Let the brake engage before moving the steering
#define STEERING_HOME_END_HOLD_MS         600U //Time to hold the steering on the right
#define STEERING_HOME_CENTER_SETTLE_MS    500U //Wait for the steering linkage to settle

/* Heading hold during straight travel. These values act on the steering servo only;
 * wheel-speed PI remains responsible for balancing encoder speeds. */
#define LINE_STEER_DEADBAND_DEG         0.01f
#define LINE_STEER_MIN_PERCENT          0.01f
#define LINE_STEER_MAX_PERCENT          40.0f
/* Heading-hold gains are selected by direction of travel.
 * Ki is zero in both directions; steering uses proportional and yaw-rate feedback. */
#define FORWARD_STEER_KP_PERCENT_PER_DEG   5.6f
#define FORWARD_STEER_KI_PERCENT_PER_DEG_S 0.0f
#define FORWARD_STEER_KD_PERCENT_PER_DPS   0.34f

#define REVERSE_STEER_KP_PERCENT_PER_DEG   5.6f
#define REVERSE_STEER_KI_PERCENT_PER_DEG_S 0.0f
#define REVERSE_STEER_KD_PERCENT_PER_DPS   0.34f

/* How a steering percentage is converted into servo CCR counts.
 *   0 = original behaviour: percent is scaled by the travel available on the
 *       side being requested (centre->left or centre->right). With centre 150,
 *       left 110, right 240 that is 40 vs 90 counts, so the same % gives very
 *       different CCR changes on each side.
 *   1 = symmetric: both sides use the SMALLER of the two spans, so +x% and -x%
 *       give the same CCR change from centre.
 * Both directions currently use the side-specific mapping (0). */
#define FORWARD_STEER_SYMMETRIC_MAP 0
#define REVERSE_STEER_SYMMETRIC_MAP 0

#define LINE_STEER_I_LIMIT_PERCENT      20.0f // Limit the integral term to prevent windup
#define LINE_STEER_SLEW_PERCENT_PER_S  90000.0f // Maximum steering correction change per second
#define LINE_STEER_D_LPF_TAU_S 0.050f // Yaw-rate filter time constant; larger values smooth more

/* IR-only rolling heading hold: initial tuning values, not measured gains. */
#define IR_HOLD_DRIVE_PERCENT            100.0f
#define IR_HOLD_KP_PERCENT_PER_DEG          4.0f
#define IR_HOLD_KI_PERCENT_PER_DEG_S        0.0f
#define IR_HOLD_KD_PERCENT_PER_DPS          FORWARD_STEER_KD_PERCENT_PER_DPS
#define IR_HOLD_DEADBAND_DEG                0.30f
#define IR_HOLD_MAX_PERCENT                40.0f
#define IR_HOLD_SLEW_PERCENT_PER_S         300.0f
#define IR_HOLD_RELEASE_PERCENT_PER_S     1200.0f
#define IR_HOLD_D_LPF_TAU_S                  0.025f
#define IR_HOLD_SYMMETRIC_MAP                FORWARD_STEER_SYMMETRIC_MAP
#define IR_HOLD_WHEEL_ENTRY_MS             200U
#define IR_HOLD_WHEEL_ENTRY_MAX_DPS         15.0f
#define IR_HOLD_WHEEL_CORR_MAX              300
#define IR_HOLD_TRACE_MS                   250U

static uint8_t ir_motion_active = 0U;
static uint8_t ir_hold_heading_lock_valid = 0U;
static uint8_t ir_feedback_keep = 0U;
static float ir_hold_heading_reference = 0.0f;
static uint32_t ir_hold_start_tick = 0U;
static uint32_t ir_hold_trace_tick = 0U;
#define FU_HEADING_TRACE_MS                  250U
static uint8_t fu_slide_heading_active = 0U;
static uint32_t fu_heading_trace_tick = 0U;

/* Only the forward FU2 following the slide pair. Restore recovery authority
 * after v16 left a persistent slant; retain its yaw damping. Reverse,
 * rear IR and ordinary/parking straights keep their existing gain sets. */
#define FU2_HOLD_KP_PERCENT_PER_DEG          5.6f // Restore recovery authority; v16 stayed slanted into FL90.
#define FU2_HOLD_KI_PERCENT_PER_DEG_S        0.0f
#define FU2_HOLD_KD_PERCENT_PER_DPS          0.45f
#define FU2_HOLD_MAX_PERCENT                40.0f
#define FU2_HOLD_D_LPF_TAU_S                 0.025f
#define FU2_HOLD_SLEW_PERCENT_PER_S         600.0f
#define FU2_HOLD_RELEASE_PERCENT_PER_S     1200.0f

/* Final FX only: damp turn-exit yaw without the ordinary forward loop's
 * stronger proportional recovery. Drive remains constant until detection. */
#define PARKING_FX_HOLD_KP_PERCENT_PER_DEG       4.0f
#define PARKING_FX_HOLD_KI_PERCENT_PER_DEG_S     0.0f
#define PARKING_FX_HOLD_KD_PERCENT_PER_DPS       0.45f
#define PARKING_FX_HOLD_D_LPF_TAU_S              0.025f

/* Keep the intended axis through connected route commands. Activated only
 * by FU after the slide pair; ordinary stand-alone/manual turns stay relative
 * to their measured start. FR/FL keep the existing arc controller and radius. */
static uint8_t route_axis_valid = 0U;
static float route_axis_heading = 0.0f;
static uint8_t route_entry_hold_pending = 0U;
static uint32_t route_entry_stable_tick = 0U;
static uint32_t route_heading_trace_tick = 0U;
#define ROUTE_ENTRY_MAX_EFFORT_L       FORWARD_DECEL_EFFORT_L
#define ROUTE_ENTRY_MAX_EFFORT_R       FORWARD_DECEL_EFFORT_R
#define ROUTE_ENTRY_MAX_YAW_DPS           15.0f
#define ROUTE_ENTRY_MAX_ERROR_DEG          3.0f
#define ROUTE_ENTRY_STABLE_MS             60U
/* FU2 with fresh forward space only: allow small steady calibration bias
 * to clear the launch gate. Close approaches retain the ordinary3deg rule. */
#define FU2_FAR_LAUNCH_MAX_ERROR_DEG       6.0f
#define FU2_FAR_LAUNCH_MIN_RANGE_CM       FU2_FAR_MIN_RANGE_CM
static float route_entry_effort_l = ROUTE_ENTRY_MAX_EFFORT_L;
static float route_entry_effort_r = ROUTE_ENTRY_MAX_EFFORT_R;



/* Integrator gating: only learn the steering bias while cruising, never during
 * launch or decel, and never while the heading error is large (avoids winding
 * up on the launch transient). */
#define LINE_STEER_I_GATING_ENABLE 		 0
#define LINE_STEER_I_MIN_DRIVE_PERCENT   60.0f // Update the integral only above this drive level
#define LINE_STEER_I_ERR_BAND_DEG        2.0f // Heading error must also stay within this band

/* Steering behaviour once planned deceleration begins.
 * 0 = heading PID keeps running through the decel (previous behaviour).
 * 1 = PID corrections (P and D) are dropped at decel start; the servo HOLDS the
 *     learned bias trim (the integral term) so the car keeps its straight line
 *     without reacting to gyro noise / the decel disturbance. Integrator is frozen. */
#define LINE_DECEL_STEER_HOLD           0

// diagnostic flags
#define LINE_DIAG_FIXED_SERVO          0 // 0 = full PID steering active (recommended); 1 = bypass, for A/B testing only
#define LINE_DIAG_OFFSET_CCR           0 // Fixed steering offset
#define LINE_DIAG_FIXED_SERVO_REVERSE  0 // 1 = the fixed-servo diagnostic also applies to reverse (use to calibrate SERVO_REV_CENTER_*)
#define WHEEL_PI_DIAG_DISABLE 			   0   // 0 = normal (recommended), 1 = force off=0 for isolation testing; off_f/err/i_acc still computed & logged
#define STEERING_DIAG_HOME_FROM_LEFT       0 // Select the left-side homing diagnostic

#if LINE_DIAG_FIXED_SERVO != 0 && LINE_DIAG_FIXED_SERVO != 1
#error "STRAIGHT_DIAG_FIXED_SERVO must be 0 or 1"
#endif
#if LINE_DIAG_FIXED_SERVO_REVERSE != 0 && LINE_DIAG_FIXED_SERVO_REVERSE != 1
#error "STRAIGHT_DIAG_FIXED_SERVO_REVERSE must be 0 or 1"
#endif
#if LINE_DIAG_OFFSET_CCR < -6 || LINE_DIAG_OFFSET_CCR > 6
#error "Keep the diagnostic steering offset within -6 to +6 CCR"
#endif

#if STEERING_DIAG_HOME_FROM_LEFT != 0 && STEERING_DIAG_HOME_FROM_LEFT != 1
#error "STEERING_DIAG_HOME_FROM_LEFT must be 0 or 1"
#endif

// post brake telemetry
#define LINE_POST_STOP_TELEMETRY_MS 300U
static volatile uint32_t straight_log_deadline = 0U;

/* float is atomic on this STM32 and has ample precision for robot yaw. */
volatile float total_angle = 0.0f;  // updated in gyroTask
/* Heading captured when a straight F/R movement begins. */
volatile float target_angle = 0.0f;
volatile float error_angle = 0.0f;  // computed in motorTask
volatile float TURN_DEG = 90.0f;

// Gyro and turn-controller state exposed to the telemetry task.
volatile float gyro_bias_dps = 0.0f;
volatile float gyro_yaw_rate_dps = 0.0f;
volatile uint8_t gyro_healthy = 0U;
volatile uint32_t gyro_last_good_tick = 0U;

//luther turn start
typedef enum
{
  ARC_IDLE = 0,
  ARC_RUNNING,
  ARC_SETTLING
} arc_phase_t;

volatile arc_phase_t turn_control_phase = ARC_IDLE;
volatile float turn_error_deg = 0.0f;
volatile float turn_effort_cmd = 0.0f;
volatile int turn_left_pwm = PWM_OFF;
volatile int turn_right_pwm = PWM_OFF;

/* Straight-controller outputs exposed to EncoderTask telemetry. */
volatile int straight_left_pwm = PWM_OFF;
volatile int straight_right_pwm = PWM_OFF;
volatile int straight_servo_ccr = STEER_CENTER_CCR;
volatile int straight_servo_center_ccr = STEER_CENTER_CCR;
static volatile uint8_t steering_ready = 0U;
volatile float straight_steer_percent = 0.0f; // Steering request as a percentage of available travel: left negative, right positive

volatile float straight_steer_p_percent = 0.0f;
volatile float straight_steer_i_percent = 0.0f;
volatile float straight_steer_d_percent = 0.0f;
volatile float straight_steer_correction_percent = 0.0f;



static float arc_integral_deg_s = 0.0f;
static uint32_t arc_pid_tick = 0U;
static uint32_t arc_settle_deadline = 0U;
static uint32_t arc_deadline = 0U;
static uint8_t arc_attempts = 0U;
// luther turn end

volatile uint32_t turn_motor_enable_tick = 0; // when motors are allowed to start for turns

#define OBST_STOP_CM 26
#define OBST_REV_CM 20

#define US_BACK_CLEAR_CM 20

volatile uint8_t arc_postforward_cm = 0;

// Gyro scale calibration. 0.989 means a raw 91-degree reading is treated as 90.
volatile float GYRO_SCALE_TRIM = 0.989f;

// ir
// ADC1 DMA buffer (PC0->ADC1_IN10, PC1->ADC1_IN11)
volatile uint16_t ir_adc[2] = {0, 0};

// ADC callback owns the obstacle flags; IR_Task updates display voltages.
volatile uint16_t ir0_raw = 0, ir1_raw = 0;
volatile float ir0_v = 0.0f, ir1_v = 0.0f;
volatile uint8_t ir0_obs = 0, ir1_obs = 0; // obstacle flags

// threshold voltage
#define IR_VOLT_OBS 0.80f // should be 30 cm for GP2Y0A21 (tune in test)

// Integer millivolts for the ADC obstacle checks.
volatile uint16_t ir0_mV = 0, ir1_mV = 0; // 0..3300
#define IR_MV_OBS 800                     // 0.80 V threshold.

// obstacle mode
#define STOP_MODE_NONE 0
#define STOP_MODE_US 1         // ultrasound
#define STOP_MODE_IR_RIGHT 2   // stop when right IR says "no obstacle" (0)
#define STOP_MODE_IR_LEFT 3    // stop for left IR
#define STOP_MODE_US_BACK 4    // reverse forever until us
#define STOP_MODE_IR_RIGHT_O 5 // stop when right IR says "obstacle" (1)
#define STOP_MODE_IR_LEFT_O 6
#define STOP_MODE_US_T 7 // for FU

volatile uint8_t obstacle_stop_mode = STOP_MODE_NONE;

volatile uint16_t us_stop_cm = 26;     // set by FUxx each time, default 26
volatile uint8_t us_allow_reverse = 1; // 1=FU (reverse on overshoot), 0=FX (just stop)
static uint8_t park_fx_active = 0U, park_fx_has_range = 0U;
static uint32_t park_fx_start_tick = 0U, park_fx_last_valid_tick = 0U;
static uint32_t park_fx_sample_seq = 0U, park_fx_range_cm = 0U;
static uint32_t park_fx_trace_tick = 0U;
static float park_fx_anchor_range_cm = 0.0f;
static uint8_t park_fx_range_confirmed = 0U, park_fx_confirm_samples = 0U;
static uint32_t park_fx_confirm_low_cm = 0U, park_fx_confirm_high_cm = 0U;
static int32_t park_fx_range_counts = 0, park_fx_confirm_start_counts = 0;
/* --- Odometry --- */
volatile int32_t odom_counts_run = 0; // counts accumulated in the current move
volatile float odom_cm_run = 0.0f; // cached last segment distance (cm)

volatile int32_t odom_counts_run_ir = 0;
volatile float odom_cm_run_ir = 0.0f;

volatile uint8_t seg_rebase = 0; // 1 = rebase last_a/last_b/last_t on next motor tick

// --- Slide control ---
typedef enum
{
  SLIDE_NO_SIDE = 0,
  SLIDE_TO_RIGHT = 1,
  SLIDE_TO_LEFT = 2
} slide_side_t;
typedef enum
{
  SLIDE_IDLE = 0,
  SLIDE_OUTWARD,
  SLIDE_INWARD,
  SLIDE_CLEARANCE
} slide_stage_t;

volatile slide_side_t slide_mode = SLIDE_NO_SIDE;     // active slide direction
volatile slide_stage_t slide_phase = SLIDE_IDLE; // OUT -> IN -> optional CLEARANCE
volatile double slide_origin_heading = 0.0;   // heading at slide start
volatile double slide_return_heading = 0.0;

#define SLIDE_ANGLE_DEG 45.0f
#define SLIDE_FIRST_PAIR_ANGLE_DEG 45.0f // The first slide includes both outward and inward bends.
static float slide_active_angle_deg = SLIDE_ANGLE_DEG;
#define SLIDE_OUT_REVERSAL_LOOKAHEAD_S 0.14f
#define SLIDE_OUT_REVERSAL_MAX_DEG     15.0f
#define SLIDE_PAIR_CLEARANCE_CM       9.0f // Forward gap before the next slide in the opposite direction.
#define SLIDE_PHASE_TIMEOUT_MS 5000U
#define SLIDE_MAX_YAW_MARGIN_DEG 30.0f
static uint32_t slide_stage_start_tick = 0U;
static float paired_slide_heading = 0.0f;
static uint8_t paired_slide_heading_valid = 0U;
static int32_t slide_clearance_start_counts = 0;

static const char *script_fault_reply = NULL;
static char slide_fault_reply[80];
#define STEER_SLIDERIGHT_CCR 227 // Existing calibrated setting.
#define STEER_SLIDELEFT_CCR 116 // Existing calibrated setting.

// --- Slide PWM tunables
#define SLIDE_OUTER_PWM 5000 // Outer wheel PWM compare
#define SLIDE_INNER_PWM 6750 // Existing calibrated setting.
#define SLIDE_DRIVE_EFFORT_SCALE 2.00f // Slide effort multiplier; wheel effort ratio stays the same.
#define SLIDE_FINAL_IN_DECEL_DEG        20.0f
#define SLIDE_FINAL_IN_MIN_SCALE         0.70f
#define SLIDE_FINAL_HANDOFF_LEAD_S       0.20f
#define SLIDE_FINAL_HANDOFF_MIN_DEG      5.0f
#define SLIDE_FINAL_HANDOFF_MAX_DEG     15.0f

// attempt for US
static volatile uint8_t us_echo_started = 0; // 0 waits for the rising edge; 1 waits for the falling edge
static volatile uint16_t us_pulse_ticks = 0;  // Echo width in us (PSC=15 gives a 1 MHz counter)

// OLED pages
volatile uint8_t display_page = 0;
#define DISPLAY_PAGES 2   // Two display pages

// IR sensor distance
volatile float ir0_distance_cm = 0.0f;
volatile float ir1_distance_cm = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void ledTask(void const * argument);
void oledTask(void const * argument);
void motorTask(void const * argument);
void encoderTask(void const * argument);
void gyroTask(void const * argument);
void ultrasoundTask(void const * argument);
void irTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ultrasonic
float speedOfSound = 0.0343 / 2;

// delay for US
static inline void wait_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks)
  {
    __NOP();
  }
}

void DWT_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void wheels_forward(void)
{
  // MOTORA on TIM4: CH3 = IN2, CH4 = IN1
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_CRUISE);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF);

  // MOTORB on TIM9: CH1 = IN2, CH2 = IN1
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_CRUISE);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_OFF);
}

static inline void wheels_reverse(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_CRUISE);

  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_OFF);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_CRUISE);
}

static inline int limit_pwm(int value);
static uint8_t slew_output_active = 0U;
static float slew_left_request = 0.0f, slew_right_request = 0.0f;
static float slew_left_effort = 0.0f, slew_right_effort = 0.0f;
static uint32_t slew_output_tick = 0U;

static inline void brake_wheels(void) // Brake for completion, S, joystick stop, or a fault
{
  slide_clearance_start_counts = 0;
  section2_exit_turn_limited = section2_exit_straight_limited = 0U;
  section2_rear_carry_active = 0U;
  section2_entry_carry_active = 0U;
  rear_unwind_active = rear_unwind_seed_pending = 0U;
  rear_unwind_stable_tick = 0U;
  section2_rear_steady_turn_active = 0U;
  section2_180_opposing_yaw_pending = 0U;
  section2_180_entry_blend_tick = 0U;
  section2_exit_stable_tick = 0U;
  ir_motion_active = ir_hold_heading_lock_valid = ir_feedback_keep = 0U;
  slew_left_request = slew_right_request = 0.0f;
  slew_left_effort = slew_right_effort = 0.0f;
  slew_output_tick = HAL_GetTick();
  Telemetry_ClearWheelPi(); // Discard the pre-brake controller snapshot.
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF);

  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_OFF);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_OFF);
}

static inline void drive_left_forward(int duty)
{
  if (slew_output_active) { slew_left_request = (float)(PWM_OFF - limit_pwm(duty)); return; }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty);    // IN2 (PWM)
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF); // IN1 high -> FWD
}
static inline void drive_right_forward(int duty)
{
  if (slew_output_active) { slew_right_request = (float)(PWM_OFF - limit_pwm(duty)); return; }
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, duty);    // IN2 (PWM)
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_OFF); // IN1 high -> FWD
}
static inline void coast_left(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}
static inline void coast_right(void)
{
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
}

static inline void drive_left_reverse(int duty)
{
  if (slew_output_active) { slew_left_request = -(float)(PWM_OFF - limit_pwm(duty)); return; }
  // reverse on left: IN1 = PWM (CH4), IN2 = HIGH (CH3)
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF); // IN2 high
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty);    // IN1 pwm
}
static inline void drive_right_reverse(int duty)
{
  if (slew_output_active) { slew_right_request = -(float)(PWM_OFF - limit_pwm(duty)); return; }
  // reverse on right: IN1 = PWM (CH2), IN2 = HIGH (CH1)
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_OFF); // IN2 high
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, duty);    // IN1 pwm
}

static float drive_slew_rate(void)
{
  if (park_turn_profile && parking_90_started == 0U &&
      uart_cmd == DRIVE_FORWARD && obstacle_stop_mode == STOP_MODE_NONE &&
      target_counts > 0 && target_counts != INT32_MAX)
      return PARK_RETURN_ACCEL_EFFORT_PER_S;
  const bool post_image = section2_entry_carry_active || section2_rear_steady_turn_active ||
      section2_rear_carry_active || rear_turn_ordinary_active || park_turn_profile;
  return post_image
      ? POST_IMAGE_ACCEL_EFFORT_PER_S : SMOOTH_ACCEL_EFFORT_PER_S;
}

/* Consume the accepted clearance with encoder travel. A later longer echo
 * must not move the remembered wall farther away during a forward approach. */
static float park_fx_estimated_range_cm(void)
{
  return fmaxf(0.0f, park_fx_anchor_range_cm -
      fmaxf(0.0f, (float)(odom_counts_run - park_fx_range_counts) / ENCODER_COUNTS_PER_CM));
}

static void park_fx_effort_limits(float *left, float *right)
{
  if (!park_fx_range_confirmed)
  {
    *left = *right = 0.0f;
    return;
  }
  const float cruise = fmaxf(0.0f, PARKING_FX_CRUISE_EFFORT_SCALE);
  const float minimum = fminf(cruise, fmaxf(0.0f, PARKING_FX_MIN_EFFORT_SCALE));
  const float stop_cm = (float)us_stop_cm + PARKING_FX_STOP_MARGIN_CM;
  const float span = fmaxf(1.0f, PARKING_FX_DECEL_START_CM - stop_cm);
  const float fraction = fminf(1.0f, fmaxf(0.0f,
      (park_fx_estimated_range_cm() - stop_cm) / span));
  float scale = minimum + (cruise - minimum) * fraction;
  if (park_fx_estimated_range_cm() - stop_cm <= PARKING_FX_NEAR_BRAKE_ZONE_CM)
      scale = fminf(scale, fmaxf(minimum, PARKING_FX_NEAR_MAX_EFFORT_SCALE));
  *left = fminf(PWM_OFF, fmaxf(0.0f,
      (PWM_OFF - (float)PWM_CRUISE * FORWARD_LEFT_COMPARE_SCALE) * scale));
  *right = fminf(PWM_OFF, fmaxf(0.0f,
      (PWM_OFF - (float)PWM_CRUISE * FORWARD_RIGHT_COMPARE_SCALE) * scale));
}

static void start_park_fx(void)
{
  park_fx_active = 1U;
  park_fx_has_range = 0U;
  park_fx_range_confirmed = park_fx_confirm_samples = 0U;
  park_fx_range_counts = park_fx_confirm_start_counts = 0;
  park_fx_anchor_range_cm = 0.0f;
  park_fx_start_tick = HAL_GetTick();
  park_fx_last_valid_tick = park_fx_trace_tick = 0U;
  const uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  park_fx_sample_seq = us_capture_seq;
  __set_PRIMASK(irq_state);
  // No carried turn drive while the wall range is still unknown.
  brake_wheels();
  char message[64];
  int n = snprintf(message, sizeof(message), "PARK_US,ARM,%u,%u,%u\n",
      (unsigned)us_stop_cm, (unsigned)(us_stop_cm + PARKING_FX_STOP_MARGIN_CM),
      (unsigned)PARKING_FX_US_STALE_MS);
  if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
  n = snprintf(message, sizeof(message), "PARK_US,HOLD,%.2f,%.2f,%.3f\n",
      (double)PARKING_FX_HOLD_KP_PERCENT_PER_DEG,
      (double)PARKING_FX_HOLD_KD_PERCENT_PER_DPS, (double)PARKING_FX_HOLD_D_LPF_TAU_S);
  if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
  n = snprintf(message, sizeof(message), "PARK_US,APPROACH,%.2f,%.2f,%.0f\n",
      (double)PARKING_FX_CRUISE_EFFORT_SCALE, (double)PARKING_FX_MIN_EFFORT_SCALE,
      (double)PARKING_FX_DECEL_START_CM);
  if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
}

static void park_fx_accept_range(uint32_t range_cm, uint32_t tick)
{
  const float estimate = park_fx_has_range
      ? fminf((float)range_cm, park_fx_estimated_range_cm()) : (float)range_cm;
  park_fx_anchor_range_cm = estimate;
  park_fx_range_cm = (uint32_t)lroundf(estimate);
  park_fx_range_counts = odom_counts_run;
  park_fx_last_valid_tick = tick;
  park_fx_has_range = park_fx_range_confirmed = 1U;
}

static bool park_fx_confirm_range(uint32_t range_cm, uint32_t tick)
{
  uint32_t low = range_cm, high = range_cm;
  bool restart = park_fx_confirm_samples == 0U;
  if (!restart)
  {
    low = range_cm < park_fx_confirm_low_cm ? range_cm : park_fx_confirm_low_cm;
    high = range_cm > park_fx_confirm_high_cm ? range_cm : park_fx_confirm_high_cm;
    restart = high - low > PARKING_FX_CONFIRM_SPREAD_CM ||
        fabsf((float)(odom_counts_run - park_fx_confirm_start_counts)) >
            PARKING_FX_CONFIRM_MAX_MOVE_CM * ENCODER_COUNTS_PER_CM;
  }
  if (restart)
  {
    park_fx_confirm_low_cm = park_fx_confirm_high_cm = range_cm;
    park_fx_confirm_start_counts = odom_counts_run;
    park_fx_confirm_samples = 1U;
  }
  else
  {
    park_fx_confirm_low_cm = low;
    park_fx_confirm_high_cm = high;
    if (park_fx_confirm_samples < PARKING_FX_CONFIRM_SAMPLES) ++park_fx_confirm_samples;
  }
  if (park_fx_confirm_samples < PARKING_FX_CONFIRM_SAMPLES) return false;
  park_fx_accept_range(park_fx_confirm_low_cm, tick);
  park_fx_confirm_samples = 0U;
  char message[48];
  int n = snprintf(message, sizeof(message), "PARK_US,READY,%lu\n",
      (unsigned long)park_fx_range_cm);
  if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
  return true;
}

static void park_fx_hold_for_range(uint32_t raw_cm)
{
  if (park_fx_range_confirmed)
  {
    char message[48];
    int n = snprintf(message, sizeof(message), "PARK_US,WAIT,%lu\n", (unsigned long)raw_cm);
    if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
  }
  park_fx_range_confirmed = park_fx_confirm_samples = 0U;
  brake_wheels();
}

static bool update_park_fx_range(void)
{
  const uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  const uint32_t seq = us_capture_seq, tick = us_capture_tick;
  const uint32_t raw_cm = us_raw_cm;
  const float sample_heading = us_raw_heading;
  const uint16_t pulse_us = us_pulse_ticks;
  const bool valid = us_capture_valid != 0U;
  __set_PRIMASK(irq_state);
  const uint32_t now = HAL_GetTick();
  const bool fresh = valid && (uint32_t)(now - tick) <= PARKING_FX_SAMPLE_MAX_AGE_MS;
  const bool close = fresh && raw_cm <= (uint32_t)us_stop_cm + PARKING_FX_STOP_MARGIN_CM;
  const bool aligned = fabsf(sample_heading - target_angle) <= PARKING_FX_SAMPLE_MAX_ERROR_DEG;
  if (!fresh || (!close && !aligned)) park_fx_hold_for_range(raw_cm);
  if (seq != park_fx_sample_seq)
  {
    park_fx_sample_seq = seq;
    // A fresh close obstacle always stops us, even if heading is still settling.
    if (fresh && (close || aligned))
    {
      const bool first_range = !park_fx_has_range;
      if (close)
      {
        brake_wheels();
        park_fx_accept_range(raw_cm, tick);
      }
      else if (park_fx_range_confirmed)
      {
        if ((float)raw_cm > park_fx_estimated_range_cm() + PARKING_FX_RANGE_INCREASE_TOLERANCE_CM)
            park_fx_hold_for_range(raw_cm);
        else park_fx_accept_range(raw_cm, tick);
      }
      else if (fabsf(gyro_yaw_rate_dps) <= PARKING_FX_CONFIRM_MAX_YAW_DPS)
          park_fx_confirm_range(raw_cm, tick);
      else park_fx_confirm_samples = 0U;
      if (close || (first_range && park_fx_range_confirmed))
      {
        // Accepted raw echo: sample sequence, cm, pulse width us, age ms.
        char echo_msg[48];
        int n = snprintf(echo_msg, sizeof(echo_msg), "PARK_US,ECHO,%lu,%lu,%u,%lu\n",
            (unsigned long)seq, (unsigned long)raw_cm, (unsigned)pulse_us,
            (unsigned long)(now - tick));
        if (n > 0 && n < (int)sizeof(echo_msg)) queue_range_reply(echo_msg);
      }
    }
  }
  if (!park_fx_range_confirmed) brake_wheels();
  const uint32_t last_seen = park_fx_has_range
      ? park_fx_last_valid_tick : park_fx_start_tick;
  if ((uint32_t)(now - last_seen) > PARKING_FX_US_STALE_MS)
  {
    brake_wheels();
    Telemetry_RecordFault("PARK_US_STALE");
    script_fault_reply = "ERR:PARK_US_STALE\r\n";
    abort_now = 1U;
    return false;
  }
  if (PARKING_FX_TRACE_MS > 0U &&
      (uint32_t)(now - park_fx_trace_tick) >= PARKING_FX_TRACE_MS)
  {
    park_fx_trace_tick = now;
    char message[64];
    int n = snprintf(message, sizeof(message), "PARK_US,RANGE,%.1f,%lu,%u,%.1f,%.1f,%.1f\n",
        (double)(park_fx_has_range ? park_fx_estimated_range_cm() : 0.0f),
        (unsigned long)(now - last_seen), (unsigned)park_fx_range_confirmed,
        (double)target_angle, (double)total_angle, (double)gyro_yaw_rate_dps);
    if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
  }
  return true;
}

static float slew_effort(float current, float target, float dt_s)
{
  const float rate = fabsf(target) > fabsf(current)
      ? drive_slew_rate() : SMOOTH_DECEL_EFFORT_PER_S;
  const float step = fmaxf(0.0f, rate) * dt_s;
  return current + fminf(step, fmaxf(-step, target - current));
}

static bool route_entry_limiting(void)
{
  return route_axis_valid && route_entry_hold_pending &&
      uart_cmd == DRIVE_FORWARD && slide_mode == SLIDE_NO_SIDE &&
      (obstacle_stop_mode == STOP_MODE_US_T || obstacle_stop_mode == STOP_MODE_NONE);
}

static void start_route_straight(void)
{
  route_entry_hold_pending = route_axis_valid;
  route_entry_stable_tick = route_heading_trace_tick = 0U;
  route_entry_effort_l = ROUTE_ENTRY_MAX_EFFORT_L;
  route_entry_effort_r = ROUTE_ENTRY_MAX_EFFORT_R;
  if (route_axis_valid && park_turn_profile && parking_90_started == 0U &&
      slew_left_effort >= 0.0f && slew_right_effort >= 0.0f)
  {
      // finish_turn already balanced the rear90's applied outputs. Retain
      // that drive during heading settle instead of forcing a low restart.
      route_entry_effort_l = fminf(PARK_RETURN_ENTRY_MAX_EFFORT_L,
          fmaxf(ROUTE_ENTRY_MAX_EFFORT_L, slew_left_effort));
      route_entry_effort_r = fminf(PARK_RETURN_ENTRY_MAX_EFFORT_R,
          fmaxf(ROUTE_ENTRY_MAX_EFFORT_R, slew_right_effort));
      char carry_msg[48];
      int n = snprintf(carry_msg, sizeof(carry_msg), "RETURN_CARRY,%.0f,%.0f\n",
          (double)route_entry_effort_l, (double)route_entry_effort_r);
      if (n > 0 && n < (int)sizeof(carry_msg)) queue_range_reply(carry_msg);
  }
  if (!route_axis_valid) return;
  char message[48];
  int n = snprintf(message, sizeof(message), "FLOW_REF,%.1f,%.1f,%.1f\n",
      (double)target_angle, (double)total_angle, (double)gyro_yaw_rate_dps);
  if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
}

static float ir_leave_effort_ceiling_l(void)
{
  return section2_rear_carry_active
      ? fminf(PWM_OFF, section2_rear_carry_effort_l +
          fmaxf(0.0f, SECTION2_REAR_CARRY_BALANCE_HEADROOM))
      : section2_entry_carry_active
          ? fminf(PWM_OFF, section2_entry_carry_effort_l + SECTION2_ENTRY_CARRY_BALANCE_HEADROOM)
          : fminf(PWM_OFF, fmaxf(0.0f, SECTION2_EXIT_STRAIGHT_MAX_EFFORT_L));
}

static float ir_leave_effort_ceiling_r(void)
{
  return section2_rear_carry_active
      ? fminf(PWM_OFF, section2_rear_carry_effort_r +
          fmaxf(0.0f, SECTION2_REAR_CARRY_BALANCE_HEADROOM))
      : section2_entry_carry_active
          ? fminf(PWM_OFF, section2_entry_carry_effort_r + SECTION2_ENTRY_CARRY_BALANCE_HEADROOM)
          : fminf(PWM_OFF, fmaxf(0.0f, SECTION2_EXIT_STRAIGHT_MAX_EFFORT_R));
}

static float section2_exit_turn_effort_ceiling(void)
{
  const float requested = fu2_entry90_active ? FU2_ENTRY90_EXIT_MAX_EFFORT
      : rear_turn_ordinary_active ? SECTION2_REAR90_EXIT_TURN_MAX_EFFORT : SECTION2_EXIT_TURN_MAX_EFFORT;
  return fminf(PWM_OFF, fmaxf(0.0f, requested));
}

static float park_turn_effort_ceiling(uint32_t now)
{
  // Ease the last degrees to reduce carried yaw into the opposite turn or FX.
  const float exit_fraction = fminf(1.0f, fabsf(arc_target_angle - total_angle) /
      fmaxf(1.0f, PARK_TURN_EXIT_EASE_START_DEG));
  const float exit_minimum = fminf(1.0f, fmaxf(0.0f, PARK_TURN_EXIT_MIN_EFFORT_SCALE));
  const float maximum = fminf(PWM_OFF, fmaxf(0.0f, PARK_TURN_OUTER_EFFORT_MAX)) *
      (exit_minimum + (1.0f - exit_minimum) * exit_fraction);
  if (PARK_TURN_CARRY_SPEED_ENABLE && peek_motion_direction() == 1)
  {
    // A common combined effort avoids a drive dip between opposite arcs.
    // Bound it by the tighter wheel split: neither mirror may demand more
    // than the calibrated outer ceiling. PWM effort is not measured speed.
    const float ratio_r = PARK_TIGHT_ARC_ENABLE
        ? PARK_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_R;
    const float ratio_l = PARK_TIGHT_ARC_ENABLE
        ? PARK_INNER_EFFORT_RATIO_L : ARC_INNER_EFFORT_RATIO_L;
    const float ratio = uart_cmd == DRIVE_FR ? ratio_r : ratio_l;
    const float combined = maximum * (1.0f + fminf(ratio_r, ratio_l));
    return fminf(maximum, combined / fmaxf(0.01f, 1.0f + ratio));
  }
  const float initial = fminf(maximum, fmaxf(0.0f, parking_entry_outer_effort));
  const float blend = PARK_TURN_ENTRY_BLEND_MS > 0U
      ? fminf(1.0f, (float)(now - turn_motion_start_tick) /
          (float)PARK_TURN_ENTRY_BLEND_MS) : 1.0f;
  return initial + (maximum - initial) * blend;
}

/* Ordinary rear90 retains the applied straight effort at entry. This only
 * limits the normal angle-law request/output; it does not enable direct drive. */
static float rear_turn_effort_ceiling(uint32_t now)
{
  const float maximum = fminf(PWM_OFF, fminf(ARC_OUTER_EFFORT_MAX,
      fmaxf(0.0f, SECTION2_REAR90_MAX_EFFORT)));
  const float initial = fminf(maximum, fmaxf(0.0f, rear_turn_entry_outer_effort));
  const float blend = SECTION2_REAR90_ENTRY_BLEND_MS > 0U
      ? fminf(1.0f, (float)(now - turn_motion_start_tick) / (float)SECTION2_REAR90_ENTRY_BLEND_MS)
      : 1.0f;
  return initial + (maximum - initial) * blend;
}

static float section2_180_effort_ceiling(uint32_t now)
{
  // Both callers are in motorTask: this also controls the applied-output cap.
  if (section2_180_opposing_yaw_pending)
  {
    const float closing_rate = uart_cmd == DRIVE_FL
        ? gyro_yaw_rate_dps : -gyro_yaw_rate_dps;
    if (closing_rate < -fmaxf(0.0f, SECTION2_180_OPPOSING_YAW_MAX_DPS))
        return fminf(ARC_OUTER_EFFORT_MAX, fminf(
            fmaxf(0.0f, SECTION2_REAR_STEADY_OUTER_EFFORT),
            fminf(fmaxf(0.0f, SECTION2_180_OPPOSING_YAW_MAX_EFFORT),
                section2_180_entry_outer_effort)));
    section2_180_opposing_yaw_pending = 0U;
    section2_180_entry_blend_tick = now;
    char release_msg[64];
    int n = snprintf(release_msg, sizeof(release_msg),
        "TURN_CFG,180_YAW_RELEASE,%.1f,%.1f\n",
        (double)total_angle, (double)gyro_yaw_rate_dps);
    if (n > 0 && n < (int)sizeof(release_msg)) queue_range_reply(release_msg);
  }
  const float steady = fminf(PWM_OFF,
      fmaxf(0.0f, SECTION2_REAR_STEADY_OUTER_EFFORT));
  const float initial = fminf(steady, fminf(
      fmaxf(0.0f, SECTION2_180_ENTRY_OUTER_EFFORT_MAX), section2_180_entry_outer_effort));
  const float blend = SECTION2_180_ENTRY_BLEND_MS > 0U
      ? fminf(1.0f, (float)(now - section2_180_entry_blend_tick) / (float)SECTION2_180_ENTRY_BLEND_MS)
      : 1.0f;
  return initial + (steady - initial) * blend;
}

/* Use one effective curvature for steady180 steering and wheel drive. */
static float section2_180_effective_turn_fraction(void)
{
  const float arc_fraction = fminf(1.0f, fmaxf(0.10f, SECTION2_180_ARC_TURN_FRACTION));
  const float unwind_fraction = SECTION2_180_PROGRESSIVE_UNWIND_ENABLE
      ? fminf(1.0f, fmaxf(0.0f, section2_180_turn_fraction)) : 1.0f;
  return arc_fraction * unwind_fraction;
}

/* Blend the curved wheel split toward calibrated straight drive while
 * preserving the requested combined effort. Used for requests AND caps. */
static void section2_turn_wheel_efforts(bool right_turn, float outer,
    float *left, float *right)
{
  const float ratio = right_turn
      ? (park_turn_profile && PARK_TIGHT_ARC_ENABLE ? PARK_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_R)
      : (park_turn_profile && PARK_TIGHT_ARC_ENABLE ? PARK_INNER_EFFORT_RATIO_L : ARC_INNER_EFFORT_RATIO_L);
  const float arc_l = right_turn ? outer : outer * ratio;
  const float arc_r = right_turn ? outer * ratio : outer;
  const float sum = arc_l + arc_r;
  const float full_l = PWM_OFF - (float)PWM_CRUISE * FORWARD_LEFT_COMPARE_SCALE;
  const float full_r = PWM_OFF - (float)PWM_CRUISE * FORWARD_RIGHT_COMPARE_SCALE;
  const float straight_l = sum * full_l / (full_l + full_r);
  const float straight_r = sum - straight_l;
  const float selected_fraction = fu2_entry90_active && FU2_ENTRY90_UNWIND_ENABLE
      ? entry90_turn_fraction : park_turn_profile && PARK_TURN_UNWIND_ENABLE
          ? park_turn_fraction : rear_turn_ordinary_active && SECTION2_REAR90_UNWIND_ENABLE
              ? rear_turn_turn_fraction : section2_rear_steady_turn_active
                  ? section2_180_effective_turn_fraction() : section2_180_turn_fraction;
  const float fraction = fminf(1.0f, fmaxf(0.0f, selected_fraction));
  *left = straight_l + fraction * (arc_l - straight_l);
  *right = straight_r + fraction * (arc_r - straight_r);
}

static float moving_turn_effort_ceiling(uint32_t now)
{
  const float maximum = fminf(PWM_OFF, fmaxf(0.0f, ROLLING_TURN_OUTER_EFFORT_MAX));
  const float initial = fminf(maximum, fmaxf(0.0f, ROLLING_TURN_INITIAL_OUTER_EFFORT));
  const float blend = ROLLING_TURN_ENTRY_BLEND_MS > 0U
      ? fminf(1.0f, (float)(now - turn_motion_start_tick) /
          (float)ROLLING_TURN_ENTRY_BLEND_MS) : 1.0f;
  return initial + (maximum - initial) * blend;
}

static void apply_slewed_drive(void)
{
  if (!slew_output_active) return;
  // This turn writes both wheel channels directly in apply_turn_drive().
  // Do not overwrite those outputs with the preceding straight's slew state.
  if ((rear_turn_direct_active || parking_first90_active || parking_final90_active) &&
      (uart_cmd == DRIVE_FL || uart_cmd == DRIVE_FR)) return;
  const uint32_t now = HAL_GetTick();
  float dt_s = fminf(0.020f, (float)(now - slew_output_tick) * 0.001f);
  slew_output_tick = now;
  float left_target = slew_left_request, right_target = slew_right_request;
  /* Both wheels cross zero before either wheel starts the opposite direction. */
  if (slew_left_effort * left_target < 0.0f || slew_right_effort * right_target < 0.0f)
    left_target = right_target = 0.0f;
  const bool paired_forward_turn =
      ((uart_cmd == DRIVE_FORWARD && slide_mode != SLIDE_NO_SIDE) ||
       ((uart_cmd == DRIVE_FR || uart_cmd == DRIVE_FL) &&
        (rear_turn_ordinary_active || (park_turn_profile && PARK_TURN_UNWIND_ENABLE) ||
         (section2_rear_steady_turn_active && !section2_180_opposing_yaw_pending)))) &&
      slew_left_effort >= 0.0f && slew_right_effort >= 0.0f &&
      left_target >= 0.0f && right_target >= 0.0f && left_target + right_target > 0.0f;
  if (paired_forward_turn)
  {
    // Transfer effort between wheels together. Clipping the old inner wheel
    // first while the outer wheel ramps up causes an avoidable drive dip.
    const float target_sum = left_target + right_target;
    const float sum = slew_effort(slew_left_effort + slew_right_effort,
        target_sum, dt_s);
    const float left_goal = sum * left_target / target_sum;
    const float transfer_step = fmaxf(0.0f,
        fminf(drive_slew_rate(), SMOOTH_DECEL_EFFORT_PER_S)) * dt_s;
    float left = slew_left_effort + fminf(transfer_step,
        fmaxf(-transfer_step, left_goal - slew_left_effort));
    left = fminf(fminf(PWM_OFF, sum), fmaxf(fmaxf(0.0f, sum - PWM_OFF), left));
    slew_left_effort = left;
    slew_right_effort = sum - left;
  }
  else
  {
    slew_left_effort = slew_effort(slew_left_effort, left_target, dt_s);
    slew_right_effort = slew_effort(slew_right_effort, right_target, dt_s);
  }
  /* Enforce the effort ceiling on applied outputs as well as requests; wheel
   * PI and the slew state must not bypass it. Physical speed may still coast. */
  float cap_l = PWM_OFF, cap_r = PWM_OFF;
  if (FU2_REVERSE_CAP_ENABLE && uart_cmd == DRIVE_REVERSE &&
      obstacle_stop_mode == STOP_MODE_US_BACK && fu_sequence_no == 2U)
  {
    cap_l = fminf(PWM_OFF, fmaxf(0.0f, FU2_REVERSE_MAX_EFFORT_L));
    cap_r = fminf(PWM_OFF, fmaxf(0.0f, FU2_REVERSE_MAX_EFFORT_R));
  }
  else if (route_entry_limiting())
  {
    cap_l = fminf(PWM_OFF, fmaxf(0.0f, route_entry_effort_l));
    cap_r = fminf(PWM_OFF, fmaxf(0.0f, route_entry_effort_r));
  }
  else if (park_turn_profile &&
      (uart_cmd == DRIVE_FR || uart_cmd == DRIVE_FL))
  {
    const float outer = park_turn_effort_ceiling(now);
    const bool right = uart_cmd == DRIVE_FR;
    const float ratio = right
        ? (PARK_TIGHT_ARC_ENABLE ? PARK_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_R)
        : (PARK_TIGHT_ARC_ENABLE ? PARK_INNER_EFFORT_RATIO_L : ARC_INNER_EFFORT_RATIO_L);
    if (PARK_TURN_UNWIND_ENABLE)
        section2_turn_wheel_efforts(right, outer, &cap_l, &cap_r);
    else
    {
        cap_l = right ? outer : outer * ratio;
        cap_r = right ? outer * ratio : outer;
    }
    if (paired_forward_turn)
    {
        cap_l = fmaxf(cap_l, slew_left_effort);
        cap_r = fmaxf(cap_r, slew_right_effort);
    }
  }
  else if (rear_turn_ordinary_active &&
      (uart_cmd == DRIVE_FR || uart_cmd == DRIVE_FL))
  {
    float outer = rear_turn_effort_ceiling(now);
    if (section2_exit_turn_limited) outer = fminf(outer, section2_exit_turn_effort_ceiling());
    const bool right = uart_cmd == DRIVE_FR;
    if (SECTION2_REAR90_UNWIND_ENABLE)
        section2_turn_wheel_efforts(right, outer, &cap_l, &cap_r);
    else
    {
        cap_l = right ? outer : outer * ARC_INNER_EFFORT_RATIO_L;
        cap_r = right ? outer * ARC_INNER_EFFORT_RATIO_R : outer;
    }
    if (paired_forward_turn)
    {
        // Allow the bounded paired transfer to finish. Its combined effort
        // already follows the capped requests; an instantaneous per-wheel
        // ratio clamp would defeat this ramp and discard the carried effort.
        cap_l = fmaxf(cap_l, slew_left_effort);
        cap_r = fmaxf(cap_r, slew_right_effort);
    }
  }
  else if (section2_exit_turn_limited &&
      (uart_cmd == DRIVE_FR || uart_cmd == DRIVE_FL))
  {
    const float outer = section2_exit_turn_effort_ceiling();
    const bool right = uart_cmd == DRIVE_FR;
    if (fu2_entry90_active && FU2_ENTRY90_UNWIND_ENABLE)
        section2_turn_wheel_efforts(right, outer, &cap_l, &cap_r);
    else
    {
        cap_l = right ? outer : outer * ARC_INNER_EFFORT_RATIO_L;
        cap_r = right ? outer * ARC_INNER_EFFORT_RATIO_R : outer;
    }
  }
  else if (fu2_entry90_active &&
      slew_left_effort >= 0.0f && slew_right_effort >= 0.0f &&
      (uart_cmd == DRIVE_FR || uart_cmd == DRIVE_FL))
  {
    // Preserve the ordinary reverse-to-forward zero crossing before capping
    // forward turn outputs; do not clip just one still-reversing wheel.
    const float outer = fminf(PWM_OFF, fmaxf(0.0f, FU2_ENTRY90_OUTER_EFFORT_MAX));
    const bool right = uart_cmd == DRIVE_FR;
    if (FU2_ENTRY90_UNWIND_ENABLE)
        section2_turn_wheel_efforts(right, outer, &cap_l, &cap_r);
    else
    {
        cap_l = right ? outer : outer * ARC_INNER_EFFORT_RATIO_L;
        cap_r = right ? outer * ARC_INNER_EFFORT_RATIO_R : outer;
    }
  }
  else if (section2_rear_steady_turn_active &&
      (uart_cmd == DRIVE_FR || uart_cmd == DRIVE_FL))
  {
    const float outer = section2_180_effort_ceiling(now);
    section2_turn_wheel_efforts(uart_cmd == DRIVE_FR, outer, &cap_l, &cap_r);
    if (paired_forward_turn && !section2_180_opposing_yaw_pending)
    {
        // Let the paired transfer redistribute the drive while unwinding.
        // Preserve the opposing-yaw entry cap whenever that guard is active.
        cap_l = fmaxf(cap_l, slew_left_effort);
        cap_r = fmaxf(cap_r, slew_right_effort);
    }
  }
  else if (moving_turn_active &&
      (uart_cmd == DRIVE_FR || uart_cmd == DRIVE_FL))
  {
    // Carried straight effort must not bypass the moving steering-entry ramp.
    const float outer = moving_turn_effort_ceiling(now);
    const bool right = uart_cmd == DRIVE_FR;
    cap_l = right ? outer : outer * ARC_INNER_EFFORT_RATIO_L;
    cap_r = right ? outer * ARC_INNER_EFFORT_RATIO_R : outer;
  }
  else if (section2_exit_straight_limited && uart_cmd == DRIVE_FORWARD && slide_mode == SLIDE_NO_SIDE)
  {
    cap_l = ir_leave_effort_ceiling_l();
    cap_r = ir_leave_effort_ceiling_r();
  }
  if (park_fx_active && uart_cmd == DRIVE_FORWARD)
  {
    float left, right;
    park_fx_effort_limits(&left, &right);
    cap_l = fminf(cap_l, left);
    cap_r = fminf(cap_r, right);
  }
  slew_left_effort = fminf(cap_l, fmaxf(-cap_l, slew_left_effort));
  slew_right_effort = fminf(cap_r, fmaxf(-cap_r, slew_right_effort));
  const int left_compare = limit_pwm(PWM_OFF - (int)(fabsf(slew_left_effort) + 0.5f));
  const int right_compare = limit_pwm(PWM_OFF - (int)(fabsf(slew_right_effort) + 0.5f));
  /* Release the old PWM leg before enabling the new leg on a direction change. */
  if (slew_left_effort >= 0.0f)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_compare);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, left_compare);
  }
  if (slew_right_effort >= 0.0f)
  {
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, right_compare);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, right_compare);
  }
}

/* Start available work immediately, consuming only the intervening EOS markers. */
static void finish_slewed_motion(void)
{
  const int continuation = peek_motion_direction();
  uart_cmd = DRIVE_IDLE;
  next_start_tick = 0U;
  if (continuation == 0) brake_wheels();
  for (uint8_t count = 0U; count < STEP_QUEUE_SIZE && uart_cmd == DRIVE_IDLE && !queue_empty(); ++count)
    if (!run_next_step()) break;
}

// luther turn start
static uint16_t steer_center_ccr = STEER_CENTER_CCR;
static inline void steer_center(void) { htim12.Instance->CCR2 = steer_center_ccr; }
static inline void steer_fr(void) { htim12.Instance->CCR2 = STEER_RIGHT_CCR; }
static inline void steer_rr(void) { htim12.Instance->CCR2 = STEER_REVERSE_RIGHT_CCR; }
static inline void steer_fl(void) { htim12.Instance->CCR2 = STEER_LEFT_CCR; }
static inline void steer_rl(void) { htim12.Instance->CCR2 = STEER_REVERSE_LEFT_CCR; }

/* 1 = steering last returned to centre from the left, 0 = from the right
 * (also the power-up default, matching homing from the right). */
static volatile uint8_t center_from_left = 0U;

static inline void center_after_left(void)
{
    center_from_left = 1U;
    steer_center_ccr = STEER_CENTER_AFTERLEFT_CCR;
    htim12.Instance->CCR2 = steer_center_ccr;
}

static inline void center_after_right(void)
{
    center_from_left = 0U;
    steer_center_ccr = STEER_CENTER_AFTERRIGHT_CCR;
    htim12.Instance->CCR2 = steer_center_ccr;
}

/* Centre for reverse straight-line steering. */
static inline int reverse_center(void)
{
    return center_from_left ? STEER_REV_CENTER_AFTERLEFT_CCR
                                 : STEER_REV_CENTER_AFTERRIGHT_CCR;
}

// luther startup steering homing from either direction
static void steering_home(void)
{
    steering_ready = 0U;

#if STEERING_DIAG_HOME_FROM_LEFT
    // load left values
    steer_fl();

    osDelay(STEERING_HOME_END_HOLD_MS);

    center_after_left();

#else

    // load from right
    steer_fr();

    osDelay(STEERING_HOME_END_HOLD_MS);

    center_after_right();

#endif

    // delay to allow for it to settle
    osDelay(STEERING_HOME_CENTER_SETTLE_MS);

    straight_servo_center_ccr = steer_center_ccr;
    straight_servo_ccr = steer_center_ccr;
    straight_steer_percent = 0.0f;

    steering_ready = 1U;
}

static inline int limit_pwm(int value)
{
  if (value < 0)
    return 0;
  if (value > PWM_OFF)
    return PWM_OFF;
  return value;
}

static inline bool command_is_turn(int command)
{
  return command == DRIVE_FR ||
         command == DRIVE_FL ||
         command == DRIVE_RR ||
         command == DRIVE_RL;
}

static inline bool command_is_reverse_turn(int command)
{
  return command == DRIVE_RR || command == DRIVE_RL;
}

static inline bool turn_steers_right(int command)
{
  return command == DRIVE_FR || command == DRIVE_RR;
}

/*
 * Sign of yaw motion produced by each command:
 *   FR and RL reduce total_angle; FL and RR increase it.
 */
static inline float turn_yaw_sign(int command)
{
  return (command == DRIVE_FL || command == DRIVE_RR) ? 1.0f : -1.0f;
}

static void steer_for_turn(int command)
{
  // A reverse-to-forward handoff still has reverse output during its zero
  // crossing. Keep steering centred until both outputs reach the new sign,
  // rather than starting a backwards arc with a forward-turn steering request.
  const float drive_sign = command_is_reverse_turn(command) ? -1.0f : 1.0f;
  if (!rear_turn_direct_active && !parking_first90_active && !parking_final90_active && slew_output_active &&
      (drive_sign * slew_left_effort < 0.0f ||
      drive_sign * slew_right_effort < 0.0f))
  {
    steer_center();
    return;
  }
  if (PARK_TIGHT_ARC_ENABLE && park_turn_profile &&
      (command == DRIVE_FR || command == DRIVE_FL))
  {
    const bool right = command == DRIVE_FR;
    const float center = right ? STEER_CENTER_AFTERRIGHT_CCR : STEER_CENTER_AFTERLEFT_CCR;
    const float endpoint = right ? PARK_SERVO_RIGHT_CCR : PARK_SERVO_LEFT_CCR;
    const float fraction = PARK_TURN_UNWIND_ENABLE ? park_turn_fraction : 1.0f;
    htim12.Instance->CCR2 = (uint32_t)lroundf(center + fraction * (endpoint - center));
    return;
  }
  if (fu2_entry90_active && (FU2_ENTRY90_TIGHT_ARC_ENABLE || FU2_ENTRY90_UNWIND_ENABLE) &&
      (command == DRIVE_FR || command == DRIVE_FL))
  {
    const bool right = command == DRIVE_FR;
    const float center = right ? STEER_CENTER_AFTERRIGHT_CCR : STEER_CENTER_AFTERLEFT_CCR;
    const float endpoint = FU2_ENTRY90_TIGHT_ARC_ENABLE
        ? (right ? FU2_ENTRY90_SERVO_RIGHT_CCR : FU2_ENTRY90_SERVO_LEFT_CCR)
        : (right ? STEER_RIGHT_CCR : STEER_LEFT_CCR);
    const float fraction = FU2_ENTRY90_UNWIND_ENABLE ? entry90_turn_fraction : 1.0f;
    htim12.Instance->CCR2 = (uint32_t)lroundf(center + fraction * (endpoint - center));
    return;
  }
  if (SECTION2_REAR90_UNWIND_ENABLE && rear_turn_ordinary_active &&
      (command == DRIVE_FR || command == DRIVE_FL))
  {
    const bool right = command == DRIVE_FR;
    const float center = right ? STEER_CENTER_AFTERRIGHT_CCR : STEER_CENTER_AFTERLEFT_CCR;
    const float endpoint = right ? STEER_RIGHT_CCR : STEER_LEFT_CCR;
    htim12.Instance->CCR2 = (uint32_t)lroundf(center +
        rear_turn_turn_fraction * (endpoint - center));
    return;
  }
  if (section2_rear_steady_turn_active &&
      (command == DRIVE_FR || command == DRIVE_FL))
  {
    const bool right = command == DRIVE_FR;
    const float center = right ? STEER_CENTER_AFTERRIGHT_CCR : STEER_CENTER_AFTERLEFT_CCR;
    const float endpoint = right ? STEER_RIGHT_CCR : STEER_LEFT_CCR;
    htim12.Instance->CCR2 = (uint32_t)lroundf(center +
        section2_180_effective_turn_fraction() * (endpoint - center));
    return;
  }
  if (command == DRIVE_FR)
    steer_fr();
  else if (command == DRIVE_FL)
    steer_fl();
  else if (command == DRIVE_RR)
    steer_rr();
  else if (command == DRIVE_RL)
    steer_rl();
}

static void recenter_after_turn(int command)
{
  if (turn_steers_right(command))
    center_after_right();
  else
    center_after_left();
}

/* Encoder/gyro displacement in the route-start forward/left frame. */
static void start_turn_odometry(void)
{
  turn_odom_active = 1U;
  turn_odom_last_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  turn_odom_last_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  turn_odom_last_heading = total_angle;
  turn_forward_cm = turn_left_cm = 0.0f;
  turn_wheel_counts_a = turn_wheel_counts_b = 0;
}

static void update_turn_odometry(void)
{
  if (!turn_odom_active) return;
  const uint16_t a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  const uint16_t b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  const int32_t da = (int16_t)(a - turn_odom_last_a);
  const int32_t db = -(int16_t)(b - turn_odom_last_b);
  turn_wheel_counts_a += da;
  turn_wheel_counts_b += db;
  const float heading = total_angle;
  const float theta = ((turn_odom_last_heading + heading) * 0.5f -
      arc_trace_heading_reference) * 0.01745329252f;
  const float travel_cm = (float)(da + db) * 0.5f / ENCODER_COUNTS_PER_CM;
  turn_forward_cm += travel_cm * cosf(theta);
  turn_left_cm += travel_cm * sinf(theta);
  turn_odom_last_a = a;
  turn_odom_last_b = b;
  turn_odom_last_heading = heading;
}

static void reset_arc_control(void)
{
  /* A turn owns the outputs; clear the preceding sensor-drive mode before
   * starting it. The final rear 90 applies its braked entry after this reset. */
  obstacle_stop_mode = STOP_MODE_NONE;
  ir_motion_active = ir_hold_heading_lock_valid = ir_feedback_keep = 0U;
  slide_mode = SLIDE_NO_SIDE;
  slide_phase = SLIDE_IDLE;
  arc_trace_tick = 0U;
  section2_180_turn_fraction = 1.0f;
  section2_180_unwind_announced = 0U;
  entry90_turn_fraction = 1.0f;
  entry90_unwind_announced = 0U;
  rear_turn_turn_fraction = 1.0f;
  rear_turn_unwind_announced = 0U;
  park_turn_fraction = 1.0f;
  parking_unwind_announced = 0U;
  section2_exit_turn_limited = section2_exit_straight_limited = 0U;
  section2_rear_carry_active = 0U;
  section2_entry_carry_active = 0U;
  rear_unwind_active = rear_unwind_seed_pending = 0U;
  rear_unwind_stable_tick = 0U;
  section2_exit_stable_tick = 0U;
  start_turn_odometry();
  arc_integral_deg_s = 0.0f;
  arc_pid_tick = HAL_GetTick();
  turn_motion_start_tick = arc_pid_tick;
  section2_180_opposing_yaw_pending = 0U;
  section2_180_entry_blend_tick = turn_motion_start_tick;
  if (rear_turn_ordinary_active || (park_turn_profile &&
      !command_is_reverse_turn((int)uart_cmd)))
  {
      const float ratio = turn_steers_right((int)uart_cmd)
          ? (park_turn_profile && PARK_TIGHT_ARC_ENABLE ? PARK_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_R)
          : (park_turn_profile && PARK_TIGHT_ARC_ENABLE ? PARK_INNER_EFFORT_RATIO_L : ARC_INNER_EFFORT_RATIO_L);
      const bool forward = slew_left_effort >= 0.0f && slew_right_effort >= 0.0f;
      const float equivalent = forward
          ? (slew_left_effort + slew_right_effort) / (1.0f + ratio) : 0.0f;
      if (rear_turn_ordinary_active)
          rear_turn_entry_outer_effort = fminf(SECTION2_REAR90_ENTRY_MAX_EFFORT, equivalent);
      else
          parking_entry_outer_effort = fmaxf(PARK_TURN_INITIAL_OUTER_EFFORT,
              fminf(PARK_TURN_CARRY_ENTRY_MAX_EFFORT, equivalent));
      const bool parking_carry = !rear_turn_ordinary_active &&
          PARK_TURN_CARRY_SPEED_ENABLE && peek_motion_direction() == 1;
      if (parking_carry)
          parking_entry_outer_effort = park_turn_effort_ceiling(arc_pid_tick);
      char entry_msg[64];
      int n = snprintf(entry_msg, sizeof(entry_msg), "TURN_CFG,%s,%.0f,%lu\n",
          rear_turn_ordinary_active ? "REAR90_CARRY" : "PARK_CARRY",
          (double)(rear_turn_ordinary_active ? rear_turn_entry_outer_effort : parking_entry_outer_effort),
          (unsigned long)(rear_turn_ordinary_active ? SECTION2_REAR90_ENTRY_BLEND_MS
              : parking_carry ? 0U : PARK_TURN_ENTRY_BLEND_MS));
      if (n > 0 && n < (int)sizeof(entry_msg)) queue_range_reply(entry_msg);
  }
  turn_progress_tick = arc_pid_tick;
  turn_progress_heading = total_angle;
  arc_settle_deadline = 0U;
  arc_deadline = arc_pid_tick + ARC_TIMEOUT_BASE_MS +
                      (uint32_t)(TURN_DEG * (float)ARC_TIMEOUT_PER_DEG_MS);
  arc_attempts = 1U;
  section2_rear_steady_turn_active = SECTION2_REAR_STEADY_TURN_ENABLE &&
      SECTION2_REAR_CARRY_ENABLE && !park_turn_profile &&
      !command_is_reverse_turn((int)uart_cmd) && TURN_DEG >= 135.0f &&
      step_uses_ir(peek_motion_kind());
  if (section2_rear_steady_turn_active)
  {
      const float ratio = turn_steers_right((int)uart_cmd)
          ? ARC_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_L;
      // Preserve combined effort when converting straight drive into an arc.
      // The ordinary output slew and both-wheel direction checks still apply.
      const float sum = fmaxf(0.0f, slew_left_effort) + fmaxf(0.0f, slew_right_effort);
      section2_180_entry_outer_effort = fminf(PWM_OFF,
          fminf(fmaxf(0.0f, SECTION2_180_ENTRY_OUTER_EFFORT_MAX), sum / (1.0f + ratio)));
      const float closing_rate = turn_yaw_sign((int)uart_cmd) * gyro_yaw_rate_dps;
      section2_180_opposing_yaw_pending = SECTION2_180_OPPOSING_YAW_LIMIT_ENABLE &&
          closing_rate < -fmaxf(0.0f, SECTION2_180_OPPOSING_YAW_MAX_DPS);
      if (section2_180_opposing_yaw_pending)
      {
          section2_180_entry_outer_effort = fminf(section2_180_entry_outer_effort,
              fmaxf(0.0f, SECTION2_180_OPPOSING_YAW_MAX_EFFORT));
          char yaw_msg[64];
          int n = snprintf(yaw_msg, sizeof(yaw_msg),
              "TURN_CFG,180_OPPOSING_YAW,%.1f,%.1f\n",
              (double)total_angle, (double)gyro_yaw_rate_dps);
          if (n > 0 && n < (int)sizeof(yaw_msg)) queue_range_reply(yaw_msg);
      }
      char entry_msg[64];
      int n = snprintf(entry_msg, sizeof(entry_msg), "TURN_CFG,180_ENTRY,%.0f,%lu\n",
          (double)section2_180_entry_outer_effort, (unsigned long)SECTION2_180_ENTRY_BLEND_MS);
      if (n > 0 && n < (int)sizeof(entry_msg)) queue_range_reply(entry_msg);
  }

  turn_error_deg = arc_target_angle - total_angle;
  turn_effort_cmd = 0.0f;
  turn_left_pwm = PWM_OFF;
  turn_right_pwm = PWM_OFF;
  turn_control_phase = ARC_RUNNING;

  char begin_msg[48];
  const char *turn_name = command_is_reverse_turn((int)uart_cmd)
      ? (turn_steers_right((int)uart_cmd) ? "RR" : "RL")
      : (turn_steers_right((int)uart_cmd) ? "FR" : "FL");
  int begin_n = snprintf(begin_msg, sizeof(begin_msg), "TURN_BEGIN,%s,%.1f,%.1f\n",
      turn_name, (double)total_angle, (double)arc_target_angle);
  if (begin_n > 0 && begin_n < (int)sizeof(begin_msg)) queue_range_reply(begin_msg);
  if (route_axis_valid)
  {
    char reference_msg[48];
    int ref_n = snprintf(reference_msg, sizeof(reference_msg), "TURN_REF,%.1f,%.1f\n",
        (double)route_axis_heading, (double)total_angle);
    if (ref_n > 0 && ref_n < (int)sizeof(reference_msg)) queue_range_reply(reference_msg);
  }
  if (rear_turn_direct_active)
  {
    char speed_msg[48];
    int n = snprintf(speed_msg, sizeof(speed_msg), "TURN_CFG,REAR90_NORMAL,%.0f,%.0f\n",
        (double)REAR90_OUTER_EFFORT_MAX, (double)REAR90_OUTER_EFFORT_MIN);
    if (n > 0 && n < (int)sizeof(speed_msg)) queue_range_reply(speed_msg);
  }
  if (rear_turn_ordinary_active)
  {
    char exit_msg[48];
    int n = snprintf(exit_msg, sizeof(exit_msg), "TURN_CFG,REAR90_EXIT,%.0f,%.0f\n",
        (double)section2_exit_turn_effort_ceiling(), (double)SECTION2_EXIT_TURN_WINDOW_DEG);
    if (n > 0 && n < (int)sizeof(exit_msg)) queue_range_reply(exit_msg);
  }
  if (fu2_entry90_active)
  {
    char speed_msg[48];
    int n = snprintf(speed_msg, sizeof(speed_msg), "TURN_CFG,FU2_ENTRY90,%.0f,%.0f\n",
        (double)FU2_ENTRY90_OUTER_EFFORT_MIN, (double)FU2_ENTRY90_OUTER_EFFORT_MAX);
    if (n > 0 && n < (int)sizeof(speed_msg)) queue_range_reply(speed_msg);
  }

  char profile_msg[48];
  const bool parking_arc = PARK_TIGHT_ARC_ENABLE && park_turn_profile &&
      !command_is_reverse_turn((int)uart_cmd);
  const float ratio = turn_steers_right((int)uart_cmd)
      ? (parking_arc ? PARK_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_R)
      : (parking_arc ? PARK_INNER_EFFORT_RATIO_L : ARC_INNER_EFFORT_RATIO_L);
  int n = snprintf(profile_msg, sizeof(profile_msg), "TURN_CFG,%c,%lu,%.2f\n",
      park_turn_profile ? 'P' : 'N',
      (unsigned long)htim12.Instance->CCR2, (double)ratio);
  if (n > 0 && n < (int)sizeof(profile_msg)) queue_range_reply(profile_msg);
  if (section2_rear_steady_turn_active)
  {
    char steady_msg[48];
    int steady_n = snprintf(steady_msg, sizeof(steady_msg), "TURN_CFG,STEADY,%.0f\n",
        (double)fminf(PWM_OFF,
            fmaxf(0.0f, SECTION2_REAR_STEADY_OUTER_EFFORT)));
    if (steady_n > 0 && steady_n < (int)sizeof(steady_msg)) queue_range_reply(steady_msg);
    char arc_msg[64];
    int arc_n = snprintf(arc_msg, sizeof(arc_msg), "TURN_CFG,180_ARC,%.2f\n",
        (double)section2_180_effective_turn_fraction());
    if (arc_n > 0 && arc_n < (int)sizeof(arc_msg)) queue_range_reply(arc_msg);
    char lead_msg[48];
    int lead_n = snprintf(lead_msg, sizeof(lead_msg), "TURN_CFG,REAR_LEAD,%.3f\n",
        (double)SECTION2_REAR_HANDOFF_LOOKAHEAD_S);
    if (lead_n > 0 && lead_n < (int)sizeof(lead_msg)) queue_range_reply(lead_msg);
    char end_msg[64];
    int end_n = snprintf(end_msg, sizeof(end_msg), "TURN_CFG,180_EXIT,%.1f,%.2f,%.1f\n",
        (double)SECTION2_180_UNWIND_START_DEG, (double)SECTION2_180_UNWIND_MIN_FRACTION,
        (double)SECTION2_180_EXIT_MAX_REMAIN_DEG);
    if (end_n > 0 && end_n < (int)sizeof(end_msg)) queue_range_reply(end_msg);
  }
  if (moving_turn_active && !park_turn_profile)
  {
    char rolling_msg[48];
    int n = snprintf(rolling_msg, sizeof(rolling_msg), "TURN_CFG,ROLLING,%.0f,%lu\n",
        (double)ROLLING_TURN_INITIAL_OUTER_EFFORT, (unsigned long)ROLLING_TURN_ENTRY_BLEND_MS);
    if (n > 0 && n < (int)sizeof(rolling_msg)) queue_range_reply(rolling_msg);
  }
  if (park_turn_profile && !command_is_reverse_turn((int)uart_cmd))
  {
    char speed_msg[48];
    int speed_n = snprintf(speed_msg, sizeof(speed_msg), "PARK_SPEED,%.0f,%.0f\n",
        (double)PARK_TURN_OUTER_EFFORT_MAX,
        (double)(PARK_TURN_CARRY_SPEED_ENABLE && peek_motion_direction() == 1
            ? park_turn_effort_ceiling(HAL_GetTick()) : PARK_TURN_OUTER_EFFORT_MIN));
    if (speed_n > 0 && speed_n < (int)sizeof(speed_msg)) queue_range_reply(speed_msg);
    if (PARK_TURN_CARRY_SPEED_ENABLE && peek_motion_direction() == 1)
        queue_range_reply("TURN_CFG,PARK_STEADY_CARRY\n");
  }
}

static void stop_arc_control(void)
{
  fu2_entry90_active = 0U;
  rear_turn_direct_active = 0U;
  rear_turn_ordinary_active = 0U;
  parking_first90_active = 0U;
  parking_final90_active = 0U;
  // Retain parking_90_started across the two PARK arcs; route/PARK resets it.
  turn_steering_entry_pending = 0U;
  arc_integral_deg_s = 0.0f;
  arc_pid_tick = 0U;
  arc_settle_deadline = 0U;
  arc_deadline = 0U;
  turn_progress_tick = 0U;
  arc_attempts = 0U;

  turn_effort_cmd = 0.0f;
  turn_left_pwm = PWM_OFF;
  turn_right_pwm = PWM_OFF;
  turn_control_phase = ARC_IDLE;
}

static int effort_to_pwm(float effort)
{
  int compare = PWM_OFF - (int)(effort + 0.5f);
  return limit_pwm(compare);
}

static bool fu2_read_fresh_range(float reference, uint32_t *range_cm)
{
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const uint32_t sample_tick = us_capture_tick;
    const uint32_t range = us_raw_cm;
    const float heading = us_raw_heading;
    const bool valid = us_capture_valid != 0U;
    __set_PRIMASK(irq_state);
    if (!valid || (uint32_t)(HAL_GetTick() - sample_tick) > FU2_EARLY_US_SAMPLE_MAX_AGE_MS ||
        fabsf(heading - reference) > FU2_EARLY_US_SAMPLE_MAX_ERROR_DEG) return false;
    *range_cm = range;
    return true;
}

static float fu2_slide_range_scale(void)
{
    if (!FU2_EARLY_US_ENABLE || fu_sequence_no != 1U ||
        slide_mode == SLIDE_NO_SIDE || slide_phase != SLIDE_INWARD ||
        peek_motion_kind() != STEP_FU ||
        fabsf(total_angle - (float)slide_origin_heading) > FU2_EARLY_US_SCAN_WINDOW_DEG) return 1.0f;
    uint32_t range_cm = 0U;
    const bool fresh = fu2_read_fresh_range(slide_origin_heading, &range_cm);
    float scale = FU2_EARLY_US_NO_SAMPLE_SCALE;
    if (fresh)
    {
        const float span = fmaxf(1.0f, FU2_EARLY_US_SLOW_RANGE_CM - FU2_EARLY_US_NEAR_RANGE_CM);
        const float blend = fminf(1.0f, fmaxf(0.0f,
            ((float)range_cm - FU2_EARLY_US_NEAR_RANGE_CM) / span));
        scale = FU2_EARLY_US_MIN_SCALE + (1.0f - FU2_EARLY_US_MIN_SCALE) * blend;
    }
    if (!fu2_slide_us_announced)
    {
        fu2_slide_us_announced = 1U;
        char message[64];
        int n = snprintf(message, sizeof(message), "SLIDE_US,SCAN,%u,%.1f,%.2f\n",
            fresh ? (unsigned)range_cm : 0U, (double)total_angle, (double)scale);
        if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
    }
    return fminf(1.0f, fmaxf(0.0f, scale));
}

static uint32_t fu2_current_range(void)
{
    // The motor-loop guard validates and snapshots this before any FU2
    // stop decision or output. Other phases retain the senior median.
    if (fu_sequence_no == 2U && us_allow_reverse) return fu2_fwd_range_cm;
    return echo_debug;
}

static int scaled_slide_pwm(int base_pwm)
{
  const float effort = (float)(PWM_OFF - limit_pwm(base_pwm));
  return effort_to_pwm(effort * fmaxf(0.0f, SLIDE_DRIVE_EFFORT_SCALE));
}

/* Apply the active phase at dispatch, rather than leaving the previous
 * steering/wheel request in place for another motor cycle. The ordinary
 * output slew still applies; the slide drive effort and wheel ratio are kept. */
static void drive_slide_stage(void)
{
  if (slide_mode == SLIDE_NO_SIDE ||
      (slide_phase != SLIDE_OUTWARD && slide_phase != SLIDE_INWARD)) return;

  const bool steer_right = (slide_mode == SLIDE_TO_RIGHT) == (slide_phase == SLIDE_OUTWARD);
  float finish_scale = 1.0f;
  if (slide_phase == SLIDE_INWARD && peek_motion_kind() == STEP_FU)
  {
    const float remaining = fabsf((float)slide_origin_heading - total_angle);
    const float fraction = SLIDE_FINAL_IN_DECEL_DEG > 0.0f
        ? fminf(1.0f, remaining / SLIDE_FINAL_IN_DECEL_DEG) : 1.0f;
    const float minimum = fminf(1.0f, fmaxf(0.0f, SLIDE_FINAL_IN_MIN_SCALE));
    finish_scale = minimum + (1.0f - minimum) * fraction;
  }
  finish_scale *= fu2_slide_range_scale();
  const int outer = effort_to_pwm(
      (float)(PWM_OFF - scaled_slide_pwm(SLIDE_OUTER_PWM)) * finish_scale);
  const int inner = effort_to_pwm(
      (float)(PWM_OFF - scaled_slide_pwm(SLIDE_INNER_PWM)) * finish_scale);
  htim12.Instance->CCR2 = steer_right ? STEER_SLIDERIGHT_CCR : STEER_SLIDELEFT_CCR;
  drive_left_forward(steer_right ? outer : inner);
  drive_right_forward(steer_right ? inner : outer);
}

/* Keep the slide's combined drive while holding its original heading.
 * A normal F command would select a different straight cruise profile. */
static void drive_slide_clearance(int servo_ccr)
{
  const float sum = (float)(PWM_OFF - scaled_slide_pwm(SLIDE_OUTER_PWM)) +
      (float)(PWM_OFF - scaled_slide_pwm(SLIDE_INNER_PWM));
  const float full_l = PWM_OFF - (float)PWM_CRUISE * FORWARD_LEFT_COMPARE_SCALE;
  const float full_r = PWM_OFF - (float)PWM_CRUISE * FORWARD_RIGHT_COMPARE_SCALE;
  const float left = sum * full_l / (full_l + full_r);
  htim12.Instance->CCR2 = (uint32_t)servo_ccr;
  drive_left_forward(effort_to_pwm(left));
  drive_right_forward(effort_to_pwm(sum - left));
}

static void apply_turn_drive(int command, int outer_pwm, int inner_pwm)
{
  int left_pwm;
  int right_pwm;

  /* With right steering the left wheel is outer; with left steering it is right. */
  if (turn_steers_right(command))
  {
    left_pwm = outer_pwm;
    right_pwm = inner_pwm;
  }
  else
  {
    left_pwm = inner_pwm;
    right_pwm = outer_pwm;
  }

  left_pwm = limit_pwm(left_pwm);
  right_pwm = limit_pwm(right_pwm);

  turn_left_pwm = left_pwm;
  turn_right_pwm = right_pwm;

  if ((rear_turn_direct_active || parking_first90_active || parking_final90_active) &&
      (command == DRIVE_FL || command == DRIVE_FR))
  {
    // Same two-wheel forward PWM mapping as Basic_SeniorFlow. Bypass the
    // request/slew wrappers for this command, and seed them from these actual
    // compare writes so the following forward leg resumes without stale drive.
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, right_pwm);
    slew_left_request = slew_left_effort = (float)(PWM_OFF - left_pwm);
    slew_right_request = slew_right_effort = (float)(PWM_OFF - right_pwm);
    slew_output_tick = HAL_GetTick();
    return;
  }

  if (command_is_reverse_turn(command))
  {
    drive_left_reverse(left_pwm);
    drive_right_reverse(right_pwm);
  }
  else
  {
    /* Always command BOTH wheels. The old FR branch omitted the left wheel. */
    drive_left_forward(left_pwm);
    drive_right_forward(right_pwm);
  }
}

/* Give the ordinary rear-edge90 a short braked steering entry. Its drive
 * still uses the normal arc controller and output slew after this wait.
 * Legacy direct-drive modes and PARK entry flags remain separate. */
static void brake_for_turn_entry(int command)
{
  if ((!rear_turn_ordinary_active && !rear_turn_direct_active &&
      !parking_first90_active && !parking_final90_active) || ARC_SERVO_ONLY_TEST) return;
  brake_wheels();
  steer_for_turn(command);
  turn_effort_cmd = 0.0f;
  turn_left_pwm = turn_right_pwm = PWM_OFF;
  const uint32_t settle_ms = parking_first90_active ? PARK_FIRST90_STEERING_SETTLE_MS
      : parking_final90_active ? PARK_FINAL90_STEERING_SETTLE_MS
      : rear_turn_ordinary_active ? SECTION2_REAR90_ORDINARY_STEER_SETTLE_MS : REAR90_STEERING_SETTLE_MS;
  turn_motor_enable_tick = HAL_GetTick() + settle_ms;
  turn_steering_entry_pending = 1U;
  char entry_msg[48];
  int n = snprintf(entry_msg, sizeof(entry_msg), "TURN_CFG,%s,%lu\n",
      parking_first90_active ? "PARK_FIRST_ENTRY" : parking_final90_active ? "PARK_FINAL_ENTRY"
          : rear_turn_ordinary_active ? "REAR90_ORDINARY_STEER" : "BRAKED_REAR90",
      (unsigned long)settle_ms);
  if (n > 0 && n < (int)sizeof(entry_msg)) queue_range_reply(entry_msg);
}

/* The motor loop must keep braking while steering settles. Rebase the yaw
 * progress deadline when drive starts, so the intentional wait is excluded. */
static bool wait_for_turn_steering(int command, uint32_t now)
{
  if (!turn_steering_entry_pending) return false;
  brake_wheels();
  steer_for_turn(command);
  turn_effort_cmd = 0.0f;
  turn_left_pwm = turn_right_pwm = PWM_OFF;
  if ((int32_t)(now - turn_motor_enable_tick) < 0) return true;

  turn_steering_entry_pending = 0U;
  turn_motor_enable_tick = 0U;
  arc_pid_tick = turn_motion_start_tick = now;
  turn_progress_tick = now;
  turn_progress_heading = total_angle;
  arc_deadline = now + ARC_TIMEOUT_BASE_MS +
      (uint32_t)(TURN_DEG * (float)ARC_TIMEOUT_PER_DEG_MS);
  queue_range_reply(parking_first90_active ? "TURN_CFG,PARK_FIRST_DRIVE\n"
      : parking_final90_active ? "TURN_CFG,PARK_FINAL_DRIVE\n"
      : rear_turn_ordinary_active ? "TURN_CFG,REAR90_ORDINARY_DRIVE\n" : "TURN_CFG,REAR90_DRIVE\n");
  return false;
}

static void balance_turn_exit(void)
{
    if (!slew_output_active || slew_left_effort < 0.0f || slew_right_effort < 0.0f)
        return;
    const float sum = slew_left_effort + slew_right_effort;
    const float full_l = PWM_OFF - (float)PWM_CRUISE * FORWARD_LEFT_COMPARE_SCALE;
    const float full_r = PWM_OFF - (float)PWM_CRUISE * FORWARD_RIGHT_COMPARE_SCALE;
    slew_left_effort = sum * full_l / (full_l + full_r);
    slew_right_effort = sum - slew_left_effort;
    slew_left_request = slew_left_effort;
    slew_right_request = slew_right_effort;
}

static void complete_turn(int completed_command)
{
  char done_msg[48];
  int n = snprintf(done_msg, sizeof(done_msg), "TURN_DONE,%.1f,%.1f,%.1f\n",
      (double)total_angle, (double)arc_target_angle, (double)gyro_yaw_rate_dps);
  if (n > 0 && n < (int)sizeof(done_msg)) queue_range_reply(done_msg);

  update_turn_odometry();
  char xy_msg[48];
  int xy_n = snprintf(xy_msg, sizeof(xy_msg), "TURN_XY,%.1f,%.1f\n",
      (double)turn_forward_cm, (double)turn_left_cm);
  if (xy_n > 0 && xy_n < (int)sizeof(xy_msg)) queue_range_reply(xy_msg);
  turn_odom_active = 0U;

#if SMOOTH_TURN_EXIT_BALANCE
  const step_kind_t next_type = peek_motion_kind();
  const bool next_straight = step_uses_ir(next_type) ||
      next_type == STEP_FORWARD_CM || next_type == STEP_FU || next_type == STEP_FX;
  if (!command_is_reverse_turn(completed_command) && next_straight &&
      slew_left_effort >= 0.0f && slew_right_effort >= 0.0f)
  {
      /* Retain combined drive effort, remove the curved outer/inner torque
       * split before the straight PID starts accelerating the wheels. */
      balance_turn_exit();
  }
#endif
  const step_kind_t exit_type = peek_motion_kind();
  const bool forward_straight_exit = step_uses_ir(exit_type) ||
      (SECTION2_EXIT_CAP_FORWARD_LEGS &&
       (exit_type == STEP_FORWARD_CM || exit_type == STEP_FU || exit_type == STEP_FX));
  section2_exit_turn_limited = 0U;
  section2_exit_straight_limited = SECTION2_EXIT_CAP_ENABLE && !park_turn_profile &&
      !command_is_reverse_turn(completed_command) && forward_straight_exit;
  section2_rear_carry_active = SECTION2_REAR_CARRY_ENABLE &&
      !park_turn_profile && !command_is_reverse_turn(completed_command) &&
      TURN_DEG >= 135.0f && step_uses_ir(exit_type) &&
      slew_left_effort >= 0.0f && slew_right_effort >= 0.0f;
  if (section2_rear_carry_active)
  {
    const float sum = (slew_left_effort + slew_right_effort) *
        fmaxf(0.0f, SECTION2_REAR_CARRY_EFFORT_SCALE);
    const float full_l = PWM_OFF - (float)PWM_CRUISE * FORWARD_LEFT_COMPARE_SCALE;
    const float full_r = PWM_OFF - (float)PWM_CRUISE * FORWARD_RIGHT_COMPARE_SCALE;
    section2_rear_carry_effort_l = fminf(full_l,
        sum * full_l / (full_l + full_r));
    section2_rear_carry_effort_r = fminf(full_r,
        sum * full_r / (full_l + full_r));
    if (section2_rear_carry_effort_l + section2_rear_carry_effort_r <= 0.0f)
      section2_rear_carry_active = 0U;
    else
    {
      char carry_msg[48];
      int carry_n = snprintf(carry_msg, sizeof(carry_msg), "REAR_CARRY,%.0f,%.0f\n",
          (double)section2_rear_carry_effort_l, (double)section2_rear_carry_effort_r);
      if (carry_n > 0 && carry_n < (int)sizeof(carry_msg)) queue_range_reply(carry_msg);
    }
  }
  section2_entry_carry_active = SECTION2_ENTRY_CARRY_ENABLE && fu2_entry90_active &&
      !park_turn_profile && !command_is_reverse_turn(completed_command) &&
      step_uses_ir(exit_type) &&
      slew_left_effort >= 0.0f && slew_right_effort >= 0.0f;
  if (section2_entry_carry_active)
  {
      // Request a modest boost from the balanced exit through normal output slew.
      section2_entry_carry_effort_l = fminf(fmaxf(0.0f, SECTION2_ENTRY_CARRY_MAX_EFFORT_L),
          slew_left_effort * SECTION2_ENTRY_CARRY_EFFORT_SCALE);
      section2_entry_carry_effort_r = fminf(fmaxf(0.0f, SECTION2_ENTRY_CARRY_MAX_EFFORT_R),
          slew_right_effort * SECTION2_ENTRY_CARRY_EFFORT_SCALE);
      if (section2_entry_carry_effort_l + section2_entry_carry_effort_r <= 0.0f)
          section2_entry_carry_active = 0U;
      else
      {
          char carry_msg[48];
          int n = snprintf(carry_msg, sizeof(carry_msg), "ENTRY_CARRY,%.0f,%.0f\n",
              (double)section2_entry_carry_effort_l, (double)section2_entry_carry_effort_r);
          if (n > 0 && n < (int)sizeof(carry_msg)) queue_range_reply(carry_msg);
      }
  }
  rear_unwind_active = SECTION2_REAR_UNWIND_ENABLE && section2_rear_carry_active;
  rear_unwind_seed_pending = rear_unwind_active;
  rear_unwind_stable_tick = 0U;
  section2_rear_steady_turn_active = 0U;
  if (section2_rear_carry_active || section2_entry_carry_active) section2_exit_straight_limited = 1U;
  if (section2_exit_straight_limited)
  {
    section2_exit_start_tick = HAL_GetTick();
    section2_exit_stable_tick = 0U;
    char cap_msg[48];
    int cap_n = snprintf(cap_msg, sizeof(cap_msg), "EXIT_CAP,STRAIGHT,%.0f,%.0f\n",
        (double)ir_leave_effort_ceiling_l(),
        (double)ir_leave_effort_ceiling_r());
    if (cap_n > 0 && cap_n < (int)sizeof(cap_msg)) queue_range_reply(cap_msg);
  }
  if (peek_motion_direction() == 0) brake_wheels();
  recenter_after_turn(completed_command);
  stop_arc_control();

  if (route_axis_valid) route_axis_heading = arc_target_angle;
  /* IR uses the intended turn endpoint, rather than capturing residual yaw. */
  ir_hold_heading_reference = arc_target_angle;
  ir_hold_heading_lock_valid = 1U;
  ir_motion_active = ir_feedback_keep = 0U;
  target_angle = total_angle;

  if (arc_postforward_cm > 0U)
  {
    enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    int32_t counts = (int32_t)(arc_postforward_cm * ENCODER_COUNTS_PER_CM + 0.5f);
    obstacle_stop_mode = STOP_MODE_NONE;
    seg_rebase = 1U;

    if (command_is_reverse_turn(completed_command))
    {
      target_counts = -counts;
      uart_cmd = DRIVE_REVERSE;
    }
    else
    {
      target_counts = counts;
      uart_cmd = DRIVE_FORWARD;
    }

    arc_postforward_cm = 0U;
  }
  else
  {
    finish_slewed_motion();
  }
} //luther turn helper end


/* ---------------- luther Joystick drive mode ----------------
 * UART ISR only copies lines. motorTask owns this state and all motor writes.
 * The gamepad sends intent only: direction, steering percentage and throttle.
 * PWM, feed-forward scaling, wheel PI, servo calibration and turn ratios remain
 * STM32-owned so full gamepad movements match the scripted motion tuning.
 */
static ManualControl joy_control;
static volatile uint8_t joy_uart_owned = 0U;
static volatile uint8_t joy_line_pending = 0U;
static volatile char joy_rx_line[MANUAL_LINE_SIZE];
static volatile uint8_t joy_driving = 0U;
static int joy_last_steer = 0;
static int joy_telemetry_kind = 0;
static int joy_throttle_percent = 0;
static int joy_motion_direction = 0;

typedef enum
{
    JOY_STOPPED = 0,
    JOY_STRAIGHT,
    JOY_TURNING
} joy_motion_t;

static joy_motion_t joy_motion = JOY_STOPPED;
static char joy_reply_pending[80];
static char joy_reply_tx[80];

static void joy_receive_line(const char *line)
{
    size_t n = strlen(line);
    if (n >= MANUAL_LINE_SIZE) return;
    /* This producer runs only inside the USART3 RX ISR. */
    for (size_t i = 0; i <= n; ++i) joy_rx_line[i] = line[i];
    if (strncmp(line, "jbegin ", 7U) == 0) joy_uart_owned = 1U;
    joy_line_pending = 1U;
}

static void joy_queue_reply(const char *text)
{
    (void)snprintf(joy_reply_pending, sizeof(joy_reply_pending), "%s", text);
}

static void joy_send_reply(void)
{
    if (joy_reply_pending[0] == '\0' || reply_tx_busy) return;
    memcpy(joy_reply_tx, joy_reply_pending, sizeof(joy_reply_tx));
    reply_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)joy_reply_tx,
                            (uint16_t)strlen(joy_reply_tx)) == HAL_OK) {
        joy_reply_pending[0] = '\0';
    } else {
        reply_tx_busy = 0U;
    }
}

static void joy_center_steering(void)
{
    if (joy_last_steer > 0) center_after_right();
    else if (joy_last_steer < 0) center_after_left();
    else steer_center();
    joy_last_steer = 0;
}

static void joy_leave_mode(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    joy_line_pending = 0U;
    joy_uart_owned = 0U;
    if (primask == 0U) __enable_irq();

    if (joy_control.owns_motors) joy_center_steering();
    Manual_Init(&joy_control);
    joy_driving = 0U;
    joy_telemetry_kind = 0;
    joy_throttle_percent = 0;
    joy_motion_direction = 0;
    joy_motion = JOY_STOPPED;
    joy_reply_pending[0] = '\0';
    seg_rebase = 1U;
}

static void joy_enter_mode(uint32_t session, uint32_t now)
{
    const int previous_command = (int)uart_cmd;
    brake_wheels();
    if (command_is_turn(previous_command)) recenter_after_turn(previous_command);
    else joy_center_steering();

    queue_clear();
    route_axis_valid = route_entry_hold_pending = 0U;
    rx_script_read = rx_script_write;
    paired_slide_heading_valid = 0U;
    park_turn_profile = moving_turn_pending = moving_turn_active = turn_odom_active = 0U;
    parking_90_started = 0U;
    park_fx_active = park_fx_has_range = park_fx_range_confirmed = park_fx_confirm_samples = 0U;
    fu_arrow_confirmed = 0U;
    clear_replies();
    script_ack_waiting = 0U;
    stop_arc_control();
    obstacle_stop_mode = STOP_MODE_NONE;
    slide_mode = SLIDE_NO_SIDE;
    slide_phase = SLIDE_IDLE;
    target_counts = 0;
    target_angle = total_angle;
    arc_postforward_cm = 0U;
    turn_motor_enable_tick = 0U;
    next_start_tick = 0U;
    odom_counts_run = 0;
    odom_cm_run = 0.0f;
    odom_counts_run_ir = 0;
    odom_cm_run_ir = 0.0f;
    seg_rebase = 1U;

    Manual_Begin(&joy_control, session, now);
    joy_uart_owned = 1U;
    joy_driving = 0U;
    joy_telemetry_kind = 0;
    joy_throttle_percent = 0;
    joy_motion_direction = 0;
    joy_motion = JOY_STOPPED;
    uart_cmd = DRIVE_JOYSTICK;
}

/* Return 0 when the normal DRIVE_FORWARD/DRIVE_REVERSE controller should run.
 * Return 1 when manual mode has already produced/braked the outputs this tick.
 */
static int joy_apply_drive(ManualSetpoint output)
{
    joy_driving = (output.direction != 0 && output.throttle_percent > 0) ? 1U : 0U;

    if (!joy_driving)
    {
        if (joy_motion != JOY_STOPPED)
        {
            brake_wheels();
            joy_center_steering();
            seg_rebase = 1U;
        }

        joy_motion = JOY_STOPPED;
        joy_motion_direction = 0;
        joy_throttle_percent = 0;
        straight_left_pwm = PWM_OFF;
        straight_right_pwm = PWM_OFF;
        straight_servo_ccr = steer_center_ccr;
        uart_cmd = DRIVE_JOYSTICK;
        return 1;
    }

    joy_throttle_percent = output.throttle_percent;

    /* Straight manual movement uses the exact same straight controller as F/R:
     * feed-forward scaling, encoder PI and gyro/servo heading hold. */
    if (output.steering == 0)
    {
        if (joy_motion != JOY_STRAIGHT ||
            joy_motion_direction != output.direction)
        {
            brake_wheels();
            joy_center_steering();
            target_angle = total_angle;
            obstacle_stop_mode = STOP_MODE_NONE;
            target_counts = (output.direction > 0) ? INT32_MAX : (INT32_MIN + 1);
            seg_rebase = 1U;
        }

        joy_motion = JOY_STRAIGHT;
        joy_motion_direction = output.direction;
        uart_cmd = (output.direction > 0) ? DRIVE_FORWARD : DRIVE_REVERSE;
        return 0;
    }

    /* Continuous manual arc. Full D-pad/trigger steering uses the same full
     * servo CCR, ARC_OUTER_EFFORT_MAX and inner-wheel ratio as FR/FL/RR/RL.
     * Partial stick steering interpolates those STM32-owned values. */
    const int reverse = (output.direction < 0);
    const int right = (output.steering > 0);
    const int magnitude = abs(output.steering);
    const int command = !reverse
        ? (right ? DRIVE_FR : DRIVE_FL)
        : (right ? DRIVE_RR : DRIVE_RL);

    if (joy_motion != JOY_TURNING ||
        joy_motion_direction != output.direction ||
        ((joy_last_steer > 0) != right))
    {
        brake_wheels();
        seg_rebase = 1U;
    }

    const int end_ccr = right
        ? (reverse ? STEER_REVERSE_RIGHT_CCR : STEER_RIGHT_CCR)
        : (reverse ? STEER_REVERSE_LEFT_CCR : STEER_LEFT_CCR);
    const int servo = (int)steer_center_ccr +
                      (end_ccr - (int)steer_center_ccr) * magnitude / 100;
    htim12.Instance->CCR2 = (uint32_t)servo;
    joy_last_steer = output.steering;

    float effort = ARC_OUTER_EFFORT_MAX * ((float)output.throttle_percent / 100.0f);
    if (effort < ARC_OUTER_EFFORT_MIN) effort = ARC_OUTER_EFFORT_MIN;
    if (effort > ARC_OUTER_EFFORT_MAX) effort = ARC_OUTER_EFFORT_MAX;

    const float full_inner_ratio = right
        ? ARC_INNER_EFFORT_RATIO_R
        : ARC_INNER_EFFORT_RATIO_L;
    const float inner_ratio = 1.0f -
        (1.0f - full_inner_ratio) * ((float)magnitude / 100.0f);

    const int outer_pwm = effort_to_pwm(effort);
    const int inner_pwm = effort_to_pwm(effort * inner_ratio);

    turn_effort_cmd = effort;
    apply_turn_drive(command, outer_pwm, inner_pwm);

    joy_motion = JOY_TURNING;
    joy_motion_direction = output.direction;
    uart_cmd = DRIVE_JOYSTICK;
    return 1;
}

static int joy_poll(void)
{
    char line[MANUAL_LINE_SIZE];
    char reply[80];
    uint8_t have_line = 0U;
    const uint32_t now = HAL_GetTick();
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (joy_line_pending) {
        for (size_t i = 0; i < sizeof(line); ++i) line[i] = joy_rx_line[i];
        joy_line_pending = 0U;
        have_line = 1U;
    }
    if (primask == 0U) __enable_irq();

    if (Manual_Tick(&joy_control, now)) {
        (void)snprintf(reply, sizeof(reply), "JFAULT %lu TIMEOUT\r\n",
                       (unsigned long)joy_control.session);
        joy_queue_reply(reply);
    }

    if (have_line) {
        ManualMessage message;
        if (Manual_ParseLine(line, &message)) {
            if (message.type == MANUAL_BEGIN) {
                joy_enter_mode(message.session, now);
                (void)snprintf(reply, sizeof(reply), "JREADY %lu\r\n",
                               (unsigned long)message.session);
                joy_queue_reply(reply);
            } else if (Manual_Accept(&joy_control, &message, now)) {
                (void)snprintf(reply, sizeof(reply), "JACK %lu %lu\r\n",
                               (unsigned long)message.session,
                               (unsigned long)message.sequence);
                joy_queue_reply(reply);
            }
        }
    }

    if (!joy_control.owns_motors) {
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        if (!joy_line_pending) joy_uart_owned = 0U;
        if (saved == 0U) __enable_irq();
        joy_send_reply();
        return 0;
    }

    const ManualSetpoint output = Manual_Output(&joy_control, now);
    const int steer_sign = (output.steering > 0) - (output.steering < 0);
    const int kind = output.direction == 0 ? 0 :
                     (output.direction > 0 ? 1 : 4) + (steer_sign + 1);

    if (kind != 0 && kind != joy_telemetry_kind) {
        const char *name = output.direction > 0
            ? (steer_sign < 0 ? "MAN_FL" : (steer_sign > 0 ? "MAN_FR" : "MAN_FWD"))
            : (steer_sign < 0 ? "MAN_RL" : (steer_sign > 0 ? "MAN_RR" : "MAN_REV"));
        Telemetry_StartCommand(name, output.throttle_percent);
    }
    joy_telemetry_kind = kind;

    const int handled = joy_apply_drive(output);
    joy_send_reply();
    return handled;
}
/* ---------------- luther End hold-to-drive manual mode ---------------- */

static const char *drive_log_name(void)
{
  switch (uart_cmd)
  {
  case DRIVE_IDLE:
    return "NONE";
  case DRIVE_FORWARD:
    return "FORWARD";
  case DRIVE_REVERSE:
    return "REVERSE";
  case DRIVE_STOP:
    return "STOP";
  case DRIVE_FR:
    return "ARC_RIGHT";
  case DRIVE_FL:
    return "ARC_LEFT";
  case DRIVE_RR:
    return "ARC_RR";
  case DRIVE_RL:
    return "ARC_RL";
  case DRIVE_JOYSTICK:
     return "MANUAL";
  default:
    return "?";
  }
}

// to take a whole string of commands and break it up
static void queue_script_line(char *line)
{
  if (strcmp(line, "smooth_rel?") == 0)
  {
    if (uart_cmd != DRIVE_IDLE || !queue_empty())
    {
      // A delayed handshake retry is a status query, not a motion abort.
      queue_range_reply("SMOOTH_REL_BUSY,50\n");
      return;
    }
    fu_sequence_no = 0U;
    park_fx_active = park_fx_has_range = park_fx_range_confirmed = park_fx_confirm_samples = 0U;
    route_axis_valid = route_entry_hold_pending = 0U;
    fu2_approach_select_pending = 0U;
    fu2_fwd_waiting_echo = fu2_rev_waiting_echo = 0U;
    fu2_fwd_near_samples = 0U;
    fu2_rev_reacquire_samples = 0U;
    fu2_entry90_active = 0U;
    park_turn_profile = moving_turn_pending = moving_turn_active = 0U;
    paired_slide_heading_valid = 0U;
    ir_motion_active = ir_hold_heading_lock_valid = ir_feedback_keep = 0U;
    section2_exit_turn_limited = section2_exit_straight_limited = 0U;
    arc_trace_heading_reference = total_angle;
    section2_rear_carry_active = 0U;
    section2_entry_carry_active = 0U;
    rear_unwind_active = rear_unwind_seed_pending = 0U;
    rear_unwind_stable_tick = 0U;
    section2_rear_steady_turn_active = 0U;
    rear_turn_direct_active = rear_turn_ordinary_active = turn_steering_entry_pending = 0U;
    parking_90_started = parking_first90_active = parking_final90_active = 0U;
    queue_range_reply("SMOOTH_REL_READY,50\n");
    return;
  }
  if (strcmp(line, "smooth_align?") == 0)
  {
    script_fault_reply = "ERR:RELATIVE_ONLY\r\n";
    abort_now = 1U;
    return;
  }

  if (strcmp(line, "smooth?") == 0)
  {
    queue_range_reply("SMOOTH_READY\n");
    return;
  }
  if (strcmp(line, "confirm") == 0)
  {
    if (obstacle_stop_mode == STOP_MODE_US_BACK && us_allow_reverse)
      fu_arrow_confirmed = 1U;
    return; // control notification, not an extra script/OK
  }
  // replace commas with spaces so strtok can split both
  for (char *p = line; *p; ++p)
    if (*p == ',')
      *p = ' ';

  char *tok = strtok(line, " \t");
  while (tok)
  {

    if (strcmp(tok, "rst") == 0)
    {
      abort_now = 1; // handled in motorTask ASAP
      // optional: drop the rest of the line
      break;
    }
    motion_step_t it = {STEP_NONE, 0};

    // tokens already lowercased in Rx callback
    if (strcmp(tok, "href") == 0 || strncmp(tok, "yaw", 3U) == 0)
    {
      script_fault_reply = "ERR:RELATIVE_ONLY\r\n";
      abort_now = 1U;
      return;
    }
    else if (strcmp(tok, "park") == 0)
    {
      it.type = STEP_PARK_PROFILE;
    }
    else if (strcmp(tok, "end") == 0)
    {
      it.type = STEP_END; // Pi stage marker inside a continuous script
    }
    else if (strcmp(tok, "confirm") == 0)
    {
      if (us_allow_reverse &&
          (obstacle_stop_mode == STOP_MODE_US_T || obstacle_stop_mode == STOP_MODE_US_BACK))
        fu_arrow_confirmed = 1U;
    }
    else if (strcmp(tok, "fir") == 0)
    {
      it.type = STEP_FIR;
      it.value = 0;
    }
    else if (strcmp(tok, "fil") == 0)
    {
      it.type = STEP_FIL;
      it.value = 0;
    }
    else if (strcmp(tok, "firo") == 0)
    {
      it.type = STEP_FIRO;
      it.value = 0;
    }
    else if (strcmp(tok, "filo") == 0)
    {
      it.type = STEP_FILO;
      it.value = 0;
    }
    else if (strcmp(tok, "sr") == 0)
    {
      it.type = STEP_SLIDE_RIGHT;
      it.value = 0;
    }
    else if (strcmp(tok, "sl") == 0)
    {
      it.type = STEP_SLIDE_LEFT;
      it.value = 0;
    }
    else if (tok[0] == 'f' && tok[1] == 'x')
    {
      it.type = STEP_FX;
      it.value = atoi(&tok[2]);
    }
    else if (tok[0] == 'f' && tok[1] == 'u')
    {
      it.type = STEP_FU;
      it.value = atoi(&tok[2]);
    }
    else if (tok[0] == 'f' && tok[1] == 'r')
    {
      it.type = STEP_FR;
      it.value = atoi(&tok[2]);
    }
    else if (tok[0] == 'f' && tok[1] == 'l')
    {
      it.type = STEP_FL;
      it.value = atoi(&tok[2]);
    }
    else if (tok[0] == 'r' && tok[1] == 'r')
    {
      it.type = STEP_RR;
      it.value = atoi(&tok[2]);
    }
    else if (tok[0] == 'r' && tok[1] == 'l')
    {
      it.type = STEP_RL;
      it.value = atoi(&tok[2]);
    }
    else if (tok[0] == 'f')
    {
      it.type = STEP_FORWARD_CM;
      it.value = atoi(&tok[1]);
    }
    else if (tok[0] == 'r')
    {
      it.type = STEP_REVERSE_CM;
      it.value = atoi(&tok[1]);
    }
    else if (tok[0] == 's')
    {
      it.type = STEP_STOP;
      it.value = 0;
    }

    if (it.type != STEP_NONE)
    {
      // motorTask parses completed RX lines and owns the script queue.
      int ok = queue_add(it);
      if (!ok)
      {
        Telemetry_RecordFault("SCRIPT_QUEUE_FULL");
        abort_now = 1U;
        return;
      }
    }
    tok = strtok(NULL, " \t");
  }
  motion_step_t eos = {STEP_END, 0};
  if (!queue_add(eos))
  {
    Telemetry_RecordFault("SCRIPT_QUEUE_FULL");
    abort_now = 1U;
  }
}

static const char *step_log_name(step_kind_t type)
{
    switch (type)
    {
    case STEP_FORWARD_CM:
        return "FWD_CM";

    case STEP_REVERSE_CM:
        return "REV_CM";

    case STEP_FR:
        return "ARC_FR";

    case STEP_FL:
        return "ARC_FL";

    case STEP_RR:
        return "ARC_RR";

    case STEP_RL:
        return "ARC_RL";

    case STEP_PARK_PROFILE:
        return NULL;

    case STEP_SLIDE_RIGHT:
        return "SLIDE_R";

    case STEP_SLIDE_LEFT:
        return "SLIDE_L";

    case STEP_STOP:
        return "STOP";

    case STEP_FIR:
        return "FIR";

    case STEP_FIL:
        return "FIL";

    case STEP_FIRO:
        return "FIRO";

    case STEP_FILO:
        return "FILO";

    case STEP_FU:
        return "FU";

    case STEP_FX:
        return "FX";

    default:
        return NULL;
    }
}

// pop from queue and set all the variables needed, then change the uart_cmd
static int run_next_step(void)
{
	if (!steering_ready || !gyro_healthy){return 0;} //disable commands before steering sequence is finalized

	motion_step_t it;
	if (!queue_take(&it))
		return 0;
    park_fx_active = 0U;

    // Capture the pair origin before the following non-slide clears validity.
    const bool fu_after_slide = it.type == STEP_FU && paired_slide_heading_valid;
    const float fu_entry_heading = fu_after_slide ? paired_slide_heading : total_angle;
    // Ordinary F between the paired slides must retain their common origin.
    // Otherwise the second slide would adopt the first slide's residual yaw.
    const step_kind_t after_item = peek_motion_kind();
    const bool slide_bridge = it.type == STEP_FORWARD_CM && it.value > 0 &&
        paired_slide_heading_valid &&
        (after_item == STEP_SLIDE_RIGHT || after_item == STEP_SLIDE_LEFT);
    if (slide_bridge)
    {
        route_axis_heading = paired_slide_heading;
        route_axis_valid = 1U;
        char bridge_msg[48];
        int n = snprintf(bridge_msg, sizeof(bridge_msg), "SLIDE_BRIDGE,%d,%.1f\n",
            it.value, (double)paired_slide_heading);
        if (n > 0 && n < (int)sizeof(bridge_msg)) queue_range_reply(bridge_msg);
    }
    if (fu_after_slide && fu_sequence_no == 1U)
    {
        route_axis_heading = fu_entry_heading;
        route_axis_valid = 1U;
    }
    if (it.type == STEP_SLIDE_RIGHT || it.type == STEP_SLIDE_LEFT ||
        it.type == STEP_STOP || it.type == STEP_RR || it.type == STEP_RL)
        route_axis_valid = 0U;
    // Metadata must not discard the preceding turn's intended axis.
    if (it.type != STEP_END && it.type != STEP_PARK_PROFILE)
        route_entry_hold_pending = 0U;
    if (it.type != STEP_END)
    {
        // Capture the previous FU context before replacing its heading flag.
        fu2_entry90_active = fu_sequence_no == 2U && fu_slide_heading_active &&
            !park_turn_profile && step_is_forward_turn(it.type) && it.value == 90;
        fu_slide_heading_active = fu_after_slide;
        // Capture the rear-pass context before the non-IR setup clears it.
        const bool rear_ir_to_90 = !park_turn_profile &&
            ir_motion_active && section2_rear_carry_active &&
            step_is_forward_turn(it.type) && it.value == 90;
        rear_turn_direct_active = rear_ir_to_90 && !REAR90_USE_ORDINARY_TURN;
        rear_turn_ordinary_active = rear_ir_to_90 && REAR90_USE_ORDINARY_TURN;
        if (rear_turn_ordinary_active)
            queue_range_reply("TURN_CFG,REAR90_ORDINARY\n");
        const bool parking_90 = park_turn_profile &&
            step_is_forward_turn(it.type) && it.value == 90;
        parking_first90_active = !PARK_CONTINUOUS_TURNS && parking_90 && parking_90_started == 0U;
        parking_final90_active = !PARK_CONTINUOUS_TURNS && parking_90 && parking_90_started == 1U;
        if (parking_90 && parking_90_started < 2U) ++parking_90_started;
        if (PARK_CONTINUOUS_TURNS && parking_90)
        {
            char entry_msg[48];
            int n = snprintf(entry_msg, sizeof(entry_msg), "TURN_CFG,PARK_ROLLING,%u\n",
                (unsigned)parking_90_started);
            if (n > 0 && n < (int)sizeof(entry_msg)) queue_range_reply(entry_msg);
        }
        turn_steering_entry_pending = 0U;
        moving_turn_active = moving_turn_pending && step_is_forward_turn(it.type) &&
            !rear_turn_direct_active && !parking_first90_active && !parking_final90_active;
        moving_turn_pending = 0U;
    }
    if (it.type != STEP_SLIDE_RIGHT && it.type != STEP_SLIDE_LEFT && it.type != STEP_END && !slide_bridge)
        paired_slide_heading_valid = 0U;
    const bool ir_command = step_uses_ir(it.type);
    if (!ir_command && it.type != STEP_END)
    {
        section2_rear_carry_active = 0U;
        section2_entry_carry_active = 0U;
        rear_unwind_active = rear_unwind_seed_pending = 0U;
        rear_unwind_stable_tick = 0U;
    }
    if (ir_command)
    {
        ir_feedback_keep = ir_motion_active;
        if (!ir_motion_active)
        {
            ir_hold_start_tick = HAL_GetTick();
            ir_hold_trace_tick = 0U;
        }
        if (!ir_hold_heading_lock_valid)
            ir_hold_heading_reference = route_axis_valid ? route_axis_heading : total_angle;
        ir_hold_heading_lock_valid = ir_motion_active = 1U;
    }
    else if (it.type != STEP_END)
    {
        ir_motion_active = ir_hold_heading_lock_valid = ir_feedback_keep = 0U;
    }
	const char *telemetry_name = step_log_name(it.type);

	if (telemetry_name != NULL)
	{
		/* EncoderTask snapshots the gains for the SPID line as soon as it sees
		 * the new command, so the gains for THIS command must be set first. */
        if (ir_command)
        {
            Telemetry_SetStraightPidGains(IR_HOLD_KP_PERCENT_PER_DEG,
                IR_HOLD_KI_PERCENT_PER_DEG_S, IR_HOLD_KD_PERCENT_PER_DPS);
        }
        else if (it.type == STEP_FX && park_turn_profile && parking_90_started >= 2U)
        {
            Telemetry_SetStraightPidGains(PARKING_FX_HOLD_KP_PERCENT_PER_DEG,
                PARKING_FX_HOLD_KI_PERCENT_PER_DEG_S, PARKING_FX_HOLD_KD_PERCENT_PER_DPS);
        }
		else if (fu_after_slide && fu_sequence_no == 1U)
		{
            Telemetry_SetStraightPidGains(FU2_HOLD_KP_PERCENT_PER_DEG,
                FU2_HOLD_KI_PERCENT_PER_DEG_S, FU2_HOLD_KD_PERCENT_PER_DPS);
		}
		else if (it.type == STEP_REVERSE_CM)
		{
			Telemetry_SetStraightPidGains(REVERSE_STEER_KP_PERCENT_PER_DEG,
										  REVERSE_STEER_KI_PERCENT_PER_DEG_S,
										  REVERSE_STEER_KD_PERCENT_PER_DPS);
		}
		else
		{
			Telemetry_SetStraightPidGains(FORWARD_STEER_KP_PERCENT_PER_DEG,
										  FORWARD_STEER_KI_PERCENT_PER_DEG_S,
										  FORWARD_STEER_KD_PERCENT_PER_DPS);
		}

		Telemetry_StartCommand(telemetry_name, it.value);
	}

	switch (it.type)
	{

        case STEP_PARK_PROFILE:
            park_turn_profile = 1U;
            parking_90_started = 0U;
            uart_cmd = DRIVE_IDLE;
            return 1;

		case STEP_FX: // fall through — same setup, just different reverse flag
		case STEP_FU:
		{
			// Forward "forever" but stop when ultrasound <= us_stop_cm
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = route_axis_valid ? route_axis_heading : fu_entry_heading;
			steer_center();

			// reset odometry for reporting
			odom_counts_run = 0;
			odom_cm_run = 0.0f;

			target_counts = INT32_MAX;                                           // run "forever"
			us_stop_cm = (it.value > 0) ? (uint16_t)it.value : OBST_STOP_CM; // per-move threshold
			us_allow_reverse = (it.type == STEP_FU) ? 1 : 0;                     // FU=reverse, FX=just stop
			obstacle_stop_mode = STOP_MODE_US_T;                                 // our new mode
			uart_cmd = DRIVE_FORWARD;
            fu_arrow_confirmed = 0U;
            fu2_approach_select_pending = 0U;
            if (it.type == STEP_FU)
            {
              ++fu_sequence_no;
              if (fu_sequence_no == 2U)
              {
                  fu2_close_approach_cm = us_stop_cm;
                  fu2_approach_sample_seq = us_capture_seq;
                  fu2_fwd_sample_seq = us_capture_seq - 1U;
                  fu2_fwd_range_cm = 0U;
                  fu2_fwd_near_samples = fu2_fwd_waiting_echo = 0U;
                  fu2_approach_select_pending = FU2_ADAPTIVE_APPROACH_ENABLE;
                  queue_range_reply("VISION,2\n");
                  char approach_msg[48];
                  int n = snprintf(approach_msg, sizeof(approach_msg), "FU_APPROACH,2,%u,%.0f,%.0f,%.0f\n",
                      (unsigned)us_stop_cm, (double)FU2_APPROACH_DECEL_CM,
                      (double)FU2_APPROACH_ENTRY_EFFORT_L,
                      (double)FU2_APPROACH_ENTRY_EFFORT_R);
                  if (n > 0 && n < (int)sizeof(approach_msg)) queue_range_reply(approach_msg);
              }
              if (fu_slide_heading_active)
              {
                  fu_heading_trace_tick = 0U;
                  char heading_msg[48];
                  int n = snprintf(heading_msg, sizeof(heading_msg), "FU_REF,%u,%.1f,%.1f,%.1f\n",
                      (unsigned)fu_sequence_no, (double)target_angle,
                      (double)total_angle, (double)gyro_yaw_rate_dps);
                  if (n > 0 && n < (int)sizeof(heading_msg)) queue_range_reply(heading_msg);
                  if (fu_sequence_no == 2U)
                  {
                      char hold_msg[48];
                      int hold_n = snprintf(hold_msg, sizeof(hold_msg), "FU_HOLD,%.2f,%.2f,%.0f,%.0f,%.0f\n",
                          (double)FU2_HOLD_KP_PERCENT_PER_DEG,
                          (double)FU2_HOLD_KD_PERCENT_PER_DPS, (double)FU2_HOLD_MAX_PERCENT,
                          (double)FU2_HOLD_SLEW_PERCENT_PER_S,
                          (double)FU2_HOLD_RELEASE_PERCENT_PER_S);
                      if (hold_n > 0 && hold_n < (int)sizeof(hold_msg)) queue_range_reply(hold_msg);
                  }
              }
            }
            if (it.type == STEP_FU && fu_sequence_no == 2U && route_axis_valid)
                start_route_straight();
			seg_rebase = 1; // clean speed/odom baselines
            if (it.type == STEP_FX && park_turn_profile && parking_90_started >= 2U)
                start_park_fx();
			return 1;
		}

		case STEP_FIRO:
		{
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = ir_hold_heading_reference;
			if (!ir_feedback_keep) steer_center();

			odom_counts_run = 0;
			odom_cm_run = 0.0f;

			target_counts = INT32_MAX; // run “forever”
			obstacle_stop_mode = STOP_MODE_IR_RIGHT_O;
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // <--- rebase on entry
			return 1;
		}

		case STEP_FILO:
		{
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = ir_hold_heading_reference;
			if (!ir_feedback_keep) steer_center();

			odom_counts_run = 0;
			odom_cm_run = 0.0f;

			target_counts = INT32_MAX;                // run “forever”
			obstacle_stop_mode = STOP_MODE_IR_LEFT_O; // NEW mode
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // <--- rebase on entry
			return 1;
		}

		case STEP_FIR:
		{
			// forward "forever" until right IR reports 0 (no obstacle)
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = ir_hold_heading_reference;
			if (!ir_feedback_keep) steer_center();

			odom_counts_run_ir = 0;
			odom_cm_run_ir = 0.0f;

			target_counts = INT32_MAX;               // sentinel: distance won't stop us
			obstacle_stop_mode = STOP_MODE_IR_RIGHT; // NEW mode
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // <--- rebase on entry
			return 1;
		}

		case STEP_FIL:
		{
			// forward "forever" until right IR reports 0 (no obstacle)
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = ir_hold_heading_reference;
			if (!ir_feedback_keep) steer_center();

			odom_counts_run_ir = 0;
			odom_cm_run_ir = 0.0f;

			target_counts = INT32_MAX;              // sentinel: distance won't stop us
			obstacle_stop_mode = STOP_MODE_IR_LEFT; // IR LEFT
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // <--- rebase on entry
			return 1;
		}

		case STEP_FORWARD_CM:
		{
			int cm = it.value;
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = route_axis_valid ? route_axis_heading : total_angle;
			steer_center();
            start_route_straight();
			if (park_turn_profile && parking_90_started == 0U && cm > 0)
            {
                char cruise_msg[48];
                int n = snprintf(cruise_msg, sizeof(cruise_msg), "TURN_CFG,RETURN_CRUISE,%.2f,%.0f,%.0f\n",
                    (double)PARK_RETURN_CRUISE_EFFORT_SCALE,
                    (double)(PARK_TURN_CARRY_SPEED_ENABLE && !PARK_RETURN_SLOWDOWN_ENABLE &&
                        step_is_forward_turn(peek_motion_kind())
                        ? 0.0f : PARK_RETURN_DECEL_CM),
                    (double)PARK_RETURN_ACCEL_EFFORT_PER_S);
                if (n > 0 && n < (int)sizeof(cruise_msg)) queue_range_reply(cruise_msg);
                n = snprintf(cruise_msg, sizeof(cruise_msg), "TURN_CFG,RETURN_BEGIN,%d,%lu\n",
                    cm, (unsigned long)HAL_GetTick());
                if (n > 0 && n < (int)sizeof(cruise_msg)) queue_range_reply(cruise_msg);
            }

			if (cm > 0)
			{
				// normal distance move
				int32_t tgt = (int32_t)(cm * ENCODER_COUNTS_PER_CM + 0.5f);
				target_counts = tgt;
				obstacle_stop_mode = STOP_MODE_NONE; // ensure off
				uart_cmd = DRIVE_FORWARD;
			}
			else
			{
				// use ultrasound just go forward forever
				obstacle_stop_mode = STOP_MODE_US;
				target_counts = INT32_MAX; // sentinel; we won't use it
				odom_counts_run = 0;       // reset US segment odometer
				odom_cm_run = 0.0f;
				uart_cmd = DRIVE_FORWARD;
			}
			seg_rebase = 1; // <--- rebase on entry
			return 1;
		}

		case STEP_REVERSE_CM:
		{
			int cm = (it.value > 0) ? it.value : 50;
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			int32_t tgt = -(int32_t)(cm * ENCODER_COUNTS_PER_CM + 0.5f);
			target_counts = tgt;
			target_angle = route_axis_valid ? route_axis_heading : total_angle;
			steer_center();
			uart_cmd = DRIVE_REVERSE;
			seg_rebase = 1;
			return 1;
		}

		case STEP_FR:
		{
			TURN_DEG = (it.value >= 0) ? (float)it.value : -(float)it.value;
			arc_target_angle = (route_axis_valid ? route_axis_heading : total_angle) - TURN_DEG;
			steer_for_turn(DRIVE_FR);

			#if ARC_SERVO_ONLY_TEST
			turn_motor_enable_tick = HAL_GetTick() + ARC_SERVO_TEST_HOLD_MS;
			#else
			turn_motor_enable_tick = HAL_GetTick() + ARC_DELAY_RIGHT;
			#endif
			arc_postforward_cm = 0;
			uart_cmd = DRIVE_FR;
			reset_arc_control();
            brake_for_turn_entry(DRIVE_FR);
			return 1;
		}

		case STEP_FL:
		{
			TURN_DEG = (it.value >= 0) ? (float)it.value : -(float)it.value;
			arc_target_angle = (route_axis_valid ? route_axis_heading : total_angle) + TURN_DEG;
			steer_for_turn(DRIVE_FL);

			#if ARC_SERVO_ONLY_TEST
			turn_motor_enable_tick = HAL_GetTick() + ARC_SERVO_TEST_HOLD_MS;
			#else
			turn_motor_enable_tick = HAL_GetTick() + ARC_DELAY_LEFT;
			#endif
			arc_postforward_cm = 0;
			uart_cmd = DRIVE_FL;
			reset_arc_control();
            brake_for_turn_entry(DRIVE_FL);
			return 1;
		}

		case STEP_RR:
		{
			TURN_DEG = (it.value >= 0) ? (float)it.value : -(float)it.value;
			arc_target_angle = total_angle + TURN_DEG;
			steer_rr();

			#if ARC_SERVO_ONLY_TEST
			turn_motor_enable_tick = HAL_GetTick() + ARC_SERVO_TEST_HOLD_MS;
			#else
			turn_motor_enable_tick = HAL_GetTick() + ARC_DELAY;
			#endif
			arc_postforward_cm = 1;
			uart_cmd = DRIVE_RR;
			reset_arc_control();
			return 1;
		}

		case STEP_RL:
		{
			TURN_DEG = (it.value >= 0) ? (float)it.value : -(float)it.value;
			arc_target_angle = total_angle - TURN_DEG;
			steer_rl();

			#if ARC_SERVO_ONLY_TEST
			turn_motor_enable_tick = HAL_GetTick() + ARC_SERVO_TEST_HOLD_MS;
			#else
			turn_motor_enable_tick = HAL_GetTick() + ARC_DELAY;
			#endif
			arc_postforward_cm = 0;
			uart_cmd = DRIVE_RL;
			reset_arc_control();
			return 1;
		}

		case STEP_STOP:
		{
			brake_wheels();
			stop_arc_control();
			steer_center();
			uart_cmd = DRIVE_STOP;
			return 1;
		}

		case STEP_END:
		{
			ack_script(); // <--- ACK the just-finished line
			// Do NOT set uart_cmd; just return. The idle loop will pick up the next item
			return 1;
		}

        case STEP_SLIDE_RIGHT:
        case STEP_SLIDE_LEFT:
        {
            if (!paired_slide_heading_valid)
            {
                paired_slide_heading = total_angle;
                paired_slide_heading_valid = 1U;
            }
            slide_origin_heading = (double)paired_slide_heading;
            slide_return_heading = slide_origin_heading;
            slide_mode = it.type == STEP_SLIDE_RIGHT ? SLIDE_TO_RIGHT : SLIDE_TO_LEFT;
            const step_kind_t opposite_slide = it.type == STEP_SLIDE_RIGHT
                ? STEP_SLIDE_LEFT : STEP_SLIDE_RIGHT;
            // Each command is a complete sideways slide: OUT by the selected
            // angle, then IN to the common original heading. Never consume the
            // next slide here; it must run its own OUT and IN bends afterwards.
            const bool first_of_pair = !queue_empty() &&
                step_queue[queue_read].type == opposite_slide;
            slide_active_angle_deg = fmaxf(1.0f, first_of_pair
                ? SLIDE_FIRST_PAIR_ANGLE_DEG : (float)SLIDE_ANGLE_DEG);
            slide_phase = SLIDE_OUTWARD;
            slide_stage_start_tick = HAL_GetTick();
            slide_clearance_start_counts = 0;
            target_angle = (float)slide_origin_heading;
            arc_target_angle = target_angle +
                (slide_mode == SLIDE_TO_RIGHT ? -slide_active_angle_deg : slide_active_angle_deg);
            enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
            enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            target_counts = INT32_MAX;
            obstacle_stop_mode = STOP_MODE_NONE;
            uart_cmd = DRIVE_FORWARD;
            seg_rebase = 1U;
            drive_slide_stage();
            char phase_msg[48];
            int n = snprintf(phase_msg, sizeof(phase_msg), "SLIDE,%c,OUT,%.1f,%.1f\n",
                slide_mode == SLIDE_TO_RIGHT ? 'R' : 'L',
                slide_origin_heading, (double)arc_target_angle);
            if (n > 0 && n < (int)sizeof(phase_msg)) queue_range_reply(phase_msg);
            char geometry_msg[64];
            int geometry_n = snprintf(geometry_msg, sizeof(geometry_msg),
                "SLIDE,%c,GEOMETRY,%.1f,%s\n",
                slide_mode == SLIDE_TO_RIGHT ? 'R' : 'L',
                (double)slide_active_angle_deg, first_of_pair ? "FIRST_FULL" : "FULL");
            if (geometry_n > 0 && geometry_n < (int)sizeof(geometry_msg))
                queue_range_reply(geometry_msg);
            char slide_speed_msg[48];
            int speed_n = snprintf(slide_speed_msg, sizeof(slide_speed_msg), "SLIDE_SPEED,%.2f,%d,%d\n",
                (double)SLIDE_DRIVE_EFFORT_SCALE,
                PWM_OFF - scaled_slide_pwm(SLIDE_OUTER_PWM),
                PWM_OFF - scaled_slide_pwm(SLIDE_INNER_PWM));
            if (speed_n > 0 && speed_n < (int)sizeof(slide_speed_msg)) queue_range_reply(slide_speed_msg);
            return 1;
        }

		default:
		break;
		}
	return 0;
}

// ultrasound

// IR conversion calculation
float Voltage_To_Distance_CM(float v) {
    if (v < 0.1f) return 80.0f; // Guard against near-zero division or noise
    float cm = 27.86f * powf(v, -1.15f);
    if (cm > 80.0f) return 80.0f;
    if (cm < 10.0f) return 10.0f;
    return cm;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	const uint32_t reset_cause_flags = RCC->CSR;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and  the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Delay_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  Telemetry_Init(&huart2);

  Telemetry_SetStraightPidGains(
      FORWARD_STEER_KP_PERCENT_PER_DEG,
      FORWARD_STEER_KI_PERCENT_PER_DEG_S,
      FORWARD_STEER_KD_PERCENT_PER_DPS
  );

  if (reset_cause_flags & RCC_CSR_IWDGRSTF)
    Telemetry_RecordFault("BOOT_IWDG");
  else if (reset_cause_flags & RCC_CSR_WWDGRSTF)
    Telemetry_RecordFault("BOOT_WWDG");
  else if (reset_cause_flags & RCC_CSR_SFTRSTF)
    Telemetry_RecordFault("BOOT_SOFTWARE");
  else if (reset_cause_flags & RCC_CSR_BORRSTF)
    Telemetry_RecordFault("BOOT_BROWNOUT");
  else if (reset_cause_flags & RCC_CSR_PINRSTF)
    Telemetry_RecordFault("BOOT_PIN");
  else if (reset_cause_flags & RCC_CSR_PORRSTF)
    Telemetry_RecordFault("BOOT_POWER");
  else
    Telemetry_RecordFault("BOOT_OTHER");

  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Start byte-by-byte UART RX */
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);

  // IR DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ir_adc, 2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, ledTask, osPriorityNormal, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of OLED_Task */
  osThreadDef(OLED_Task, oledTask, osPriorityBelowNormal, 0, 384);
  OLED_TaskHandle = osThreadCreate(osThread(OLED_Task), NULL);

  /* definition and creation of Motor_Task */
  osThreadDef(Motor_Task, motorTask, osPriorityAboveNormal, 0, 1024);
  Motor_TaskHandle = osThreadCreate(osThread(Motor_Task), NULL);

  /* definition and creation of Encoder_Task */
  osThreadDef(Encoder_Task, encoderTask, osPriorityBelowNormal, 0, 512);
  Encoder_TaskHandle = osThreadCreate(osThread(Encoder_Task), NULL);

  /* definition and creation of Gyro_Task */
  osThreadDef(Gyro_Task, gyroTask, osPriorityNormal, 0, 384);
  Gyro_TaskHandle = osThreadCreate(osThread(Gyro_Task), NULL);

  /* definition and creation of Ultrasound_Task */
  osThreadDef(Ultrasound_Task, ultrasoundTask, osPriorityBelowNormal, 0, 256);
  Ultrasound_TaskHandle = osThreadCreate(osThread(Ultrasound_Task), NULL);

  /* definition and creation of IR_Task */
  osThreadDef(IR_Task, irTask, osPriorityIdle, 0, 128);
  IR_TaskHandle = osThreadCreate(osThread(IR_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 7199;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 160;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RES_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RES_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RES_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : US_Trig_Pin */
  GPIO_InitStruct.Pin = US_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_Trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_Button_Pin */
  GPIO_InitStruct.Pin = OLED_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OLED_Button_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HCSR04_Trigger(void)
{
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);
  HAL_Delay(1); // 1 ms pulse; ultrasoundTask uses trigger_us_pulse.
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
}

static inline void trigger_us_pulse(void)
{
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
  wait_us(2);
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);
  wait_us(10);
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
}

static inline uint32_t median_of_three(uint32_t a, uint32_t b, uint32_t c)
{
  uint32_t max = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
  uint32_t min = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
  return a + b + c - max - min;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM8 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) return;

  if (!us_echo_started) {
    // Rising edge: reset counter and switch to falling edge
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    us_echo_started = 1;
  } else {
    // Falling edge: capture width and switch back to rising edge
    us_pulse_ticks = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
    us_echo_started = 0;

    // Convert ticks to cm and update globals
    uint32_t raw_cm = (uint32_t)(us_pulse_ticks * (0.0343f / 2.0f) + 0.5f);
    us_history[us_history_pos] = raw_cm;
    us_history_pos = (us_history_pos + 1U) % 3U;
    if (us_history_used < 3U) {
      us_history_used++;
    }
    uint32_t filtered_cm = raw_cm;
    if (us_history_used == 3U) {
      filtered_cm = median_of_three(us_history[0], us_history[1], us_history[2]);
    }
    distance = (float)filtered_cm;
    echo_debug = filtered_cm;
    us_raw_cm = raw_cm;
    us_raw_heading = total_angle;
    us_capture_tick = HAL_GetTick();
    us_capture_valid = raw_cm >= US_VALID_MIN_CM && raw_cm <= US_VALID_MAX_CM;
    if (us_capture_valid) us_valid_echo_tick = us_capture_tick;
    ++us_capture_seq;
    new_measurement_ready = true;
  }
}

// luther new callback for controller pad
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3) return;
  if (huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
    // HAL can deliver the error byte before calling ErrorCallback. Never
    // enqueue a line terminated by a corrupt byte as a valid motion script.
    rx_pos = 0U;
    rx_line_overflow = 1U;
    // ErrorCallback rearms RX after HAL has classified the error. Rearming
    // here would clear ErrorCode before HAL checks whether ORE is blocking.
    return;
  }
  const uint8_t c = rx_byte;
  if (c == '\r' || c == '\n') {
    rx_line[rx_pos] = '\0';
    if (!rx_line_overflow && rx_pos > 0U) {
      for (uint16_t i = 0U; i < rx_pos; ++i) {
        if (rx_line[i] >= 'A' && rx_line[i] <= 'Z')
          rx_line[i] = (char)(rx_line[i] - 'A' + 'a');
      }
      if (strcmp(rx_line, "rst") == 0) {
        /* Priority abort; never enqueue it behind a held movement. */
        abort_now = 1U;
      } else if (rx_line[0] == 'j') {
        joy_receive_line(rx_line);
      } else if (!joy_uart_owned && !abort_now) {
        const uint8_t next = (uint8_t)((rx_script_write + 1U) % RX_SCRIPT_SLOTS);
        if (next == rx_script_read) abort_now = 1U;
        else {
          memcpy(rx_script_lines[rx_script_write], rx_line, (size_t)rx_pos + 1U);
          rx_script_write = next;
        }
      }
      /* Ordinary scripted commands are ignored while manual mode owns motors. */
    }
    rx_pos = 0U;
    rx_line_overflow = 0U;
  } else if (!rx_line_overflow) {
    if (c < 32U || c > 126U) {
      if (c != '\t') rx_line_overflow = 1U;
      else if (rx_pos < sizeof(rx_line) - 1U) rx_line[rx_pos++] = (char)c;
      else rx_line_overflow = 1U;
    } else if (rx_pos < sizeof(rx_line) - 1U) {
      rx_line[rx_pos++] = (char)c;
    } else {
      /* Discard the WHOLE overlong line, including its tail, until a delimiter. */
      rx_line_overflow = 1U;
    }
  }
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3) return;
  uart3_rx_error_flags |= huart->ErrorCode;
  ++uart3_rx_error_count;
  rx_pos = 0U;
  rx_line_overflow = 1U; // Discard the damaged line through its next delimiter.
  __HAL_UART_CLEAR_OREFLAG(huart); // SR/DR read clears ORE/FE/NE/PE.
  (void)HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1U);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    // raw -> mV (rounded)
    uint16_t mV0 = (uint16_t)((3300u * ir_adc[0] + 2047u) / 4095u);
    uint16_t mV1 = (uint16_t)((3300u * ir_adc[1] + 2047u) / 4095u);

    // EMA in integer domain: y = 0.8*y + 0.2*x  ->  y = (4*y + x) / 5
    ir0_mV = (uint16_t)((4u * ir0_mV + mV0) / 5u);
    ir1_mV = (uint16_t)((4u * ir1_mV + mV1) / 5u);

    ir0_raw = ir_adc[0];
    ir1_raw = ir_adc[1];

    ir0_obs = (ir0_mV >= IR_MV_OBS);
    ir1_obs = (ir1_mV >= IR_MV_OBS);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ledTask */
/**
 * @brief  Function implementing the LED_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_ledTask */
void ledTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint8_t ch = 'A';
  for (;;)
  {
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_oledTask */
/**
 * @brief Function implementing the OLED_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_oledTask */
void oledTask(void const * argument)
{
  /* USER CODE BEGIN oledTask */
  char line[32];
  static uint8_t display_page = 0;
  #define DISPLAY_PAGES 2

  static GPIO_PinState last_btn_state = GPIO_PIN_SET;

  for (;;)
  {
    // ---- Check Button State (Polling) ----
    GPIO_PinState current_btn_state = HAL_GPIO_ReadPin(OLED_Button_GPIO_Port, OLED_Button_Pin);
    if (last_btn_state == GPIO_PIN_SET && current_btn_state == GPIO_PIN_RESET)
    {
      display_page = (display_page + 1) % DISPLAY_PAGES;
      osDelay(50); // Software debounce delay
    }
    last_btn_state = current_btn_state;

    if (!gyro_healthy)
    {
      OLED_ShowString(0, 0, (uint8_t *)"GYRO NOT READY  ");
      OLED_ShowString(0, 16, (uint8_t *)"MOTION DISABLED ");
      OLED_ShowString(0, 32, (uint8_t *)"AUTO RETRY...   ");
      OLED_ShowString(0, 48, (uint8_t *)"KEEP CAR STILL  ");
      OLED_Refresh_Gram();
      osDelay(20);
      continue;
    }

    if (display_page == 0)
    {
    	// --- PAGE 0: Yaw, Target, Speeds, and CMD ---
		snprintf(line, sizeof(line), "Yaww: %-11.1f", (double)total_angle);
		OLED_ShowString(0, 0, (uint8_t *)line);

		snprintf(line, sizeof(line), "Tgt: %-11d", (int)arc_target_angle);
		OLED_ShowString(0, 16, (uint8_t *)line);

		snprintf(line, sizeof(line), "A:%5d B:%5d", (int)delta_a, (int)delta_b);
		OLED_ShowString(0, 32, (uint8_t *)line);

		snprintf(line, sizeof(line), "CMD: %-11s", drive_log_name());
		OLED_ShowString(0, 48, (uint8_t *)line);
	}
    else if (display_page == 1)
    {
		// --- PAGE 1: Ultrasonic, IR mV/Status, and IR Distances ---

		// Line 1 ($y = 0): Ultrasonic Distance
		snprintf(line, sizeof(line), "US Dist: %3d cm    ", (int)echo_debug);
		OLED_ShowString(0, 0, (uint8_t *)line);

		// Line 2 ($y = 16): IR0 Millivolts and Status
		snprintf(line, sizeof(line), "L:%4umV [%d]   ", ir0_mV, ir0_obs);
		OLED_ShowString(0, 16, (uint8_t *)line);

		// Line 3 ($y = 32): IR1 Millivolts and Status
		snprintf(line, sizeof(line), "R:%4umV [%d]   ", ir1_mV, ir1_obs);
		OLED_ShowString(0, 32, (uint8_t *)line);

		// Line 4 ($y = 48): Calculated Distances for both IR sensors
		snprintf(line, sizeof(line), "L:%dcm R:%dcm   ", (int)ir0_distance_cm, (int)ir1_distance_cm);
		OLED_ShowString(0, 48, (uint8_t *)line);
    }

    OLED_Refresh_Gram();
    osDelay(20);
  }
  /* USER CODE END oledTask */
}

/* USER CODE BEGIN Header_motorTask */
/**
 * @brief Function implementing the Motor_Taskl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_motorTask */
/* No forward output or reverse decision is allowed from stale/off-axis
 * sonar. Retain the queued vision/turn script while waiting at rest. */
static bool fu2_range_ready(void)
{
    if (joy_control.owns_motors || slide_mode != SLIDE_NO_SIDE ||
        fu_sequence_no != 2U || !us_allow_reverse || uart_cmd != DRIVE_FORWARD ||
        obstacle_stop_mode != STOP_MODE_US_T) return true;
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const uint32_t seq = us_capture_seq;
    const uint32_t tick = us_capture_tick;
    const uint32_t range = us_raw_cm;
    const float heading = us_raw_heading;
    const bool valid = us_capture_valid != 0U;
    __set_PRIMASK(irq_state);
    const uint32_t now = HAL_GetTick();
    const bool fresh = valid && (uint32_t)(now - tick) <= FU2_EARLY_US_SAMPLE_MAX_AGE_MS &&
        fabsf(heading - target_angle) <= FU2_EARLY_US_SAMPLE_MAX_ERROR_DEG;
    if (!fresh) fu2_fwd_near_samples = 0U;
    else
    {
        fu2_fwd_range_cm = range;
        if (seq != fu2_fwd_sample_seq)
        {
            fu2_fwd_sample_seq = seq;
            if (range <= FU2_REVERSE_TRIGGER_CM)
            {
                if (fu2_fwd_near_samples < FU2_REVERSE_START_CONFIRM_SAMPLES)
                    ++fu2_fwd_near_samples;
            }
            else fu2_fwd_near_samples = 0U;
        }
    }
    if (fresh && (range > FU2_REVERSE_TRIGGER_CM ||
        fu2_fwd_near_samples >= FU2_REVERSE_START_CONFIRM_SAMPLES))
    {
        if (fu2_fwd_waiting_echo)
        {
            char message[48];
            int n = snprintf(message, sizeof(message), "FU_US,READY,%lu,%u\n",
                (unsigned long)range, (unsigned)fu2_fwd_near_samples);
            if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
        }
        fu2_fwd_waiting_echo = 0U;
        return true;
    }

    brake_wheels();
    if (!fu2_fwd_waiting_echo)
    {
        fu2_fwd_wait_tick = now;
        fu2_fwd_waiting_echo = 1U;
        char message[64];
        int n = snprintf(message, sizeof(message), "FU_US,HOLD,%c,%lu,%lu,%.1f\n",
            fresh ? 'C' : 'S', (unsigned long)range, (unsigned long)(now - tick),
            (double)(heading - target_angle));
        if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
    }
    if ((uint32_t)(now - fu2_fwd_wait_tick) >= FU2_FORWARD_US_WAIT_MS)
    {
        Telemetry_RecordFault("FU2_US_UNCONFIRMED");
        script_fault_reply = "ERR:FU2_US_UNCONFIRMED\r\n";
        abort_now = 1U;
    }
    return false;
}

/* Select once from a new, valid echo after the slide pair, while pointing
 * near the approach axis. Raw range avoids using the median's slide-era
 * history; FU2 stops also use raw data validated by the motor-loop guard.
 * Until a usable sample arrives, retain the requested close threshold. */
static void choose_fu2_approach(void)
{
    if (!fu2_approach_select_pending || fu_sequence_no != 2U || !us_allow_reverse ||
        uart_cmd != DRIVE_FORWARD || obstacle_stop_mode != STOP_MODE_US_T) return;
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const uint32_t seq = us_capture_seq;
    const uint32_t sample_tick = us_capture_tick;
    const uint32_t range_cm = us_raw_cm;
    const float sample_heading = us_raw_heading;
    const bool valid = us_capture_valid != 0U;
    __set_PRIMASK(irq_state);
    if (seq == fu2_approach_sample_seq) return;
    fu2_approach_sample_seq = seq;
    if (!valid || (uint32_t)(HAL_GetTick() - sample_tick) > FU2_APPROACH_SAMPLE_MAX_AGE_MS ||
        fabsf(target_angle - sample_heading) > FU2_APPROACH_SAMPLE_MAX_ERROR_DEG) return;

    const uint16_t far_stop_cm = FU2_FAR_APPROACH_CM > fu2_close_approach_cm
        ? FU2_FAR_APPROACH_CM : fu2_close_approach_cm;
    const uint32_t far_min_cm = (uint32_t)fmaxf((float)FU2_FAR_MIN_RANGE_CM,
        (float)far_stop_cm + FU2_APPROACH_DECEL_CM);
    us_stop_cm = range_cm >= far_min_cm ? far_stop_cm : fu2_close_approach_cm;
    fu2_approach_select_pending = 0U;
    char message[48];
    int n = snprintf(message, sizeof(message), "FU_SELECT,2,%lu,%u,%c\n",
        (unsigned long)range_cm, (unsigned)us_stop_cm,
        range_cm >= far_min_cm ? 'F' : 'C');
    if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
}

static void start_fu2_reverse(uint32_t entry_cm)
{
    if (fu_sequence_no != 2U) return;
    fu2_approach_select_pending = 0U;
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    fu2_rev_sample_seq = us_capture_seq;
    __set_PRIMASK(irq_state);

    fu2_rev_clear_cm = (uint16_t)fminf((float)us_stop_cm,
        (float)FU2_REVERSE_CLEAR_CM);
    const float minimum_cm = fmaxf(0.0f, (float)fu2_rev_clear_cm - (float)entry_cm);
    fu2_rev_min_counts = (int32_t)ceilf(minimum_cm * ENCODER_COUNTS_PER_CM);
    fu2_rev_max_counts = fu2_rev_min_counts +
        (int32_t)ceilf(FU2_REVERSE_EXTRA_TRAVEL_CM * ENCODER_COUNTS_PER_CM);
    fu2_rev_clear_samples = 0U;
    fu2_rev_start_tick = HAL_GetTick();
    fu2_rev_reference_cm = entry_cm;
    fu2_rev_reference_counts = 0;
    fu2_rev_reacquire_samples = 0U;
    fu2_rev_last_range_cm = entry_cm;
    fu2_rev_last_usable_tick = fu2_rev_start_tick;
    fu2_rev_waiting_echo = 0U;
    char message[48];
    int n = snprintf(message, sizeof(message), "REV_CLEAR,2,%lu,%u,%.1f\n",
        (unsigned long)entry_cm, (unsigned)fu2_rev_clear_cm, (double)minimum_cm);
    if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
}

/* Called once per new, valid echo while braked, aligned and at low yaw.
 * Do not use a single far reading to release the next turn. */
static bool fu2_reverse_check_range(uint32_t range_cm)
{
    const uint16_t a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    const uint16_t b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint32_t low = range_cm, high = range_cm;
    bool restart = fu2_rev_reacquire_samples == 0U;
    if (!restart)
    {
        low = range_cm < fu2_rev_reacquire_low_cm ? range_cm : fu2_rev_reacquire_low_cm;
        high = range_cm > fu2_rev_reacquire_high_cm ? range_cm : fu2_rev_reacquire_high_cm;
        const float movement_limit = FU2_REVERSE_REACQUIRE_MAX_MOVE_CM * ENCODER_COUNTS_PER_CM;
        restart = high - low > FU2_REVERSE_REACQUIRE_SPREAD_CM ||
            fabsf((float)(int16_t)(a - fu2_rev_reacquire_start_a)) > movement_limit ||
            fabsf((float)(int16_t)(b - fu2_rev_reacquire_start_b)) > movement_limit;
    }
    if (restart)
    {
        fu2_rev_reacquire_low_cm = fu2_rev_reacquire_high_cm = range_cm;
        fu2_rev_reacquire_start_a = a;
        fu2_rev_reacquire_start_b = b;
        fu2_rev_reacquire_samples = 1U;
    }
    else
    {
        fu2_rev_reacquire_low_cm = low;
        fu2_rev_reacquire_high_cm = high;
        if (fu2_rev_reacquire_samples < FU2_REVERSE_REACQUIRE_CONFIRM_SAMPLES)
            ++fu2_rev_reacquire_samples;
    }
    if (fu2_rev_reacquire_samples < FU2_REVERSE_REACQUIRE_CONFIRM_SAMPLES) return false;

    // Use the nearest reading in the cluster. A confirmed larger range can
    // disprove the original close echo and its excessive encoder minimum.
    fu2_rev_reference_cm = fu2_rev_reacquire_low_cm;
    fu2_rev_reference_counts = reverse_travel_counts;
    const float remaining_cm = fmaxf(0.0f,
        (float)fu2_rev_clear_cm - (float)fu2_rev_reference_cm);
    const int32_t revised_minimum = reverse_travel_counts +
        (int32_t)ceilf(remaining_cm * ENCODER_COUNTS_PER_CM);
    if (revised_minimum < fu2_rev_min_counts)
        fu2_rev_min_counts = revised_minimum;
    fu2_rev_clear_samples = fu2_rev_reference_cm >= fu2_rev_clear_cm
        ? FU2_REVERSE_CLEAR_CONFIRM_SAMPLES : 0U;
    fu2_rev_reacquire_samples = 0U;
    char message[48];
    int n = snprintf(message, sizeof(message), "FU_US,REV_REBASE,%lu,%.1f\n",
        (unsigned long)fu2_rev_reference_cm,
        (double)((float)(fu2_rev_min_counts - reverse_travel_counts) / ENCODER_COUNTS_PER_CM));
    if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
    return true;
}

static bool fu2_reverse_is_clear(void)
{
    // Snapshot the ISR-owned fields together. Never count a cached echo twice.
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const uint32_t seq = us_capture_seq;
    const uint32_t sample_tick = us_capture_tick;
    const uint32_t range_cm = us_raw_cm;
    const float sample_heading = us_raw_heading;
    const bool valid = us_capture_valid != 0U;
    __set_PRIMASK(irq_state);
    const uint32_t now = HAL_GetTick();
    const float travelled_cm = (float)reverse_travel_counts / ENCODER_COUNTS_PER_CM;
    const bool fresh = valid &&
        (uint32_t)(now - sample_tick) <= FU2_EARLY_US_SAMPLE_MAX_AGE_MS &&
        fabsf(sample_heading - target_angle) <= FU2_EARLY_US_SAMPLE_MAX_ERROR_DEG;
    const float expected_max_cm = (float)fu2_rev_reference_cm +
        fmaxf(0.0f, (float)(reverse_travel_counts - fu2_rev_reference_counts) / ENCODER_COUNTS_PER_CM) +
        FU2_REVERSE_RANGE_TOLERANCE_CM;
    bool usable = fresh && (float)range_cm <= expected_max_cm;
    const bool can_reacquire = fresh && !usable && fu2_rev_waiting_echo &&
        fabsf(sample_heading - target_angle) <= FU2_REVERSE_REACQUIRE_MAX_ERROR_DEG &&
        fabsf(gyro_yaw_rate_dps) <= FU2_REVERSE_REACQUIRE_MAX_YAW_DPS;
    if (!can_reacquire) fu2_rev_reacquire_samples = 0U;
    const bool new_sample = seq != fu2_rev_sample_seq;
    if (new_sample)
    {
        fu2_rev_sample_seq = seq;
        if (can_reacquire) usable = fu2_reverse_check_range(range_cm);
        if (usable)
        {
            fu2_rev_last_usable_tick = now;
            fu2_rev_last_range_cm = range_cm;
        }
    }
    if (!usable)
    {
        if (!fu2_rev_waiting_echo)
        {
            char message[64];
            int n = snprintf(message, sizeof(message), "FU_US,REV_HOLD,%lu,%.1f,%.1f\n",
                (unsigned long)range_cm, (double)travelled_cm,
                (double)(sample_heading - target_angle));
            if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
            n = snprintf(message, sizeof(message), "FU_US,REV_CHECK,%.1f,%lu,%u\n",
                (double)expected_max_cm, (unsigned long)(now - sample_tick), (unsigned)valid);
            if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
        }
        fu2_rev_waiting_echo = 1U;
        fu2_rev_clear_samples = 0U;
        brake_wheels();
    }
    else fu2_rev_waiting_echo = 0U;

    if ((uint32_t)(now - fu2_rev_last_usable_tick) >= FU2_REVERSE_US_STALE_MS)
    {
        brake_wheels();
        Telemetry_RecordFault("FU2_US_STALE");
        script_fault_reply = "ERR:FU2_US_STALE\r\n";
        abort_now = 1U;
        return false;
    }
    if (!usable || reverse_travel_counts < fu2_rev_min_counts)
    {
        fu2_rev_clear_samples = 0U;
    }
    else if (new_sample)
    {
        if (range_cm >= fu2_rev_clear_cm)
        {
            if (fu2_rev_clear_samples < FU2_REVERSE_CLEAR_CONFIRM_SAMPLES)
                ++fu2_rev_clear_samples;
        }
        else fu2_rev_clear_samples = 0U;
    }
    if (fu2_rev_clear_samples >= FU2_REVERSE_CLEAR_CONFIRM_SAMPLES) return true;
    if (usable && reverse_travel_counts >= fu2_rev_min_counts &&
        range_cm >= fu2_rev_clear_cm)
    {
        // Clearance is already indicated. Confirm it at rest instead of
        // continuing to back toward obstacle 1 for another sonar period.
        brake_wheels();
        fu2_rev_waiting_echo = 1U;
        if (new_sample)
        {
            char message[48];
            int n = snprintf(message, sizeof(message), "FU_US,REV_CONFIRM,%lu,%.1f\n",
                (unsigned long)range_cm, (double)travelled_cm);
            if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
        }
    }
    // Stop rather than backing indefinitely toward obstacle 1 if the front
    // face is lost. This bound derives from entry range, not obstacle spacing.
    if (reverse_travel_counts >= fu2_rev_max_counts && !fu2_rev_waiting_echo)
    {
        brake_wheels();
        Telemetry_RecordFault("FU2_REVERSE_LIMIT");
        script_fault_reply = "ERR:FU2_REVERSE_LIMIT\r\n";
        abort_now = 1U;
    }
    return false;
}

/* Start reverse at its calibrated centre; subsequent ticks use gyro heading
 * correction. A centred request alone did not maintain a straight reverse. */
static void center_fu_reverse(void)
{
    const int center = fu_sequence_no == 2U
        ? reverse_center() : (int)steer_center_ccr;
    htim12.Instance->CCR2 = (uint32_t)center;
    if (fu_sequence_no == 2U)
    {
        char message[48];
        int n = snprintf(message, sizeof(message), "REV_HEADING,2,%d\n", center);
        if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
    }
}

void motorTask(void const * argument)
{
	/* USER CODE BEGIN motorTask */

	steering_ready = 0U;

	// PWM channels not active yet, simply prepares registers
	coast_left();
	coast_right();

	// MOTORA Start PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // PB8 -> IN2
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // PB9 -> IN1

	// MOTORB Start PWM
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); //
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); //

	brake_wheels(); // actively brake to reduce movement

	osDelay(STEERING_HOME_BRAKE_SETTLE_MS); // delay

	// decides which homing direction to use
	#if STEERING_DIAG_HOME_FROM_LEFT
	    htim12.Instance->CCR2 = STEER_HOME_LEFT_CCR;
	#else
	    htim12.Instance->CCR2 = STEER_HOME_RIGHT_CCR;
	#endif

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	steering_home();

	while (!gyro_healthy)
	{
		osDelay(20);
	}

	brake_wheels();

	Manual_Init(&joy_control);
    queue_range_reply("SMOOTH_BOOT,50\n");

	char command_line[sizeof(waiting_line)];

	for (;;)
	{
	check_uart_receiver();
	if (!abort_now && !joy_uart_owned)
	{
	  check_script_status();
	  send_queued_reply();
	}
	while (rx_script_read != rx_script_write && !abort_now && !joy_uart_owned)
	{
          // The ISR cannot reuse this slot until head advances after the copy.
          memcpy(command_line, rx_script_lines[rx_script_read], sizeof(command_line));
          rx_script_read = (uint8_t)((rx_script_read + 1U) % RX_SCRIPT_SLOTS);

	  command_line[sizeof(command_line) - 1U] = '\0';
	  for (size_t index = 0U; command_line[index] != '\0'; ++index)
	  {
		if (command_line[index] >= 'A' && command_line[index] <= 'Z')
		  command_line[index] = (char)(command_line[index] - 'A' + 'a');
		if (command_line[index] == ';' || command_line[index] == ',')
		  command_line[index] = ' ';
	  }

	  queue_script_line(command_line);
	  if (uart_cmd == DRIVE_IDLE && !abort_now)
		(void)run_next_step();
	}

	/* Every motion mode depends on valid heading feedback. Never continue a
	 * command with a failed or stale gyro. */
	if (uart_cmd != DRIVE_IDLE && uart_cmd != DRIVE_STOP)
	{
	  const uint32_t now = HAL_GetTick();
	  const bool stale = gyro_healthy &&
						 ((uint32_t)(now - gyro_last_good_tick) >
						  IMU_STALE_TIMEOUT_MS);

	  if (!gyro_healthy || stale)
	  {
		Telemetry_RecordFault(stale ? "GYRO_STALE" : "GYRO_NOT_READY");
		abort_now = 1U;
	  }
	}


	// emergency-stop
	if (abort_now)
	{

		const int interrupted_command = (int)uart_cmd;
		joy_leave_mode();

		abort_now = 0;
		obstacle_stop_mode = STOP_MODE_NONE;
		odom_counts_run = 0;
		odom_cm_run = 0.0f;
		odom_counts_run_ir = 0;
		odom_cm_run_ir = 0;
		slide_mode = SLIDE_NO_SIDE;
		slide_phase = SLIDE_IDLE;

		brake_wheels();
		next_start_tick = 0;

		// luther centering
		if (command_is_turn(interrupted_command))
		{
			recenter_after_turn(interrupted_command);
		}
		else
		{
			steer_center();
		}

		// Reset distance and PID state before restarting.
		target_counts = 0;

		// clear outstanding script
		queue_clear();
        rx_script_read = rx_script_write;
        fu_arrow_confirmed = 0U;
        fu_sequence_no = 0U;
        fu2_rev_reacquire_samples = 0U;
        route_axis_valid = route_entry_hold_pending = 0U;
        paired_slide_heading_valid = 0U;
        park_turn_profile = moving_turn_pending = moving_turn_active = turn_odom_active = 0U;
        parking_90_started = 0U;
        park_fx_active = park_fx_has_range = park_fx_range_confirmed = park_fx_confirm_samples = 0U;
		clear_replies();
		script_ack_waiting = 0U;

		uart_cmd = DRIVE_IDLE;
		turn_motor_enable_tick = 0;
		arc_postforward_cm = 0;
		stop_arc_control();


		/* luther
		 * Send the reset acknowledgement if UART3 is available.
		 * static keeps the buffer valid during interrupt transmission.
		 */

        joy_queue_reply(script_fault_reply != NULL ? script_fault_reply : "RST\r\n");
        script_fault_reply = NULL;
		joy_send_reply();

		/*const char rst_message[] = "RST\r\n";

		if (!reply_tx_busy)
		{
			reply_tx_busy = 1;

			if (HAL_UART_Transmit_IT(
					&huart3,
					(uint8_t *)rst_message,
					sizeof(rst_message) - 1) != HAL_OK)
			{
				//Transmission did not start, so release the busy flag.
				reply_tx_busy = 0;
			}
		}*/

		osDelay(5);

		/* Skip normal motor processing and restart the motor loop. */
		continue;

	}

	slew_output_active = 0U;
	if (joy_poll())
	{
		osDelay(5);
		continue;
	}

	slew_output_active = !joy_control.owns_motors;
	switch (uart_cmd)
	{
	case DRIVE_FORWARD:
	case DRIVE_REVERSE:
	{
      const bool ir_hold = ir_motion_active && uart_cmd == DRIVE_FORWARD && !joy_control.owns_motors;
      if (slide_mode != SLIDE_NO_SIDE)
      {
          const uint32_t elapsed = HAL_GetTick() - slide_stage_start_tick;
          const bool expired = elapsed >= SLIDE_PHASE_TIMEOUT_MS;
          const bool yaw_outside = fabsf(total_angle - (float)slide_origin_heading) >
              slide_active_angle_deg + SLIDE_MAX_YAW_MARGIN_DEG;
          if (expired || yaw_outside)
          {
              const slide_side_t fault_side = slide_mode;
              const slide_stage_t fault_phase = slide_phase;
              const float fault_heading = total_angle;
              const float fault_rate = gyro_yaw_rate_dps;
              const uint32_t fault_servo = htim12.Instance->CCR2;
              brake_wheels();
              Telemetry_RecordFault(expired ? "SLIDE_TIMEOUT" : "SLIDE_YAW_LIMIT");
              snprintf(slide_fault_reply, sizeof(slide_fault_reply), "ERR:%s,%c,%s,%.1f,%.1f,%.1f,%lu\r\n",
                  expired ? "SLIDE_TIMEOUT" : "SLIDE_YAW_LIMIT",
                  fault_side == SLIDE_TO_RIGHT ? 'R' : 'L',
                  fault_phase == SLIDE_OUTWARD ? "OUT" :
                      fault_phase == SLIDE_CLEARANCE ? "CLEARANCE" : "IN",
                  (double)fault_heading, slide_origin_heading, (double)fault_rate,
                  (unsigned long)fault_servo);
              script_fault_reply = slide_fault_reply;
              abort_now = 1U;
              break;
          }
      }
	  // Encoder-speed balance gains.
	  const float Kp = 0.250f;
	  const float Ki = 0.010f;
	  const float Kd = 0.010f;

	  // Controller state persists between motor ticks.
	  static int32_t i_acc = 0;
	  static int32_t prev_err = 0;
	  static float cpsA_f = 0.0f;
	  static float cpsB_f = 0.0f;
	  static float steer_i_acc_deg_s = 0.0f;   // heading-hold PID integral accumulator (deg*s)
	  static float steer_last_percent = 0.0f;  // previous commanded correction %, for slew-rate limiting
	  static bool  steer_hold_latched = false;   // true once decel has started and the servo trim is latched
	  static float steer_hold_percent = 0.0f;    // latched servo correction (integral trim) held through decel
	  static float steer_d_rate_f = 0.0f;

	  /*
	   * Straight drive profile
	   *
	   * 0%   -> essentially zero motor drive
	   * 100% -> calibrated PWM_CRUISE motor drive
	   */
	  static float drive_profile_percent = 0.0f;
	  static uint32_t drive_profile_last_tick = 0U;

	  /* Used only for temporary launch compensation. */
	  static uint32_t drive_profile_start_tick = 0U;

	  // Terminal anti-stall state. increased gradually until movement resumes.
	  static float terminal_floor_L = 0.0f;
	  static float terminal_floor_R = 0.0f;

	  static uint32_t terminal_last_progress_tick = 0U;
	  static int32_t terminal_last_progress_counts = 0;

	  /* 32-bit segment distance accumulator. The hardware encoder counters are
	   * only 16-bit, so subtracting the command-start counter directly fails
	   * once a move exceeds about 32767 encoder counts (roughly 4.4 m here).
	   * Small wrap-safe deltas are accumulated into int32_t instead. */
	  static uint16_t dist_last_a = 0U;
	  static uint16_t dist_last_b = 0U;
	  static int32_t dist_accum_a = 0;
	  static int32_t dist_accum_b = 0;
	  static int dist_direction = 0;
	  static uint8_t dist_initialized = 0U;

	  uint16_t now_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
	  uint16_t now_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
	  const int commanded_direction = (uart_cmd == DRIVE_REVERSE) ? -1 : 1;
	  uint32_t profile_now = HAL_GetTick();

	  if (!dist_initialized || seg_rebase || dist_direction != commanded_direction)
	  {
		  dist_last_a = now_a;
		  dist_last_b = now_b;

		  dist_accum_a = 0;
		  dist_accum_b = 0;

		  dist_direction = commanded_direction;
		  dist_initialized = 1U;

		  /*
		   * Starting a new straight
		   * begin from zero drive instead of immediately applying PWM_CRUISE.
		   */
		  drive_profile_percent = 0.0f;
		  drive_profile_last_tick = profile_now;
          const bool already_moving =
              (float)commanded_direction * slew_left_effort > 1.0f &&
              (float)commanded_direction * slew_right_effort > 1.0f;
		  drive_profile_start_tick = already_moving
              ? profile_now - LINE_ACCEL_RAMP_MS : profile_now;

		  // New command = reset terminal anti-stall state.
		  terminal_floor_L = commanded_direction < 0
		      ? REVERSE_TERMINAL_EFFORT_L : FORWARD_TERMINAL_EFFORT_L;
		  terminal_floor_R = commanded_direction < 0
		      ? REVERSE_TERMINAL_EFFORT_R : FORWARD_TERMINAL_EFFORT_R;
		  terminal_last_progress_tick = profile_now;
		  terminal_last_progress_counts = 0;

	  }
	  else
	  {
		dist_accum_a += (int16_t)(now_a - dist_last_a);
		dist_accum_b += -(int16_t)(now_b - dist_last_b);
		dist_last_a = now_a;
		dist_last_b = now_b;
	  }

	  uint32_t profile_dt_ms = profile_now - drive_profile_last_tick;

	  drive_profile_last_tick = profile_now;

	  /* Prevent an unusual task delay from making one giant ramp step. luther */
	  if (profile_dt_ms > 50U)
	  {
		  profile_dt_ms = 50U;
	  }

	  int32_t avg_moved = (dist_accum_a + dist_accum_b) / 2;

	  // Terminal anti-stall progress detector
	  int32_t terminal_progress_delta =
	      avg_moved - terminal_last_progress_counts;

	  if (terminal_progress_delta >= LINE_STALL_PROGRESS_COUNTS ||
	      terminal_progress_delta <= -LINE_STALL_PROGRESS_COUNTS)
	  {
	      // The drivetrain is still physically moving.
	      terminal_last_progress_counts = avg_moved;
	      terminal_last_progress_tick = profile_now;
	  }

	  // Remaining distance in encoder counts. Keep it positive regardless of FWD/REV direction. luther
	  int32_t remaining_counts;

      if (target_counts == INT32_MAX || target_counts == (INT32_MIN + 1))
          remaining_counts = INT32_MAX;
      else
      {
          const int64_t remaining = commanded_direction > 0
              ? (int64_t)target_counts - avg_moved
              : (int64_t)avg_moved - target_counts;
          remaining_counts = remaining <= 0 ? 0
              : remaining > INT32_MAX ? INT32_MAX : (int32_t)remaining;
      }

	  // ====== Stop conditions ======
      if (!fu2_range_ready())
      {
          i_acc = 0;
          prev_err = 0;
          osDelay(5);
          continue;
      }
      choose_fu2_approach();

	  // ====== Exact encoder stop condition ======
	  bool done_by_counts = false;

      if (target_counts == INT32_MAX || target_counts == (INT32_MIN + 1))
          done_by_counts = false;
	  else if (target_counts >= 0)
	  {
	      done_by_counts = (avg_moved >= target_counts);
	  }
	  else
	  {
	      done_by_counts = (avg_moved <= target_counts);
	  }

	  bool done_by_obst = false;
	  if (uart_cmd == DRIVE_FORWARD && obstacle_stop_mode != STOP_MODE_NONE)
	  {
        const uint32_t us_range_cm = fu2_current_range();
		switch (obstacle_stop_mode)
		{

		case STOP_MODE_US_T:
		  // Stop condition for FUxx: ultrasound <= per-move threshold
          if (park_fx_active)
          {
              if (!update_park_fx_range()) break;
              done_by_obst = park_fx_has_range && park_fx_range_confirmed &&
                  (uint32_t)(HAL_GetTick() - park_fx_last_valid_tick) <= PARKING_FX_SAMPLE_MAX_AGE_MS &&
                  park_fx_estimated_range_cm() <= (float)us_stop_cm + PARKING_FX_STOP_MARGIN_CM;
              if (done_by_obst)
              {
                  // Stop immediately; do not slew down or wait for the median.
                  brake_wheels();
                  park_fx_range_cm = (uint32_t)lroundf(park_fx_estimated_range_cm());
                  char message[48];
                  int n = snprintf(message, sizeof(message), "PARK_US,BRAKE,%lu,%u\n",
                      (unsigned long)park_fx_range_cm,
                      (unsigned)(us_stop_cm + PARKING_FX_STOP_MARGIN_CM));
                  if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
              }
          }
          else done_by_obst = (us_range_cm <= us_stop_cm);

		  // If extremely close, reverse only if allowed (FU), otherwise just stop (FX)
		  if (us_range_cm <= (fu_sequence_no == 2U ? FU2_REVERSE_TRIGGER_CM : OBST_REV_CM) && us_allow_reverse)
		  {

			{
		  char us_msg[32];
			  int n = snprintf(us_msg, sizeof(us_msg), "us0.0,%d\n", (int)us_range_cm);
		  if (n > 0 && n < (int)sizeof(us_msg))
		    queue_range_reply(us_msg);
		}

			// Keep heading straight; hand over to reverse controller
            // Retain FU's captured heading through automatic reverse.
            reverse_travel_counts = 0;
            start_fu2_reverse(us_range_cm);
			center_fu_reverse();

			obstacle_stop_mode = STOP_MODE_US_BACK; // Reverse until clearance is reached.
			target_counts = INT32_MIN + 1;          // sentinel: counts won’t finish it

			// Clean PID handover
			i_acc = 0;
			prev_err = 0;

			done_by_obst = false; // we’re not finishing here; switching to REVERSE
			seg_rebase = 1;
			uart_cmd = DRIVE_REVERSE;
            char reverse_msg[32];
            snprintf(reverse_msg, sizeof(reverse_msg), "REVERSE,%u\n", (unsigned)fu_sequence_no);
            queue_range_reply(reverse_msg);
            if (FU2_REVERSE_CAP_ENABLE && fu_sequence_no == 2U)
            {
                char speed_msg[48];
                int speed_n = snprintf(speed_msg, sizeof(speed_msg), "REV_SPEED,2,%.0f,%.0f\n",
                    (double)FU2_REVERSE_MAX_EFFORT_L, (double)FU2_REVERSE_MAX_EFFORT_R);
                if (speed_n > 0 && speed_n < (int)sizeof(speed_msg)) queue_range_reply(speed_msg);
            }
		  }
		  break;

		case STOP_MODE_US:
		  // ultrasound: stop when we are closer than threshold
		  done_by_obst = (us_range_cm <= OBST_STOP_CM);
		  // reverse

		  if (us_range_cm <= OBST_REV_CM)
		  { // reverse bump

			{
		  char us_msg[32];
			  int n = snprintf(us_msg, sizeof(us_msg), "us0.0,%d\n",
							   (int)us_range_cm);
		  if (n > 0 && n < (int)sizeof(us_msg))
		    queue_range_reply(us_msg);
		}

			// Keep heading straight
            // Retain FU's captured heading through automatic reverse.
            reverse_travel_counts = 0;
            start_fu2_reverse(us_range_cm);
			center_fu_reverse();

			// --- switch to reverse-until-clear mode ---
			obstacle_stop_mode = STOP_MODE_US_BACK;
			target_counts = INT32_MIN + 1; // sentinel so counts never end us

			// Clean PID handover
			i_acc = 0;
			prev_err = 0;

			done_by_obst = false;
			seg_rebase = 1; // <--- rebase on entry
			uart_cmd = DRIVE_REVERSE;
            char reverse_msg[32];
            snprintf(reverse_msg, sizeof(reverse_msg), "REVERSE,%u\n", (unsigned)fu_sequence_no);
            queue_range_reply(reverse_msg);
            if (FU2_REVERSE_CAP_ENABLE && fu_sequence_no == 2U)
            {
                char speed_msg[48];
                int speed_n = snprintf(speed_msg, sizeof(speed_msg), "REV_SPEED,2,%.0f,%.0f\n",
                    (double)FU2_REVERSE_MAX_EFFORT_L, (double)FU2_REVERSE_MAX_EFFORT_R);
                if (speed_n > 0 && speed_n < (int)sizeof(speed_msg)) queue_range_reply(speed_msg);
            }
		  }
		  break;
		case STOP_MODE_IR_RIGHT:
		  done_by_obst = (ir1_obs == 0);
		  break;
		case STOP_MODE_IR_LEFT:
		  done_by_obst = (ir0_obs == 0);
		  break;

		case STOP_MODE_IR_RIGHT_O: // NEW firo (stop when obstacle IS present)
		  done_by_obst = (ir1_obs == 1);
		  break;

		case STOP_MODE_IR_LEFT_O: // NEW filo (stop when obstacle IS present)
		  done_by_obst = (ir0_obs == 1);
		  break;
		default:
		  break;
		}
	  }

      if (abort_now) break;
	  if (uart_cmd == DRIVE_REVERSE && obstacle_stop_mode == STOP_MODE_US_BACK)
	  {
        // Obstacle 2 must finish its full clearance reverse even when vision
        // confirms early. A queued turn starts immediately after clearance.
        // The existing obstacle-1 slide release remains separate.
        done_by_obst = fu_sequence_no == 2U ? fu2_reverse_is_clear() :
            ((fu_sequence_no == 1U && fu_arrow_confirmed) || (echo_debug >= US_BACK_CLEAR_CM));
        if (abort_now) continue;
        if (fu_sequence_no == 2U && fu2_rev_waiting_echo)
        {
            i_acc = 0;
            prev_err = 0;
            osDelay(5);
            continue;
        }
	  }

	  // transmit US distance travelled and detected
	  if (done_by_obst && (obstacle_stop_mode == STOP_MODE_US || obstacle_stop_mode == STOP_MODE_US_T))
	  {
		odom_cm_run = (float)odom_counts_run / ENCODER_COUNTS_PER_CM;

		{
		  char us_msg[32];
		  int n = snprintf(us_msg, sizeof(us_msg), "us%.1f,%d\n",
						   (double)odom_cm_run,
                           (int)(park_fx_active ? park_fx_range_cm :
                               fu_sequence_no == 2U && us_allow_reverse
                                   ? fu2_fwd_range_cm : echo_debug));
		  if (n > 0 && n < (int)sizeof(us_msg))
		    queue_range_reply(us_msg);
		}
	  }

      if (done_by_obst && uart_cmd == DRIVE_REVERSE && obstacle_stop_mode == STOP_MODE_US_BACK)
      {
          char reverse_msg[48];
          int n = snprintf(reverse_msg, sizeof(reverse_msg), "REV_DONE,%u,%lu,%.1f\n",
              (unsigned)fu_sequence_no, (unsigned long)(fu_sequence_no == 2U
                  ? fu2_rev_last_range_cm : echo_debug),
              (double)((float)reverse_travel_counts / ENCODER_COUNTS_PER_CM));
          if (n > 0 && n < (int)sizeof(reverse_msg)) queue_range_reply(reverse_msg);
      }

	  // Copy the senior-style distance response before the local buffer expires.
	  if (done_by_obst && (obstacle_stop_mode == STOP_MODE_IR_RIGHT || obstacle_stop_mode == STOP_MODE_IR_LEFT))
	  {
	      odom_cm_run_ir = (float)odom_counts_run_ir / ENCODER_COUNTS_PER_CM;
	      char ir_msg[32];
	      int n = snprintf(ir_msg, sizeof(ir_msg), "ir%.1f\n", (double)odom_cm_run_ir);
	      if (n > 0 && n < (int)sizeof(ir_msg))
	          queue_range_reply(ir_msg);
          char edge_msg[48];
          const bool right_edge = obstacle_stop_mode == STOP_MODE_IR_RIGHT;
          int edge_n = snprintf(edge_msg, sizeof(edge_msg), "IR_EDGE,%c,%u,%.1f,%.1f\n",
              right_edge ? 'R' : 'L', (unsigned)(right_edge ? ir1_mV : ir0_mV),
              (double)total_angle, (double)gyro_yaw_rate_dps);
          if (edge_n > 0 && edge_n < (int)sizeof(edge_msg)) queue_range_reply(edge_msg);
	  }

	  if (done_by_counts || done_by_obst)
	  {
          if (done_by_counts && park_turn_profile && parking_90_started == 0U &&
              uart_cmd == DRIVE_FORWARD && obstacle_stop_mode == STOP_MODE_NONE &&
              target_counts > 0 && target_counts != INT32_MAX)
          {
              // Separate the commanded straight from the neighbouring arcs.
              char return_msg[48];
              int n = snprintf(return_msg, sizeof(return_msg),
                  "TURN_CFG,RETURN_DONE,%.1f,%.1f,%lu\n",
                  (double)((float)target_counts / ENCODER_COUNTS_PER_CM),
                  (double)((float)avg_moved / ENCODER_COUNTS_PER_CM),
                  (unsigned long)HAL_GetTick());
              if (n > 0 && n < (int)sizeof(return_msg)) queue_range_reply(return_msg);
          }
          const bool fu2_forward_turn_entry = uart_cmd == DRIVE_FORWARD &&
              us_allow_reverse && fu_sequence_no == 2U &&
              obstacle_stop_mode == STOP_MODE_US_T;
          moving_turn_pending = uart_cmd == DRIVE_FORWARD &&
              step_is_forward_turn(peek_motion_kind()) &&
              obstacle_stop_mode == STOP_MODE_NONE && target_counts > 0 &&
              target_counts != INT32_MAX;
          if (peek_motion_direction() == 0) brake_wheels();

	      straight_left_pwm = PWM_OFF;
	      straight_right_pwm = PWM_OFF;

	      // Keep the active correction through a rolling IR-to-IR handoff.
          const bool keep_ir_feedback = ir_motion_active && step_uses_ir(peek_motion_kind());
	      if (!keep_ir_feedback) steer_center();

	      straight_servo_ccr = steer_center_ccr;
	      straight_servo_center_ccr = steer_center_ccr;
	      straight_steer_percent = 0.0f;
	      straight_steer_p_percent = 0.0f;
	      straight_steer_i_percent = 0.0f;
	      straight_steer_d_percent = 0.0f;
	      straight_steer_correction_percent = 0.0f;

	      straight_log_deadline =
	          HAL_GetTick() + LINE_POST_STOP_TELEMETRY_MS;

	      obstacle_stop_mode = STOP_MODE_NONE;

	      if (!keep_ir_feedback)
          {
              i_acc = 0;
              prev_err = 0;
          }

          fu_arrow_confirmed = 0U;
	      park_fx_active = park_fx_has_range = park_fx_range_confirmed = park_fx_confirm_samples = 0U;
	      finish_slewed_motion();
          if (fu2_forward_turn_entry && !abort_now && !ARC_SERVO_ONLY_TEST &&
              (int32_t)(HAL_GetTick() - turn_motor_enable_tick) >= 0 && TURN_DEG == 90.0f &&
              (uart_cmd == DRIVE_FL || uart_cmd == DRIVE_FR))
          {
              // Prime the ordinary 90's wheel requests in this same cycle.
              // Keep output slew, normal turn launch and relative yaw target.
              const float ratio = uart_cmd == DRIVE_FR
                  ? ARC_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_L;
              apply_turn_drive((int)uart_cmd,
                  effort_to_pwm(FU2_ENTRY90_OUTER_EFFORT_MAX),
                  effort_to_pwm(FU2_ENTRY90_OUTER_EFFORT_MAX * ratio));
              queue_range_reply("TURN_CFG,FU2_FORWARD_ENTRY\n");
          }

	      break;
	  }

	  // ====== Wheel speeds (counts per second) @ ~10 ms ======
	  static uint16_t last_a = 0, last_b = 0;
	  static uint32_t last_t = 0;
	  uint32_t t = HAL_GetTick();

	  if (last_t == 0)
	  {
		last_t = t;
		last_a = now_a;
		last_b = now_b;
	  }

	  // If a new segment just started, rebase baselines and skip this tick’s accumulation
	  if (seg_rebase)
	  {
		last_a = now_a;
		last_b = now_b;
		last_t = t;
		seg_rebase = 0;
        // FIRO/FILO may finish immediately, before its first feedback tick.
        // Keep the handoff pending until a real rear-straight tick initializes.
        if (!ir_feedback_keep || (ir_hold && rear_unwind_seed_pending))
        {
		cpsA_f = 0.0f;
		cpsB_f = 0.0f;


		// optional: also clear PID transients to avoid a kick
		i_acc = 0;
		prev_err = 0;
		steer_i_acc_deg_s = 0.0f;
		steer_last_percent = 0.0f;
		steer_hold_latched = false;
		steer_hold_percent = 0.0f;
		// FU after the slides retains real rotation; zero would delay damping.
        steer_d_rate_f = (ir_hold || fu_slide_heading_active || route_entry_hold_pending || park_fx_active)
            ? gyro_yaw_rate_dps : 0.0f;

		straight_steer_p_percent = 0.0f;
		straight_steer_i_percent = 0.0f;
		straight_steer_d_percent = 0.0f;
		straight_steer_correction_percent = 0.0f;

        }
        ir_feedback_keep = 0U;

		// Do NOT compute da/db or update odometers this tick
	  }

	  int16_t da = (int16_t)(now_a - last_a);
	  int16_t db = (int16_t)(now_b - last_b);
	  db = -db; // keep right reversed consistently

	  // record distance travelled during US/FIR/FIL movement
	  if (uart_cmd == DRIVE_FORWARD && (obstacle_stop_mode == STOP_MODE_US || obstacle_stop_mode == STOP_MODE_US_T))
	  {
		int32_t inc_counts = ((int32_t)da + (int32_t)db) / 2;
		odom_counts_run += inc_counts;
	  }

	  if (uart_cmd == DRIVE_FORWARD && (obstacle_stop_mode == STOP_MODE_IR_RIGHT || obstacle_stop_mode == STOP_MODE_IR_LEFT))
	  {
		int32_t inc_counts = ((int32_t)da + (int32_t)db) / 2;
		odom_counts_run_ir += inc_counts;
	  }

      if (uart_cmd == DRIVE_REVERSE && obstacle_stop_mode == STOP_MODE_US_BACK)
      {
          const int32_t backwards = -((int32_t)da + (int32_t)db) / 2;
          if (backwards > 0) reverse_travel_counts += backwards;
      }

	  uint32_t dt_ms = (t - last_t);
	  if (dt_ms == 0)
		dt_ms = 1;

	  int32_t cpsA = (int32_t)da * 1000 / (int32_t)dt_ms;
	  int32_t cpsB = (int32_t)db * 1000 / (int32_t)dt_ms;

	  // ---- Low-pass filter for PID feedback ----
	  float dt_s = (float)dt_ms / 1000.0f;
	  float alpha = dt_s / (SPEED_FILTER_TAU + dt_s);

	  cpsA_f += alpha * ((float)cpsA - cpsA_f);
	  cpsB_f += alpha * ((float)cpsB - cpsB_f);

	  last_a = now_a;
	  last_b = now_b;
	  last_t = t;

	  // Balance the filtered encoder speeds; the servo handles heading below.
	  int32_t speed_err = (int32_t)(cpsA_f - cpsB_f); // filtered values

	  // Calculate heading error BEFORE using it for gyro feedback
	  error_angle = target_angle - total_angle;

	  /* Wheel PI keeps the encoder speeds balanced. The servo handles heading correction
	   * below. Feeding yaw into both loops makes them fight each other. */
	  int32_t err = speed_err;

      const bool ir_wheel_entry = ir_hold &&
          ((uint32_t)(t - ir_hold_start_tick) < IR_HOLD_WHEEL_ENTRY_MS ||
           fabsf(gyro_yaw_rate_dps) > IR_HOLD_WHEEL_ENTRY_MAX_DPS);
      if (ir_wheel_entry)
      {
          // Curved wheel speeds after the arc are not a straight-line bias.
          i_acc = 0;
          prev_err = err;
      }
      else i_acc += err;
	  const int32_t IACC_CLAMP = 40000; // Limit integral windup.
	  if (i_acc > IACC_CLAMP)
		i_acc = IACC_CLAMP;
	  if (i_acc < -IACC_CLAMP)
		i_acc = -IACC_CLAMP;

	  int32_t d = err - prev_err;
	  prev_err = err;

	  float off_f = Kp * (float)err + Ki * (float)i_acc + Kd * (float)d;
		#if WHEEL_PI_DIAG_DISABLE
		  int off = 0;              // suppressed; off_f/err/i_acc still logged above
		#else
		  int off = (int)off_f;
		#endif
      if (ir_hold)
      {
          if (ir_wheel_entry) off = 0;
          if (off > IR_HOLD_WHEEL_CORR_MAX) off = IR_HOLD_WHEEL_CORR_MAX;
          if (off < -IR_HOLD_WHEEL_CORR_MAX) off = -IR_HOLD_WHEEL_CORR_MAX;
      }



	  /*
	   * Select the same calibrated full-speed compare used by scripted F/R.
	   * In manual straight mode, throttle only interpolates from brake/zero
	   * drive (PWM_OFF) to that already-calibrated full-speed point. At 100%
	   * the values are exactly the same as a normal scripted F/R command.
	   */
	  //start new acceleration luther
	  const bool is_reverse = (uart_cmd == DRIVE_REVERSE);

	  const bool manual_straight =
		  joy_control.owns_motors &&
		  joy_motion == JOY_STRAIGHT;
      const bool fu2_photo_reverse = !manual_straight &&
          is_reverse && obstacle_stop_mode == STOP_MODE_US_BACK && fu_sequence_no == 2U;
      const bool fu2_reverse_limited = FU2_REVERSE_CAP_ENABLE && fu2_photo_reverse;
      const bool fu2_forward_approach = !manual_straight && !is_reverse &&
          slide_mode == SLIDE_NO_SIDE && us_allow_reverse &&
          obstacle_stop_mode == STOP_MODE_US_T && fu_sequence_no == 2U;
      const bool fu2_heading_hold = fu2_forward_approach && fu_slide_heading_active;
      const bool parking_fx_heading_hold = park_fx_active && !manual_straight && !is_reverse;
      bool flow_entry_limited = !manual_straight && route_entry_limiting();
      if (flow_entry_limited)
      {
          // fu2_range_ready() has validated this loop's raw sample.
          // Require both the far-window threshold and room for FU decel.
          const bool far_fu2_launch = fu2_heading_hold &&
              (float)fu2_current_range() >= fmaxf((float)FU2_FAR_LAUNCH_MIN_RANGE_CM,
                  (float)us_stop_cm + FU2_APPROACH_DECEL_CM);
          const float launch_error_limit = far_fu2_launch
              ? fmaxf(ROUTE_ENTRY_MAX_ERROR_DEG, FU2_FAR_LAUNCH_MAX_ERROR_DEG)
              : ROUTE_ENTRY_MAX_ERROR_DEG;
          const bool stable = fabsf(target_angle - total_angle) <= launch_error_limit &&
              fabsf(gyro_yaw_rate_dps) <= ROUTE_ENTRY_MAX_YAW_DPS;
          if (!stable) route_entry_stable_tick = 0U;
          else if (route_entry_stable_tick == 0U) route_entry_stable_tick = profile_now;
          if (stable && (uint32_t)(profile_now - route_entry_stable_tick) >= ROUTE_ENTRY_STABLE_MS)
          {
              route_entry_hold_pending = 0U;
              flow_entry_limited = false;
              if (far_fu2_launch)
              {
                  char launch_msg[80];
                  int n = snprintf(launch_msg, sizeof(launch_msg),
                      "FLOW_ENTRY_READY,FU2_FAR,%.1f,%.1f,%.1f,%lu\n",
                      (double)launch_error_limit, (double)(target_angle - total_angle),
                      (double)gyro_yaw_rate_dps, (unsigned long)fu2_current_range());
                  if (n > 0 && n < (int)sizeof(launch_msg)) queue_range_reply(launch_msg);
              }
              else queue_range_reply("FLOW_ENTRY_READY\n");
          }
      }

	  /*
	   * Requested level before applying the scripted acceleration/deceleration
	   * profile.
	   */
	  float requested_percent = manual_straight
	      ? (float)joy_throttle_percent : ir_hold ? IR_HOLD_DRIVE_PERCENT : 100.0f;
	  requested_percent = fminf(100.0f, fmaxf(0.0f, requested_percent));

	  // Planned slowdown applies only to finite scripted F/R distance moves.
	  const bool finite_scripted_move =
	      !manual_straight &&
	      obstacle_stop_mode == STOP_MODE_NONE &&
	      target_counts != INT32_MAX &&
	      target_counts != (INT32_MIN + 1);

	  const float left_compare_scale =
	      is_reverse ? REVERSE_LEFT_COMPARE_SCALE : FORWARD_LEFT_COMPARE_SCALE;
	  const float right_compare_scale =
	      is_reverse ? REVERSE_RIGHT_COMPARE_SCALE : FORWARD_RIGHT_COMPARE_SCALE;
	  const int full_base_L = limit_pwm(
	      (int)((float)PWM_CRUISE * left_compare_scale + 0.5f));
	  const int full_base_R = limit_pwm(
	      (int)((float)PWM_CRUISE * right_compare_scale + 0.5f));
      // Only the first finite forward move after PARK is the long return.
      // The counter advances at its first90, excluding later F/R and FX22.
      const bool parking_return_cruise = finite_scripted_move && !is_reverse &&
          park_turn_profile && parking_90_started == 0U;
      const float cruise_scale = parking_return_cruise
          ? fmaxf(1.0f, PARK_RETURN_CRUISE_EFFORT_SCALE) : 1.0f;
	  const float full_effort_L = fminf(PWM_OFF, (float)(PWM_OFF - full_base_L) * cruise_scale);
	  const float full_effort_R = fminf(PWM_OFF, (float)(PWM_OFF - full_base_R) * cruise_scale);
	  const float decel_effort_L = is_reverse
	      ? REVERSE_DECEL_EFFORT_L : FORWARD_DECEL_EFFORT_L;
	  const float decel_effort_R = is_reverse
	      ? REVERSE_DECEL_EFFORT_R : FORWARD_DECEL_EFFORT_R;

	  float effort_L = full_effort_L * requested_percent * 0.01f;
	  float effort_R = full_effort_R * requested_percent * 0.01f;
	  bool decel_active = false;

      const bool flowing_same_direction = peek_motion_direction() == (is_reverse ? -1 : 1);
      const bool rolling_turn_next = !is_reverse && step_is_forward_turn(peek_motion_kind());
      const bool parking_carry_into_turn = PARK_TURN_CARRY_SPEED_ENABLE &&
          park_turn_profile && finite_scripted_move && !is_reverse && rolling_turn_next &&
          !(parking_return_cruise && PARK_RETURN_SLOWDOWN_ENABLE);
      const float rolling_entry_l = park_turn_profile
          ? PARK_TURN_ENTRY_EFFORT_L : ROLLING_TURN_ENTRY_EFFORT_L;
      const float rolling_entry_r = park_turn_profile
          ? PARK_TURN_ENTRY_EFFORT_R : ROLLING_TURN_ENTRY_EFFORT_R;
      if (fu2_forward_approach)
      {
          const float range_remaining_cm = fmaxf(0.0f,
              (float)fu2_current_range() - (float)us_stop_cm);
          const float blend = FU2_APPROACH_DECEL_CM > 0.0f
              ? fminf(1.0f, range_remaining_cm / FU2_APPROACH_DECEL_CM) : 1.0f;
          const float entry_l = fminf(full_effort_L,
              fmaxf(0.0f, FU2_APPROACH_ENTRY_EFFORT_L));
          const float entry_r = fminf(full_effort_R,
              fmaxf(0.0f, FU2_APPROACH_ENTRY_EFFORT_R));
          effort_L = entry_l + (full_effort_L - entry_l) * blend;
          effort_R = entry_r + (full_effort_R - entry_r) * blend;
          decel_active = blend < 1.0f;
      }
      else if (park_fx_active && !manual_straight && !is_reverse)
      {
          park_fx_effort_limits(&effort_L, &effort_R);
      }
	  else if (finite_scripted_move && (!flowing_same_direction || rolling_turn_next) &&
          !parking_carry_into_turn)
	  {
	      const float remaining_cm = (float)remaining_counts / ENCODER_COUNTS_PER_CM;
          const float decel_distance_cm = parking_return_cruise
              ? fmaxf(LINE_DECEL_CM, PARK_RETURN_DECEL_CM) : LINE_DECEL_CM;
	      decel_active = remaining_cm < decel_distance_cm;

	      if (!rolling_turn_next && decel_active && remaining_cm <= LINE_TERMINAL_CM &&
	          (uint32_t)(profile_now - terminal_last_progress_tick) >=
	              LINE_STALL_DETECT_MS)
	      {
	          terminal_floor_L = straight_boost_terminal(
	              terminal_floor_L, decel_effort_L, full_effort_L);
	          terminal_floor_R = straight_boost_terminal(
	              terminal_floor_R, decel_effort_R, full_effort_R);
	          terminal_last_progress_tick = profile_now;
	      }

	      const float target_effort_L = straight_slowdown_target(
	          remaining_cm, full_effort_L,
              rolling_turn_next ? rolling_entry_l : decel_effort_L,
              rolling_turn_next ? rolling_entry_l : terminal_floor_L, decel_distance_cm);
	      const float target_effort_R = straight_slowdown_target(
	          remaining_cm, full_effort_R,
              rolling_turn_next ? rolling_entry_r : decel_effort_R,
              rolling_turn_next ? rolling_entry_r : terminal_floor_R, decel_distance_cm);

	      // Keep the calibrated quadratic launch ramp. A short move cannot
	      // jump to a slowdown target above its current acceleration limit.
	      const uint32_t accel_elapsed_ms = profile_now - drive_profile_start_tick;
	      float accel_fraction = LINE_ACCEL_RAMP_MS > 0U
	          ? (float)accel_elapsed_ms / (float)LINE_ACCEL_RAMP_MS : 1.0f;
	      accel_fraction = fminf(1.0f, fmaxf(0.0f, accel_fraction));
	      const float accel_fraction_squared = accel_fraction * accel_fraction;
          if (PARK_FORWARD_USE_OUTPUT_RAMP && park_turn_profile && rolling_turn_next)
          {
              /* Parking forward legs use the existing motor-output slew.
               * Avoid restarting the zero-effort launch profile after the
               * moving home-facing turn; retain the near-corner slowdown. */
              effort_L = target_effort_L;
              effort_R = target_effort_R;
          }
          else
          {
              effort_L = fminf(target_effort_L,
                  full_effort_L * accel_fraction_squared);
              effort_R = fminf(target_effort_R,
                  full_effort_R * accel_fraction_squared);
          }
	  }

      if (fu2_reverse_limited)
      {
          effort_L = fminf(effort_L, fmaxf(0.0f, FU2_REVERSE_MAX_EFFORT_L));
          effort_R = fminf(effort_R, fmaxf(0.0f, FU2_REVERSE_MAX_EFFORT_R));
      }
      if (flow_entry_limited)
      {
          effort_L = fminf(effort_L, fmaxf(0.0f, route_entry_effort_l));
          effort_R = fminf(effort_R, fmaxf(0.0f, route_entry_effort_r));
      }
      if (section2_rear_carry_active && ir_hold && !manual_straight && !is_reverse)
      {
          effort_L = fminf(effort_L, section2_rear_carry_effort_l);
          effort_R = fminf(effort_R, section2_rear_carry_effort_r);
      }
      if (section2_entry_carry_active && ir_hold && !manual_straight && !is_reverse)
      {
          effort_L = fminf(effort_L, section2_entry_carry_effort_l);
          effort_R = fminf(effort_R, section2_entry_carry_effort_r);
      }
      bool straight_exit_limited = (SECTION2_EXIT_CAP_ENABLE || section2_rear_carry_active || section2_entry_carry_active) && section2_exit_straight_limited &&
          !manual_straight && !is_reverse && slide_mode == SLIDE_NO_SIDE;
      if (straight_exit_limited)
      {
          const bool stable = fabsf(gyro_yaw_rate_dps) <= SECTION2_EXIT_STABLE_YAW_DPS &&
              fabsf(target_angle - total_angle) <= SECTION2_EXIT_STABLE_ERROR_DEG;
          if (!stable) section2_exit_stable_tick = 0U;
          else if (section2_exit_stable_tick == 0U) section2_exit_stable_tick = profile_now;
          if (!section2_rear_carry_active && !section2_entry_carry_active &&
              (uint32_t)(profile_now - section2_exit_start_tick) >= SECTION2_EXIT_MIN_HOLD_MS &&
              section2_exit_stable_tick != 0U &&
              (uint32_t)(profile_now - section2_exit_stable_tick) >= SECTION2_EXIT_STABLE_MS)
          {
              section2_exit_straight_limited = 0U;
              straight_exit_limited = false;
              queue_range_reply("EXIT_CAP,RELEASE\n");
          }
          else
          {
              effort_L = fminf(effort_L, ir_leave_effort_ceiling_l());
              effort_R = fminf(effort_R, ir_leave_effort_ceiling_r());
          }
      }

	  // Percent is now only a derived value for wheel-PI scaling, steering
	  // gating and telemetry. The slowdown endpoints are absolute PWM effort.
	  const float left_profile_percent = full_effort_L > 0.0f
	      ? 100.0f * effort_L / full_effort_L : 0.0f;
	  const float right_profile_percent = full_effort_R > 0.0f
	      ? 100.0f * effort_R / full_effort_R : 0.0f;
	  drive_profile_percent = fminf(100.0f, fmaxf(0.0f,
	      fminf(left_profile_percent, right_profile_percent)));
	  const int throttle_percent = (int)(drive_profile_percent + 0.5f);

	  int base_L = limit_pwm(PWM_OFF - (int)lroundf(effort_L));
	  int base_R = limit_pwm(PWM_OFF - (int)lroundf(effort_R));
	  if (!finite_scripted_move && !straight_exit_limited && !fu2_reverse_limited &&
          !fu2_forward_approach && !flow_entry_limited && !park_fx_active)
	  {
	      // Preserve the existing integer PWM mapping for manual/sensor moves.
	      const int requested_throttle = (int)(requested_percent + 0.5f);
	      base_L = PWM_OFF -
	          ((PWM_OFF - full_base_L) * requested_throttle) / 100;
	      base_R = PWM_OFF -
	          ((PWM_OFF - full_base_R) * requested_throttle) / 100;
	  }

	  // Forward launch-only left motor trim, Apply only to normal finite scripted FORWARD moves. temporarily weakens the left motor.
	  /*
		* Launch trim for forward and reverse.
		*
		* Applies only to finite scripted straight movements.
		* Trim is a percentage of CURRENT left-wheel motor effort,
		* not a fixed PWM compare offset.
		*
		* Positive percentage = weaken left motor.
		* Negative percentage = strengthen left motor.
		*/
		if (finite_scripted_move &&
			drive_profile_start_tick != 0U)
		{
			const uint32_t launch_trim_ms =
				is_reverse
					? REVERSE_LAUNCH_TRIM_MS
					: FORWARD_LAUNCH_TRIM_MS;

			const float launch_trim_percent =
				is_reverse
					? REVERSE_LAUNCH_LEFT_EFFORT_TRIM_PERCENT
					: FORWARD_LAUNCH_LEFT_EFFORT_TRIM_PERCENT;

			const uint32_t launch_elapsed_ms =
				profile_now - drive_profile_start_tick;

			if (launch_trim_ms > 0U &&
				launch_elapsed_ms < launch_trim_ms)
			{
				/*
				* 1.0 at launch start,
				* gradually falls to 0.0.
				*/
				float launch_fraction =
					1.0f -
					((float)launch_elapsed_ms /
					(float)launch_trim_ms);

				if (launch_fraction < 0.0f)
					launch_fraction = 0.0f;

				if (launch_fraction > 1.0f)
					launch_fraction = 1.0f;

				/*
				* PWM is inverted:
				*
				* PWM_OFF = no drive
				* smaller compare = more drive
				*
				* Therefore positive motor effort is:
				*/
				const float left_effort =
					(float)(PWM_OFF - base_L);

				/*
				* Work out how much effort to remove/add.
				*/
				const float trim_compare_f =
					left_effort *
					(launch_trim_percent * 0.01f) *
					launch_fraction;

				const int trim_compare =
					(int)lroundf(trim_compare_f);

				/*
				* Positive trim raises PWM compare -> LESS drive.
				* Negative trim lowers PWM compare -> MORE drive.
				*/
				base_L =
					limit_pwm(
						base_L + trim_compare
					);
			}
		}

	  // Scale wheel-speed PI correction with the actual drive profile.
	  if (throttle_percent < 100)
		  off = (off * throttle_percent) / 100;

	  int lDuty;
	  int rDuty;

	  if (!is_reverse)
	  {
		  /*
		   * Larger compare = less power.
		   * Positive off slows left and speeds up right.
		   */
		  lDuty = base_L + off;
		  rDuty = base_R - off;
	  }
	  else
	  {
		  /*
		   * Encoder speeds are negative in reverse, so the correction
		   * direction must be inverted.
		   */
		  lDuty = base_L - off;
		  rDuty = base_R + off;
	  }

	  lDuty = limit_pwm(lDuty);
	  rDuty = limit_pwm(rDuty);
      if (straight_exit_limited)
      {
          const int cap_compare_l = effort_to_pwm(
              ir_leave_effort_ceiling_l());
          const int cap_compare_r = effort_to_pwm(
              ir_leave_effort_ceiling_r());
          if (lDuty < cap_compare_l) lDuty = cap_compare_l;
          if (rDuty < cap_compare_r) rDuty = cap_compare_r;
      }
      if (fu2_reverse_limited)
      {
          const int cap_compare_l = effort_to_pwm(
              fmaxf(0.0f, FU2_REVERSE_MAX_EFFORT_L));
          const int cap_compare_r = effort_to_pwm(
              fmaxf(0.0f, FU2_REVERSE_MAX_EFFORT_R));
          if (lDuty < cap_compare_l) lDuty = cap_compare_l;
          if (rDuty < cap_compare_r) rDuty = cap_compare_r;
      }
      if (flow_entry_limited)
      {
          const int cap_compare_l = effort_to_pwm(route_entry_effort_l);
          const int cap_compare_r = effort_to_pwm(route_entry_effort_r);
          if (lDuty < cap_compare_l) lDuty = cap_compare_l;
          if (rDuty < cap_compare_r) rDuty = cap_compare_r;
      }

	  /* Save actual straight motor controller outputs for telemetry. */
	  straight_left_pwm = lDuty;
	  straight_right_pwm = rDuty;

	  // Hold the heading captured at the start of THIS straight segment.
	  error_angle = target_angle - total_angle;

	  /* =========================================================
	   * Gyro correction for straight travel
	   * ========================================================= */

	  float steering_error = error_angle;

	  /* Ignore tiny gyro noise. */
	  if (fabsf(steering_error) < (ir_hold ? IR_HOLD_DEADBAND_DEG : LINE_STEER_DEADBAND_DEG))
	  {
		  steering_error = 0.0f;
	  }

	  /*
	   * Heading-hold PID, computed as a percentage of available
	   * steering travel rather than raw CCR counts.
	   *
	   * P alone cannot fully cancel a persistent yaw bias (uneven
	   * friction, motor mismatch beyond what the wheel PI corrects,
	   * mechanical misalignment): a P-only loop settles at whatever
	   * non-zero error produces the output needed to hold yaw rate at
	   * zero, so the robot drives straight but at a small constant
	   * angle offset from the intended heading. The integral term
	   * below removes that residual offset over time.
	   *
	   * The derivative term uses the gyro's own measured yaw RATE
	   * rather than differencing the (noisier) integrated angle:
	   * d(error)/dt = -d(total_angle)/dt = -gyro_yaw_rate_dps.
	   */
		#if LINE_STEER_I_GATING_ENABLE

		const bool steer_i_enabled =
			!decel_active &&
			drive_profile_percent >= LINE_STEER_I_MIN_DRIVE_PERCENT &&
			fabsf(steering_error) < LINE_STEER_I_ERR_BAND_DEG;

		#else

		/* Allow the integrator to learn throughout the whole straight movement. */
		const bool steer_i_enabled = true;

		#endif

	  /*
	   * Choose the steering gains for forward or reverse travel.
	   * Both directions use the same PID calculation below.
	   */
      const float steer_kp = ir_hold ? IR_HOLD_KP_PERCENT_PER_DEG
          : parking_fx_heading_hold ? PARKING_FX_HOLD_KP_PERCENT_PER_DEG
          : fu2_heading_hold ? FU2_HOLD_KP_PERCENT_PER_DEG
          : is_reverse ? REVERSE_STEER_KP_PERCENT_PER_DEG : FORWARD_STEER_KP_PERCENT_PER_DEG;
      const float steer_ki = ir_hold ? IR_HOLD_KI_PERCENT_PER_DEG_S
          : parking_fx_heading_hold ? PARKING_FX_HOLD_KI_PERCENT_PER_DEG_S
          : fu2_heading_hold ? FU2_HOLD_KI_PERCENT_PER_DEG_S
          : is_reverse ? REVERSE_STEER_KI_PERCENT_PER_DEG_S : FORWARD_STEER_KI_PERCENT_PER_DEG_S;
      const float steer_kd = ir_hold ? IR_HOLD_KD_PERCENT_PER_DPS
          : parking_fx_heading_hold ? PARKING_FX_HOLD_KD_PERCENT_PER_DPS
          : fu2_heading_hold ? FU2_HOLD_KD_PERCENT_PER_DPS
          : is_reverse ? REVERSE_STEER_KD_PERCENT_PER_DPS : FORWARD_STEER_KD_PERCENT_PER_DPS;

	  /*
	   * Integral term and windup limit.
	   *
	   * Ki may intentionally be set to zero while tuning.
	   * Do not divide by Ki when it is disabled.
	   */
	  if (steer_ki > 0.0f)
	  {
	      if (steer_i_enabled)
	      {
	          steer_i_acc_deg_s += steering_error * dt_s;
	      }

	      const float i_limit_deg_s =
	          LINE_STEER_I_LIMIT_PERCENT /
	          steer_ki;

	      if (steer_i_acc_deg_s > i_limit_deg_s)
	      {
	          steer_i_acc_deg_s = i_limit_deg_s;
	      }
	      else if (steer_i_acc_deg_s < -i_limit_deg_s)
	      {
	          steer_i_acc_deg_s = -i_limit_deg_s;
	      }
	  }
	  else
	  {
	      /*
	       * Ki is disabled.
	       * Keep the accumulator clean so enabling Ki later
	       * cannot suddenly apply an old accumulated error.
	       */
	      steer_i_acc_deg_s = 0.0f;
	  }

	  const float steer_p_term = steer_kp * steering_error;
	  const float steer_i_term = steer_ki * steer_i_acc_deg_s;
	  /*
	   * Low-pass filter the gyro yaw rate used by the derivative term.
	   *
	   * This keeps real chassis rotation but rejects much of the
	   * high-frequency gyro/vibration noise that D would otherwise amplify.
	   */
	  const float d_alpha =
	      dt_s / ((ir_hold ? IR_HOLD_D_LPF_TAU_S : parking_fx_heading_hold
              ? PARKING_FX_HOLD_D_LPF_TAU_S : fu2_heading_hold
              ? FU2_HOLD_D_LPF_TAU_S : LINE_STEER_D_LPF_TAU_S) + dt_s);

	  steer_d_rate_f +=
	      d_alpha * (gyro_yaw_rate_dps - steer_d_rate_f);

      const bool rear_unwind = ir_hold && section2_rear_carry_active && rear_unwind_active;
      const float rear_predicted_yaw = rear_unwind
          ? fminf(SECTION2_REAR_UNWIND_MAX_DEG,
              fmaxf(-SECTION2_REAR_UNWIND_MAX_DEG,
                  SECTION2_REAR_UNWIND_LOOKAHEAD_S * steer_d_rate_f)) : 0.0f;
      // P(error - predicted yaw) + ordinary yaw damping. This does not
      // replace the intended heading with the early handoff heading.
      const float steer_d_term = -steer_kd * steer_d_rate_f
          - steer_kp * rear_predicted_yaw;

      if (rear_unwind)
      {
          if (fabsf(error_angle) <= SECTION2_EXIT_STABLE_ERROR_DEG &&
              fabsf(gyro_yaw_rate_dps) <= SECTION2_EXIT_STABLE_YAW_DPS)
          {
              if (rear_unwind_stable_tick == 0U) rear_unwind_stable_tick = t;
              if ((uint32_t)(t - rear_unwind_stable_tick) >= SECTION2_REAR_UNWIND_STABLE_MS)
              {
                  rear_unwind_active = 0U;
                  queue_range_reply("REAR_UNWIND,DONE\n");
              }
          }
          else rear_unwind_stable_tick = 0U;
      }

	  float correction_percent = steer_p_term + steer_i_term + steer_d_term;

#if LINE_DECEL_STEER_HOLD
	  /* Planned decel: stop reacting, hold the learned bias trim (I term) only. */
	  if (decel_active)
	  {
		  if (!steer_hold_latched)
		  {
			  steer_hold_latched = true;
			  steer_hold_percent = steer_i_term;
		  }
		  correction_percent = steer_hold_percent;
	  }
	  else
	  {
		  steer_hold_latched = false;
	  }
#endif

	  /*
	   * Once outside the deadband, make sure the servo moves enough
	   * to overcome linkage/servo dead travel.
	   */
	  if (!ir_hold && steering_error != 0.0f &&
		  fabsf(correction_percent) < LINE_STEER_MIN_PERCENT)
	  {
		  correction_percent =
			  (correction_percent > 0.0f)
				  ? LINE_STEER_MIN_PERCENT
				  : -LINE_STEER_MIN_PERCENT;
	  }

      const float correction_limit = ir_hold ? IR_HOLD_MAX_PERCENT : fu2_heading_hold
          ? FU2_HOLD_MAX_PERCENT : LINE_STEER_MAX_PERCENT;
      correction_percent = fminf(correction_limit, fmaxf(-correction_limit, correction_percent));
      if (rear_unwind && rear_unwind_seed_pending)
      {
          // Seed only this 180-to-IR transition, not slides/FU/parking or 90s.
          // Avoid spending the first ~130 ms slewing from stale/zero trim.
          steer_last_percent = correction_percent;
          rear_unwind_seed_pending = 0U;
          char unwind_msg[64];
          int n = snprintf(unwind_msg, sizeof(unwind_msg),
              "REAR_UNWIND,BEGIN,%.1f,%.1f,%.1f\n",
              (double)error_angle, (double)gyro_yaw_rate_dps,
              (double)correction_percent);
          if (n > 0 && n < (int)sizeof(unwind_msg)) queue_range_reply(unwind_msg);
      }

	  /*
	   * Slew-rate limit the commanded correction so the servo moves
	   * smoothly instead of jumping between values on every ~10 ms
	   * tick, which otherwise shows up as a wiggling/oscillating line.
	   */
	  {
        const bool releasing = correction_percent * steer_last_percent < 0.0f ||
            fabsf(correction_percent) < fabsf(steer_last_percent);
        const float slew = ir_hold
            ? (releasing ? IR_HOLD_RELEASE_PERCENT_PER_S : IR_HOLD_SLEW_PERCENT_PER_S)
            : fu2_heading_hold
                ? (releasing ? FU2_HOLD_RELEASE_PERCENT_PER_S : FU2_HOLD_SLEW_PERCENT_PER_S)
            : LINE_STEER_SLEW_PERCENT_PER_S;
		const float max_step_percent = slew * dt_s;

		  if (correction_percent > steer_last_percent + max_step_percent)
		  {
			  correction_percent = steer_last_percent + max_step_percent;
		  }
		  else if (correction_percent < steer_last_percent - max_step_percent)
		  {
			  correction_percent = steer_last_percent - max_step_percent;
		  }
	  }

	  steer_last_percent = correction_percent;

	  /*
	   * Save the applied PID values for the telemetry record.
	   *
	   * P/I/D are the raw PID contributions.
	   *
	   * correction_percent is the FINAL controller-domain request after:
	   * - decel hold
	   * - minimum output
	   * - output clamping
	   * - slew limiting
	   */
	  straight_steer_p_percent =
	      steer_p_term;

	  straight_steer_i_percent =
	      steer_i_term;

	  straight_steer_d_percent =
	      steer_d_term;

	  straight_steer_correction_percent =
	      correction_percent;

	  /*
	   * Map the yaw correction to left or right steering.
	   *
	   * Positive physical_steer_percent = RIGHT
	   * Negative physical_steer_percent = LEFT
	   *
	   * Reverse requires the steering correction direction to flip.
	   */
	  float physical_steer_percent;

	  if (uart_cmd == DRIVE_FORWARD)
	  {
		  physical_steer_percent = -correction_percent;
	  }
	  else
	  {
		  physical_steer_percent = correction_percent;
	  }

	  /* Use the calibrated centre for this direction of travel. */
	  int center = is_reverse ? reverse_center()
							  : (int)steer_center_ccr;

	  /*
	   * Use the appropriate steering endpoints for the direction
	   * of travel.
	   */
	  int left_limit;
	  int right_limit;

	  if (uart_cmd == DRIVE_REVERSE)
	  {
		  left_limit = STEER_REVERSE_LEFT_CCR;
		  right_limit = STEER_REVERSE_RIGHT_CCR;
	  }
	  else
	  {
		  left_limit = STEER_LEFT_CCR;
		  right_limit = STEER_RIGHT_CCR;
	  }

	  int servo = center;

	  /*
	   * Travel available on each side of centre. With the symmetric map
	   * enabled for this direction, both sides use the smaller span so that
	   * +x% and -x% produce the same CCR change from centre.
	   */
	  float right_span = (float)(right_limit - center);
	  float left_span  = (float)(center - left_limit);

	  if (ir_hold ? IR_HOLD_SYMMETRIC_MAP : is_reverse ? REVERSE_STEER_SYMMETRIC_MAP : FORWARD_STEER_SYMMETRIC_MAP)
	  {
		  const float sym_span = (right_span < left_span) ? right_span : left_span;
		  right_span = sym_span;
		  left_span  = sym_span;
	  }

	  if (physical_steer_percent > 0.0f)
	  {
		  /* RIGHT steering */
		  float available = right_span;

		  servo = center +
			  (int)lroundf(
				  available *
				  physical_steer_percent /
				  100.0f
			  );
	  }
	  else if (physical_steer_percent < 0.0f)
	  {
		  /* LEFT steering */
		  float available = left_span;

		  servo = center +
			  (int)lroundf(
				  available *
				  physical_steer_percent /
				  100.0f
			  );
	  }

	  /* BEGIN STRAIGHT FIXED-STEERING DIAGNOSTIC */
#if LINE_DIAG_FIXED_SERVO
	  if (finite_scripted_move && (!is_reverse || LINE_DIAG_FIXED_SERVO_REVERSE) && slide_mode == SLIDE_NO_SIDE)
	  {
		  servo = center + LINE_DIAG_OFFSET_CCR;
		  if (servo < left_limit) servo = left_limit;
		  if (servo > right_limit) servo = right_limit;

		  /* Report the fixed REQUEST, not the bypassed heading-controller
		   * request. This is not a measurement of front-wheel angle. */
		  const float span = (servo >= center)
			  ? (float)(right_limit - center)
			  : (float)(center - left_limit);
		  physical_steer_percent = (span > 0.0f)
			  ? 100.0f * (float)(servo - center) / span : 0.0f;
	  }
#endif
	  /* END STRAIGHT FIXED-STEERING DIAGNOSTIC */

      // FU2 uses the ordinary reverse heading PD again. Version 10's fixed
      // centre suppressed correction while heading drifted at about 10 deg/s.

	  /* Final mechanical safety clamp. */
	  if (servo < left_limit)
	  {
		  servo = left_limit;
	  }

	  if (servo > right_limit)
	  {
		  servo = right_limit;
	  }

	  /* Save controller state for telemetry. */
	  straight_servo_center_ccr = center;
	  straight_steer_percent = physical_steer_percent;
	  straight_servo_ccr = servo;
#if IR_HOLD_TRACE_MS > 0U
      if (ir_hold && (uint32_t)(t - ir_hold_trace_tick) >= IR_HOLD_TRACE_MS)
      {
          char hold_msg[48];
          int n = snprintf(hold_msg, sizeof(hold_msg), "IRH,%u,%.1f,%.1f,%.1f,%d\n",
              (unsigned)obstacle_stop_mode, (double)error_angle,
              (double)gyro_yaw_rate_dps, (double)correction_percent, servo);
          if (n > 0 && n < (int)sizeof(hold_msg)) queue_range_reply(hold_msg);
          // Capture both sensors together; these are ADC counts, filtered mV
          // and obstacle flags. This does not change edge decisions.
          const uint32_t irq_state = __get_PRIMASK();
          __disable_irq();
          const uint16_t raw_l = ir0_raw, raw_r = ir1_raw;
          const uint16_t mv_l = ir0_mV, mv_r = ir1_mV;
          const uint8_t obs_l = ir0_obs, obs_r = ir1_obs;
          __set_PRIMASK(irq_state);
          char sensor_msg[48];
          int sensor_n = snprintf(sensor_msg, sizeof(sensor_msg), "IRS,%u,%u,%u,%u,%u,%u,%u\n",
              (unsigned)obstacle_stop_mode, (unsigned)raw_l, (unsigned)mv_l,
              (unsigned)obs_l, (unsigned)raw_r, (unsigned)mv_r, (unsigned)obs_r);
          if (sensor_n > 0 && sensor_n < (int)sizeof(sensor_msg)) queue_range_reply(sensor_msg);
          ir_hold_trace_tick = t;
      }
#endif

      if (fu_slide_heading_active && slide_mode == SLIDE_NO_SIDE &&
          FU_HEADING_TRACE_MS > 0U && (fu_heading_trace_tick == 0U ||
          (uint32_t)(t - fu_heading_trace_tick) >= FU_HEADING_TRACE_MS))
      {
          char heading_msg[48];
          int n = snprintf(heading_msg, sizeof(heading_msg), "FUH,%c,%.1f,%.1f,%.1f,%d\n",
              is_reverse ? 'R' : 'F', (double)target_angle, (double)total_angle,
              (double)gyro_yaw_rate_dps, servo);
          if (n > 0 && n < (int)sizeof(heading_msg)) queue_range_reply(heading_msg);
          fu_heading_trace_tick = t;
      }
      if (route_axis_valid && !fu_slide_heading_active && uart_cmd == DRIVE_FORWARD &&
          slide_mode == SLIDE_NO_SIDE && obstacle_stop_mode == STOP_MODE_NONE &&
          (route_heading_trace_tick == 0U || (uint32_t)(t - route_heading_trace_tick) >= 250U))
      {
          char message[48];
          int n = snprintf(message, sizeof(message), "FLOWH,%.1f,%.1f,%.1f,%d\n",
              (double)target_angle, (double)total_angle, (double)gyro_yaw_rate_dps, servo);
          if (n > 0 && n < (int)sizeof(message)) queue_range_reply(message);
          route_heading_trace_tick = t;
      }
	  // === Slide state machine overrides servo while active ===
	  if (slide_mode != SLIDE_NO_SIDE)
	  {
        const float outward_sign = slide_mode == SLIDE_TO_RIGHT ? -1.0f : 1.0f;
        const float outward_remaining = outward_sign * (arc_target_angle - total_angle);
        const float outward_rate = fmaxf(0.0f, outward_sign * gyro_yaw_rate_dps);
        const float reversal_margin = fminf(SLIDE_OUT_REVERSAL_MAX_DEG,
            fmaxf(0.0f, SLIDE_OUT_REVERSAL_LOOKAHEAD_S) * outward_rate);
        // Reverse steering before the target to allow for servo/wheel response
        // and residual yaw, without reducing the slide drive request.
        if (slide_phase == SLIDE_OUTWARD && outward_remaining <= reversal_margin)
        {
            slide_phase = SLIDE_INWARD;
            fu2_slide_us_announced = 0U;
            slide_stage_start_tick = HAL_GetTick();
            char phase_msg[48];
            int n = snprintf(phase_msg, sizeof(phase_msg), "SLIDE,%c,IN,%.1f,%.1f\n",
                slide_mode == SLIDE_TO_RIGHT ? 'R' : 'L',
                (double)total_angle, slide_origin_heading);
            if (n > 0 && n < (int)sizeof(phase_msg)) queue_range_reply(phase_msg);
        }

        bool slide_complete = false;
        drive_slide_stage();
        if (slide_phase == SLIDE_INWARD)
        {
            arc_target_angle = slide_origin_heading;
            float release_margin = 5.0f;
            if (peek_motion_kind() == STEP_FU)
            {
                const float closing_rate = slide_mode == SLIDE_TO_RIGHT
                    ? gyro_yaw_rate_dps : -gyro_yaw_rate_dps;
                release_margin = fminf(SLIDE_FINAL_HANDOFF_MAX_DEG,
                    fmaxf(SLIDE_FINAL_HANDOFF_MIN_DEG,
                        0.5f + SLIDE_FINAL_HANDOFF_LEAD_S * fmaxf(0.0f, closing_rate)));
            }

            if ((slide_mode == SLIDE_TO_RIGHT && total_angle >= arc_target_angle - release_margin) ||
                (slide_mode == SLIDE_TO_LEFT && total_angle <= arc_target_angle + release_margin))
            {
                const step_kind_t opposite_slide = slide_mode == SLIDE_TO_RIGHT
                    ? STEP_SLIDE_LEFT : STEP_SLIDE_RIGHT;
                // Keep each queued slide intact. Move forward before beginning
                // an immediate opposite slide, so it does not return inward early.
                if (SLIDE_PAIR_CLEARANCE_CM > 0.0f && !queue_empty() &&
                    step_queue[queue_read].type == opposite_slide)
                {
                    slide_phase = SLIDE_CLEARANCE;
                    slide_stage_start_tick = HAL_GetTick();
                    slide_clearance_start_counts = avg_moved;
                    target_angle = (float)slide_origin_heading;
                    if (slide_mode == SLIDE_TO_RIGHT) center_after_left();
                    else center_after_right();
                    // The normal heading controller ran during the curved
                    // phases; discard its accumulated correction before reuse.
                    i_acc = prev_err = 0;
                    steer_i_acc_deg_s = steer_last_percent = 0.0f;
                    steer_hold_latched = false;
                    steer_hold_percent = 0.0f;
                    steer_d_rate_f = gyro_yaw_rate_dps;
                    servo = steer_center_ccr;
                    straight_servo_ccr = straight_servo_center_ccr = servo;
                    straight_steer_percent = 0.0f;
                    straight_steer_p_percent = straight_steer_i_percent = 0.0f;
                    straight_steer_d_percent = straight_steer_correction_percent = 0.0f;
                    char clearance_msg[48];
                    int n = snprintf(clearance_msg, sizeof(clearance_msg),
                        "SLIDE,CLEARANCE,BEGIN,%c,%.1f\n",
                        slide_mode == SLIDE_TO_RIGHT ? 'R' : 'L',
                        (double)SLIDE_PAIR_CLEARANCE_CM);
                    if (n > 0 && n < (int)sizeof(clearance_msg))
                        queue_range_reply(clearance_msg);
                }
                else slide_complete = true;
            }
        }

        if (slide_phase == SLIDE_CLEARANCE)
        {
            drive_slide_clearance(servo);
            const float clearance_cm =
                (float)(avg_moved - slide_clearance_start_counts) / ENCODER_COUNTS_PER_CM;
            if (clearance_cm >= fmaxf(0.0f, SLIDE_PAIR_CLEARANCE_CM))
            {
                char clearance_msg[48];
                int n = snprintf(clearance_msg, sizeof(clearance_msg),
                    "SLIDE,CLEARANCE,DONE,%c,%.1f\n",
                    slide_mode == SLIDE_TO_RIGHT ? 'R' : 'L', (double)clearance_cm);
                if (n > 0 && n < (int)sizeof(clearance_msg))
                    queue_range_reply(clearance_msg);
                slide_complete = true;
            }
        }

        if (slide_complete)
        {
            const slide_side_t completed_slide = slide_mode;
            char phase_msg[48];
            int n = snprintf(phase_msg, sizeof(phase_msg), "SLIDE,%c,DONE,%.1f,%.1f,%.1f\n",
                completed_slide == SLIDE_TO_RIGHT ? 'R' : 'L',
                (double)total_angle, slide_origin_heading, (double)gyro_yaw_rate_dps);
            if (n > 0 && n < (int)sizeof(phase_msg)) queue_range_reply(phase_msg);
            target_angle = (float)slide_origin_heading;
            arc_target_angle = slide_origin_heading;
            slide_mode = SLIDE_NO_SIDE;
            slide_phase = SLIDE_IDLE;
            slide_clearance_start_counts = 0;
            // IN bends opposite to OUT; use the centre for the last steering side.
            if (completed_slide == SLIDE_TO_RIGHT) center_after_left();
            else center_after_right();
            const step_kind_t slide_exit = peek_motion_kind();
            if (slide_exit == STEP_FU || slide_exit == STEP_FX ||
                slide_exit == STEP_FORWARD_CM || step_uses_ir(slide_exit))
                balance_turn_exit();
            if (peek_motion_direction() == 0) brake_wheels();
            obstacle_stop_mode = STOP_MODE_NONE;
            i_acc = prev_err = 0;
            finish_slewed_motion();
            // The next command owns its outputs; skip this slide's stale
            // straight-controller calculations for the rest of the tick.
            break;
        }
        else if (slide_phase != SLIDE_OUTWARD && slide_phase != SLIDE_INWARD &&
                 slide_phase != SLIDE_CLEARANCE)
        {
            htim12.Instance->CCR2 = servo;
        }
	  }
	  else
	  { // no servoslide mode
		// no slide: normal straight correction
		htim12.Instance->CCR2 = servo;
		// luther: straight steering uses the computed correction.

	  }


      if (uart_cmd == DRIVE_FORWARD && slide_mode == SLIDE_NO_SIDE)
      {
          drive_left_forward(lDuty);
          drive_right_forward(rDuty);
      }
      else if (uart_cmd == DRIVE_REVERSE)
      {
          drive_left_reverse(lDuty);
          drive_right_reverse(rDuty);
      }

	  /* Capture exactly the PI state used above. UART work stays in EncoderTask. */
	  if (slide_mode == SLIDE_NO_SIDE &&
	      (uart_cmd == DRIVE_FORWARD || uart_cmd == DRIVE_REVERSE))
	  {
	      Telemetry_PublishWheelPi(
	          t, dt_ms, cpsA_f, cpsB_f, speed_err,
	          i_acc, IACC_CLAMP, off, throttle_percent, lDuty, rDuty
	      );
	  }

	  break;
	}

	// luther turn start
	case DRIVE_FR:
	case DRIVE_FL:
	case DRIVE_RR:
	case DRIVE_RL:
	{

		const int command = (int)uart_cmd;
		const uint32_t now = HAL_GetTick();
        update_turn_odometry();
        const bool parking_forward_arc = park_turn_profile &&
            !command_is_reverse_turn(command);
        const bool controlled_entry = moving_turn_active || parking_forward_arc;
        float outer_effort_max = parking_forward_arc ? park_turn_effort_ceiling(now) : controlled_entry
            ? ROLLING_TURN_OUTER_EFFORT_MAX : ARC_OUTER_EFFORT_MAX;
        float outer_effort_min = parking_forward_arc ? PARK_TURN_OUTER_EFFORT_MIN : controlled_entry
            ? ROLLING_TURN_OUTER_EFFORT_MIN : ARC_OUTER_EFFORT_MIN;
        if (controlled_entry && !parking_forward_arc && ROLLING_TURN_ENTRY_BLEND_MS > 0U)
        {
            outer_effort_max = fminf(outer_effort_max, moving_turn_effort_ceiling(now));
        }
        if (rear_turn_direct_active)
        {
            outer_effort_max = fminf(ARC_OUTER_EFFORT_MAX,
                fmaxf(0.0f, REAR90_OUTER_EFFORT_MAX));
            outer_effort_min = fminf(outer_effort_max,
                fmaxf(0.0f, REAR90_OUTER_EFFORT_MIN));
        }
        if (rear_turn_ordinary_active)
        {
            outer_effort_max = fminf(outer_effort_max, rear_turn_effort_ceiling(now));
            outer_effort_min = fmaxf(outer_effort_min, SECTION2_REAR90_MIN_EFFORT);
        }
        if (fu2_entry90_active)
        {
            outer_effort_max = fminf(outer_effort_max, FU2_ENTRY90_OUTER_EFFORT_MAX);
            outer_effort_min = fmaxf(outer_effort_min, FU2_ENTRY90_OUTER_EFFORT_MIN);
        }
        outer_effort_min = fminf(outer_effort_min, outer_effort_max);

		#if ARC_SERVO_ONLY_TEST

			coast_left();
			coast_right();

			if (turn_motor_enable_tick == 0U)
			{
				steer_for_turn(command);

				turn_motor_enable_tick =
					now + ARC_SERVO_TEST_HOLD_MS;

				break;
			}

			if ((int32_t)(now - turn_motor_enable_tick) < 0)
			{
				coast_left();
				coast_right();
				break;
			}

			recenter_after_turn(command);

			coast_left();
			coast_right();

			stop_arc_control();

			target_angle = total_angle;
			turn_motor_enable_tick = 0U;
			arc_postforward_cm = 0U;

			uart_cmd = DRIVE_IDLE;
			next_start_tick = HAL_GetTick() + STEP_GAP_MS;

			break;

		#endif

		/* Safety: a lifted/stalled robot has no yaw feedback and must not run forever. */
		if (arc_deadline != 0U &&
		  (int32_t)(now - arc_deadline) >= 0)
		{
			brake_wheels();
			Telemetry_RecordFault("TURN_TIMEOUT");
			script_fault_reply = "ERR:TURN_TIMEOUT\r\n";
			abort_now = 1U;
			break;
		}

        if (wait_for_turn_steering(command, now)) break;

		/* Let the steering servo reach its requested angle before driving. */
		if (turn_motor_enable_tick != 0U &&
		  (int32_t)(now - turn_motor_enable_tick) < 0)
		{
			brake_wheels();
			turn_effort_cmd = 0.0f;
			turn_left_pwm = PWM_OFF;
			turn_right_pwm = PWM_OFF;
			break;
		}
		turn_motor_enable_tick = 0U;

		if (turn_control_phase == ARC_IDLE)
			reset_arc_control();

		const float direction = turn_yaw_sign(command);
		const float signed_error = arc_target_angle - total_angle;
		const float remaining = direction * signed_error;
		const float closing_rate = direction * gyro_yaw_rate_dps;
        if (parking_forward_arc && parking_90_started >= 2U &&
            remaining <= PARK_FINAL90_US_GUARD_WINDOW_DEG && !queue_empty() &&
            step_queue[queue_read].type == STEP_FX)
        {
            // The last arc can reach the wall before FX owns the motors.
            const uint32_t irq_state = __get_PRIMASK();
            __disable_irq();
            const uint32_t wall_cm = us_raw_cm, wall_tick = us_capture_tick;
            const bool wall_valid = us_capture_valid != 0U;
            __set_PRIMASK(irq_state);
            const uint32_t wall_stop_cm = (step_queue[queue_read].value > 0
                ? (uint32_t)step_queue[queue_read].value : OBST_STOP_CM) + PARKING_FX_STOP_MARGIN_CM;
            if (wall_valid && (uint32_t)(now - wall_tick) <= PARKING_FX_SAMPLE_MAX_AGE_MS &&
                wall_cm <= wall_stop_cm)
            {
                brake_wheels();
                char wall_msg[48];
                int n = snprintf(wall_msg, sizeof(wall_msg), "PARK_US,TURN_BRAKE,%lu,%.1f\n",
                    (unsigned long)wall_cm, (double)remaining);
                if (n > 0 && n < (int)sizeof(wall_msg)) queue_range_reply(wall_msg);
                if (fabsf(signed_error) <= PARK_FINAL90_NEAR_WALL_HANDOFF_DEG)
                    complete_turn(command);
                else
                {
                    Telemetry_RecordFault("PARK_WALL_TURN");
                    script_fault_reply = "ERR:PARK_WALL_TURN\r\n";
                    abort_now = 1U;
                }
                break;
            }
        }
        if (PARK_TURN_UNWIND_ENABLE && parking_forward_arc)
        {
            const bool first_parking90 = parking_90_started == 1U;
            const float window = fmaxf(1.0f, first_parking90
                ? PARK_FIRST90_UNWIND_START_DEG : PARK_TURN_UNWIND_START_DEG);
            const float minimum = fminf(1.0f, fmaxf(0.0f, first_parking90
                ? PARK_FIRST90_UNWIND_MIN_FRACTION : PARK_TURN_UNWIND_MIN_FRACTION));
            const float fraction = fminf(1.0f, fmaxf(minimum, remaining / window));
            park_turn_fraction = fminf(park_turn_fraction, fraction);
            if (!parking_unwind_announced && remaining <= window)
            {
                parking_unwind_announced = 1U;
                char unwind_msg[64];
                int n = snprintf(unwind_msg, sizeof(unwind_msg),
                    "TURN_UNWIND,PARK%u,%.1f,%.1f,%.1f\n",
                    (unsigned)parking_90_started, (double)window,
                    (double)remaining, (double)gyro_yaw_rate_dps);
                if (n > 0 && n < (int)sizeof(unwind_msg)) queue_range_reply(unwind_msg);
            }
        }
        if (FU2_ENTRY90_UNWIND_ENABLE && fu2_entry90_active)
        {
            const float window = fmaxf(1.0f, FU2_ENTRY90_UNWIND_START_DEG);
            const float minimum = fminf(1.0f, fmaxf(0.0f, FU2_ENTRY90_UNWIND_MIN_FRACTION));
            const float fraction = fminf(1.0f, fmaxf(minimum, remaining / window));
            entry90_turn_fraction = fminf(entry90_turn_fraction, fraction);
            if (!entry90_unwind_announced && remaining <= window)
            {
                entry90_unwind_announced = 1U;
                char unwind_msg[64];
                int n = snprintf(unwind_msg, sizeof(unwind_msg),
                    "TURN_UNWIND,ENTRY90,%.1f,%.1f,%.1f\n",
                    (double)window, (double)remaining, (double)gyro_yaw_rate_dps);
                if (n > 0 && n < (int)sizeof(unwind_msg)) queue_range_reply(unwind_msg);
            }
        }
        if (SECTION2_REAR90_UNWIND_ENABLE && rear_turn_ordinary_active)
        {
            const float window = fmaxf(1.0f, SECTION2_REAR90_UNWIND_START_DEG);
            const float minimum = fminf(1.0f, fmaxf(0.0f, SECTION2_REAR90_UNWIND_MIN_FRACTION));
            const float fraction = fminf(1.0f, fmaxf(minimum, remaining / window));
            rear_turn_turn_fraction = fminf(rear_turn_turn_fraction, fraction);
            if (!rear_turn_unwind_announced && remaining <= window)
            {
                rear_turn_unwind_announced = 1U;
                char unwind_msg[64];
                int n = snprintf(unwind_msg, sizeof(unwind_msg),
                    "TURN_UNWIND,REAR90,%.1f,%.1f,%.1f\n",
                    (double)window, (double)remaining, (double)gyro_yaw_rate_dps);
                if (n > 0 && n < (int)sizeof(unwind_msg)) queue_range_reply(unwind_msg);
            }
        }
        if (SECTION2_180_PROGRESSIVE_UNWIND_ENABLE && section2_rear_steady_turn_active)
        {
            const float window = fmaxf(1.0f, SECTION2_180_UNWIND_START_DEG);
            const float minimum = fminf(1.0f, fmaxf(0.0f, SECTION2_180_UNWIND_MIN_FRACTION));
            const float fraction = fminf(1.0f, fmaxf(minimum, remaining / window));
            // Do not wind steering back in if a noisy heading tick recedes.
            section2_180_turn_fraction = fminf(section2_180_turn_fraction, fraction);
            if (!section2_180_unwind_announced && remaining <= window)
            {
                section2_180_unwind_announced = 1U;
                char unwind_msg[64];
                int n = snprintf(unwind_msg, sizeof(unwind_msg),
                    "TURN_UNWIND,BEGIN,%.1f,%.1f,%.1f\n",
                    (double)window, (double)remaining, (double)gyro_yaw_rate_dps);
                if (n > 0 && n < (int)sizeof(unwind_msg)) queue_range_reply(unwind_msg);
            }
        }
        if (SECTION2_EXIT_CAP_ENABLE && !rear_turn_direct_active &&
            !controlled_entry && !section2_rear_steady_turn_active &&
            !command_is_reverse_turn(command) && peek_motion_direction() == 1 &&
            remaining <= SECTION2_EXIT_TURN_WINDOW_DEG)
        {
            if (!section2_exit_turn_limited) queue_range_reply("EXIT_CAP,TURN\n");
            section2_exit_turn_limited = 1U;
            outer_effort_max = fminf(outer_effort_max, section2_exit_turn_effort_ceiling());
            outer_effort_min = fminf(outer_effort_min, outer_effort_max);
        }

		turn_error_deg = signed_error;
        if (peek_motion_direction() != 0)
        {
          const float handoff_lookahead = section2_rear_steady_turn_active
              ? SECTION2_REAR_HANDOFF_LOOKAHEAD_S : SMOOTH_TURN_HANDOFF_LOOKAHEAD_S;
          const float predicted_margin = SMOOTH_TURN_HANDOFF_TOL_DEG +
              fmaxf(0.0f, handoff_lookahead) * fmaxf(0.0f, closing_rate);
          const float rear_handoff_margin = fminf(
              fmaxf(SMOOTH_TURN_HANDOFF_TOL_DEG, SECTION2_REAR_HANDOFF_MAX_DEG),
              predicted_margin);
          // At high yaw, the old0.12s lead released17-18 degrees early,
          // before steering could finish unwinding. An immediate IR edge
          // could then dispatch another90 while still finishing this arc.
          const float handoff_margin = section2_rear_steady_turn_active
              ? fminf(rear_handoff_margin,
                  fmaxf(SMOOTH_TURN_HANDOFF_TOL_DEG, SECTION2_180_EXIT_MAX_REMAIN_DEG))
              : predicted_margin;
          if (remaining <= handoff_margin)
          {
            complete_turn(command);
            break;
          }
          if (turn_control_phase == ARC_SETTLING)
          {
            turn_control_phase = ARC_RUNNING;
            arc_pid_tick = now;
            steer_for_turn(command);
          }
        }


		/*
		* After braking, wait for mechanical motion to stop before deciding
		* whether the command is complete or needs one gentle re-approach.
		*/
		if (turn_control_phase == ARC_SETTLING)
		{
			brake_wheels();
			turn_effort_cmd = 0.0f;
			turn_left_pwm = PWM_OFF;
			turn_right_pwm = PWM_OFF;

			if ((int32_t)(now - arc_settle_deadline) < 0)
				break;

			if (fabsf(signed_error) <= ARC_FINISH_TOL_DEG || remaining <= 0.0f || arc_attempts >= ARC_MAX_APPROACHES)
			{
				complete_turn(command);
				break;
			}

			/* Stopped short: restore steering and make a slower approach. */
			arc_attempts++;
			arc_integral_deg_s = 0.0f;
			arc_pid_tick = now;
			turn_control_phase = ARC_RUNNING;
			steer_for_turn(command);
			turn_motor_enable_tick = now + ARC_RETRY_SERVO_MS;
			break;
		}

		/*
		* Predict motion during sensing + braking delay. At 100 deg/s and
		* 45 ms look-ahead this begins braking about 4.5 degrees early.
		*/
		const float positive_closing_rate = (closing_rate > 0.0f) ? closing_rate : 0.0f;
		const float brake_margin = ARC_FINISH_TOL_DEG +
								 ARC_BRAKE_LOOKAHEAD_S * positive_closing_rate;

		if (peek_motion_direction() == 0 && remaining <= brake_margin)
		{
			brake_wheels();
			recenter_after_turn(command);
			turn_effort_cmd = 0.0f;
			turn_left_pwm = PWM_OFF;
			turn_right_pwm = PWM_OFF;
			turn_control_phase = ARC_SETTLING;
			arc_settle_deadline = now + ARC_SETTLE_MS;
			break;
		}

        if (ARC_PROGRESS_CHECK_ENABLE)
        {
            if (direction * (total_angle - turn_progress_heading) >= ARC_PROGRESS_MIN_DEG)
            {
                turn_progress_heading = total_angle;
                turn_progress_tick = now;
            }
            else if (remaining > ARC_PROGRESS_MIN_REMAIN_DEG &&
                (uint32_t)(now - turn_progress_tick) >= ARC_PROGRESS_TIMEOUT_MS)
            {
                brake_wheels();
                Telemetry_RecordFault("TURN_NO_YAW");
                script_fault_reply = "ERR:TURN_NO_YAW\r\n";
                abort_now = 1U;
                break;
            }
        }
		float dt_s = (float)(now - arc_pid_tick) * 0.001f;
		if (dt_s < 0.001f)
			dt_s = 0.001f;
		if (dt_s > 0.050f)
			dt_s = 0.050f;
		arc_pid_tick = now;

		float candidate_i = arc_integral_deg_s + remaining * dt_s;
		if (candidate_i > ARC_I_LIMIT_DEG_S)
			candidate_i = ARC_I_LIMIT_DEG_S;
		if (candidate_i < -ARC_I_LIMIT_DEG_S)
			candidate_i = -ARC_I_LIMIT_DEG_S;

		float effort = ARC_KP_EFFORT_PER_DEG * remaining +
					 ARC_KI_EFFORT_PER_DEG_S * candidate_i -
					 ARC_KD_EFFORT_PER_DPS * closing_rate;
        if (section2_rear_steady_turn_active)
        {
            /* Blend from carried entry effort to steady drive without a
             * low-effort restart. Remaining angle does not taper the arc;
             * gyro feedback still determines its handoff to the straight. */
            effort = section2_180_effort_ceiling(now);
            outer_effort_max = effort;
            outer_effort_min = fminf(outer_effort_min, effort);
        }
        else if (parking_forward_arc && PARK_TURN_CARRY_SPEED_ENABLE &&
            peek_motion_direction() == 1)
        {
            // Keep a shared drive budget through both PARK arcs. Steering
            // still unwinds and the gyro still determines the endpoint;
            // remaining angle no longer requests the old3200 drive tail.
            effort = park_turn_effort_ceiling(now);
            outer_effort_max = effort;
            outer_effort_min = fminf(outer_effort_min, effort);
        }

		/* Conditional integration prevents wind-up while output is saturated. */
		if (effort < outer_effort_max)
			arc_integral_deg_s = candidate_i;

		if (effort > outer_effort_max)
			effort = outer_effort_max;
		if (effort < outer_effort_min)
			effort = outer_effort_min;

		const bool parking_arc = PARK_TIGHT_ARC_ENABLE && park_turn_profile &&
            !command_is_reverse_turn(command);
        const float inner_ratio = turn_steers_right(command)
            ? (parking_arc ? PARK_INNER_EFFORT_RATIO_R : ARC_INNER_EFFORT_RATIO_R)
            : (parking_arc ? PARK_INNER_EFFORT_RATIO_L : ARC_INNER_EFFORT_RATIO_L);

        float outer_drive = effort;
        float inner_drive = effort * inner_ratio;
        if (section2_rear_steady_turn_active || (fu2_entry90_active && FU2_ENTRY90_UNWIND_ENABLE) ||
            (rear_turn_ordinary_active && SECTION2_REAR90_UNWIND_ENABLE) ||
            (parking_forward_arc && PARK_TURN_UNWIND_ENABLE))
        {
            float left, right;
            const bool right_turn = turn_steers_right(command);
            section2_turn_wheel_efforts(right_turn, effort, &left, &right);
            outer_drive = right_turn ? left : right;
            inner_drive = right_turn ? right : left;
        }
		const int outer_pwm = effort_to_pwm(outer_drive);
		const int inner_pwm = effort_to_pwm(inner_drive);

		turn_effort_cmd = effort;
        /* Keep turn steering authoritative on every drive tick, rather than
         * relying only on the one-time IR-to-turn setup write. */
        steer_for_turn(command);
		apply_turn_drive(command, outer_pwm, inner_pwm);
        if (ARC_TRACE_MS > 0U && (arc_trace_tick == 0U ||
            (uint32_t)(now - arc_trace_tick) >= ARC_TRACE_MS))
        {
            char control_msg[48], motor_msg[48];
            int control_n = snprintf(control_msg, sizeof(control_msg), "TDRV,%c,%.1f,%.1f,%lu,%.0f\n",
                turn_steers_right(command) ? 'R' : 'L', (double)remaining,
                (double)gyro_yaw_rate_dps, (unsigned long)htim12.Instance->CCR2, (double)effort);
            int motor_n = snprintf(motor_msg, sizeof(motor_msg), "TMOT,%d,%d,%.0f,%.0f\n",
                PWM_OFF - turn_left_pwm, PWM_OFF - turn_right_pwm,
                (double)slew_left_effort, (double)slew_right_effort);
            if (control_n > 0 && control_n < (int)sizeof(control_msg)) queue_range_reply(control_msg);
            if (motor_n > 0 && motor_n < (int)sizeof(motor_msg)) queue_range_reply(motor_msg);
            if (parking_forward_arc)
            {
                // Measured wheel travel distinguishes wheel stall from yaw loss.
                char wheel_msg[48];
                int wheel_n = snprintf(wheel_msg, sizeof(wheel_msg),
                    "TURN_CFG,PARK_WHEEL,%.1f,%.1f\n",
                    (double)((float)turn_wheel_counts_a / ENCODER_COUNTS_PER_CM),
                    (double)((float)turn_wheel_counts_b / ENCODER_COUNTS_PER_CM));
                if (wheel_n > 0 && wheel_n < (int)sizeof(wheel_msg))
                    queue_range_reply(wheel_msg);
            }
            // Register diagnostics show timer/pin configuration, not proof of
            // a physical pulse at the servo or of mechanical steering motion.
            char servo_msg[48];
            int servo_n = snprintf(servo_msg, sizeof(servo_msg), "SVPWM,%lu,%lu,%lu,%u,%u,%u,%u,%u\n",
                (unsigned long)htim12.Instance->CCR2,
                (unsigned long)htim12.Instance->PSC,
                (unsigned long)htim12.Instance->ARR,
                (unsigned)((htim12.Instance->CR1 & TIM_CR1_CEN) != 0U),
                (unsigned)((htim12.Instance->CCER & TIM_CCER_CC2E) != 0U),
                (unsigned)((GPIOB->MODER >> 30U) & 3U),
                (unsigned)((GPIOB->AFR[1] >> 28U) & 15U),
                (unsigned)((htim12.Instance->CCMR1 >> 12U) & 7U));
            if (servo_n > 0 && servo_n < (int)sizeof(servo_msg)) queue_range_reply(servo_msg);
            arc_trace_tick = now;
        }
		break;
	} // luther turn end

	case DRIVE_STOP:
		brake_wheels();
		uart_cmd = DRIVE_IDLE;
		next_start_tick = HAL_GetTick() + SLIDE_CMD_MS;

		break;

	case DRIVE_IDLE:
	default:
	{
		if (!queue_empty())
		{
			if (HAL_GetTick() >= next_start_tick)
			{
				if (!run_next_step())
					{
					// queue emptied right here -> ACK script finished
					}
			}
		}
		else
		{
			// idle with empty queue: make sure next_start_tick doesn’t block future scripts
			next_start_tick = 0;
		}
		break;
	}
	}
    if (uart_cmd == DRIVE_IDLE && (int32_t)(HAL_GetTick() - next_start_tick) >= 0)
    {
      for (uint8_t count = 0U; count < STEP_QUEUE_SIZE && uart_cmd == DRIVE_IDLE && !queue_empty(); ++count)
        if (!run_next_step()) break;
      if (uart_cmd == DRIVE_IDLE && queue_empty()) brake_wheels();
    }
    check_script_status();
    send_queued_reply();
    apply_slewed_drive();
	osDelay(5);
	}
	  /* USER CODE END motorTask */
}

/* USER CODE BEGIN Header_encoderTask */
/**
 * @brief Function implementing the Encoder_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_encoderTask */
void encoderTask(void const * argument)
{
  /* USER CODE BEGIN encoderTask */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Wheel encoder deltas a
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Wheel encoder deltas b

  uint16_t last_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); // get the last count of encoderA
  uint16_t last_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint32_t last_tick = HAL_GetTick(); // get last time

  for (;;)
  {
	  uint32_t sample_tick = HAL_GetTick();
	  if ((sample_tick - last_tick) >= 20U) // nominal 50 Hz
		  {
			uint32_t elapsed_ms = sample_tick - last_tick;
			uint16_t sample_dt_ms = (elapsed_ms > UINT16_MAX)
			                          ? UINT16_MAX
			                          : (uint16_t)elapsed_ms;
			// ---- Encoder A (TIM2) ----
			uint16_t now_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			delta_a = (int16_t)(now_a - last_a);
			last_a = now_a;

			// ---- Encoder B (TIM3) ----
			uint16_t now_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			delta_b = (int16_t)(now_b - last_b);
			delta_b = -delta_b;
			last_b = now_b;

			/* Schedule from the current tick rather than adding 20. After a delayed task that causes rapid
			 * catch-up samples and false speed spikes. */
			last_tick = sample_tick;


			// luther telementry
				Telemetry_SendEncoder(
				    (int16_t)delta_a,
				    (int16_t)delta_b,
				    joy_control.owns_motors
				        ? joy_driving
				        : ((uart_cmd != DRIVE_IDLE) ? 1U : 0U),
				    sample_dt_ms
				);

				Telemetry_SendTurn(
				    total_angle,
				    arc_target_angle,
				    gyro_yaw_rate_dps,
				    turn_error_deg,
				    turn_effort_cmd,
				    turn_left_pwm,
				    turn_right_pwm,
				    (uint8_t)turn_control_phase,
				    command_is_turn((int)uart_cmd) ? 1U : 0U
				);

				const uint32_t telemetry_now = HAL_GetTick();

				const uint8_t straight_telemetry_active =
				    (uart_cmd == DRIVE_FORWARD ||
				     uart_cmd == DRIVE_REVERSE ||
				     (int32_t)(straight_log_deadline - telemetry_now) > 0)
				        ? 1U
				        : 0U;

				Telemetry_SendStraight(
				    total_angle,
				    target_angle,
				    gyro_yaw_rate_dps,
				    error_angle,

				    straight_steer_p_percent,
				    straight_steer_i_percent,
				    straight_steer_d_percent,
				    straight_steer_correction_percent,

				    straight_servo_ccr,
				    straight_servo_center_ccr,
				    straight_steer_percent,

				    straight_left_pwm,
				    straight_right_pwm,

				    straight_telemetry_active
				);

				/* WPI is diagnostic only; exclude slides and post-stop STR records. */
				Telemetry_SendWheelPi(
				    ((uart_cmd == DRIVE_FORWARD || uart_cmd == DRIVE_REVERSE) &&
				     slide_mode == SLIDE_NO_SIDE) ? 1U : 0U
				);
			  }
    osDelay(1);
  }
  /* USER CODE END encoderTask */
}

/* USER CODE BEGIN Header_gyroTask */
/**
 * @brief Function implementing the Gyro_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_gyroTask */
void gyroTask(void const * argument)
{

  /* USER CODE BEGIN gyroTask */

	GyroSafe gyro = {0};

	for (;;)
	{
		gyro_healthy = 0U;
		gyro_yaw_rate_dps = 0.0f;

		while (!steering_ready)
		{
			osDelay(20);
		}

		if (!GyroSafe_Recover(&gyro, &hi2c2))
		{
			Telemetry_RecordFault("GYRO_INIT");
			osDelay(IMU_RETRY_DELAY_MS);
			continue;
		}

		/* Keep the robot completely still for this approximately two-second
		 * calibration. Discard the calibration if any sample fails. */
		if (!GyroSafe_Calibrate(&gyro, 200U, IMU_PERIOD_MS))
		{
			Telemetry_RecordFault("GYRO_CAL");
			osDelay(IMU_RETRY_DELAY_MS);
			continue;
		}

		gyro_bias_dps = gyro.bias_raw / 16.4f;

		/* Heading cannot be reconstructed across a sensor failure, so rebase it
		 * after a successful recovery. motorTask prevents motion while unhealthy. */
		total_angle = 0.0f;
		target_angle = 0.0f;
		arc_target_angle = 0.0f;

		uint32_t tick = HAL_GetTick();
		float previous_rate_dps = 0.0f;
		uint8_t consecutive_failures = 0U;

		gyro_last_good_tick = tick;
		gyro_healthy = 1U;

		for (;;)
		{
			float rate_dps;
			osDelay(IMU_PERIOD_MS);

			if (!GyroSafe_ReadRateDps(&gyro, GYRO_SCALE_TRIM, &rate_dps))
			{
				consecutive_failures++;

				if (consecutive_failures >= IMU_MAX_CONSECUTIVE_FAILURES)
				{
					gyro_healthy = 0U;
					gyro_yaw_rate_dps = 0.0f;
					Telemetry_RecordFault("GYRO_I2C");
					break;
				}

				continue;
			}

			consecutive_failures = 0U;

			uint32_t now = HAL_GetTick();
			float dt_s = (float)(now - tick) * 0.001f;
			tick = now;
			gyro_last_good_tick = now;

			if (dt_s > 0.050f)
				dt_s = 0.050f;

			if (fabsf(rate_dps) < IMU_RATE_DEADBAND_DPS)
				rate_dps = 0.0f;

			total_angle += 0.5f * (previous_rate_dps + rate_dps) * dt_s;
			previous_rate_dps = rate_dps;
			gyro_yaw_rate_dps = rate_dps;
		}

		osDelay(IMU_RETRY_DELAY_MS);
	}
	/* USER CODE END gyroTask */
}

/* USER CODE BEGIN Header_ultrasoundTask */
/**
 * @brief Function implementing the Ultrasound_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ultrasoundTask */
void ultrasoundTask(void const * argument)
{
  /* USER CODE BEGIN ultrasoundTask */
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);

  for (;;)
  {
    const uint32_t trigger_tick = HAL_GetTick();
    // Clear BEFORE triggering: a quick echo must not be erased afterward.
    new_measurement_ready = false;
    trigger_us_pulse();
    while (!new_measurement_ready &&
        (uint32_t)(HAL_GetTick() - trigger_tick) < US_ECHO_TIMEOUT_MS)
    {
      osDelay(1);
    }
    if (!new_measurement_ready)
    {
      const uint32_t irq_state = __get_PRIMASK();
      __disable_irq();
      if (!new_measurement_ready)
      {
        // Recover from missing rise OR fall; leave old range explicitly invalid.
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
        __HAL_TIM_SET_COUNTER(&htim8, 0);
        us_echo_started = 0U;
        us_capture_valid = 0U;
        us_capture_tick = HAL_GetTick();
        ++us_capture_seq;
      }
      __set_PRIMASK(irq_state);
    }
    // Trigger spacing includes the time spent waiting for the echo.
    const uint32_t elapsed = HAL_GetTick() - trigger_tick;
    if (elapsed < US_TRIGGER_PERIOD_MS) osDelay(US_TRIGGER_PERIOD_MS - elapsed);
  }
  /* USER CODE END ultrasoundTask */
}

/* USER CODE BEGIN Header_irTask */
/**
 * @brief Function implementing the IR_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_irTask */
void irTask(void const * argument)
{
  /* USER CODE BEGIN irTask */
  /* Infinite loop */
  // Display smoothing only. ADC callback owns the IR obstacle flags.
  const float alpha = 0.20f; // 0..1; higher = faster
  float v0 = 0.0f, v1 = 0.0f;

  for (;;)
  {
    // Read latest DMA samples (16-bit reads are atomic on M4)
    uint16_t r0 = ir_adc[0];
    uint16_t r1 = ir_adc[1];

    float f0 = 3.3f * (float)r0 / 4095.0f;
    float f1 = 3.3f * (float)r1 / 4095.0f;

    v0 = (1.0f - alpha) * v0 + alpha * f0;
    v1 = (1.0f - alpha) * v1 + alpha * f1;

    // Publish for OLED (volatile globals)
    ir0_v = v0;
    ir1_v = v1;

    // Calculate real-time distance using the curve-fit formula
    ir0_distance_cm = Voltage_To_Distance_CM(v0);
    ir1_distance_cm = Voltage_To_Distance_CM(v1);

    // Edge detection is handled by the ADC callback, as in the senior code.

    osDelay(10);
  }
  /* USER CODE END irTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
