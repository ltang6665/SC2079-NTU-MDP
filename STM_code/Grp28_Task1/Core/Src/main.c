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

static char rx_line[256]; // Incoming command line
static uint16_t rx_pos = 0;
static uint8_t rx_line_overflow = 0U;
static char waiting_line[256];
static volatile uint8_t waiting_line_ready = 0U;

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

/* Command acknowledgements */

static volatile uint8_t reply_tx_busy = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
    reply_tx_busy = 0;
}

static void ack_script(void)
{
  static const uint8_t msg[] = "OK\n";
  if (reply_tx_busy)
    return; // UART3 is still sending the previous reply.
  reply_tx_busy = 1;
  (void)HAL_UART_Transmit_IT(&huart3, (uint8_t *)msg, sizeof(msg) - 1);
  queue_clear();
}

// Reply when a motion step finishes
static void ack_step(void)
{
  static const uint8_t msg[] = "done\n";
  if (reply_tx_busy)
    return; // UART3 is still sending the previous reply.
  reply_tx_busy = 1;
  (void)HAL_UART_Transmit_IT(&huart3, (uint8_t *)msg, sizeof(msg) - 1);
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
#define PWM_CRUISE 3500 // Timer compare at the calibrated cruise setting.
#define PWM_SLOW 6800
#define PWM_INNER_WHEEL 6000

//luther direction higher value = slower
#define FORWARD_LEFT_COMPARE_SCALE   1.10f // Left-wheel drive trim from the WPI readings.
#define FORWARD_RIGHT_COMPARE_SCALE  1.000f

/* Reverse wheel compare calibration. */
#define REVERSE_LEFT_COMPARE_SCALE   0.993f // A lower timer compare increases drive.
#define REVERSE_RIGHT_COMPARE_SCALE  1.000f

/*
 * Straight motion profile. luther
 *
 * ACCEL_RAMP_MS:
 *     Time taken to ramp from zero drive to 100% drive.
 *
 * DECEL_CM:
 *     Distance before the target at which planned deceleration begins.
 *
 * DECEL_MIN_PERCENT:
 *     Lowest drive level before the final brake.
 *     Keeping this above zero reduces the chance of stalling before
 *     reaching target_counts.
 */
#define LINE_ACCEL_RAMP_MS        500U
#define LINE_DECEL_CM             8.0f // Distance before target where planned quadratic decel begins.
#define LINE_DECEL_MIN_PERCENT    15.0f // Configurable minimum drive percent reached right before final brake.
#define LINE_DECEL_EXPONENT       2.0f // Quadratic deceleration curve: 2.0 = smooth entry, curving down to MIN_PERCENT.

// Temporary left-wheel trim during launch.
#define FORWARD_LAUNCH_TRIM_MS                        300U
#define FORWARD_LAUNCH_LEFT_EFFORT_TRIM_PERCENT       0.0f //9.0f

#define REVERSE_LAUNCH_TRIM_MS                        400U
#define REVERSE_LAUNCH_LEFT_EFFORT_TRIM_PERCENT       0.0f

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

#define ARC_KP_EFFORT_PER_DEG       80.0f
#define ARC_KI_EFFORT_PER_DEG_S      0.0f  // Leave at zero while tuning to avoid added overshoot
#define ARC_KD_EFFORT_PER_DPS        0.0f
#define ARC_I_LIMIT_DEG_S           40.0f

#define ARC_OUTER_EFFORT_MAX      5000.0f  // Outer-wheel effort ceiling.
#define ARC_OUTER_EFFORT_MIN      1000.0f
#define ARC_INNER_EFFORT_RATIO_R     0.2f
#define ARC_INNER_EFFORT_RATIO_L     0.42f

#define ARC_FINISH_TOL_DEG           0.1f // Yaw tolerance at the end of a turn.
#define ARC_BRAKE_LOOKAHEAD_S        0.0855f // Seconds of yaw travel allowed for braking.
#define ARC_SETTLE_MS               150U
#define ARC_RETRY_SERVO_MS           80U
#define ARC_MAX_APPROACHES            3U
#define ARC_TIMEOUT_BASE_MS        1500U
#define ARC_TIMEOUT_PER_DEG_MS       80U

#define IMU_PERIOD_MS                10U
#define IMU_RATE_DEADBAND_DPS         0.01f
#define IMU_STALE_TIMEOUT_MS         100U
#define IMU_RETRY_DELAY_MS          1000U
#define IMU_MAX_CONSECUTIVE_FAILURES   3U

#define STEP_GAP_MS 100 // Pause between queued steps, in ms.
#define ARC_DELAY 50
#define ARC_DELAY_LEFT 100
#define ARC_DELAY_RIGHT 100
#define SLIDE_CMD_MS 350

// --- Steering mode (no angles; tokens like "fr", "fl") ---
typedef enum
{
  TURN_NO_SIDE = 0,
  TURN_TO_LEFT,
  TURN_TO_RIGHT
} turn_side_t;
volatile turn_side_t cmd_turn = TURN_NO_SIDE;


// luther CCR
#define STEER_CENTER_CCR 152            // Default straight steering position.
#define STEER_CENTER_AFTERLEFT_CCR 161  // After-left calibration: 162; testing at 163 gave about -1.2 degrees
#define STEER_CENTER_AFTERRIGHT_CCR 152 // Keep the after-right setting at 150 (about +2.2 degrees)
#define STEER_RIGHT_CCR 240             // Steering position for FR.
#define STEER_LEFT_CCR 110              // Steering position for FL.
#define STEER_REVERSE_LEFT_CCR 110
#define STEER_REVERSE_RIGHT_CCR 240

/* Reverse straight-line centre depends on the last steering direction.
 * Calibrate these with LINE_DIAG_FIXED_SERVO_REVERSE. */
#define STEER_REV_CENTER_AFTERLEFT_CCR  STEER_CENTER_AFTERLEFT_CCR
#define STEER_REV_CENTER_AFTERRIGHT_CCR STEER_CENTER_AFTERRIGHT_CCR

#define ARC_SERVO_ONLY_TEST       0 // for testing
#define ARC_SERVO_TEST_HOLD_MS    10000U //Steering hold time during the test

#define SPEED_FILTER_TAU 0.05f // Encoder-speed filter time constant: 50 ms

#define STEER_HOME_RIGHT_CCR              STEER_RIGHT_CCR // Homing endpoint before returning to centre
#define STEER_HOME_LEFT_CCR               STEER_LEFT_CCR // for testing
#define STEERING_HOME_BRAKE_SETTLE_MS     50U // Let the brake engage before moving the steering
#define STEERING_HOME_END_HOLD_MS         600U // Dwell at the chosen steering endpoint.
#define STEERING_HOME_CENTER_SETTLE_MS    500U //Wait for the steering linkage to settle

/* Heading hold during straight travel. These values act on the steering servo only;
 * wheel-speed PI remains responsible for balancing encoder speeds. */
#define LINE_STEER_DEADBAND_DEG         0.01f
#define LINE_STEER_MIN_PERCENT          7.50f
#define LINE_STEER_MAX_PERCENT          60.0f
/* Direction-specific heading-hold gains.
 * Forward uses the user's calibrated values; reverse has a separate gain set. */
#define FORWARD_STEER_KP_PERCENT_PER_DEG   5.60f // Steering P gain, percent per degree.
#define FORWARD_STEER_KI_PERCENT_PER_DEG_S 1.10f   // Steering I gain, percent per degree-second.
#define FORWARD_STEER_KD_PERCENT_PER_DPS   0.94f  // Tuned using the user's F200_with_updated_params telemetry.

#define REVERSE_STEER_KP_PERCENT_PER_DEG   5.60f // Reverse gain; check against r200 telemetry.
#define REVERSE_STEER_KI_PERCENT_PER_DEG_S 0.00f  // Steering integral is off.
#define REVERSE_STEER_KD_PERCENT_PER_DPS   0.94f // Steering D gain, percent per degree/second.

/* How a steering percentage is converted into servo CCR counts.
 *   0 = original behaviour: percent is scaled by the travel available on the
 *       side being requested (centre->left or centre->right). With centre 150,
 *       left 110, right 240 that is 40 vs 90 counts, so the same % gives very
 *       different CCR changes on each side.
 *   1 = symmetric: both sides use the SMALLER of the two spans, so +x% and -x%
 *       give the same CCR change from centre.
 * Forward keeps 0 so your tuned forward behaviour does not change. */
#define FORWARD_STEER_SYMMETRIC_MAP 0
#define REVERSE_STEER_SYMMETRIC_MAP 0

#define LINE_STEER_I_LIMIT_PERCENT      20.0f // Limit the integral term to prevent windup
#define LINE_STEER_SLEW_PERCENT_PER_S  900000.0f // Maximum steering correction change per second
#define LINE_STEER_D_LPF_TAU_S 0.050f // Yaw-rate filter time constant; larger values smooth more


/* When enabled, integral gating excludes deceleration, low drive levels,
 * and large heading errors. */
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
#define LINE_DIAG_FIXED_SERVO          0 // Set to 1 for fixed-steering calibration.
#define LINE_DIAG_OFFSET_CCR           0 // Fixed steering offset
#define LINE_DIAG_FIXED_SERVO_REVERSE  0 // 1 = the fixed-servo diagnostic also applies to reverse (use to calibrate SERVO_REV_CENTER_*)
#define WHEEL_PI_DIAG_DISABLE 			   1   // 1 disables the applied wheel correction; controller values remain logged.
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

// Values produced by IR_Task, consumed by OLED_Task
volatile uint16_t ir0_raw = 0, ir1_raw = 0;
volatile float ir0_v = 0.0f, ir1_v = 0.0f;
volatile uint8_t ir0_obs = 0, ir1_obs = 0; // obstacle flags

// threshold voltage
#define IR_VOLT_OBS 0.80f // should be 30 cm for GP2Y0A21 (tune in test)

// floating problem
volatile uint16_t ir0_mV = 0, ir1_mV = 0; // 0..3300
#define IR_MV_OBS 800                     // 800 mV threshold.

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
  SLIDE_CENTERING
} slide_stage_t;

volatile slide_side_t slide_mode = SLIDE_NO_SIDE;     // active slide direction
volatile slide_stage_t slide_phase = SLIDE_IDLE; // OUT -> IN
volatile double slide_origin_heading = 0.0;   // heading at slide start
volatile double slide_return_heading = 0.0;

#define SLIDE_ANGLE_DEG 50.0
#define STEER_SLIDERIGHT_CCR 245
#define STEER_SLIDELEFT_CCR 100

// --- Slide PWM tunables
#define SLIDE_OUTER_PWM 5000 // Outer wheel PWM compare
#define SLIDE_INNER_PWM 6500 // Inner wheel PWM compare

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

static inline void brake_wheels(void) // Both bridge inputs high for active braking
{
  Telemetry_ClearWheelPi(); // Discard the pre-brake controller snapshot.
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF);

  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_OFF);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_OFF);
}

static inline void drive_left_forward(int duty)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty);    // IN2 (PWM)
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF); // IN1 high -> FWD
}
static inline void drive_right_forward(int duty)
{
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
  // reverse on left: IN1 = PWM (CH4), IN2 = HIGH (CH3)
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF); // IN2 high
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty);    // IN1 pwm
}
static inline void drive_right_reverse(int duty)
{
  // reverse on right: IN1 = PWM (CH2), IN2 = HIGH (CH1)
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_OFF); // IN2 high
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, duty);    // IN1 pwm
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

static void reset_arc_control(void)
{
  arc_integral_deg_s = 0.0f;
  arc_pid_tick = HAL_GetTick();
  arc_settle_deadline = 0U;
  arc_deadline = arc_pid_tick + ARC_TIMEOUT_BASE_MS +
                      (uint32_t)(TURN_DEG * (float)ARC_TIMEOUT_PER_DEG_MS);
  arc_attempts = 1U;

  turn_error_deg = arc_target_angle - total_angle;
  turn_effort_cmd = 0.0f;
  turn_left_pwm = PWM_OFF;
  turn_right_pwm = PWM_OFF;
  turn_control_phase = ARC_RUNNING;
}

static void stop_arc_control(void)
{
  arc_integral_deg_s = 0.0f;
  arc_pid_tick = 0U;
  arc_settle_deadline = 0U;
  arc_deadline = 0U;
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

static void complete_turn(int completed_command)
{
  brake_wheels();
  recenter_after_turn(completed_command);
  stop_arc_control();

  /* A following straight segment should hold the heading actually reached. */
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
    uart_cmd = DRIVE_IDLE;
    next_start_tick = HAL_GetTick() + STEP_GAP_MS;
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

// Parse a command line into queued movements in motorTask.
static void queue_script_line(char *line)
{
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
      // Ignore commands after reset on this line.
      break;
    }
    motion_step_t it = {STEP_NONE, 0};

    // tokens already lowercased in Rx callback
    if (strcmp(tok, "fir") == 0)
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
      int ok = queue_add(it);
      if (!ok)
      {
        const char emsg[] = "ERR:QUEUE\r\n";
        // drop the rest if full
        break;
      }
    }
    tok = strtok(NULL, " \t");
  }
  motion_step_t eos = {STEP_END, 0};
  (void)queue_add(eos);
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

	const char *telemetry_name = step_log_name(it.type);

	if (telemetry_name != NULL)
	{
		/* EncoderTask snapshots the gains for the SPID line as soon as it sees
		 * the new command, so the gains for THIS command must be set first. */
		if (it.type == STEP_REVERSE_CM)
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

		case STEP_FX: // fall through — same setup, just different reverse flag
		case STEP_FU:
		{
			// Forward "forever" but stop when ultrasound <= us_stop_cm
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = total_angle;
			steer_center();

			// reset odometry for reporting
			odom_counts_run = 0;
			odom_cm_run = 0.0f;

			target_counts = INT32_MAX;                                           // run "forever"
			us_stop_cm = (it.value > 0) ? (uint16_t)it.value : OBST_STOP_CM; // per-move threshold
			us_allow_reverse = (it.type == STEP_FU) ? 1 : 0;                     // FU=reverse, FX=just stop
			obstacle_stop_mode = STOP_MODE_US_T;                                 // Ultrasound threshold mode.
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // clean speed/odom baselines
			return 1;
		}

		case STEP_FIRO:
		{
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = total_angle;
			steer_center();

			odom_counts_run = 0;
			odom_cm_run = 0.0f;

			target_counts = INT32_MAX; // run “forever”
			obstacle_stop_mode = STOP_MODE_IR_RIGHT_O;
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // Reset encoder baselines for this segment.
			return 1;
		}

		case STEP_FILO:
		{
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = total_angle;
			steer_center();

			odom_counts_run = 0;
			odom_cm_run = 0.0f;

			target_counts = INT32_MAX;                // run “forever”
			obstacle_stop_mode = STOP_MODE_IR_LEFT_O;
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // Reset encoder baselines for this segment.
			return 1;
		}

		case STEP_FIR:
		{
			// forward "forever" until right IR reports 0 (no obstacle)
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = total_angle;
			steer_center();

			odom_counts_run_ir = 0;
			odom_cm_run_ir = 0.0f;

			target_counts = INT32_MAX;               // sentinel: distance won't stop us
			obstacle_stop_mode = STOP_MODE_IR_RIGHT;
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // Reset encoder baselines for this segment.
			return 1;
		}

		case STEP_FIL:
		{
			// forward "forever" until right IR reports 0 (no obstacle)
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = total_angle;
			steer_center();

			odom_counts_run_ir = 0;
			odom_cm_run_ir = 0.0f;

			target_counts = INT32_MAX;              // sentinel: distance won't stop us
			obstacle_stop_mode = STOP_MODE_IR_LEFT; // IR LEFT
			uart_cmd = DRIVE_FORWARD;
			seg_rebase = 1; // Reset encoder baselines for this segment.
			return 1;
		}

		case STEP_FORWARD_CM:
		{
			int cm = it.value;
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			target_angle = total_angle;
			steer_center();

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
			seg_rebase = 1; // Reset encoder baselines for this segment.
			return 1;
		}

		case STEP_REVERSE_CM:
		{
			int cm = (it.value > 0) ? it.value : 50;
			enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
			enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			int32_t tgt = -(int32_t)(cm * ENCODER_COUNTS_PER_CM + 0.5f);
			target_counts = tgt;
			target_angle = total_angle;
			steer_center();
			uart_cmd = DRIVE_REVERSE;
			seg_rebase = 1;
			return 1;
		}

		case STEP_FR:
		{
			TURN_DEG = (it.value >= 0) ? (float)it.value : -(float)it.value;
			arc_target_angle = total_angle - TURN_DEG;
			steer_fr();

			#if ARC_SERVO_ONLY_TEST
			turn_motor_enable_tick = HAL_GetTick() + ARC_SERVO_TEST_HOLD_MS;
			#else
			turn_motor_enable_tick = HAL_GetTick() + ARC_DELAY_RIGHT;
			#endif
			arc_postforward_cm = 0;
			uart_cmd = DRIVE_FR;
			reset_arc_control();
			return 1;
		}

		case STEP_FL:
		{
			TURN_DEG = (it.value >= 0) ? (float)it.value : -(float)it.value;
			arc_target_angle = total_angle + TURN_DEG;
			steer_fl();

			#if ARC_SERVO_ONLY_TEST
			turn_motor_enable_tick = HAL_GetTick() + ARC_SERVO_TEST_HOLD_MS;
			#else
			turn_motor_enable_tick = HAL_GetTick() + ARC_DELAY_LEFT;
			#endif
			arc_postforward_cm = 0;
			uart_cmd = DRIVE_FL;
			reset_arc_control();
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
			ack_script(); // Acknowledge the completed command line.
			// Do NOT set uart_cmd; just return. The idle loop will pick up the next item
			return 1;
		}

		case STEP_SLIDE_RIGHT:
		{
			// arm the slide state machine (RIGHT: heading goes -45)
			slide_origin_heading = arc_target_angle; // save the current heading for slide back
			//		slide_out_target = arc_target_angle - SLIDE_ANGLE_DEG;
			slide_return_heading = arc_target_angle + SLIDE_ANGLE_DEG;
			arc_target_angle = arc_target_angle - SLIDE_ANGLE_DEG;
			//		arc_target_angle = arc_target_angle;
			slide_mode = SLIDE_TO_RIGHT;
			slide_phase = SLIDE_OUTWARD;
			// ensure we are in forward mode; if idle or not forward, start forward-forever
			if (uart_cmd != DRIVE_FORWARD)
			{
				enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
				enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
				//            target_angle     = total_angle;
				//            arc_target_angle = total_angle;           // lock straight reference
				steer_center();

				target_counts = INT32_MAX; // forward "forever"
				obstacle_stop_mode = STOP_MODE_NONE;
				uart_cmd = DRIVE_FORWARD;
			}

			return 1;
		}

		case STEP_SLIDE_LEFT:
		{
			// arm the slide state machine (LEFT: heading goes +45)
			slide_origin_heading = arc_target_angle;
			//    		slide_out_target = arc_target_angle + SLIDE_ANGLE_DEG;
			slide_return_heading = arc_target_angle - SLIDE_ANGLE_DEG;
			arc_target_angle = arc_target_angle + SLIDE_ANGLE_DEG;
			//		arc_target_angle = arc_target_angle;
			slide_mode = SLIDE_TO_LEFT;
			slide_phase = SLIDE_OUTWARD;
			// ensure we are in forward mode; if idle or not forward, start forward-forever
			if (uart_cmd != DRIVE_FORWARD)
			{
				enc_start_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
				enc_start_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
				//                target_angle     = total_angle;
				//                arc_target_angle = total_angle;           // lock straight reference
				steer_center();

				target_counts = INT32_MAX; // forward "forever"
				obstacle_stop_mode = STOP_MODE_NONE;
				uart_cmd = DRIVE_FORWARD;
			}

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
// callback function, when rxbuffer comes in, go to this function
//  my callback function will parse the command

/* --- IR Sensor Helpers --- */

// Convert ADC raw value (0–4095) to volts (0.0–3.3 V)
/*float ADC_To_Voltage(uint16_t raw) {
    return (3.3f * raw) / 4095.0f;
}

// Very rough mapping from voltage to distance in cm
float Voltage_To_Distance_CM(float v) {
    if (v < 0.4f) return 80.0f;
    else if (v < 0.5f) return 60.0f;
    else if (v < 0.7f) return 40.0f;
    else if (v < 1.1f) return 30.0f;
    else if (v < 1.5f) return 20.0f;
    else if (v < 2.2f) return 15.0f;
    else return 10.0f;
}*/

void HCSR04_Trigger(void)
{
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);
  HAL_Delay(1); // Short delay for 10µs pulse
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
    new_measurement_ready = true;
  }
}

/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET)
  {
    tc1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
  }
  else
  {
    tc2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    echo = (tc2 > tc1) ? (tc2 - tc1) : (65536 - tc1 + tc2);
    uint32_t raw_cm = (uint32_t)(echo * (0.0343f / 2.0f) + 0.5f);
    us_history[us_history_pos] = raw_cm;
    us_history_pos = (us_history_pos + 1U) % 3U;
    if (us_history_used < 3U)
    {
      us_history_used++;
    }
    uint32_t filtered_cm = raw_cm;
    if (us_history_used == 3U)
    {
      filtered_cm = median_of_three(us_history[0], us_history[1], us_history[2]);
    }
    distance = (float)filtered_cm;
    echo_debug = filtered_cm;
    new_measurement_ready = true;
  }
}*/

// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//     if (htim->Instance != TIM8 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) return;
//
//     if (!us_echo_started) {
//         // Rising edge: start timing
//         __HAL_TIM_SET_COUNTER(htim, 0);
//         __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
//         us_echo_started = 1;
//     } else {
//         // Falling edge: stop timing
//         us_pulse_ticks = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);  // microseconds
//         __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
//         us_echo_started = 0;
//         new_measurement_ready = true;  // tell the task there’s a fresh width
//     }
// }

// keep this callback short, just update the buffer and add to queue once \n
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint8_t c = rx_byte;

    if (c == '\r' || c == '\n')
    {
      /* Do not call strtok(), atoi(), queue code, or command-start code in
       * interrupt context. Publish one complete line for motorTask instead. */
	/*if (rx_pos > 0U && !waiting_line_ready)
      {
        rx_line[rx_pos] = '\0';
        memcpy(waiting_line, rx_line, (size_t)rx_pos + 1U);
        waiting_line_ready = 1U;
      }
      rx_pos = 0U;
    }
    else
    {
      if (rx_pos < sizeof(rx_line) - 1)
      {
        rx_line[rx_pos++] = (char)c;
      }
      else
      {
        rx_pos = 0U;
      }
    }

    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte, 1);
  }
}*/

// luther new callback for controller pad
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3) return;
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
        for (uint16_t i = 0U; i < rx_pos; ++i)
          if (rx_line[i] == ';' || rx_line[i] == ',') rx_line[i] = ' ';
        queue_script_line(rx_line);
        if (uart_cmd == DRIVE_IDLE && !abort_now) (void)run_next_step();
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
    	// --- PAGE 0: Yaww, Target, Speeds, and CMD ---
		snprintf(line, sizeof(line), "Yaw: %-11.1f", (double)total_angle);
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

	char command_line[sizeof(waiting_line)];

	for (;;)
	{
	if (waiting_line_ready)
	{
	  uint32_t primask = __get_PRIMASK();
	  __disable_irq();
	  memcpy(command_line, waiting_line, sizeof(command_line));
	  waiting_line_ready = 0U;
	  if (primask == 0U)
		__enable_irq();

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

		// Clear the interrupted movement target.
		target_counts = 0;

		// clear outstanding script
		queue_clear();

		uart_cmd = DRIVE_IDLE;
		turn_motor_enable_tick = 0;
		arc_postforward_cm = 0;
		stop_arc_control();


		/* luther
		 * Queue the reset acknowledgement for UART3 transmission.
		 */

		joy_queue_reply("RST\r\n");
		joy_send_reply();


		osDelay(5);

		/* Skip normal motor processing and restart the motor loop. */
		continue;

	}

	if (joy_poll())
	{
		osDelay(5);
		continue;
	}

	switch (uart_cmd)
	{
	case DRIVE_FORWARD:
	case DRIVE_REVERSE:
	{
	  // Wheel-speed balance gains.
	  const float Kp = 0.250f;
	  const float Ki = 0.010f;
	  const float Kd = 0.010f;

	  //  ====== Persistent PID state ======
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
		  drive_profile_start_tick = profile_now;

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

	  // Remaining distance in encoder counts. Keep it positive regardless of FWD/REV direction. luther
	  int32_t remaining_counts;

	  if (commanded_direction > 0)
	  {
		  remaining_counts = target_counts - avg_moved;
	  }
	  else
	  {
		  remaining_counts = avg_moved - target_counts;
	  }

	  if (remaining_counts < 0)
	  {
		  remaining_counts = 0;
	  }

	  // ====== Stop conditions ======

	  // ====== Exact encoder stop condition ======
	  bool done_by_counts = false;

	  if (target_counts >= 0)
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
		switch (obstacle_stop_mode)
		{

		case STOP_MODE_US_T:
		  // Stop condition for FUxx: ultrasound <= per-move threshold
		  done_by_obst = (echo_debug <= us_stop_cm);

		  // If extremely close, reverse only if allowed (FU), otherwise just stop (FX)
		  if (echo_debug <= OBST_REV_CM && us_allow_reverse)
		  {

			// Report the close obstacle before switching to reverse.
			if (!reply_tx_busy)
			{
			  char us_msg[32];
			  int n = snprintf(us_msg, sizeof(us_msg), "us0.0,%d\n", (int)echo_debug);
			  reply_tx_busy = 1;
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *)us_msg, (uint16_t)n);
			}

			// Keep heading straight; hand over to reverse controller
			target_angle = arc_target_angle;
			steer_center();

			obstacle_stop_mode = STOP_MODE_US_BACK; // Reverse until the ultrasound clearance threshold is reached.
			target_counts = INT32_MIN + 1;          // sentinel: counts won’t finish it

			// Clean PID handover
			i_acc = 0;
			prev_err = 0;

			done_by_obst = false; // we’re not finishing here; switching to REVERSE
			seg_rebase = 1;
			uart_cmd = DRIVE_REVERSE;
		  }
		  break;

		case STOP_MODE_US:
		  // ultrasound: stop when we are closer than threshold
		  done_by_obst = (echo_debug <= OBST_STOP_CM);
		  // reverse

		  if (echo_debug <= OBST_REV_CM)
		  { // reverse bump

			if (!reply_tx_busy)
			{
			  char us_msg[32];
			  int n = snprintf(us_msg, sizeof(us_msg), "us0.0,%d\n",
							   (int)echo_debug);
			  reply_tx_busy = 1;
			  HAL_UART_Transmit_IT(&huart3, (uint8_t *)us_msg, (uint16_t)n);
			}

			// Keep heading straight
			target_angle = arc_target_angle;
			steer_center();

			// --- switch to reverse-until-clear mode ---
			//						  odom_counts_run = 0;                  // track how far we reverse
			obstacle_stop_mode = STOP_MODE_US_BACK;
			target_counts = INT32_MIN + 1; // sentinel so counts never end us

			// Clean PID handover
			i_acc = 0;
			prev_err = 0;

			done_by_obst = false;
			seg_rebase = 1; // Reset encoder baselines for this segment.
			uart_cmd = DRIVE_REVERSE;
		  }
		  break;
		case STOP_MODE_IR_RIGHT:
		  // FIR semantics: keep moving UNTIL right-IR says "no obstacle" (0)
		  // -> stop when ir1_obs == 0
		  done_by_obst = (ir1_obs == 0); // PC1
		  break;
		case STOP_MODE_IR_LEFT:
		  done_by_obst = (ir0_obs == 0); // PC0
		  break;

		case STOP_MODE_IR_RIGHT_O: // FIRO stops when an obstacle is present.
		  done_by_obst = (ir1_obs == 1);
		  break;

		case STOP_MODE_IR_LEFT_O: // FILO stops when an obstacle is present.
		  done_by_obst = (ir0_obs == 1);
		  break;
		default:
		  break;
		}
	  }

	  if (uart_cmd == DRIVE_REVERSE && obstacle_stop_mode == STOP_MODE_US_BACK)
	  {
		done_by_obst = (echo_debug >= US_BACK_CLEAR_CM);
	  }

	  // transmit US distance travelled and detected
	  if (done_by_obst && (obstacle_stop_mode == STOP_MODE_US || obstacle_stop_mode == STOP_MODE_US_T))
	  {
		odom_cm_run = (float)odom_counts_run / ENCODER_COUNTS_PER_CM;

		if (!reply_tx_busy)
		{
		  char us_msg[32];
		  int n = snprintf(us_msg, sizeof(us_msg), "us%.1f,%d\n",
						   (double)odom_cm_run, (int)echo_debug);
		  reply_tx_busy = 1;
		  HAL_UART_Transmit_IT(&huart3, (uint8_t *)us_msg, (uint16_t)n);
		}
	  }

	  // transmit IR distance travelled
	  if (done_by_obst && (obstacle_stop_mode == STOP_MODE_IR_RIGHT || obstacle_stop_mode == STOP_MODE_IR_LEFT))
	  {
		odom_cm_run_ir = (float)odom_counts_run_ir / ENCODER_COUNTS_PER_CM;

		if (!reply_tx_busy)
		{
		  char ir_msg[32];
		  int n = snprintf(ir_msg, sizeof(ir_msg), "ir%.1f\n", (double)odom_cm_run_ir);
		  reply_tx_busy = 1;
		  HAL_UART_Transmit_IT(&huart3, (uint8_t *)ir_msg, (uint16_t)n);
		}
	  }


	  if (done_by_counts || done_by_obst)
	  {
	      brake_wheels();

	      straight_left_pwm = PWM_OFF;
	      straight_right_pwm = PWM_OFF;

	      // return to the currently calibrated mechanical centre.
	      steer_center();

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

	      i_acc = 0;
	      prev_err = 0;

	      uart_cmd = DRIVE_IDLE;
	      next_start_tick = HAL_GetTick() + STEP_GAP_MS;

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
		cpsA_f = 0.0f;
		cpsB_f = 0.0f;


		// optional: also clear PID transients to avoid a kick
		i_acc = 0;
		prev_err = 0;
		steer_i_acc_deg_s = 0.0f;
		steer_last_percent = 0.0f;
		steer_hold_latched = false;
		steer_hold_percent = 0.0f;
		steer_d_rate_f = 0.0f;

		straight_steer_p_percent = 0.0f;
		straight_steer_i_percent = 0.0f;
		straight_steer_d_percent = 0.0f;
		straight_steer_correction_percent = 0.0f;

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

	  uint32_t dt_ms = (t - last_t);
	  if (dt_ms == 0)
		dt_ms = 1;

	  int32_t cpsA = (int32_t)da * 1000 / (int32_t)dt_ms;
	  int32_t cpsB = (int32_t)db * 1000 / (int32_t)dt_ms;

	  // ---- Low-pass filter for PID feedback ----

	  float dt_s = (float)dt_ms / 1000.0f;
	  float alpha = dt_s / (SPEED_FILTER_TAU + dt_s); // Filter coefficient follows the sample interval.

	  cpsA_f += alpha * ((float)cpsA - cpsA_f);
	  cpsB_f += alpha * ((float)cpsB - cpsB_f);

	  last_a = now_a;
	  last_b = now_b;
	  last_t = t;


	  // Wheel-speed balance feedback.
	  // Speed-based error (motor balance)
	  int32_t speed_err = (int32_t)(cpsA_f - cpsB_f); // filtered values

	  // Current heading error for the steering controller.
	  error_angle = target_angle - total_angle;

	  /* Wheel PI keeps the encoder speeds balanced. The servo handles heading correction
	   * below. Feeding yaw into both loops makes them fight each other. */
	  int32_t err = speed_err;

	  i_acc += err;
	  const int32_t IACC_CLAMP = 40000; // Integral accumulator limit.
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

	  /*
	   * Requested level before applying the scripted acceleration/deceleration
	   * profile.
	   */
	  float requested_percent = manual_straight ? (float)joy_throttle_percent : 100.0f;

	  /*
	   * Apply the automatic motion profile only to normal finite F/R commands.
	   *
	   * Emergency/obstacle/manual stops should retain their immediate safety
	   * behaviour.
	   */
	  const bool finite_scripted_move =
		  !manual_straight &&
		  obstacle_stop_mode == STOP_MODE_NONE &&
		  target_counts != INT32_MAX &&
		  target_counts != (INT32_MIN + 1);

	  float profile_target = requested_percent;
	  bool decel_active = false; // true while inside the planned-deceleration zone (used by the steering block)

	  if (finite_scripted_move)
	  {
		  // Planned quadratic deceleration down to configurable LINE_DECEL_MIN_PERCENT.
		  const float decel_counts = LINE_DECEL_CM * ENCODER_COUNTS_PER_CM;

		  if (decel_counts > 0.0f && (float)remaining_counts < decel_counts)
		  {
		      decel_active = true;

		      // decel_progress: 0.0 at decel start -> 1.0 when remaining_counts reaches 0 (target reached).
		      float decel_progress = 1.0f - ((float)remaining_counts / decel_counts);
		      if (decel_progress < 0.0f) decel_progress = 0.0f;
		      if (decel_progress > 1.0f) decel_progress = 1.0f;

		      // Quadratic curve matching the acceleration shape (starts gentle, smoothly slopes down):
		      // decel_percent = 100 - (100 - MIN) * (decel_progress ^ 2)
		      float decel_percent = 100.0f - (100.0f - LINE_DECEL_MIN_PERCENT) * powf(decel_progress, LINE_DECEL_EXPONENT);

		      if (decel_percent < profile_target)
		      {
		          profile_target = decel_percent;
		      }
		  }

		  // non linear accel ramp
		  // calcs amount of time elapsed since drive start
		  uint32_t accel_elapsed_ms = profile_now - drive_profile_start_tick;

		  // calcs elapsed time as a percentage of the amount of time given to accel
		  float accel_fraction = (float)accel_elapsed_ms / (float)LINE_ACCEL_RAMP_MS;

		  //safety clamps
		  if (accel_fraction < 0.0f)
		  {
		      accel_fraction = 0.0f;
		  }

		  if (accel_fraction > 1.0f)
		  {
		      accel_fraction = 1.0f;
		  }

		  // drive % = 100 × x², generates quadratic curve since accel_fraction^2
		  float accel_limit_percent =
		      100.0f *
		      accel_fraction *
		      accel_fraction;

		  // Apply whichever limit requests less drive.
		  if (profile_target < accel_limit_percent)
		  {
		      drive_profile_percent = profile_target;
		  }
		  else
		  {
		      drive_profile_percent = accel_limit_percent;
		  }
	  }
	  else
	  {
		  /* Manual and obstacle movements use the requested throttle directly. */
		  drive_profile_percent = requested_percent;
	  }

	  if (drive_profile_percent < 0.0f)
		  drive_profile_percent = 0.0f;

	  if (drive_profile_percent > 100.0f)
		  drive_profile_percent = 100.0f;

	  const int throttle_percent =
		  (int)(drive_profile_percent + 0.5f);
	  //end new acceleration luther

	  const float left_compare_scale =
		  is_reverse ? REVERSE_LEFT_COMPARE_SCALE : FORWARD_LEFT_COMPARE_SCALE;
	  const float right_compare_scale =
		  is_reverse ? REVERSE_RIGHT_COMPARE_SCALE : FORWARD_RIGHT_COMPARE_SCALE;

	  const int full_base_L = limit_pwm(
		  (int)((float)PWM_CRUISE * left_compare_scale + 0.5f));
	  const int full_base_R = limit_pwm(
		  (int)((float)PWM_CRUISE * right_compare_scale + 0.5f));

	  int base_L = PWM_OFF -
		  ((PWM_OFF - full_base_L) * throttle_percent) / 100;
	  int base_R = PWM_OFF -
		  ((PWM_OFF - full_base_R) * throttle_percent) / 100;

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
	  if (fabsf(steering_error) < LINE_STEER_DEADBAND_DEG)
	  {
		  steering_error = 0.0f;
	  }

	  /* Heading PID outputs a percentage of available steering travel.
	   * Integral action compensates steady bias when enabled. The derivative
	   * uses measured yaw rate: d(error)/dt = -gyro_yaw_rate_dps. */
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
	  const float steer_kp = is_reverse ? REVERSE_STEER_KP_PERCENT_PER_DEG
											: FORWARD_STEER_KP_PERCENT_PER_DEG;
	  const float steer_ki = is_reverse ? REVERSE_STEER_KI_PERCENT_PER_DEG_S
											: FORWARD_STEER_KI_PERCENT_PER_DEG_S;
	  const float steer_kd = is_reverse ? REVERSE_STEER_KD_PERCENT_PER_DPS
											: FORWARD_STEER_KD_PERCENT_PER_DPS;

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
	      dt_s / (LINE_STEER_D_LPF_TAU_S + dt_s);

	  steer_d_rate_f +=
	      d_alpha * (gyro_yaw_rate_dps - steer_d_rate_f);

	  const float steer_d_term =
	      -steer_kd * steer_d_rate_f;

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
	  if (steering_error != 0.0f &&
		  fabsf(correction_percent) < LINE_STEER_MIN_PERCENT)
	  {
		  correction_percent =
			  (correction_percent > 0.0f)
				  ? LINE_STEER_MIN_PERCENT
				  : -LINE_STEER_MIN_PERCENT;
	  }

	  /* Limit heading correction. */
	  if (correction_percent > LINE_STEER_MAX_PERCENT)
	  {
		  correction_percent = LINE_STEER_MAX_PERCENT;
	  }
	  else if (correction_percent < -LINE_STEER_MAX_PERCENT)
	  {
		  correction_percent = -LINE_STEER_MAX_PERCENT;
	  }

	  /*
	   * Slew-rate limit the commanded correction so the servo moves
	   * smoothly instead of jumping between values on every ~10 ms
	   * tick, which otherwise shows up as a wiggling/oscillating line.
	   */
	  {
		  const float max_step_percent = LINE_STEER_SLEW_PERCENT_PER_S * dt_s;

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

	  if (is_reverse ? REVERSE_STEER_SYMMETRIC_MAP : FORWARD_STEER_SYMMETRIC_MAP)
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

	  // add sliding right
	  // === Slide state machine overrides servo while active ===
	  if (slide_mode != SLIDE_NO_SIDE)
	  {

		if (slide_phase == SLIDE_OUTWARD)
		{
		  // steer toward target side until reaching ±45° from origin
		  htim12.Instance->CCR2 = (slide_mode == SLIDE_TO_RIGHT) ? STEER_SLIDERIGHT_CCR : STEER_SLIDELEFT_CCR;

		  if (slide_mode == SLIDE_TO_RIGHT)
		  {
			// Right slide -> steer right, so LEFT is outer, RIGHT is inner
			int outer = (int)(SLIDE_OUTER_PWM);
			int inner = (int)(SLIDE_INNER_PWM);
			drive_left_forward(outer);
			drive_right_forward(inner);
		  }
		  else
		  { // SLIDE_TO_LEFT
			// Left slide -> steer left, so RIGHT is outer, LEFT is inner
			int outer = (int)(SLIDE_OUTER_PWM);
			int inner = (int)(SLIDE_INNER_PWM);
			drive_right_forward(outer);
			drive_left_forward(inner);
		  }
		  if ((slide_mode == SLIDE_TO_RIGHT && total_angle <= arc_target_angle) ||
			  (slide_mode == SLIDE_TO_LEFT && total_angle >= arc_target_angle))
		  {
			slide_phase = SLIDE_INWARD; // start coming back
		  }
		}
		else if (slide_phase == SLIDE_INWARD)
		{
		  // steer opposite side until we’re back at original heading (within tolerance)
		  htim12.Instance->CCR2 = (slide_mode == SLIDE_TO_RIGHT) ? STEER_SLIDELEFT_CCR : STEER_SLIDERIGHT_CCR;

		  if (slide_mode == SLIDE_TO_RIGHT)
		  {
			// Coming back from right -> steer left, so RIGHT is outer now
			int outer = (int)(SLIDE_OUTER_PWM);
			int inner = (int)(SLIDE_INNER_PWM);
			drive_right_forward(outer);
			drive_left_forward(inner);
		  }
		  else
		  { // SLIDE_TO_LEFT
			// Coming back from left -> steer right, so LEFT is outer now
			int outer = (int)(SLIDE_OUTER_PWM);
			int inner = (int)(SLIDE_INNER_PWM);
			drive_left_forward(outer);
			drive_right_forward(inner);
		  }
		  arc_target_angle = slide_origin_heading;

		  if ((slide_mode == SLIDE_TO_RIGHT && total_angle >= arc_target_angle - 5.0) || // slide_return_heading
			  (slide_mode == SLIDE_TO_LEFT && total_angle <= arc_target_angle + 5.0))
		  {
			const slide_side_t completed_slide = slide_mode;
			// Resume straight-line heading control.
			slide_mode = SLIDE_NO_SIDE;
			slide_phase = SLIDE_IDLE; // complete sliding

			arc_target_angle = slide_origin_heading; // restore straight reference
			htim12.Instance->CCR2 = servo;           // normal correction resumes

			if (completed_slide == SLIDE_TO_RIGHT)
			{
			  center_after_right();
			}


			if (completed_slide == SLIDE_TO_LEFT)
			{
			  center_after_left();
			}
			// braking logic, exit command
			brake_wheels();
			obstacle_stop_mode = STOP_MODE_NONE; // clear mode
			i_acc = 0;
			prev_err = 0; // reset PID state
			uart_cmd = DRIVE_IDLE;
			next_start_tick = HAL_GetTick() + STEP_GAP_MS;
			//
		  }

		}
		else
		{
		  // safety fallback
		  htim12.Instance->CCR2 = servo;
		}
	  }
	  else
	  { // no servoslide mode
		// no slide: normal straight correction
		htim12.Instance->CCR2 = servo;
		//htim12.Instance->CCR2 = STEER_CENTER_CCR; luther

	  }

	  //			htim12.Instance->CCR2 = servo;

	  if (uart_cmd == DRIVE_FORWARD && slide_mode == SLIDE_NO_SIDE)
	  {
		// Left motor (TIM4: CH3=IN2, CH4=IN1)
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, lDuty);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF);
		// Right motor (TIM9: CH1=IN2, CH2=IN1)
	  //  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, rDuty * 0.94); // 0.930
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, rDuty);

		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_OFF);
	  }
	  else if (uart_cmd == DRIVE_REVERSE)
	  { // DRIVE_REVERSE
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, lDuty);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_OFF);
	   // __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, rDuty * 1.025); // 0.940
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, rDuty);

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
			abort_now = 1U;
			break;
		}

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

		turn_error_deg = signed_error;

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

		if (remaining <= brake_margin)
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

		/* Conditional integration prevents wind-up while output is saturated. */
		if (effort < ARC_OUTER_EFFORT_MAX)
			arc_integral_deg_s = candidate_i;

		if (effort > ARC_OUTER_EFFORT_MAX)
			effort = ARC_OUTER_EFFORT_MAX;
		if (effort < ARC_OUTER_EFFORT_MIN)
			effort = ARC_OUTER_EFFORT_MIN;

		const float inner_ratio = turn_steers_right(command)
								  ? ARC_INNER_EFFORT_RATIO_R
								  : ARC_INNER_EFFORT_RATIO_L;

		const int outer_pwm = effort_to_pwm(effort);
		const int inner_pwm = effort_to_pwm(effort * inner_ratio);

		turn_effort_cmd = effort;
		apply_turn_drive(command, outer_pwm, inner_pwm);
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
					//		                ack_script();
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
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2); // Ultrasound echo capture.

  for (;;)
  {
	//HCSR04_Trigger();
    trigger_us_pulse();              // send trigger pulse
                                   //    	trigger_us_pulse();
    new_measurement_ready = false; // reset flag

    // wait until ISR sets the flag
    while (!new_measurement_ready)
    {
      osDelay(1);
    }

    // here, 'distance' has been updated in the callback
    osDelay(60); // Interval before the next trigger.
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
  // EMA smoothing for display & threshold stability
  const float alpha = 0.20f; // 0..1; higher = faster
  float v0 = 0.0f, v1 = 0.0f;

  // IR return a raw value that i convert to voltage
  // above voltage threshold = obstacle detected
  for (;;)
  {
    // Read latest DMA samples (16-bit reads are atomic on M4)
    uint16_t r0 = ir_adc[0];
    uint16_t r1 = ir_adc[1];

    // Convert to volts
    float f0 = 3.3f * (float)r0 / 4095.0f;
    float f1 = 3.3f * (float)r1 / 4095.0f;

    // Smooth
    v0 = (1.0f - alpha) * v0 + alpha * f0;
    v1 = (1.0f - alpha) * v1 + alpha * f1;

    // Publish for OLED (volatile globals)
    ir0_raw = r0;
    ir1_raw = r1;
    ir0_v = v0;
    ir1_v = v1;

    // Calculate real-time distance using the curve-fit formula
    ir0_distance_cm = Voltage_To_Distance_CM(v0);
    ir1_distance_cm = Voltage_To_Distance_CM(v1);

    // Simple obstacle flags (tune IR_VOLT_OBS)
    ir0_obs = (v0 >= IR_VOLT_OBS);
    ir1_obs = (v1 >= IR_VOLT_OBS);

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
