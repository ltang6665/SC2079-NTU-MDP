#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* Manual-control safety/protocol limits.
 * Motor PWM, wheel scaling, steering ratios, and other motion tuning stay in
 * the STM32 motion-control code; the gamepad sends only operator intent.
 */
#define MANUAL_WATCHDOG_MS       300U
#define MANUAL_TRANSITION_MS     120U
#define MANUAL_MAX_THROTTLE      100
#define MANUAL_LINE_SIZE         96U

typedef struct {
    int direction;        /* -1 reverse, 0 brake, +1 forward */
    int steering;         /* -100 left .. +100 right */
    int throttle_percent; /* 0..100 operator demand, not PWM/effort */
} ManualSetpoint;

typedef enum { MANUAL_BEGIN, MANUAL_FRAME } ManualMessageType;
typedef struct {
    ManualMessageType type;
    uint32_t session;
    uint32_t sequence;
    ManualSetpoint setpoint;
} ManualMessage;

/* Owned by motorTask only. The UART ISR passes lines through a separate mailbox. */
typedef struct {
    bool owns_motors;
    bool locked;
    bool have_sequence;
    bool neutral_seen;
    uint32_t session;
    uint32_t last_sequence;
    uint32_t last_update_ms;
    uint32_t drive_after_ms;
    ManualSetpoint requested;
} ManualControl;

bool Manual_ParseLine(const char *line, ManualMessage *message);
void Manual_Init(ManualControl *control);
void Manual_Begin(ManualControl *control, uint32_t session, uint32_t now_ms);
/* Returns true only for a newly expired lease. Expiry latches until a new BEGIN. */
bool Manual_Tick(ManualControl *control, uint32_t now_ms);
bool Manual_Accept(ManualControl *control, const ManualMessage *message, uint32_t now_ms);
ManualSetpoint Manual_Output(const ManualControl *control, uint32_t now_ms);

#endif
