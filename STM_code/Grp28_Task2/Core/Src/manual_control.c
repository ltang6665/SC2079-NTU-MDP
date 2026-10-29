#include "manual_control.h"
#include <stddef.h>
#include <string.h>

static const ManualSetpoint neutral_output = {0, 0, 0};

static int value_sign(int value)
{
    return (value > 0) - (value < 0);
}

/* Parse digits with a limit so malformed input cannot overflow an integer. */
static void skip_blanks(const char **scan)
{
    while (**scan == ' ' || **scan == '\t') ++*scan;
}

static bool parse_unsigned(const char **scan, uint32_t limit, uint32_t *number)
{
    const char *next_char = *scan;
    uint32_t value = 0U;
    skip_blanks(&next_char);
    if (*next_char < '0' || *next_char > '9') return false;
    do {
        uint32_t next_digit = (uint32_t)(*next_char - '0');
        if (next_digit > limit || value > (limit - next_digit) / 10U) return false;
        value = value * 10U + next_digit;
        ++next_char;
    } while (*next_char >= '0' && *next_char <= '9');
    if (*next_char != '\0' && *next_char != ' ' && *next_char != '\t') return false;
    *scan = next_char;
    *number = value;
    return true;
}

static bool parse_signed(const char **scan, int limit, int *number)
{
    uint32_t absolute_value;
    int sign = 1;
    skip_blanks(scan);
    if (**scan == '-') { sign = -1; ++*scan; }
    if (**scan < '0' || **scan > '9') return false;
    if (!parse_unsigned(scan, (uint32_t)limit, &absolute_value)) return false;
    *number = sign * (int)absolute_value;
    return true;
}

static bool setpoint_valid(ManualSetpoint request)
{
    if (request.direction < -1 || request.direction > 1 ||
        request.steering < -100 || request.steering > 100 ||
        request.throttle_percent < 0 || request.throttle_percent > MANUAL_MAX_THROTTLE)
        return false;

    if (request.direction == 0)
        return request.steering == 0 && request.throttle_percent == 0;

    return request.throttle_percent > 0;
}

bool Manual_ParseLine(const char *input, ManualMessage *packet)
{
    ManualMessage decoded = {0};
    const char *next_char;
    uint32_t throttle_value;

    if (input == NULL || packet == NULL) return false;

    if (strncmp(input, "jbegin ", 7U) == 0) {
        next_char = input + 7;
        decoded.type = MANUAL_BEGIN;
        if (!parse_unsigned(&next_char, UINT32_MAX, &decoded.session) || decoded.session == 0U)
            return false;
    } else if (strncmp(input, "j ", 2U) == 0) {
        next_char = input + 2;
        decoded.type = MANUAL_FRAME;
        if (!parse_unsigned(&next_char, UINT32_MAX, &decoded.session) || decoded.session == 0U ||
            !parse_unsigned(&next_char, UINT32_MAX, &decoded.sequence) || decoded.sequence == 0U ||
            !parse_signed(&next_char, 1, &decoded.setpoint.direction) ||
            !parse_signed(&next_char, 100, &decoded.setpoint.steering) ||
            !parse_unsigned(&next_char, MANUAL_MAX_THROTTLE, &throttle_value))
            return false;

        decoded.setpoint.throttle_percent = (int)throttle_value;
        if (!setpoint_valid(decoded.setpoint)) return false;
    } else {
        return false;
    }

    skip_blanks(&next_char);
    if (*next_char != '\0') return false;
    *packet = decoded;
    return true;
}

void Manual_Init(ManualControl *state)
{
    memset(state, 0, sizeof(*state));
}

void Manual_Begin(ManualControl *state, uint32_t session, uint32_t tick_ms)
{
    Manual_Init(state);
    state->owns_motors = true;
    state->session = session;
    state->last_update_ms = tick_ms;
    state->drive_after_ms = tick_ms;
}

bool Manual_Tick(ManualControl *state, uint32_t tick_ms)
{
    if (state->owns_motors && !state->locked &&
        (uint32_t)(tick_ms - state->last_update_ms) >= MANUAL_WATCHDOG_MS) {
        state->locked = true;
        state->requested = neutral_output;
        return true;
    }
    return false;
}

bool Manual_Accept(ManualControl *state, const ManualMessage *packet, uint32_t tick_ms)
{
    uint32_t sequence_step;

    (void)Manual_Tick(state, tick_ms);
    if (!state->owns_motors || state->locked ||
        packet->type != MANUAL_FRAME ||
        packet->session != state->session ||
        packet->sequence == 0U ||
        !setpoint_valid(packet->setpoint))
        return false;

    sequence_step = packet->sequence - state->last_sequence;
    if (state->have_sequence && (sequence_step == 0U || sequence_step >= 0x80000000U))
        return false;
    if (!state->neutral_seen && packet->setpoint.direction != 0)
        return false;

    if (packet->setpoint.direction != 0 &&
        (packet->setpoint.direction != state->requested.direction ||
         value_sign(packet->setpoint.steering) != value_sign(state->requested.steering)))
        state->drive_after_ms = tick_ms + MANUAL_TRANSITION_MS;

    state->requested = packet->setpoint;
    state->last_sequence = packet->sequence;
    state->have_sequence = true;
    state->neutral_seen = true;
    state->last_update_ms = tick_ms;
    return true;
}

ManualSetpoint Manual_Output(const ManualControl *state, uint32_t tick_ms)
{
    ManualSetpoint drive_request;

    if (!state->owns_motors || state->locked ||
        (uint32_t)(tick_ms - state->last_update_ms) >= MANUAL_WATCHDOG_MS)
        return neutral_output;

    drive_request = state->requested;
    if (drive_request.direction != 0 && (int32_t)(tick_ms - state->drive_after_ms) < 0) {
        drive_request.direction = 0;
        drive_request.throttle_percent = 0;
        /* Hold the requested steering during braking and the servo settling delay. */
    }
    return drive_request;
}
