#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#define MOTOR_JOG_MIN_MS 20U
#define MOTOR_JOG_MAX_MS 20000U

typedef enum {
    CMD_INVALID,
    CMD_STOP,
    CMD_ARM,
    CMD_JOG,
    CMD_STATUS,
    CMD_HELP,
    CMD_SENSOR,
    CMD_MOVE,
    CMD_CAL,
    CMD_STATS,
    CMD_TRACE_ON,
    CMD_TRACE_OFF
} MotorCommandType;

typedef struct {
    MotorCommandType type;
    int percent;          /* Signed PWM duty: negative reverses direction. */
    float target_mm;
    unsigned duration_ms;
} MotorCommand;

MotorCommand MotorCommand_Parse(const char *line);
#endif
