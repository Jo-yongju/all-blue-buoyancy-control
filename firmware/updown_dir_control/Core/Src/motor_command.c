#include "motor_command.h"
#include <ctype.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

MotorCommand MotorCommand_Parse(const char *line)
{
    MotorCommand command = {0};
    command.type = CMD_INVALID;
    if (line == NULL) {
        return command;
    }

    if (strcmp(line, "STOP") == 0) {
        command.type = CMD_STOP;
    } else if (strcmp(line, "ARM") == 0) {
        command.type = CMD_ARM;
    } else if (strcmp(line, "STATUS") == 0) {
        command.type = CMD_STATUS;
    } else if (strcmp(line, "SENSOR") == 0) {
        command.type = CMD_SENSOR;
    } else if (strcmp(line, "HELP") == 0) {
        command.type = CMD_HELP;
    } else if (strcmp(line, "CAL") == 0) {
        command.type = CMD_CAL;
    } else if (strcmp(line, "STATS") == 0) {
        command.type = CMD_STATS;
    } else if (strcmp(line, "TRACE ON") == 0) {
        command.type = CMD_TRACE_ON;
    } else if (strcmp(line, "TRACE OFF") == 0) {
        command.type = CMD_TRACE_OFF;
    } else if (strncmp(line, "MOVE ", 5) == 0) {
        char *end;
        errno = 0;
        float target = strtof(line + 5, &end);
        if (errno || end == line + 5 || !isfinite(target)) return command;
        while (isspace((unsigned char)*end)) ++end;
        if (*end != '\0' || target < 0 || target > 1000) return command;
        command.type = CMD_MOVE;
        command.target_mm = target;
    } else if (strncmp(line, "JOG ", 4) == 0) {
        /* Format: JOG signed_PWM_percent duration_in_milliseconds */
        char *end;
        errno = 0;
        long duty = strtol(line + 4, &end, 10);
        if (errno || end == line + 4 || !isspace((unsigned char)*end) ||
            duty == 0 || duty < -100 || duty > 100) {
            return command;
        }

        const char *duration_start = end;
        errno = 0;
        long duration = strtol(duration_start, &end, 10);
        if (errno || end == duration_start || duration < (long)MOTOR_JOG_MIN_MS ||
            duration > (long)MOTOR_JOG_MAX_MS) {
            return command;
        }

        while (isspace((unsigned char)*end)) {
            ++end;
        }
        if (*end != '\0') {
            return command;
        }

        command.type = CMD_JOG;
        command.percent = (int)duty;
        command.duration_ms = (unsigned)duration;
    }
    return command;
}
