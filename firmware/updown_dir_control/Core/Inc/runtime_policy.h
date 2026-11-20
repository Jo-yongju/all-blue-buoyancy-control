#ifndef RUNTIME_POLICY_H
#define RUNTIME_POLICY_H
/* Initial scheduling policy, not proven worst-case performance.
 * Rationale and validation gates: docs/RTOS_DESIGN_KO.txt. */
#define CONTROL_PERIOD_MS       10U
#define TELEMETRY_PERIOD_MS     200U
#define CONSOLE_RX_BUDGET       32U
#define CONSOLE_TX_BUDGET       16U
#define SENSOR_TOTAL_BUDGET_MS  4U
#define CONTROL_PRIORITY       4U
#define RECEIVER_PRIORITY      3U
#define IMU_PRIORITY           2U
#define CONSOLE_PRIORITY       1U
#define RECEIVER_BYTE_BUDGET    64U
#define IMU_REPORT_US          2500U
#define IMU_PACKET_BUDGET       4U
#define IMU_EVENT_CHECK_MS      10U
#define INPUT_BACKLOG_YIELD_MS  1U
#define CONSOLE_SERVICE_MS     1U
#endif
