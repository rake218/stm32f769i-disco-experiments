## Experiment: CPU Load Measurement (Idle Hook)

### Objective
Measure CPU utilization without debugger or external tools.

### Method
Used FreeRTOS idle hook to count idle loop executions over a fixed
1-second window. CPU load computed as inverse of idle time.

### Calibration
System was run with only the FreeRTOS idle task active.

Measured value:
IDLE_MAX_COUNT = 1,210,322 (STM32F769 Discovery)

This represents 100% idle CPU time.

### Experiments Performed
- Added high-priority CPU-intensive task (busy loop) (HPTask)
- Observed CPU load increase to ~90%
- Increased osDelay in task
- Observed CPU load drop to ~46%

### Observations
- CPU load depends on execution time, not task priority
- Blocking tasks contribute negligible CPU load
- Idle-hook method reacts correctly to workload changes

### Conclusion
Idle hook based CPU load estimation is effective for coarse system-level
CPU utilization measurement and system health monitoring.
