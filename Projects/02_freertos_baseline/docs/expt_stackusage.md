## Experiment: Stack Usage and Overflow Detection

### Objective
Measure task stack usage and validate stack overflow detection.

### RTOS Diagnostics Configuration

The following FreeRTOS diagnostics are enabled for learning and validation:

- configUSE_TRACE_FACILITY = 1  
- INCLUDE_uxTaskGetStackHighWaterMark = 1  

This allows:
- Per-task stack usage measurement
- Validation of stack sizing decisions
- Early detection of stack overflows

### Method
- Enabled FreeRTOS stack overflow checking (mode 2)
- Measured stack high-water mark per task
- Deliberately caused stack overflow using local array

### Observation
- Stack usage varies per task
- Overflow is detected reliably
- System halts safely via hook without undefined behavior

### Conclusion
Each FreeRTOS task must be sized individually.
Stack overflow detection is mandatory in RTOS systems.
