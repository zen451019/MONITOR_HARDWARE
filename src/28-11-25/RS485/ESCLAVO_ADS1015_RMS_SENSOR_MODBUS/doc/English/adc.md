@file adc.md
@defgroup group_adc_rms ADC Processing and RMS Calculation

@section group_adc_rms_overview Overview
This module implements analog signal acquisition via ADS1015 and the complete flow of:
- ADC sample acquisition in round-robin mode between 3 channels using interrupts.
- Real-time processing with circular FIFOs for efficient RMS calculation.
- RMS value history storage in thread-safe buffers.
- Application of channel-specific conversion factors for individual calibration.

@section Structure_and_components Structure and Components

- Data and configuration
  - ADC_Sample: sample structure with value and associated channel.
  - RMS_FIFO: circular buffer with cumulative sums for incremental calculation.
  - CONVERSION_FACTORS[]: channel-specific calibration factors.

- Acquisition and processing
  - queue_adc_samples: FreeRTOS queue for acquisition → processing transfer.
  - fifos[NUM_CHANNELS]: array of circular FIFOs for each channel.
  - rms_history_ch0/ch1/ch2: independent histories of RMS values.

- Main tasks
  - `task_adquisicion`: ADC conversion management with improved multiplexer.
  - `task_procesamiento`: incremental RMS calculation and history update.
  - ISR: `on_adc_data_ready()` for conversion completion notification.

@section group_adc_rms_api Main API
- Configuration: `FIFO_SIZE`, `PROCESS_INTERVAL_MS`, `RMS_HISTORY_SIZE`, `CONVERSION_FACTORS[]`.
- Structures: `ADC_Sample`, `RMS_FIFO`.
- Data: `queue_adc_samples`, `fifos[]`, `rms_history_ch0/ch1/ch2`, `rms_history_mutex`.
- Functions: `task_adquisicion()`, `task_procesamiento()`, `get_rms_history()`, `on_adc_data_ready()`.

@section group_adc_rms_usage Usage
Flow summary:

1) ADC initialization
   - ADS1015 configuration: TWOTHIRDS gain (±6.144V), 3300 SPS speed.
   - Interrupt configuration on ALERT pin for completed conversions.
   - Creation of `queue_adc_samples` queue with FIFO_SIZE capacity.
   - Initialization of `rms_history_mutex` mutex for history protection.

2) Sample acquisition
   - `task_adquisicion` executes on core 0 with priority 5.
   - Round-robin sequence between channels 0, 1, 2 using explicit multiplexer.
   - Direct mapping: channel → MUX configuration (SINGLE_0/SINGLE_1/SINGLE_2).
   - ISR `on_adc_data_ready()` notifies conversion completion via `adc_data_ready` flag.
   - Samples queued in `queue_adc_samples` with ADC_Sample structure.

3) RMS processing
   - `task_procesamiento` consumes samples from queue on core 0 with priority 3.
   - Circular FIFO update with incremental algorithm:
     - Sum maintenance: sum_x (∑x) and sum_x2 (∑x²).
     - Circular buffer with automatic replacement upon reaching FIFO_SIZE=320.
     - Mean calculation: μ = sum_x / count.
     - Variance calculation: σ² = (sum_x2 / count) - μ².
     - RMS value: √σ² with negative variance protection.

4) History management
   - RMS calculation every PROCESS_INTERVAL_MS=1000ms.
   - Storage in independent circular buffers per channel.
   - Thread-safe protection with `rms_history_mutex`.
   - `get_rms_history()` function for extracting last N values.

5) Channel calibration
   - `CONVERSION_FACTORS[]` array with specific factors:
     - Channel 0: 0.653f
     - Channel 1: 0.679f  
     - Channel 2: 1.133f
   - Applied in `dataUpdateTask` to compensate sensor differences.
   - Allows individual adjustment of measurement ranges and conditioning.

6) Incremental RMS algorithm
   - O(1) efficiency for new sample insertion.
   - No need to traverse entire buffer for calculation.
   - Automatic maintenance of cumulative statistics.
   - Robust overflow management in circular buffer.

Warnings and best practices:
- Verify ADS1015 power supply stability for accurate conversions.
- Configure external pullups on I2C lines if communication issues occur.
- Validate ALERT pin connection before enabling interrupts.
- Protect RMS history access with appropriate mutex timeout.
- Calibrate CONVERSION_FACTORS[] according to actual sensor characteristics.
- Monitor processing latency to avoid queue saturation.
- Verify that FIFO_SIZE is sufficient for required analysis window.