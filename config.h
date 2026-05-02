#pragma once

/*
 * Main feature flags
 *
 * This file controls the glove's runtime behavior, boot calibration flow,
 * debugging output, visualization, and sensor backend selection.
 *
 * Recommended baseline for normal day-to-day use:
 * - Keep recognition enabled.
 * - Keep automatic software neutral capture enabled.
 * - Keep boot-time hardware offset measurement disabled unless you are doing a
 *   deliberate calibration session with the glove held still.
 * - Keep serial quaternion logging off unless you are tuning poses or debugging.
 * - Keep simultaneous drift reset enabled for finger sensors only.
 * - Keep wrist mouse emulation and thumb mouse emulation off unless you are
 *   actively using the glove as a mouse.
 * - Keep raw visualization and FIFO boot diagnostics off in production.
 *
 * Reading guide:
 * - Boolean feature flags use 0 = disabled, 1 = enabled.
 * - Repeated families such as per-sensor calibration flags are documented only
 *   at their first entry; later members follow the same rule.
 * - "Recommended" means a stable default for most builds, not a hard rule.
 */

#ifndef GAG_ENABLE_BLE_MOUSE
// BLE mouse output. Enable only when the build should send HID mouse events.
// Recommended: 1 for mouse-control builds, 0 for gesture-only builds.
// #define GAG_ENABLE_BLE_MOUSE GAG_HAVE_BLE_MOUSE
#define GAG_ENABLE_BLE_MOUSE 1
#endif
// Hand-role marker used by some split-hand or role-specific logic.
// Recommended: leave as 1 on the main/controller hand build.
#define MASTER_HAND 1

#ifndef GAG_ENABLE_VIBRATION
// Haptic feedback for recognized actions and status cues.
// Recommended: 1 if vibration hardware is present, otherwise 0.
#define GAG_ENABLE_VIBRATION 1
#endif

#ifndef GAG_AUTO_CAPTURE_SW_NEUTRAL
// Captures the current boot pose as the software neutral reference.
// Recommended: 1 for quick startup on a consistently worn glove, 0 only when
// you need fixed, hand-tuned mounting corrections.
#define GAG_AUTO_CAPTURE_SW_NEUTRAL 1
#endif

#ifndef GAG_AUTO_CAPTURE_MINOR_ROTATION_FIX
// Measures minor per-sensor rotational alignment corrections at boot.
// Recommended: 1 while refining mounting compensation, 0 if this causes
// unwanted boot-time adjustment on your hardware.
#define GAG_AUTO_CAPTURE_MINOR_ROTATION_FIX 1
#endif

#ifndef GAG_APPLY_MINOR_ROTATION_OFFSET
// Applies the captured or predefined minor rotation corrections.
// Recommended: 1 once the offsets are trustworthy.
#define GAG_APPLY_MINOR_ROTATION_OFFSET 1
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_AT_BOOT
// Runs hardware bias estimation during boot while the glove is held still.
// Recommended: 0 for normal use; set to 1 only during explicit calibration.
#define GAG_MEASURE_HW_OFFSETS_AT_BOOT 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_THUMB
// Per-sensor participation in boot-time hardware calibration.
// Recommended: keep all of these at 0 for normal use; enable only the sensors
// you are currently calibrating in a controlled still pose.
#define GAG_MEASURE_HW_OFFSETS_SENSOR_THUMB 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_INDEX
#define GAG_MEASURE_HW_OFFSETS_SENSOR_INDEX 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_MIDDLE
#define GAG_MEASURE_HW_OFFSETS_SENSOR_MIDDLE 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_RING
#define GAG_MEASURE_HW_OFFSETS_SENSOR_RING 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_LITTLE
#define GAG_MEASURE_HW_OFFSETS_SENSOR_LITTLE 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_GY25
#define GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_GY25 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_MPU9250
#define GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_MPU9250 0
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_GY511
#define GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_GY511 0
#endif

#ifndef GAG_HW_CALIBRATION_REQUIRED_SAMPLES
// Boot-time hardware calibration window.
// Recommended: keep defaults unless calibration is too noisy or too slow.
#define GAG_HW_CALIBRATION_REQUIRED_SAMPLES 500
#endif

#ifndef GAG_HW_CALIBRATION_SAMPLE_DELAY_MS
#define GAG_HW_CALIBRATION_SAMPLE_DELAY_MS 100
#endif

#ifndef GAG_HW_CALIBRATION_APPLY_UNSTABLE
#define GAG_HW_CALIBRATION_APPLY_UNSTABLE 1
#endif

#ifndef GAG_TFT_ROTATION
// TFT orientation for the TTGO display.
// Recommended: keep 0 unless the physical mounting changed.
#define GAG_TFT_ROTATION 0
#endif

#ifndef GAG_ENABLE_FIFO_REPORT
// FIFO/backend diagnostics and probe logging.
// Recommended: keep these off in production; enable temporarily for sensor
// backend debugging or data-path validation.
#define GAG_ENABLE_FIFO_REPORT 0
#endif

#ifndef GAG_ENABLE_WRIST_MPU_PROBE_LOG
#define GAG_ENABLE_WRIST_MPU_PROBE_LOG 0
#endif

#ifndef GAG_ENABLE_LOGICAL_WRIST_MPU9250_MAG_SOURCE
#define GAG_ENABLE_LOGICAL_WRIST_MPU9250_MAG_SOURCE 0
#endif

#ifndef GAG_ENABLE_SERIAL_SENSOR_QUAT_LOG
// High-volume serial logging for quaternion inspection and gesture tuning.
// Recommended: 0 in normal use, 1 only while debugging or collecting samples.
#define GAG_ENABLE_SERIAL_SENSOR_QUAT_LOG 1
#endif

#ifndef GAG_ENABLE_RECOGNITION
// Core gesture recognizer.
// Recommended: 1 for almost all builds; disable only for raw sensor bring-up.
#define GAG_ENABLE_RECOGNITION 1
#endif

#ifndef GAG_ENABLE_RECOGNITION_DRIFT_RESET_OFFSET
// Lets recognition consume quaternions with drift-reset compensation applied.
// Recommended: 1 unless you are comparing raw vs corrected behavior.
#define GAG_ENABLE_RECOGNITION_DRIFT_RESET_OFFSET 1
#endif

#ifndef GAG_ENABLE_IMU_ONLY_MOUSE
// Mouse control derived directly from IMU motion rather than gesture actions.
// Recommended: 0 unless you are specifically tuning IMU-driven pointer control.
#define GAG_ENABLE_IMU_ONLY_MOUSE 0
#endif

#ifndef GAG_ENABLE_WRIST_MOUSE_EMULATION
// Enables continuous wrist-driven mouse movement logic.
// Recommended: 0 by default; enable only on dedicated mouse-control profiles.
#define GAG_ENABLE_WRIST_MOUSE_EMULATION 1
#endif

#ifndef GAG_SOFT_RESET_ON_WRIST_MOUSE_TOGGLE
// Re-centers orientation after toggling wrist mouse mode.
// Recommended: 0 by default; use 1 if toggling leaves the cursor biased.
#define GAG_SOFT_RESET_ON_WRIST_MOUSE_TOGGLE 1
#endif

#ifndef GAG_WRIST_MOUSE_TOGGLE_SOFT_RESET_DELAY_MS
#define GAG_WRIST_MOUSE_TOGGLE_SOFT_RESET_DELAY_MS 100UL
#endif

#ifndef GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_THUMB_MS
#define GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_THUMB_MS 350UL
#endif

#ifndef GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_INDEX_MS
#define GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_INDEX_MS 50UL
#endif

#ifndef GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_MIDDLE_MS
#define GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_MIDDLE_MS 50UL
#endif

#ifndef GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_RING_MS
#define GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_RING_MS 50UL
#endif

#ifndef GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_LITTLE_MS
#define GAG_GESTURE_FINGER_DRIFT_RESET_DELAY_LITTLE_MS 50UL
#endif

#ifndef GAG_DISABLE_VIBRATION_WHEN_BLE_SEND_OFF
// Suppresses haptics when BLE sending is disabled.
// Recommended: 1 to avoid misleading feedback when mouse output is inactive.
#define GAG_DISABLE_VIBRATION_WHEN_BLE_SEND_OFF 1
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET
// Background drift correction while the glove is still.
// Recommended: 1 for finger sensors, with wrist sensors kept disabled unless
// you have verified stable wrist behavior on your hardware.
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET 1
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_RELATIVE_WRIST
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_RELATIVE_WRIST 1
#endif

#ifndef GAG_ENABLE_FINGER_RELATIVE_DRIFT_RESET_OFFSET
#define GAG_ENABLE_FINGER_RELATIVE_DRIFT_RESET_OFFSET 1
#endif



#ifndef GAG_ENABLE_FINGER_RELATIVE_ROTATION_CONSTRAINT
// Constrains finger motion into the expected primary bend axis to reduce
// side-axis artifacts from sensor mounting and wrist coupling.
// Recommended: 1 unless it clips a legitimate gesture on your hardware.
#define GAG_ENABLE_FINGER_RELATIVE_ROTATION_CONSTRAINT 1
#endif

#ifndef GAG_FINGER_RELATIVE_CONSTRAINT_SECONDARY_LIMIT_DEG
// Finger relative-rotation constraint tuning.
// Recommended: keep defaults and change only after reviewing logged quaternions
// and confirmed false positives/false negatives.
#define GAG_FINGER_RELATIVE_CONSTRAINT_SECONDARY_LIMIT_DEG 10.0f
#endif

#ifndef GAG_FINGER_RELATIVE_CONSTRAINT_THUMB_SECONDARY_LIMIT_DEG
#define GAG_FINGER_RELATIVE_CONSTRAINT_THUMB_SECONDARY_LIMIT_DEG 12.0f
#endif

#ifndef GAG_FINGER_RELATIVE_CONSTRAINT_SNAP_DOMINANCE_RATIO
#define GAG_FINGER_RELATIVE_CONSTRAINT_SNAP_DOMINANCE_RATIO 0.60f
#endif

#ifndef GAG_FINGER_RELATIVE_CONSTRAINT_SNAP_PRIMARY_BELOW_DEG
#define GAG_FINGER_RELATIVE_CONSTRAINT_SNAP_PRIMARY_BELOW_DEG 8.0f
#endif

#ifndef GAG_FINGER_RELATIVE_CONSTRAINT_SIGN_UPDATE_DEG
#define GAG_FINGER_RELATIVE_CONSTRAINT_SIGN_UPDATE_DEG 4.0f
#endif


#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_THUMB
// Per-sensor participation in simultaneous drift reset.
// Recommended: fingers = 1, wrist sensors = 0 unless carefully validated.
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_THUMB 1
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_INDEX
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_INDEX 1
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_MIDDLE
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_MIDDLE 1
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_RING
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_RING 1
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_LITTLE
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_LITTLE 1
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_GY25
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_GY25 0
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_MPU9250
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_MPU9250 0
#endif  

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_GY511
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_GY511 0
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_SENSOR_MASK
// Shared timing/threshold tuning for simultaneous drift reset.
// Recommended: keep defaults until you have measured drift-reset behavior with
// real glove data.
#define GAG_SIMULTANEOUS_DRIFT_RESET_SENSOR_MASK 0xFFu
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_INTERVAL_MS
#define GAG_SIMULTANEOUS_DRIFT_RESET_INTERVAL_MS 5UL
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_STILL_MS
// #define GAG_SIMULTANEOUS_DRIFT_RESET_STILL_MS 150UL
#define GAG_SIMULTANEOUS_DRIFT_RESET_STILL_MS 350UL
#endif

#ifndef GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_GESTURE_PROGRESS_GUARD
#define GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_GESTURE_PROGRESS_GUARD 0
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_GESTURE_PROGRESS_MIN_QUATS
#define GAG_SIMULTANEOUS_DRIFT_RESET_GESTURE_PROGRESS_MIN_QUATS 2u
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_MOVEMENT_THRESHOLD_DEG
#define GAG_SIMULTANEOUS_DRIFT_RESET_MOVEMENT_THRESHOLD_DEG 4.00f
// #define GAG_SIMULTANEOUS_DRIFT_RESET_MOVEMENT_THRESHOLD_DEG 6.00f
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_RATE_PER_SEC
#define GAG_SIMULTANEOUS_DRIFT_RESET_RATE_PER_SEC 3.0f
// #define GAG_SIMULTANEOUS_DRIFT_RESET_RATE_PER_SEC 6.0f
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_DEADBAND_DEG
#define GAG_SIMULTANEOUS_DRIFT_RESET_DEADBAND_DEG 0.50f
#endif

#ifndef GAG_SIMULTANEOUS_DRIFT_RESET_MAX_CORRECTION_DEG
#define GAG_SIMULTANEOUS_DRIFT_RESET_MAX_CORRECTION_DEG 360.0f
#endif

#ifndef GAG_ENABLE_DELAYED_GESTURE_SOFT_RESET
// Gesture-triggered soft reset coordination.
// Recommended: keep enabled so post-gesture recentering happens after the
// gesture action instead of interfering with the pose match itself.
#define GAG_ENABLE_DELAYED_GESTURE_SOFT_RESET 1
#endif

#ifndef GAG_ENABLE_GESTURE_DRIFT_RESET_COOLDOWN
#define GAG_ENABLE_GESTURE_DRIFT_RESET_COOLDOWN 1
#endif

#ifndef GAG_DEFAULT_GESTURE_DRIFT_RESET_BLOCK_MS
#define GAG_DEFAULT_GESTURE_DRIFT_RESET_BLOCK_MS 160UL
#endif


#ifndef GAG_DEFAULT_GESTURE_SOFT_RESET_DELAY_MS
#define GAG_DEFAULT_GESTURE_SOFT_RESET_DELAY_MS 100UL
#endif

#ifndef GAG_MAX_PENDING_GESTURE_SOFT_RESETS
#define GAG_MAX_PENDING_GESTURE_SOFT_RESETS 8
#endif

#ifndef GAG_ENABLE_THUMB_MOUSE_EMULATION
// Thumb-driven continuous mouse control mode.
// Recommended: 0 by default; enable only in a dedicated mouse profile.
#define GAG_ENABLE_THUMB_MOUSE_EMULATION 0
#endif

#ifndef GAG_VIZ_HAND_RELATIVE_ROTATION
// Display visualization options.
// Recommended: hand relative = 1, cubes relative = 1 for intuitive debugging;
// raw cubes = 0 for production and 1 only when comparing corrected vs raw data.
#define GAG_VIZ_HAND_RELATIVE_ROTATION 1
#endif

#ifndef GAG_VIZ_CUBES_RELATIVE_ROTATION
// #define GAG_VIZ_CUBES_RELATIVE_ROTATION 0
#define GAG_VIZ_CUBES_RELATIVE_ROTATION 1
#endif

#ifndef GAG_ENABLE_RAW_CUBES_VISUALIZATION
#define GAG_ENABLE_RAW_CUBES_VISUALIZATION 1
#endif

#ifndef GAG_APPLY_GY511_WRIST_PIVOT_ROTATION_FIX
#define GAG_APPLY_GY511_WRIST_PIVOT_ROTATION_FIX 1
#endif

#ifndef GAG_SERIAL_SENSOR_QUAT_LOG_INTERVAL_MS
// Additional serial logging controls.
// Recommended: keep defaults and enable individual logs only during tuning.
#define GAG_SERIAL_SENSOR_QUAT_LOG_INTERVAL_MS 100
#endif

#ifndef GAG_ENABLE_SERIAL_SENSOR_QUAT_LOG_INDEX
#define GAG_ENABLE_SERIAL_SENSOR_QUAT_LOG_INDEX 1
#endif

#ifndef GAG_ENABLE_SERIAL_SENSOR_QUAT_LOG_LOGICAL_WRIST
#define GAG_ENABLE_SERIAL_SENSOR_QUAT_LOG_LOGICAL_WRIST 1
#endif

#ifndef GAG_PRINT_MINOR_ROTATION_OFFSET_CANDIDATES
#define GAG_PRINT_MINOR_ROTATION_OFFSET_CANDIDATES 0
#endif

#ifndef GAG_MINOR_ROTATION_OFFSET_PRINT_INTERVAL_MS
#define GAG_MINOR_ROTATION_OFFSET_PRINT_INTERVAL_MS 5000UL
#endif

#ifndef GAG_ENABLE_GY25_RUNTIME_BIAS_LOG
#define GAG_ENABLE_GY25_RUNTIME_BIAS_LOG 0
#endif

#ifndef GAG_TTGO_RIGHT_BUTTON_PIN
// TTGO button mappings and button-assisted capture/reset behavior.
// Recommended: keep pin defaults for the current TTGO wiring and only change
// them if the hardware layout or electrical polarity changes.
#define GAG_TTGO_RIGHT_BUTTON_PIN 0
#endif

#ifndef GAG_TTGO_BUTTON_ACTIVE_LOW
#define GAG_TTGO_BUTTON_ACTIVE_LOW 1
#endif

#ifndef GAG_TTGO_BUTTON_DEBOUNCE_MS
#define GAG_TTGO_BUTTON_DEBOUNCE_MS 250UL
#endif

#ifndef GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
#define GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE 0
#endif

#ifndef GAG_PAIR_CONFIRM_BUTTON_PIN
#define GAG_PAIR_CONFIRM_BUTTON_PIN 35
#endif

#ifndef GAG_PAIR_CONFIRM_BUTTON_ACTIVE_LOW
#define GAG_PAIR_CONFIRM_BUTTON_ACTIVE_LOW 1
#endif

#ifndef GAG_PAIR_CONFIRM_TIMEOUT_MS
#define GAG_PAIR_CONFIRM_TIMEOUT_MS 10000UL
#endif

// Periodic automatic soft reset. Useful for experiments, but it can interrupt
// normal usage if the interval is too aggressive.
// Recommended: 0 for normal use.
#define GAG_ENABLE_PERIODIC_SOFT_SENSOR_RESET 0
// #ifndef GAG_PERIODIC_SOFT_SENSOR_RESET
  #ifdef GAG_ENABLE_PERIODIC_SOFT_SENSOR_RESET
    // #define GAG_PERIODIC_SOFT_SENSOR_RESET GAG_ENABLE_PERIODIC_SOFT_SENSOR_RESET
    #define GAG_PERIODIC_SOFT_SENSOR_RESET 0
  #else
    #define GAG_PERIODIC_SOFT_SENSOR_RESET 0
  #endif
// #endif

#ifndef GAG_PERIODIC_SOFT_SENSOR_RESET_INTERVAL_MS
// #define GAG_PERIODIC_SOFT_SENSOR_RESET_INTERVAL_MS 300000UL
// #define GAG_PERIODIC_SOFT_SENSOR_RESET_INTERVAL_MS 15000UL
#define GAG_PERIODIC_SOFT_SENSOR_RESET_INTERVAL_MS 3000000UL
#endif

#ifndef GAG_USE_MPU_DMP_QUAT_FIFO
// MPU FIFO / DMP backend selection and tuning.
// Recommended: keep the current proven backend settings unless you are actively
// profiling FIFO stability, latency, or overflow behavior on the target board.
#define GAG_USE_MPU_DMP_QUAT_FIFO 0
#endif

#ifndef GAG_ENABLE_MPU_FIFO
#define GAG_ENABLE_MPU_FIFO 1
#endif

#ifndef GAG_ENABLE_MPU9250_FIFO
#define GAG_ENABLE_MPU9250_FIFO 1
#endif

#ifndef GAG_FIFO_RESET_INTERVAL_MS
#define GAG_FIFO_RESET_INTERVAL_MS 50
#endif

#ifndef GAG_MPU_FIFO_DLPF_CFG
#define GAG_MPU_FIFO_DLPF_CFG 0x04u
#endif

#ifndef GAG_MPU_FIFO_SMPLRT_DIV
#define GAG_MPU_FIFO_SMPLRT_DIV 9u
#endif

#ifndef GAG_ENABLE_FIFO_BOOT_TEST
#define GAG_ENABLE_FIFO_BOOT_TEST 1
#endif

#ifndef GAG_MPU6050_FIFO_MAX_BYTES
#define GAG_MPU6050_FIFO_MAX_BYTES 1024U
#endif

#ifndef GAG_MPU9250_FIFO_MAX_BYTES
#define GAG_MPU9250_FIFO_MAX_BYTES 512U
#endif

#define GAG_PRIMARY_WRIST_SENSOR_GY25 0
#define GAG_PRIMARY_WRIST_SENSOR_MPU9250 1
#define GAG_PRIMARY_WRIST_SENSOR_GY511 2

#ifndef GAG_PRIMARY_WRIST_SENSOR
// Selects the logical wrist source used by recognition and the main hand model.
// Recommended: GY25 if it is your most stable wrist orientation source; switch
// to MPU9250 or GY511 only after validating their runtime behavior.
#define GAG_PRIMARY_WRIST_SENSOR GAG_PRIMARY_WRIST_SENSOR_GY25
#endif
