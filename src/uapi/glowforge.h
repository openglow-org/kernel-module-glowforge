/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * glowforge.h
 *
 * Userspace API to the Glowforge kernel module.
 *
 * Copyright (C) 2015-2021 Glowforge, Inc. <opensource@glowforge.com>
 * Written by Matt Sarnoff with contributions from Taylor Vaughn.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef KERNEL_UAPI_GLOWFORGE_H_
#define KERNEL_UAPI_GLOWFORGE_H_

#define STR2(x) #x
#define STR(x)  STR2(x)

#define ROOT_KOBJ_NAME            "glowforge"
#define CNC_GROUP_NAME            "cnc"
#define PIC_GROUP_NAME            "pic"
#define THERMAL_GROUP_NAME        "thermal"
#define HEAD_GROUP_NAME           "head"

#define ROOT_CLASS_NAME           "class"
#define LED_GROUP_NAME            "leds"

#define PULSE_DEVICE_NAME         "glowforge"
#define PULSE_DEVICE_DIR          "/dev/"
#define PULSE_DEVICE_PATH         PULSE_DEVICE_DIR PULSE_DEVICE_NAME

#define ATTR_STATE                state
#define ATTR_FAULTS               faults
#define ATTR_IGNORED_FAULTS       ignored_faults
#define ATTR_STEP_FREQ            step_freq
#define ATTR_RAMP_RATE            ramp_rate
#define ATTR_RUN                  run
#define ATTR_STOP                 stop
#define ATTR_HALT                 halt
#define ATTR_RESUME               resume
#define ATTR_DISABLE              disable
#define ATTR_ENABLE               enable
#define ATTR_Z_STEP               z_step
#define ATTR_LASER_LATCH          laser_latch
#define ATTR_LASER_ENABLE          laser_enable
#define ATTR_LASER_ON              laser_on
#define ATTR_LASER_ON_SAMPLED      laser_on_sampled
#define ATTR_LASER_PGOOD           laser_pgood
#define ATTR_LASER_PGOOD_SAMPLED   laser_pgood_sampled
#define ATTR_INTERLOCK_CIRCUIT     interlock_circuit
#define ATTR_INTERLOCK_LATCH_RESET interlock_latch_reset
#define ATTR_BUTTON_LATCH          button_latch
#define ATTR_CHARGE_PUMP_ALIVE     charge_pump_alive
#define ATTR_MOTOR_LOCK           motor_lock
#define ATTR_POSITION             position
#define ATTR_FREE                 free
#define ATTR_MAX_BACKTRACK        max_backtrack
#define ATTR_STREAMING            streaming
#define ATTR_UNDERRUNS            underruns
#define ATTR_SDMA_CONTEXT         sdma_context
#define ATTR_X_MODE               x_mode
#define ATTR_X_DECAY              x_decay
#define ATTR_Y_MODE               y_mode
#define ATTR_Y_DECAY              y_decay

#define ATTR_PIC_ID               id
#define ATTR_WATER_TEMP_1         water_temp_1
#define ATTR_WATER_TEMP_2         water_temp_2
#define ATTR_TEC_TEMP             tec_temp
#define ATTR_PWR_TEMP             pwr_temp
#define ATTR_LID_IR_1             lid_ir_1
#define ATTR_LID_IR_2             lid_ir_2
#define ATTR_LID_IR_3             lid_ir_3
#define ATTR_LID_IR_4             lid_ir_4
#define ATTR_HV_CURRENT           hv_current
#define ATTR_HV_VOLTAGE           hv_voltage
#define ATTR_X_CURRENT            x_step_current
#define ATTR_Y_CURRENT            y_step_current
#define ATTR_LID_LED              lid_led
#define ATTR_BUTTON_LED_1         button_led_1
#define ATTR_BUTTON_LED_2         button_led_2
#define ATTR_BUTTON_LED_3         button_led_3
#define ATTR_PIC_ALL              grp_all
#define ATTR_PIC_SENSORS          grp_sensors
#define ATTR_PIC_OUTPUTS          grp_outputs
#define ATTR_PIC_BUTTON_LEDS      grp_button_leds
#define ATTR_PIC_HV               grp_hv
#define ATTR_PIC_RAW              raw
#define ATTR_PIC_HEX              hex

#define ATTR_INTAKE1_TACH         tach_intake_1
#define ATTR_INTAKE2_TACH         tach_intake_2
#define ATTR_EXHAUST_TACH         tach_exhaust
#define ATTR_INTAKE_PWM           intake_pwm
#define ATTR_EXHAUST_PWM          exhaust_pwm
#define ATTR_HEATER_PWM           heater_pwm
#define ATTR_WATER_PUMP_ON        water_pump_on
#define ATTR_TEC_ON               tec_on
#define ATTR_WATER_TEMP_1         water_temp_1
#define ATTR_WATER_TEMP_2         water_temp_2

#define ATTR_INFO                 info
#define ATTR_Z_ENABLE             z_enable
#define ATTR_Z_CURRENT            z_current
#define ATTR_Z_MODE               z_mode
#define ATTR_PURGE_AIR            purge_air
#define ATTR_PURGE_CURRENT        purge_air_current
#define ATTR_HALL_SENSOR          hall_sensor
#define ATTR_ACCEL_IRQ            accel_irq
#define ATTR_BEAM_DETECT_DIGITAL  beam_detect_digital
#define ATTR_BEAM_DETECT_ANALOG   beam_detect_analog
#define ATTR_AIR_ASSIST_PWM       air_assist_pwm
#define ATTR_AIR_ASSIST_TACH      air_assist_tach
#define ATTR_WHITE_LED            white_led
#define ATTR_UV_LED               uv_led
#define ATTR_MEASURE_LASER        measure_laser

/**
 * Expected value of PIC register 0.
 */
#define PIC_MAGIC_NUMBER  0x4d53

/**
 * Driver states.
 * Uses an X-macro that must be defined before use.
 */
#define DRIVER_STATES \
  X(STATE_IDLE, "idle")         /** Steppers are on but no cut is in progress */ \
  X(STATE_RUNNING, "running")   /** A cut is in progress */ \
  X(STATE_DISABLED, "disabled") /** Steppers were explicitly disabled (for debugging) */ \
  X(STATE_FAULT, "fault")     /** Stepper driver fault */ \
  X(STATE_UNDERRUN, "underrun") /** Streaming feeder let the pulse buffer run dry mid-run; \
                                    position may be desynced (steps skipped at speed). \
                                    Only entered when the "streaming" attr is 1. \
                                    Acknowledge by writing "1" to the stop attr. */
#undef X
#define X(e,s) e,
enum cnc_state {
  DRIVER_STATES
};

/** Axis constants */
enum cnc_axis {
  AXIS_X,
  AXIS_Y,
  NUM_AXES
};

/** Microstepping modes */
enum cnc_microstep_mode {
  MODE_FULL_STEP     = 1,
  MODE_MICROSTEPS_2  = 2,
  MODE_MICROSTEPS_4  = 4,
  MODE_MICROSTEPS_8  = 8,
  MODE_MICROSTEPS_16 = 16,
  MODE_MICROSTEPS_32 = 32,
};

/** Decay modes */
enum cnc_decay_mode {
  MODE_DECAY_SLOW,
  MODE_DECAY_MIXED,
  MODE_DECAY_FAST
};

/** Bits in the triggered_faults field */
enum fault_condition {
  FAULT_X,
  FAULT_Y1,
  FAULT_Y2,
  NUM_FAULT_CONDITIONS
};

/** Bits in the motor_lock field */
enum motor_lock_options {
  MOTOR_LOCK_X  = 1 << 0,
  MOTOR_LOCK_Y1 = 1 << 1,
  MOTOR_LOCK_Y2 = 1 << 2,
  MOTOR_LOCK_Z  = 1 << 3,
};

/** Struct returned by the `position` sysfs attribute */
struct cnc_position {
  int32_t x_step_pos;
  int32_t y_step_pos;
  int32_t z_step_pos;
  uint32_t bytes_processed;
  uint32_t bytes_total;
  int32_t reserved[3];
} __attribute__((packed));

/** Special lseek() offsets for /dev/glowforge */
enum cnc_lseek_options {
  PULSEDEV_LSEEK_CLEAR_DATA_AND_POSITION,
  PULSEDEV_LSEEK_CLEAR_DATA,
  PULSEDEV_LSEEK_CLEAR_POSITION,
  PULSEDEV_LSEEK_MAX_VALID_OFFSET = PULSEDEV_LSEEK_CLEAR_POSITION
};

/**
 * These match the register names used in the PIC source code,
 * which match the signal names on the schematic.
 * The sysfs attribute names are a little more human-readable.
 */
#define ENUM_LAST_MARKER(e) _XX##e, e = _XX##e-1
enum pic_register
{
  PIC_FIRST_REGISTER = 0,
  PIC_FIRST_SENSOR_REGISTER = PIC_FIRST_REGISTER,

  PIC_REG_DUMMY = PIC_FIRST_SENSOR_REGISTER,
  PIC_REG_WATER_THERM1,
  PIC_REG_WATER_THERM2,
  PIC_REG_TEC_THERM,
  PIC_REG_PWR_THERM,
  PIC_REG_LID_IR_DET1,
  PIC_REG_LID_IR_DET2,
  PIC_REG_LID_IR_DET3,
  PIC_REG_LID_IR_DET4,

  ENUM_LAST_MARKER(PIC_LAST_SENSOR_REGISTER),
  PIC_FIRST_OUTPUT_REGISTER,

  PIC_REG_STEP_DAC_X = PIC_FIRST_OUTPUT_REGISTER,
  PIC_REG_STEP_DAC_Y,
  PIC_REG_LID_LED,

  PIC_FIRST_BUTTON_LED_REGISTER,
  PIC_REG_BUTTON_LED1 = PIC_FIRST_BUTTON_LED_REGISTER,
  PIC_REG_BUTTON_LED2,
  PIC_REG_BUTTON_LED3,

  ENUM_LAST_MARKER(PIC_LAST_BUTTON_LED_REGISTER),
  ENUM_LAST_MARKER(PIC_LAST_OUTPUT_REGISTER),

  /* of the sensor registers */
  PIC_FIRST_HV_REGISTER,
  PIC_REG_HV_ISENSE = PIC_FIRST_HV_REGISTER,
  PIC_REG_HV_VSENSE,

  ENUM_LAST_MARKER(PIC_LAST_HV_REGISTER),
  ENUM_LAST_MARKER(PIC_LAST_REGISTER),

  PIC_NUM_REGISTERS,
  PIC_NUM_BUTTON_LED_REGISTERS = PIC_LAST_BUTTON_LED_REGISTER-PIC_FIRST_BUTTON_LED_REGISTER+1,
  PIC_NUM_OUTPUT_REGISTERS = PIC_LAST_OUTPUT_REGISTER-PIC_FIRST_OUTPUT_REGISTER+1
};

typedef uint8_t pic_command;
typedef uint16_t pic_value;

#endif
