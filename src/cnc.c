// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * cnc.c
 *
 * Drives the stepper motors and laser.
 *
 * Copyright (C) 2020-2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
 * Copyright (C) 2015-2021 Glowforge, Inc. <opensource@glowforge.com>
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
 *
 * The step and direction pins are controlled by an SDMA script;
 * this subsystem is responsible for overseeing the operation of
 * the SDMA engine, the periodic interval timer (EPIT) that drives it,
 * and the GPIO port settings.
 */

#include "cnc_private.h"
#include "io.h"
#include "notifiers.h"
#include "sdma_macros.h"

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/of_reserved_mem.h>
#include <linux/panic_notifier.h>

/** Module parameters */
extern int cnc_enabled;

/**
 * If 1, the module starts up in the DISABLED state when it's loaded, and does
 * not enable the 40V supply.
 * If 0, the module starts up in the IDLE state when it's loaded, and enables
 * the 40V supply.
 */
#define INITIAL_STATE_DISABLED  1

/** Minimum, maximum, and default step frequencies. */
#define STEP_FREQUENCY_MIN      1000
#define STEP_FREQUENCY_MAX      200000
#define STEP_FREQUENCY_DEFAULT  10000

/**
 * The step frequency is the EPIT clock divided by an integer, so a divisor of
 * at least this much is needed for the requested frequency to be met closely
 * (100 gives 1 % worst-case quantization). Anything below the resulting clock
 * rate means the device tree has paired the timer with an unexpected clock.
 */
#define MIN_EPIT_DIVISOR        100
#define MIN_USABLE_EPIT_RATE    (STEP_FREQUENCY_MAX * MIN_EPIT_DIVISOR)

/** The SDMA has 32 channels; channel 0 is the command channel. */
#define MAX_SDMA_CHANNELS       32

/** Number of bits of PWM resolution */
#define LASER_PWM_BITS          7
/** Laser power duty cycle when idle */
#define LASER_PWM_IDLE_DUTY     65535

/**
 * How often the charge pump input is pulsed while running
 * (to keep the laser firing)
 */
#define CHARGE_PUMP_INTERVAL_NS               (200 * NSEC_PER_MSEC)

/**
 * Minimum step frequency for controlled decelerations and accelerations.
 */
#define RAMP_UPDATE_INTERVAL_NS               (10 * NSEC_PER_MSEC)

/**
 * Minimum step frequency for controlled decelerations and accelerations.
 * (Deceleration stops when this frequency is reached, and acceleration begins
 * at this frequency.)
 */
#define RAMP_MIN_STEP_FREQUENCY               900

/**
 * Number of ramp updates per second, derived from the update interval.
 * The ramp rate is specified in Hz/s, so the per-update step-frequency delta
 * is (ramp_rate / RAMP_UPDATES_PER_SEC).
 */
#define RAMP_UPDATES_PER_SEC                  (NSEC_PER_SEC / RAMP_UPDATE_INTERVAL_NS)

/**
 * Controlled acceleration/deceleration rate, in Hz/s (the amount by which the
 * step frequency is changed per second). Settable at runtime via the
 * "ramp_rate" attribute. The bounds and default match the factory firmware;
 * the default of 125000 Hz/s reproduces the legacy behavior (freq >> 3 per
 * 10 ms update) at the default 10 kHz step frequency.
 */
#define RAMP_RATE_MIN_HZ_PER_S                10000
#define RAMP_RATE_MAX_HZ_PER_S                500000
#define RAMP_RATE_DEFAULT_HZ_PER_S            125000

/**
 * Laser safety-line sampling. A free-running timer polls the LASER_ON and
 * LASER_PGOOD inputs; every LASER_SAMPLE_WINDOW samples (~1 second) it latches
 * the number of samples in which each line read low, exposed via the
 * laser_on_sampled / laser_pgood_sampled attributes.
 */
#define LASER_SAMPLE_WINDOW                   255
#define LASER_SAMPLE_INTERVAL_NS              (NSEC_PER_SEC / LASER_SAMPLE_WINDOW)


static const struct pwm_channel_config laser_pwm_config = {
  "laser-pwm", BITS_TO_PERIOD_NS(LASER_PWM_BITS)
};

#define NUM_STEPPER_FAULT_SIGNALS 3

static const pin_id stepper_fault_gpios[NUM_STEPPER_FAULT_SIGNALS] = {
  [FAULT_X]  = PIN_X_FAULT,
  [FAULT_Y1] = PIN_Y1_FAULT,
  [FAULT_Y2] = PIN_Y2_FAULT
};

/**
 * Fatal fault conditions require the driver to stop immediately and enter the
 * FAULT state. If a non-fatal fault occurs, the driver will attept a controlled
 * deceleration and enter the IDLE state.
 */
static const u32 fatal_fault_conditions =
  (1 << FAULT_X) |
  (1 << FAULT_Y1) |
  (1 << FAULT_Y2);
#define FAULT_IS_FATAL(num) (fatal_fault_conditions & (1<<(num)))

#define FAULT_DEV_ID_FROM_CNC_AND_SIGNAL(dr, s) (void *)(((u32)dr) | (s & 3U))
#define CNC_FROM_FAULT_DEV_ID(dev_id) (struct cnc *)((u32)(dev_id) & (~3U))
#define SIGNAL_FROM_FAULT_DEV_ID(dev_id) ((u32)(dev_id) & 3U)

static const u32 sdma_script[] = {
  /* Assembled from asm/sdma.asm by tools/sdma_asm.pl at build time (see
   * Makefile). Edit the .asm; never hand-edit the generated header. */
#include "sdma.asm.h"
};

static const ktime_t ramp_update_interval_ktime = RAMP_UPDATE_INTERVAL_NS;
static const ktime_t laser_sample_interval_ktime = LASER_SAMPLE_INTERVAL_NS;
static const ktime_t charge_pump_interval_ktime  = CHARGE_PUMP_INTERVAL_NS;

extern struct kobject *glowforge_kobj;

static void _cnc_ramp_stop(struct cnc *self);
static void beam_detect_latch_reset(struct cnc *self);
static void toggle_charge_pump(struct cnc *self);

static int load_sdma_script(struct cnc *self)
{
  int ret;
  const u32 *script = sdma_script;
  size_t script_len = sizeof(sdma_script);
  /* set the script arguments and initial PC */
  /* see the specific asm file for argument requirements */
  struct sdma_context_data initial_context = {
    .channel_state = { .pc = self->sdma_script_origin * 2 },
    .gReg = {
      [4] = cnc_buffer_fifo_bitmask(self),
      [5] = io_pwm_sample_register_address(&self->laser_pwm),
      [6] = cnc_buffer_fifo_start_phys(self),
      [7] = sdma_context_address_for_channel(self->sdma_ch_num)
    },
    .pda = epit_status_register_address(self->epit),
    .mda = io_base_address(self->gpios, NUM_GPIO_PINS, cnc_sdma_pin_set),
    .ms = 0x00000000, /* source and destination address frozen; start in read mode */
    .ps = 0x000c0400, /* destination address frozen; 32-bit write size; start in write mode */
    .scratch6 = 1,    /* direction = forward */
  };

  /* write the test script code to RAM */
  /* don't use sdma_load_script() because the assembler output */
  /* is already in the correct endianness */
  dev_dbg(self->dev, "loading SDMA script (%d bytes)...", script_len);
  ret = sdma_write_datamem(self->sdma, (void *)script, script_len, self->sdma_script_origin);
  if (ret) {
    dev_err(self->dev, "failed to load script");
    return ret;
  }

  /* acquire the channel; it's triggered externally by the EPIT */
  sdma_setup_channel(self->sdmac, true);

  /* load the initial context */
  ret = sdma_load_partial_context(self->sdmac, &initial_context, 0, sizeof(initial_context));
  if (ret) {
    dev_err(self->dev, "failed to set up channel");
    return ret;
  }

  dev_dbg(self->dev, "script loaded");
  return ret;
}


/**
 * Verifies the SDMA program RAM still holds our script, reloading it if
 * anything clobbered it. The imx-sdma driver loads the NXP RAM firmware
 * asynchronously (request_firmware_nowait), at an arbitrary time relative to
 * our probe; the script origin is placed above the firmware's RAM span, but
 * firmware growth would silently corrupt the script again, so trust nothing.
 * Rewrites only program memory, never the channel context, which holds live
 * position/head/tail state between runs. May sleep (channel-0 transfers);
 * call only outside atomic context, before a run starts.
 */
static int verify_sdma_script(struct cnc *self)
{
  u32 readback[ARRAY_SIZE(sdma_script)];
  int ret = sdma_fetch_datamem(self->sdma, readback, sizeof(readback),
    self->sdma_script_origin);
  if (ret) {
    dev_err(self->dev, "failed to read back SDMA script: %d", ret);
    return ret;
  }
  if (memcmp(readback, sdma_script, sizeof(sdma_script)) != 0) {
    dev_warn(self->dev, "SDMA script corrupted (RAM firmware overlap?); reloading");
    ret = sdma_write_datamem(self->sdma, (void *)sdma_script,
      sizeof(sdma_script), self->sdma_script_origin);
  }
  return ret;
}


int cnc_get_position(struct cnc *self, struct cnc_position *pos)
{
  /* Fetch current byte and head position from sdma engine */
  int ret = sdma_fetch_partial_context(self->sdmac, pos, offsetof(struct sdma_context_data, scratch0), sizeof(*pos));
  if (ret != 0) {
    return ret;
  }
  /* Splice in the total number of bytes enqueued */
  pos->bytes_total = cnc_buffer_total_bytes(self);
  return 0;
}


u32 cnc_get_step_frequency(struct cnc *self)
{
  return self->step_freq;
}


u32 cnc_get_ramp_rate_hz_per_s(struct cnc *self)
{
  return self->ramp_step_freq_delta * RAMP_UPDATES_PER_SEC;
}


int cnc_set_ramp_rate_hz_per_s(struct cnc *self, u32 hz_per_s)
{
  int ret = 0;
  if (hz_per_s < RAMP_RATE_MIN_HZ_PER_S || hz_per_s > RAMP_RATE_MAX_HZ_PER_S) {
    return -ERANGE;
  }

  spin_lock_bh(&self->status_lock);
  /* Ramp rate changes are forbidden while running */
  /* (this includes controlled acceleration/deceleration) */
  if (unlikely(self->status.state == STATE_RUNNING)) {
    ret = -EBUSY;
  } else {
    self->ramp_step_freq_delta = hz_per_s / RAMP_UPDATES_PER_SEC;
  }
  spin_unlock_bh(&self->status_lock);

  return ret;
}


int cnc_set_step_frequency(struct cnc *self, u32 freq)
{
  int ret = 0;
  if (freq < STEP_FREQUENCY_MIN || freq > STEP_FREQUENCY_MAX) {
    return -ERANGE;
  }

  spin_lock_bh(&self->status_lock);
  /* Step frequency changes are forbidden while running */
  /* (this includes controlled acceleration/deceleration) */
  if (unlikely(self->status.state == STATE_RUNNING)) {
    ret = -EBUSY;
  } else {
    self->step_freq = freq;
    self->ramp_step_freq = freq;
    /* ramp_step_freq_delta is independent of step_freq; it is set via ramp_rate */
  }
  spin_unlock_bh(&self->status_lock);

  return ret;
}


/* Powers on the steppers without checking fault states */
static void stepper_power_on_unchecked(struct cnc *self)
{
  if (!regulator_is_enabled(self->supply_40v)) {
    if (regulator_enable(self->supply_40v)) {
      dev_err(self->dev, "unable to enable 40V supply");
    } else {
      dev_info(self->dev, "40V on");
    }
  }
  io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_startup_pin_changes);
}


static int _stepper_power_on(struct cnc *self, int faults)
{
  /* Don't power on the steppers if the drivers are asserting a fault. */
  /* (It's possible that *enabling* the steppers and the 40V supply could */
  /* trigger a fault, but in that case, they'll be disabled immediately.) */
  if (faults & fatal_fault_conditions) {
    dev_err(self->dev, "driver(s) in fault state; not powering on");
    return -1;
  }
  stepper_power_on_unchecked(self);
  return 0;
}


/* acquires status_lock */
__maybe_unused static int stepper_power_on(struct cnc *self)
{
  return _stepper_power_on(self, cnc_triggered_faults(self));
}


/* Disables the 40V supply only. May sleep; never call under status_lock. */
static void stepper_supply_off(struct cnc *self)
{
  if (regulator_is_enabled(self->supply_40v)) {
    if (regulator_disable(self->supply_40v)) {
      dev_err(self->dev, "unable to disable 40V supply");
    } else {
      dev_info(self->dev, "40V off");
    }
  }
}


/* May sleep; never call under status_lock. */
static void stepper_power_off(struct cnc *self)
{
  stepper_supply_off(self);
  io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_shutdown_pin_changes);
}


/* Must be called with status_lock held */
static void _driver_stop(struct cnc *self, enum cnc_state next_state)
{
  dev_dbg(self->dev, "stopping cut...");
  epit_stop(self->epit);
  /* Stop the HV-watchdog feed. From softirq (SDMA callback, ramp timer)
   * skip the synchronous cancel: the callback self-terminates as soon as it
   * sees the state leave RUNNING, costing at most one extra ~200 ms feed
   * with all output lines already forced low. */
  if (!in_softirq()) {
    hrtimer_cancel(&self->charge_pump_timer);
  }
  sdma_event_disable(self->sdmac, epit_sdma_event(self->epit));
  _cnc_ramp_stop(self);

  /* Clear run-scoped flags so nothing stale leaks into the next run. */
  self->status.waypoint_armed = false;
  self->status.decel_on_interrupt = false;
  self->status.enable_laser_on_interrupt = false;
  self->status.running_backward = false;

  /* Drive the outputs to their safe state. The 40 V supply itself is NOT
   * touched here: regulator calls may sleep, and _driver_stop runs under
   * status_lock and from atomic contexts (SDMA tasklet, fault tasklet,
   * panic notifier). Callers that disable (cnc_disable, deadman close)
   * power the supply off after releasing the lock. */
  if (next_state == STATE_DISABLED) {
    io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_shutdown_pin_changes);
  }
  /* Otherwise, just ensure the laser and stepper lines are low */
  else {
    io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_stop_pin_changes);
  }

  dev_dbg(self->dev, "stopped.");
  self->status.state = next_state;
  cnc_notify_state_changed(self);
}


/**
 * Common code for starting a controlled acceleration/deceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_ramp_start(struct cnc *self)
{
  /* Disable DMA control of laser enable; force the line low. */
  gpio_direction_input(self->gpios[PIN_LASER_ON]);
  /* Begin periodic updates */
  hrtimer_start(&self->ramp_timer, ramp_update_interval_ktime, HRTIMER_MODE_REL_SOFT);
}


/**
 * Stops a controlled acceleration/deceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_ramp_stop(struct cnc *self)
{
  /* If called by ramp_update_tasklet_fn, don't cancel the timer, because */
  /* we're already running in a tasklet and the kernel doesn't like that */
  if (!in_softirq()) {
    hrtimer_cancel(&self->ramp_timer);
  }
  self->status.decelerating = false;
  self->status.accelerating = false;
}


/**
 * Starts a controlled deceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_decel_start(struct cnc *self)
{
  if (self->status.decelerating) {
    return;
  }
  dev_dbg(self->dev, "starting deceleration");
  if (!self->status.accelerating) {
    /* Don't suddenly jump the step frequency if we're already accelerating */
    self->ramp_step_freq = self->step_freq;
  }
  self->status.accelerating = false;
  self->status.decelerating = true;
  _cnc_ramp_start(self);
}


/**
 * Starts a controlled acceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_accel_start(struct cnc *self)
{
  if (self->status.accelerating) {
    return;
  }
  dev_dbg(self->dev, "starting acceleration");
  if (!self->status.decelerating) {
    /* Don't suddenly jump the step frequency if we're already decelerating */
    self->ramp_step_freq = RAMP_MIN_STEP_FREQUENCY;
  }
  self->status.accelerating = true;
  self->status.decelerating = false;
  _cnc_ramp_start(self);
}


/**
 * Controlled acceleration/deceleration update step.
 */
static enum hrtimer_restart ramp_update_tasklet_fn(struct hrtimer *timer)
{
  /* We don't need to protect the status field in this function, */
  /* because the tasklet won't ever run concurrently with itself or any other */
  /* tasklet (uniprocessor system, tasklets can't be preempted), and won't */
  /* run in the sections protected by spin_lock_bh()/spin_unlock_bh(). */
  struct cnc *self = container_of(timer, struct cnc, ramp_timer);

  /* sanity check */
  if (!self->status.decelerating && !self->status.accelerating) {
    return HRTIMER_NORESTART;
  }

  if (self->status.decelerating) {
    if (self->ramp_step_freq <= RAMP_MIN_STEP_FREQUENCY) {
      dev_dbg(self->dev, "stopping deceleration");
      _driver_stop(self, STATE_IDLE);
      return HRTIMER_NORESTART;
    }
    /* Saturating decrement: an unsigned underflow here would program
     * the EPIT with a degenerate divisor (see epit_hz_to_divisor) and
     * turn the decel tail into a max-rate burst - or, in the wrap
     * case, wedge this loop in RUNNING for hours. */
    if (self->ramp_step_freq > RAMP_MIN_STEP_FREQUENCY + self->ramp_step_freq_delta) {
      self->ramp_step_freq -= self->ramp_step_freq_delta;
    } else {
      self->ramp_step_freq = RAMP_MIN_STEP_FREQUENCY;
    }
  }
  else if (self->status.accelerating) {
    self->ramp_step_freq += self->ramp_step_freq_delta;
    if (self->ramp_step_freq >= self->step_freq) {
      dev_dbg(self->dev, "stopping acceleration");
      epit_set_hz(self->epit, self->step_freq); /* restore full step freq */
      return HRTIMER_NORESTART;
    }
  }

  epit_set_hz(self->epit, self->ramp_step_freq);
  hrtimer_forward_now(timer, ramp_update_interval_ktime);
  return HRTIMER_RESTART;
}


/**
 * Called when the SDMA script raises the host interrupt flag for our channel
 * (via a "notify 3" instruction, at a waypoint or at end-of-data).
 * This callback executes in tasklet context.
 *
 * Signal decoding uses only host-side state (never a channel context fetch,
 * which may sleep): if a waypoint is outstanding, the signal is the waypoint;
 * otherwise it is end-of-data. Two notifies raised before the ARM services
 * the first merge into ONE callback, so a coalesced waypoint+end-of-data
 * signal is decoded as just the waypoint here; the script re-raises the
 * end-of-data interrupt every ~255 parked iterations until we stop the EPIT,
 * so the lost half is redelivered shortly.
 */
static void cnc_sdma_interrupt(void *param)
{
  struct cnc *self = (struct cnc *)param;
  spin_lock_bh(&self->status_lock);
  if (unlikely(self->status.waypoint_armed)) {
    /* Waypoint reached (scratch7 hit zero). Consume the armed actions. */
    self->status.waypoint_armed = false;
    if (self->status.enable_laser_on_interrupt) {
      /* Re-enable SDMA control of the laser line (end of a resume ramp)
       * - gated on the laser latch exactly like a run start: a locked
       * latch means the run stays laser-less, waypoint or no waypoint. */
      self->status.enable_laser_on_interrupt = false;
      if (gpio_get_value(self->gpios[PIN_LASER_LATCH_RESET]) == 0) {
        gpio_direction_output(self->gpios[PIN_LASER_ON], 0);
      }
    }
    if (self->status.decel_on_interrupt) {
      /* Start the controlled deceleration (end of a backtrack). */
      self->status.decel_on_interrupt = false;
      _cnc_decel_start(self);
    }
  }
  else if (self->status.state == STATE_RUNNING) {
    /* End of data: normal completion, or an underrun, if a streaming
     * feeder declared itself. The stop is laser-safe either way (the script
     * forces the laser/step lines low at end-of-data before signaling);
     * an underrun additionally means steps may have been skipped at speed,
     * so the position counters can no longer be trusted. */
    if (unlikely(self->status.streaming)) {
      self->underrun_count++;
      dev_warn(self->dev, "pulse data underrun (#%u); position no longer trusted",
        self->underrun_count);
      _driver_stop(self, STATE_UNDERRUN);
    } else {
      _driver_stop(self, STATE_IDLE);
    }
  }
  /* else: a stale re-notify delivered after we already stopped (end-of-data
   * plus a queued re-notify, or a fault/halt stopped us first). Ignore it
   * rather than clobbering the current state. */
  spin_unlock_bh(&self->status_lock);
}


/**
 * cnc_run_with_options() needs a lot of arguments, so pack them all into
 * a 32-bit struct instead of passing them all individually.
 */
struct cnc_run_options {
  /** If != 0, SDMA will trigger a waypoint interrupt after this many steps. */
  unsigned int num_steps:28;
  /** 0 to run forward, 1 to run backward. */
  unsigned int backward:1;
  /** 0 to start at full speed, 1 to start with acceleration. */
  unsigned int accelerate:1;
  /** 0 to come to an immediate stop at end of data, 1 to decelerate. */
  unsigned int decelerate:1;
  /** 0 to reset laser power PWM duty cycle at end of data, 1 to preserve. */
  unsigned int preserve_power:1;
} __attribute__((packed));

/**
 * Used for run, backtrack, and resume.
 * Idle: start cutting if there is data
 * Running: do nothing
 * Disabled: enable the steppers and start cutting
 * Fault: do nothing (error state must be explicitly cleared)
 */
static int cnc_run_with_options(struct cnc *self, struct cnc_run_options opts)
{
  int ret = 0;
  bool need_to_start = false;
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_RUNNING:
      ret = -EPERM;
      break;

    case STATE_FAULT:
    default:
      dev_err(self->dev, "cannot start in fault state");
      ret = -EPERM;
      break;

    case STATE_DISABLED:
    case STATE_IDLE:
      /* defer loading until we're out of atomic context */
      need_to_start = true;
      break;
  }
  spin_unlock_bh(&self->status_lock);

  if (need_to_start) {
    uint32_t regs[3];
    uint32_t num_steps = opts.num_steps;
    bool was_powered;

    /* Ensure there is enough data enqueued. (may sleep) A run request on an
     * empty ring is refused with -ENODATA and logged: a feeder asks for a run
     * right after queueing bytes, so head == tail here means either a feeder
     * defect or an engine that is not moving data (a head sync that reads
     * back stale), never routine. This line was the only kernel-log trace of
     * a gated SDMA. */
    if (cnc_buffer_is_empty(self)) {
      dev_err(self->dev, "run requested with no data enqueued");
      return -ENODATA;
    }

    /* Ensure the SDMA program RAM is intact before starting. (may sleep) */
    ret = verify_sdma_script(self);
    if (ret) {
      return ret;
    }

    /* The backtrack check, the scratch-register snapshot, and the state
     * commit below must be consistent against a concurrent writer or
     * clear on the shared (inheritable) fd. */
    mutex_lock(&self->pulsebuf_lock);

    /* A backward run may only walk over data that is played, genuine and
     * still resident. Refuse a longer request rather than quietly running
     * a shorter one: the caller sizes its laser-on lead to the distance it
     * asked for, so a silent shortfall puts the beam back on ahead of the
     * pause point and leaves an unburned length in the cut. */
    if (opts.backward) {
      uint32_t max_steps = cnc_buffer_max_backtrack_length(self);
      if (num_steps > max_steps) {
        dev_err(self->dev, "backtrack of %u steps refused; %u retained",
                num_steps, max_steps);
        mutex_unlock(&self->pulsebuf_lock);
        return -EPERM;
      }
    }

    /* Set direction and interrupt point. */
    /* If processing backward, set scratch5 to ensure the DMA engine doesn't */
    /* go past the oldest data byte. Wraparound in subtraction is OK. */
    regs[0] /* scratch5 */ =
     (!opts.backward) ? self->pulsebuf_fifo.kfifo.in
                      : cnc_buffer_backtrack_dead_stop(self);
      /* Note when running backward: it would be incorrect to set scratch5 */
      /* to (self->pulsebuf_fifo.kfifo.in-num_steps). When head == tail, */
      /* the DMA engine will come to a dead stop. But when backtracking we */
      /* want to *decelerate* after num_steps, and only come to a dead stop */
      /* if we run out of room to backtrack. The dead stop is the oldest */
      /* byte that is still genuine job data: the start of the job while it */
      /* fits the ring, one ring back once a live feed has wrapped it. */
    regs[1] /* scratch6 */ = (!opts.backward) ? 0x00000001 : 0xFFFFFFFF;
    regs[2] /* scratch7 */ = num_steps;

    ret = sdma_set_regs(self->sdmac, regs, scratch5, sizeof(regs));
    if (ret) {
      dev_err(self->dev, "failed to set channel context");
      mutex_unlock(&self->pulsebuf_lock);
      return ret;
    }

    /* Power the steppers and reset the laser PWM duty BEFORE taking the
     * status lock: regulator and PWM operations may sleep (regulator rdev
     * mutex; pwm_apply_might_sleep() on pwm-imx27), so they must never run
     * under spin_lock_bh. The fault check below still gates the actual run
     * start; if a fault raced in, we compensate by powering back off,
     * but only if the steppers were off when we entered. */
    was_powered = (regulator_is_enabled(self->supply_40v) > 0);
    stepper_power_on_unchecked(self);
    if (!opts.preserve_power) {
      io_pwm_set_duty_cycle(&self->laser_pwm, LASER_PWM_IDLE_DUTY);
    }

    /* We could have transitioned to FAULT between the start of the function */
    /* and now, so we have to lock and check the fault state. */
    /* If a fault occurs during the execution of this block, we'll get a */
    /* callback immediately after we release the lock, which will transition */
    /* the driver to the FAULT state. */
    spin_lock_bh(&self->status_lock);
    if (self->status.triggered_faults) {
      ret = -EPERM;
    } else {
      /* The waypoint action flags are only meaningful when a waypoint is
       * actually armed (num_steps > 0): scratch7 == 0 never fires, and a
       * stale action flag would eat the end-of-data signal, wedging the
       * driver in RUNNING. */
      bool waypoint_armed = (num_steps > 0);
      self->status.state = STATE_RUNNING;
      self->status.waypoint_armed = waypoint_armed;
      self->status.decel_on_interrupt = opts.decelerate && waypoint_armed;
      self->status.enable_laser_on_interrupt =
        (opts.accelerate && !opts.backward && waypoint_armed);
      self->status.running_backward = opts.backward;

      /* _driver_stop parks the FIRE line Hi-Z at every stop; if the
       * latch is unlocked, restore SDMA drive for this run. A resume
       * ramp keeps it parked until its waypoint re-enables it. */
      if (!self->status.enable_laser_on_interrupt &&
          gpio_get_value(self->gpios[PIN_LASER_LATCH_RESET]) == 0) {
        gpio_direction_output(self->gpios[PIN_LASER_ON], 0);
      }

      /* clear all fault conditions */
      self->status.triggered_faults &= fatal_fault_conditions;

      cnc_notify_state_changed(self);

      dev_dbg(self->dev, "starting cut...");
      beam_detect_latch_reset(self);
      toggle_charge_pump(self); /* pulse once to prime charge pump before cut start */
      hrtimer_start(&self->charge_pump_timer, charge_pump_interval_ktime, HRTIMER_MODE_REL_SOFT);
      /* Enable timer events */
      sdma_event_enable(self->sdmac, epit_sdma_event(self->epit));

      /* Start generating periodic events */
      /* If not accelerating, start at full speed */
      if (!opts.accelerate) {
        epit_start_hz(self->epit, self->step_freq);
      } else {
        epit_start_hz(self->epit, RAMP_MIN_STEP_FREQUENCY);
        _cnc_accel_start(self);
      }

      /* Set a nonzero priority to start the script */
      sdma_set_channel_priority(self->sdmac, 6);
      dev_dbg(self->dev, "started.");
    }
    spin_unlock_bh(&self->status_lock);
    mutex_unlock(&self->pulsebuf_lock);

    if (ret == -EPERM) {
      dev_err(self->dev, "attempt to start in fault state");
      if (!was_powered) {
        stepper_power_off(self); /* undo the speculative power-on */
      }
    }
  }

  return ret;
}


int cnc_run(struct cnc *self)
{
  /* Run normally; no acceleration, deceleration, or waypoint interrupt */
  return cnc_run_with_options(self, (struct cnc_run_options){
    .num_steps = 0,
    .backward = false,
    .accelerate = false,
    .decelerate = false,
    .preserve_power = false
  });
}


int cnc_backtrack(struct cnc *self, uint32_t num_steps)
{
  if (num_steps == 0) {
    return -EINVAL;
  }
  /* How far back the ring can be walked is a property of the ring, not of
   * how it was filled: cnc_run_with_options() refuses a request longer than
   * the retained history, and the engine's dead stop bounds the run at the
   * oldest genuine byte either way. A live feed keeps at least the writer's
   * gap of history, so a streamed job pauses like a preloaded one. */
  /* Run backward, with acceleration, deceleration, and waypoint interrupt. */
  /* (Waypoint interrupt starts deceleration after num_steps.) */
  return cnc_run_with_options(self, (struct cnc_run_options){
    .num_steps = num_steps,
    .backward = true,
    .accelerate = true,
    .decelerate = true,
    .preserve_power = true
  });

}


int cnc_resume(struct cnc *self, uint32_t laser_delay_steps)
{
  /* Run forward, with acceleration and waypoint interrupt. */
  /* (Waypoint interrupt enables laser after laser_delay_steps.) */
  return cnc_run_with_options(self, (struct cnc_run_options){
    .num_steps = laser_delay_steps,
    .backward = false,
    .accelerate = true,
    .decelerate = false,
    .preserve_power = true
  });
}


/**
 * Idle: do nothing
 * Running: stop cut (controlled deceleration)
 * Disabled: do nothing (remain in disabled state)
 * Underrun: acknowledge; return to idle (feeder must re-home before trusting position)
 * Fault: do nothing (return error)
 */
int cnc_stop(struct cnc *self)
{
  int ret = 0;
  spin_lock_bh(&self->status_lock);

  switch (self->status.state) {
    case STATE_IDLE:
    case STATE_DISABLED:
      break;

    case STATE_UNDERRUN:
      /* Acknowledge the underrun. The feeder is expected to have raised its
       * alarm; position must be re-homed before it can be trusted again. */
      self->status.state = STATE_IDLE;
      cnc_notify_state_changed(self);
      break;

    case STATE_FAULT:
    default:
      ret = -EPERM;
      break;

    case STATE_RUNNING:
      /* Start a controlled deceleration. */
      _cnc_decel_start(self);
      break;
  }

  spin_unlock_bh(&self->status_lock);
  return ret;
}


/**
 * Idle: do nothing
 * Running: stop cut instantly (no deceleration; may lose steps at speed)
 * Disabled: do nothing (remain in disabled state)
 * Fault: do nothing (return error)
 */
int cnc_halt(struct cnc *self)
{
  int ret = 0;
  spin_lock_bh(&self->status_lock);

  switch (self->status.state) {
    case STATE_IDLE:
    case STATE_DISABLED:
      break;

    case STATE_FAULT:
    default:
      ret = -EPERM;
      break;

    case STATE_RUNNING:
      /* Stop processing pulse data immediately. */
      _driver_stop(self, STATE_IDLE);
      break;
  }

  spin_unlock_bh(&self->status_lock);
  return ret;
}


/**
 * Idle: power off steppers
 * Running: stop everything and power off steppers
 * Underrun: stop everything and power off steppers
 * Disabled: do nothing
 * Fault: do nothing
 */
int cnc_disable(struct cnc *self)
{
  int ret = 0;
  bool powered_down = false;
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_IDLE:
    case STATE_RUNNING:
    case STATE_UNDERRUN:
      _driver_stop(self, STATE_DISABLED);
      powered_down = true;
      break;
    case STATE_DISABLED:
    case STATE_FAULT:
      break;
    default:
      ret = -EPERM;
      break;
  }
  spin_unlock_bh(&self->status_lock);
  if (powered_down) {
    /* Regulator ops may sleep: outside the lock. The output lines are
     * already in their shutdown state from _driver_stop. */
    stepper_supply_off(self);
  }
  return ret;
}


/**
 * Idle: do nothing
 * Running: error
 * Disabled: enable steppers
 * Fault: recover to idle, but only once every (non-ignored) fault line
 *        has physically cleared
 */
int cnc_enable(struct cnc *self)
{
  int ret = 0;
  bool need_power_on = false;
  int i;
  unsigned long live_faults = 0;

  /* Live fault-line snapshot (active low) for the recovery gate below. */
  for (i = 0; i < NUM_STEPPER_FAULT_SIGNALS; i++) {
    if (gpio_get_value(self->gpios[stepper_fault_gpios[i]]) == 0) {
      live_faults |= (1 << i);
    }
  }

  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_DISABLED:
      if (self->status.triggered_faults & fatal_fault_conditions) {
        dev_err(self->dev, "driver(s) in fault state; not powering on");
        ret = -EPERM;
      } else {
        need_power_on = true;
      }
      break;
    case STATE_FAULT:
      /* The documented fault -> idle recovery: an explicit enable is the
       * operator's deliberate lever, so a single edge glitch does not
       * brick motion until module reload - but a line that still reads
       * asserted refuses, so a genuinely faulted driver is never
       * cleared blind. */
      if (live_faults & ~self->ignored_faults) {
        dev_err(self->dev, "fault line(s) still asserted; not recovering");
        ret = -EPERM;
      } else {
        self->status.triggered_faults = 0;
        need_power_on = true;
      }
      break;
    case STATE_IDLE:
      break;
    default:
      ret = -EPERM;
      break;
  }
  spin_unlock_bh(&self->status_lock);

  if (!need_power_on) {
    return ret;
  }

  /* Regulator ops may sleep: power on outside the lock, then re-check that
   * no fault raced in before committing the state change. */
  stepper_power_on_unchecked(self);
  spin_lock_bh(&self->status_lock);
  if (self->status.triggered_faults & fatal_fault_conditions) {
    ret = -EPERM;
  } else if (self->status.state == STATE_DISABLED ||
             self->status.state == STATE_FAULT) {
    self->status.state = STATE_IDLE;
    cnc_notify_state_changed(self);
  }
  spin_unlock_bh(&self->status_lock);
  if (ret) {
    dev_err(self->dev, "fault during power-on; disabling again");
    stepper_power_off(self);
  }
  return ret;
}


/**
 * Idle: take step
 * Running: error
 * Disabled: take step (Z axis enable isn't controlled by kernel module)
 * Fault: error (TODO)
 */
int cnc_single_z_step(struct cnc *self, bool direction)
{
  int ret = 0;
  int z_step_gpio, z_dir_gpio;
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_RUNNING:
    case STATE_FAULT:
      ret = -EPERM;
      break;
    default:
      ret = 0;
      break;
  }
  spin_unlock_bh(&self->status_lock);

  if (ret) {
    return ret;
  }

  z_step_gpio = self->gpios[PIN_Z_STEP];
  z_dir_gpio = self->gpios[PIN_Z_DIR];

  /* Hardware-verified: Z_DIR driven HIGH moves the lens UP, away from the
   * bed - the documented attr convention (1 = positive = away from the bed)
   * is physically true with the raw drive. */
  gpio_set_value(z_dir_gpio, direction);
  gpio_set_value(z_step_gpio, 1);
  udelay(2); /* DRV8818 wants a minimum 1us pulse duration */
  gpio_set_value(z_step_gpio, 0);
  gpio_set_value(z_dir_gpio, 0);
  return 0;
}


int cnc_clear_pulse_data(struct cnc *self, enum cnc_lseek_options opts)
{
  int ret = 0;
  uint32_t clear_flags = 0;
  switch (opts) {
    case PULSEDEV_LSEEK_CLEAR_DATA_AND_POSITION:
      clear_flags = CNC_BUFFER_CLEAR_DATA|CNC_BUFFER_CLEAR_POSITION;
      break;
    case PULSEDEV_LSEEK_CLEAR_DATA:
      clear_flags = CNC_BUFFER_CLEAR_DATA;
      break;
    case PULSEDEV_LSEEK_CLEAR_POSITION:
      clear_flags = CNC_BUFFER_CLEAR_POSITION;
      break;
    default:
      return -EINVAL;
  }

  /* The state check and the reset must be one atomic unit against a run
   * start (cnc_run_with_options holds the same lock through its state
   * commit), or a run raced in between them and the reset yanks the
   * fifo out from under a playing engine. */
  mutex_lock(&self->pulsebuf_lock);
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_RUNNING:
      ret = -EPERM;
      break;
    default:
      break;
  }
  spin_unlock_bh(&self->status_lock);

  /* don't touch the context if we're returning an error */
  if (!ret) {
    ret = cnc_buffer_clear(self, clear_flags);
  }
  mutex_unlock(&self->pulsebuf_lock);
  return ret;
}


int cnc_set_laser_latch(struct cnc *self, int value)
{
  /* If value == 0, latch is unlocked and LASER_ON is an output. */
  /* Otherwise, latch is locked and LASER_ON is high impedance. */
  spin_lock_bh(&self->status_lock);
  gpio_set_value(self->gpios[PIN_LASER_LATCH_RESET], value);
  if (value == 0) {
    /* Allow LASER_ON to be driven by sdma - but never while a run or a
     * ramp is in flight: a controlled stop keeps consuming pulse bytes
     * with the FIRE line parked Hi-Z, and restoring drive here would
     * play the remaining fire bits at reduced speed (a deeper burn at
     * the stop point). Run start (and the resume waypoint) restore the
     * drive themselves when the latch is unlocked. The lock also keeps
     * this write from racing _driver_stop into an idle state with FIRE
     * left driven. */
    if (self->status.state != STATE_RUNNING &&
        !self->status.accelerating && !self->status.decelerating) {
      gpio_direction_output(self->gpios[PIN_LASER_ON], 0);
    }
  } else {
    gpio_direction_input(self->gpios[PIN_LASER_ON]);
  }
  spin_unlock_bh(&self->status_lock);
  return 0;
}


static void toggle_charge_pump(struct cnc *self)
{
  int gpio = self->gpios[PIN_CHARGE_PUMP];
  gpio_set_value(gpio, 0);
  gpio_set_value(gpio, 1);
  gpio_set_value(gpio, 0);
}


/* Called in softirq context (soft hrtimer, same domain as the stop logic:
 * if softirqs are starved, this feed starves with them and the hardware
 * HV watchdog disarms the chain instead of being kept alive blind). */
static enum hrtimer_restart charge_pump_timer_cb(struct hrtimer *timer)
{
  struct cnc *self = container_of(timer, struct cnc, charge_pump_timer);
  /* Feed the HV watchdog only while a cut is genuinely in progress; if the
   * driver left RUNNING (stop, fault, underrun), let the feed die even when
   * the synchronous cancel in _driver_stop was skipped. */
  if (self->status.state != STATE_RUNNING) {
    return HRTIMER_NORESTART;
  }
  toggle_charge_pump(self);
  hrtimer_forward_now(timer, charge_pump_interval_ktime);
  return HRTIMER_RESTART;
}


/*
 * Laser-safety-chain readbacks. These expose the hardware safety signals for
 * monitoring; actual enforcement is in the hardware AND-gate, not here.
 * LASER_ON is active low, so its getter returns the logical (asserted) state.
 * LASER_PGOOD is the supply's power-good line, driven high while the supply
 * reports its outputs within spec (it does not follow HV_ENABLE or emission),
 * so its getter returns the pin level: 1 = good. The others return the raw
 * pin level.
 */
int cnc_get_laser_enable(struct cnc *self)
{
  /* PIN_LASER_ON is the laser-enable / FIRE drive line. */
  return gpio_get_value(self->gpios[PIN_LASER_ON]);
}

int cnc_get_laser_on(struct cnc *self)
{
  return !gpio_get_value(self->gpios[PIN_LASER_ON_READBACK]);
}

int cnc_get_laser_pgood(struct cnc *self)
{
  return gpio_get_value(self->gpios[PIN_LASER_PGOOD]);
}

int cnc_get_button_latch(struct cnc *self)
{
  return gpio_get_value(self->gpios[PIN_BUTTON_LATCH]);
}

int cnc_get_charge_pump_alive(struct cnc *self)
{
  /* The one-shot's Q reaches the SoC through an inverter: low = alive. */
  return !gpio_get_value(self->gpios[PIN_CHARGE_PUMP_ALIVE]);
}

int cnc_get_interlock_latch_reset(struct cnc *self)
{
  return gpio_get_value(self->gpios[PIN_INTERLOCK_LATCH_RESET]);
}


/* Drive callback for the interlock-latch policy (cnc_interlock.c). Runs
 * from the input event path with interrupts disabled, so this must stay a
 * plain non-sleeping GPIO write. */
static void cnc_interlock_drive(struct cnc_interlock *il, int level)
{
  struct cnc *self = container_of(il, struct cnc, interlock);
  gpio_set_value(self->gpios[PIN_INTERLOCK_LATCH_RESET], level);
}

int cnc_get_laser_on_sampled(struct cnc *self)
{
  return self->laser_on_sampled;
}

int cnc_get_laser_pgood_sampled(struct cnc *self)
{
  return self->laser_pgood_sampled;
}

/*
 * Raw snapshot of the safety-chain GPIOs as a bitmask:
 *   bit 0: LASER_ON   bit 1: LASER_ENABLE   bit 2: BUTTON_LATCH
 *   bit 3: LASER_LATCH   bit 4: INTERLOCK_LATCH_RESET
 *   bit 5: CHARGE_PUMP_ALIVE (raw pin: 0 = alive)
 */
int cnc_get_interlock_circuit(struct cnc *self)
{
  int v = 0;
  if (gpio_get_value(self->gpios[PIN_LASER_ON_READBACK]))     { v |= 1; }
  if (gpio_get_value(self->gpios[PIN_LASER_ON]))              { v |= 2; }
  if (gpio_get_value(self->gpios[PIN_BUTTON_LATCH]))          { v |= 4; }
  if (gpio_get_value(self->gpios[PIN_LASER_LATCH_RESET]))     { v |= 8; }
  if (gpio_get_value(self->gpios[PIN_INTERLOCK_LATCH_RESET])) { v |= 16; }
  if (gpio_get_value(self->gpios[PIN_CHARGE_PUMP_ALIVE]))     { v |= 32; }
  return v;
}

/*
 * Free-running sampler. Counts how many of the last LASER_SAMPLE_WINDOW samples
 * read each line asserted (LASER_ON low, LASER_PGOOD high), latching the counts
 * once per window (~1 s).
 */
static enum hrtimer_restart laser_sample_timer_cb(struct hrtimer *timer)
{
  struct cnc *self = container_of(timer, struct cnc, laser_sample_timer);
  if (gpio_get_value(self->gpios[PIN_LASER_ON_READBACK]) == 0) {
    self->laser_on_low_count++;
  }
  if (gpio_get_value(self->gpios[PIN_LASER_PGOOD]) != 0) {
    self->laser_pgood_low_count++;
  }
  if (++self->laser_sample_count >= LASER_SAMPLE_WINDOW) {
    self->laser_on_sampled = self->laser_on_low_count;
    self->laser_pgood_sampled = self->laser_pgood_low_count;
    self->laser_on_low_count = 0;
    self->laser_pgood_low_count = 0;
    self->laser_sample_count = 0;
  }
  hrtimer_forward_now(timer, laser_sample_interval_ktime);
  return HRTIMER_RESTART;
}


enum cnc_state cnc_state(struct cnc *self)
{
  enum cnc_state ret;
  spin_lock_bh(&self->status_lock);
  ret = self->status.state;
  spin_unlock_bh(&self->status_lock);
  return ret;
}


#undef X
#define X(e,s) case e: return s;
const char *cnc_string_for_state(enum cnc_state st)
{
  switch (st) {
    DRIVER_STATES
    default: return "unknown";
  }
}


const char *cnc_state_string(struct cnc *self)
{
  return cnc_string_for_state(cnc_state(self));
}


u32 cnc_triggered_faults(struct cnc *self)
{
  u32 ret;
  spin_lock_bh(&self->status_lock);
  ret = self->status.triggered_faults;
  spin_unlock_bh(&self->status_lock);
  return ret;
}


ssize_t cnc_print_sdma_context(struct cnc *self, char *buf)
{
  return sdma_print_context(self->sdma, self->sdma_ch_num, buf);
}


int cnc_set_microstep_mode(struct cnc *self, enum cnc_axis axis, enum cnc_microstep_mode mode)
{
  int mode_binary, pin_mode0, pin_mode1, pin_mode2;
  switch (mode) {
    case MODE_FULL_STEP:     mode_binary = 0b000; break;
    case MODE_MICROSTEPS_2:  mode_binary = 0b001; break;
    case MODE_MICROSTEPS_4:  mode_binary = 0b010; break;
    case MODE_MICROSTEPS_8:  mode_binary = 0b011; break;
    case MODE_MICROSTEPS_16: mode_binary = 0b100; break;
    case MODE_MICROSTEPS_32: mode_binary = 0b101; break;
    default:                 return -EINVAL;
  }
  switch (axis) {
    case AXIS_X: pin_mode0 = PIN_X_MODE0; pin_mode1 = PIN_X_MODE1; pin_mode2 = PIN_X_MODE2; break;
    case AXIS_Y: pin_mode0 = PIN_Y_MODE0; pin_mode1 = PIN_Y_MODE1; pin_mode2 = PIN_Y_MODE2; break;
    default:     return -EINVAL;
  }
  gpio_set_value(self->gpios[pin_mode0], mode_binary & 0b001);
  gpio_set_value(self->gpios[pin_mode1], mode_binary & 0b010);
  gpio_set_value(self->gpios[pin_mode2], mode_binary & 0b100);
  return 0;
}


enum cnc_microstep_mode cnc_get_microstep_mode(struct cnc *self, enum cnc_axis axis)
{
  int mode_binary, pin_mode0, pin_mode1, pin_mode2;
  switch (axis) {
    case AXIS_X: pin_mode0 = PIN_X_MODE0; pin_mode1 = PIN_X_MODE1; pin_mode2 = PIN_X_MODE2; break;
    case AXIS_Y: pin_mode0 = PIN_Y_MODE0; pin_mode1 = PIN_Y_MODE1; pin_mode2 = PIN_Y_MODE2; break;
    default:     return -EINVAL;
  }
  mode_binary = (gpio_get_value(self->gpios[pin_mode0]) != 0) |
                ((gpio_get_value(self->gpios[pin_mode1]) != 0) << 1) |
                ((gpio_get_value(self->gpios[pin_mode2]) != 0) << 2);
  switch ((mode_binary) & 0b111) {
    case 0b000: return MODE_FULL_STEP;     break;
    case 0b001: return MODE_MICROSTEPS_2;  break;
    case 0b010: return MODE_MICROSTEPS_4;  break;
    case 0b011: return MODE_MICROSTEPS_8;  break;
    case 0b100: return MODE_MICROSTEPS_16; break;
    default:    return MODE_MICROSTEPS_32; break;
  }
}



int cnc_set_decay_mode(struct cnc *self, enum cnc_axis axis, enum cnc_decay_mode mode)
{
  int pin;
  switch (axis) {
    case AXIS_X: pin = PIN_X_DECAY; break;
    case AXIS_Y: pin = PIN_Y_DECAY; break;
    default:     return -EINVAL;
  }
  switch (mode) {
    case MODE_DECAY_SLOW:  gpio_direction_output(self->gpios[pin], 0); break;
    case MODE_DECAY_MIXED: gpio_direction_input(self->gpios[pin]); break;
    case MODE_DECAY_FAST:  gpio_direction_output(self->gpios[pin], 1); break;
    default:               return -EINVAL;
  }
  self->decay_mode[axis] = mode;
  return 0;
}


enum cnc_decay_mode cnc_get_decay_mode(struct cnc *self, enum cnc_axis axis)
{
  /* Mixed decay is the pin's high-impedance state, so the mode cannot be read
   * back from the pin value alone and gpiolib exposes no stable way to read a
   * line's direction from a driver. The commanded mode is tracked instead;
   * cnc_set_decay_mode is the only writer of these pins, and probe seeds the
   * value from the direction the pins are requested with. */
  if (axis < 0 || axis >= NUM_AXES) {
    return -EINVAL;
  }
  return self->decay_mode[axis];
}


int cnc_set_motor_lock(struct cnc *self, u32 motor_lock_bits)
{
  /* Convert argument to a GPIO port bitmask. */
  u32 lock_val = 0;
  if (motor_lock_bits & MOTOR_LOCK_X)  { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_X_STEP])); }
  if (motor_lock_bits & MOTOR_LOCK_Y1) { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_Y1_STEP])); }
  if (motor_lock_bits & MOTOR_LOCK_Y2) { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_Y2_STEP])); }
  if (motor_lock_bits & MOTOR_LOCK_Z)  { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_Z_STEP])); }
  return sdma_set_reg(self->sdmac, &lock_val, ca);
}


u32 cnc_get_motor_lock(struct cnc *self)
{
  u32 lock_val = 0, retval = 0;
  if (sdma_get_reg(self->sdmac, &lock_val, ca)) {
    return 0;
  }
  /* Extract bits from the GPIO port bitmask. */
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_X_STEP])))  { retval |= MOTOR_LOCK_X; }
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_Y1_STEP]))) { retval |= MOTOR_LOCK_Y1; }
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_Y2_STEP]))) { retval |= MOTOR_LOCK_Y2; }
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_Z_STEP])))  { retval |= MOTOR_LOCK_Z; }
  return retval;
}


static void fault_tasklet_fn(unsigned long data)
{
  /* We don't need to protect the status field in this function */
  /* (see note in ramp_update_tasklet_fn()) */
  struct cnc *self = (struct cnc *)data;
  bool need_to_halt = false;
  int bit;

  /* Process each pending fault condition. */
  /* If a fatal fault has occurred, stop the driver immediately. */
  /* If a non-fatal fault has occurred, and the driver is running, */
  /* attempt a controlled deceleration. */
  for (bit = 0; bit < NUM_FAULT_CONDITIONS; bit++) {
    /* test_and_clear_bit() is atomic */
    if (test_and_clear_bit(bit, &self->pending_faults)) {
      dev_err(self->dev, "fault %d", (1 << bit));
      self->status.triggered_faults |= (1 << bit);
      if (FAULT_IS_FATAL(bit)) {
        need_to_halt = true;
      }
    }
  }

  /* sanity check: don't stop if no faults have occurred */
  if (self->status.triggered_faults) {
    if (need_to_halt) {
      dev_err(self->dev, "critical fault occurred: emergency stop");
      _driver_stop(self, STATE_FAULT);
    } else if (self->status.state == STATE_RUNNING) {
      _cnc_decel_start(self);
    }
  }
}


static inline void cnc_assert_fault(struct cnc *self, int fault_num)
{
  /* set_bit() is atomic */
  set_bit(fault_num, &self->pending_faults);
  tasklet_hi_schedule(&self->fault_tasklet);
}


/**
 * Handles FAULT falling edges.
 * Lower 2 bits of dev_id encode the fault signal that was asserted.
 */
static irqreturn_t cnc_fault_irq_handler(int irq, void *dev_id)
{
  struct cnc *self = CNC_FROM_FAULT_DEV_ID(dev_id);
  u32 fault_num = SIGNAL_FROM_FAULT_DEV_ID(dev_id);
  int pin, gpio;

  if (fault_num >= NUM_STEPPER_FAULT_SIGNALS) {
    return IRQ_HANDLED;
  }

  /* De-glitch: only fault if the line is actually low */
  pin = stepper_fault_gpios[fault_num];
  gpio = self->gpios[pin];
  if (gpio_get_value(gpio) != 0) {
    return IRQ_HANDLED;
  }

  if ((self->ignored_faults & (1 << fault_num)) == 0) {
    dev_err(self->dev, "driver fault detected! %d", fault_num);
    cnc_assert_fault(self, fault_num);
  }
  return IRQ_HANDLED;
}


static int cnc_register_fault_irqs(struct cnc *self)
{
  int i;
  int fault_irqs[NUM_STEPPER_FAULT_SIGNALS];
  int initial_fault_state = 0;

  /* Read the initial fault states and look up the irq numbers */
  for (i = 0; i < NUM_STEPPER_FAULT_SIGNALS; i++) {
    int pin_id = stepper_fault_gpios[i];
    int gpio = self->gpios[pin_id];
    int irq = gpio_to_irq(gpio);
    if (irq < 0) {
      dev_err(self->dev, "gpio %d has no irq", gpio);
      return irq;
    }
    fault_irqs[i] = irq;
    /* Fault signals are active low */
    initial_fault_state |= ((gpio_get_value(self->gpios[pin_id]) == 0) << i);
  }

  /* Are we initially in a fault state? Judge on the MASKED value: an
   * ignored fault line must not seed STATE_FAULT. */
  spin_lock_bh(&self->status_lock);
  self->status.triggered_faults = initial_fault_state & (~self->ignored_faults);
  if (self->status.triggered_faults) {
    self->status.state = STATE_FAULT;
  }
  spin_unlock_bh(&self->status_lock);

  /* Register interrupt handlers */
  for (i = 0; i < NUM_STEPPER_FAULT_SIGNALS; i++) {
    int pin_id = stepper_fault_gpios[i];
    int irq = fault_irqs[i];
    /* Encode the fault signal number in the lower 2 bits of the dev_id. */
    /* Only care about falling edges (fault conditions) for now. */
    int ret = devm_request_irq(self->dev,
      irq,
      cnc_fault_irq_handler,
      IRQF_TRIGGER_FALLING,
      pin_configs[pin_id].name,
      FAULT_DEV_ID_FROM_CNC_AND_SIGNAL(self, i));
    if (ret) {
      dev_err(self->dev, "devm_request_irq(%d) failed: %d", irq, ret);
      return ret;
    }
  }
  return 0;
}


static void beam_detect_latch_reset(struct cnc *self)
{
  /* Pulse BEAM_DET_LATCH_RST high then low. */
  gpio_set_value(self->gpios[PIN_BEAM_LATCH_RESET], 1);
  gpio_set_value(self->gpios[PIN_BEAM_LATCH_RESET], 0);
}


#if INSTALL_PANIC_HANDLER
static int cnc_panic_handler(struct notifier_block *nb, unsigned long action, void *data)
{
  struct cnc *self = container_of(nb, struct cnc, panic_notifier);
  /* Runs on the atomic panic chain, so this does only what cannot sleep,
   * take a mutex, or wait on another context:
   *
   *   epit_stop()     two register writes. Stops the tick that clocks pulse
   *                   bytes out of the ring - without it SDMA and EPIT play
   *                   the rest of the ring out with no kernel alive.
   *   io_change_pins() direct GPIO writes. Parks the FIRE line Hi-Z, drops
   *                   the charge pump (so the hardware watchdog cannot be
   *                   fed and the laser dies within its timeout), asserts
   *                   the laser latch reset, and de-energizes the steppers.
   *
   * Deliberately NOT done: _driver_stop() (cancels an hrtimer, which can wait
   * on a callback that will never run again, and notifies sysfs), the
   * regulator (sleeps), and the DMS chain (blocking; SPI and I2C safing). */
  epit_stop(self->epit);
  io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_shutdown_pin_changes);
  return NOTIFY_DONE;
}
#endif


int cnc_probe(struct platform_device *pdev)
{
  struct cnc *self;
  struct device_node *epit_np = NULL;
  u32 sdma_params[2];
  u32 epit_rate;
  int ret = 0;
  /* Contract guard: the header's mirrored SDMA context layout must match the
   * size the imx-sdma driver asserts against the same constant. */
  BUILD_BUG_ON(sizeof(struct sdma_context_data) != SDMA_CONTEXT_DATA_EXPECTED_SIZE);
  if (!cnc_enabled) { dev_info(&pdev->dev, "%s: disabled, skipping", __func__); return 0; }
  dev_info(&pdev->dev, "%s: started", __func__);

  /* Allocate driver data */
  self = devm_kzalloc(&pdev->dev, sizeof(*self), GFP_KERNEL);
  if (!self) {
    return -ENOMEM;
  }
  self->dev = &pdev->dev;

  spin_lock_init(&self->status_lock);
  mutex_init(&self->pulsebuf_lock);
  tasklet_init(&self->fault_tasklet, fault_tasklet_fn, (unsigned long)self);
  cnc_set_step_frequency(self, STEP_FREQUENCY_DEFAULT);
  cnc_set_ramp_rate_hz_per_s(self, RAMP_RATE_DEFAULT_HZ_PER_S);
  hrtimer_init(&self->ramp_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_SOFT);
  self->ramp_timer.function = ramp_update_tasklet_fn;
  hrtimer_init(&self->charge_pump_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_SOFT);
  self->charge_pump_timer.function = charge_pump_timer_cb;
  hrtimer_init(&self->laser_sample_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_SOFT);
  self->laser_sample_timer.function = laser_sample_timer_cb;

#if INITIAL_STATE_DISABLED
  self->status.state = STATE_DISABLED;
#else
  self->status.state = STATE_IDLE;
#endif

  /*
   * Attach a dedicated coherent DMA pool if the DT provides one (a
   * memory-region -> non-reusable shared-dma-pool). The pulse ring is too
   * large and alignment-sensitive to pull reliably from the shared CMA
   * once boot has fragmented it (cma_alloc -EBUSY). The generic device core
   * only auto-attaches "restricted-dma-pool" nodes, so we must do it here.
   * -ENODEV just means no memory-region: fall back to the default allocator.
   */
  ret = of_reserved_mem_device_init(self->dev);
  if (ret && ret != -ENODEV) {
    dev_warn(self->dev, "no dedicated DMA pool (%d); using default allocator", ret);
  }

  /* Reserve memory for pulse data */
  ret = cnc_buffer_init(self);
  if (ret) {
    goto failed_buffer_init;
  }

  /* Set up GPIOs */
  ret = io_init_gpios(pdev->dev.of_node, pin_configs, self->gpios, NUM_GPIO_PINS);
  if (ret) {
    goto failed_io_init;
  }
  /* Both decay pins are requested as inputs, which is mixed decay. */
  self->decay_mode[AXIS_X] = MODE_DECAY_MIXED;
  self->decay_mode[AXIS_Y] = MODE_DECAY_MIXED;
  /* The SDMA script writes a single GPIO data register whose address is
   * derived from the Linux GPIO numbers, assuming the static alias layout
   * (gpio N lives in bank N/32+1). Check that against the device tree so a
   * numbering change shows up in the log instead of silently retargeting
   * every step and laser write. */
  io_verify_base_address(&pdev->dev, pdev->dev.of_node, pin_configs,
    self->gpios, NUM_GPIO_PINS, cnc_sdma_pin_set,
    io_base_address(self->gpios, NUM_GPIO_PINS, cnc_sdma_pin_set));

  /* Set up PWM */
  ret = io_init_pwms(&pdev->dev, &laser_pwm_config, &self->laser_pwm, 1);
  if (ret) {
    goto failed_pwm_init;
  }
  io_pwm_set_duty_cycle(&self->laser_pwm, LASER_PWM_IDLE_DUTY);

  /* Set up timer */
  epit_np = of_parse_phandle(pdev->dev.of_node, "timer", 0);
  if (!epit_np) { /* of_parse_phandle() returns NULL on failure, not ERR_PTR */
    ret = dev_err_probe(&pdev->dev, -ENODEV, "no timer specified");
    goto failed_epit_init;
  }
  self->epit = epit_get(epit_np);
  of_node_put(epit_np);
  if (!self->epit) {
    ret = dev_err_probe(&pdev->dev, -ENODEV, "failed to get EPIT timer");
    goto failed_epit_init;
  }
  ret = epit_init_freerunning(self->epit, NULL, NULL);
  if (ret) {
    goto failed_epit_init;
  }
  /* Step frequencies are this clock divided by an integer, so the clock the
   * device tree pairs with the EPIT sets both the ceiling and the
   * quantization. Read it back (the divisor for 1 Hz is the rate) and state
   * it, rather than leaving the pairing implicit. */
  epit_rate = epit_hz_to_divisor(self->epit, 1);
  if (!epit_rate) {
    ret = dev_err_probe(&pdev->dev, -ENODEV, "EPIT clock rate reads as zero");
    goto failed_epit_init;
  }
  dev_info(&pdev->dev, "EPIT clock %u Hz\n", epit_rate);
  if (epit_rate < MIN_USABLE_EPIT_RATE) {
    dev_warn(&pdev->dev, "EPIT clock %u Hz quantizes step frequencies coarsely "
      "above %u Hz; check which clock the device tree pairs with the timer\n",
      epit_rate, epit_rate / MIN_EPIT_DIVISOR);
  }

  /* Read SDMA channel number and load address */
  if (of_property_read_u32_array(pdev->dev.of_node, "sdma-params",
    sdma_params, ARRAY_SIZE(sdma_params)) == 0) {
    self->sdma_ch_num = sdma_params[0];
    self->sdma_script_origin = sdma_params[1];
  } else {
    ret = dev_err_probe(&pdev->dev, -ENODEV, "sdma-params property not specified");
    goto failed_sdma_init;
  }
  /* Channel 0 is the SDMA command channel and 32 channels exist. */
  if (self->sdma_ch_num == 0 || self->sdma_ch_num >= MAX_SDMA_CHANNELS) {
    ret = dev_err_probe(&pdev->dev, -EINVAL,
      "sdma-params channel %u out of range (1..%u)",
      self->sdma_ch_num, MAX_SDMA_CHANNELS - 1);
    goto failed_sdma_init;
  }
  /* Set up SDMA and get a channel reference */
  self->sdma = sdma_engine_get();
  if (!self->sdma) {
    ret = dev_err_probe(&pdev->dev, -ENODEV, "failed to get SDMA engine");
    goto failed_sdma_init;
  }
  self->sdmac = sdma_get_channel(self->sdma, self->sdma_ch_num);
  if (!self->sdmac) {
    ret = dev_err_probe(&pdev->dev, -ENODEV, "failed to get SDMA channel %d",
      self->sdma_ch_num);
    goto failed_sdma_init;
  }
  /* This channel is driven directly, outside dmaengine's allocator, so it
   * must not also be handed to a peripheral through a dmas property. State
   * the claim in the log; the pre-run script verification catches the case
   * where something else has since reprogrammed it. */
  dev_info(&pdev->dev, "SDMA channel %u reserved for pulse playback "
    "(script at halfword %u)\n", self->sdma_ch_num, self->sdma_script_origin * 2);

  /* Load the SDMA script */
  ret = load_sdma_script(self);
  if (ret) {
    goto failed_load_sdma_script;
  }
  sdma_set_channel_interrupt_callback(self->sdmac, cnc_sdma_interrupt, self);

  platform_set_drvdata(pdev, self);

  /* Acquire the 40V supply (the DT cnc node must carry 40v-supply).
   * PTR_ERR propagation matters: -EPROBE_DEFER must reach the driver core. */
  self->supply_40v = devm_regulator_get_exclusive(&pdev->dev, "40v");
  if (IS_ERR(self->supply_40v)) {
    ret = dev_err_probe(&pdev->dev, PTR_ERR(self->supply_40v),
      "failed to get 40V regulator");
    goto failed_regulator_get;
  }

  /* Register fault interrupt handlers (devm-managed) */
  ret = cnc_register_fault_irqs(self);
  if (ret) {
    goto failed_register_fault_irqs;
  }

  /* Create sysfs attributes */
  ret = sysfs_create_group(&pdev->dev.kobj, &cnc_attr_group);
  if (ret < 0) {
    dev_err(&pdev->dev, "failed to register attribute group");
    goto failed_create_group;
  }
  self->state_attr_node = sysfs_get_dirent(pdev->dev.kobj.sd, STR(ATTR_STATE));
  if (!self->state_attr_node) {
    ret = dev_err_probe(&pdev->dev, -ENODEV, "could not get node for state attribute");
    goto failed_get_dirent;
  }

  /* Add a link in /sys/glowforge */
  ret = sysfs_create_link(glowforge_kobj, &pdev->dev.kobj, CNC_GROUP_NAME);
  if (ret) {
    goto failed_create_link;
  }

  /* Interlock-latch drive: blocked from here (the pin also powers up high)
   * until the gpio-keys switch device attaches and reports the loop closed. */
  cnc_interlock_init(&self->interlock, cnc_interlock_drive);
  ret = cnc_interlock_register(&self->interlock, "glowforge-interlock");
  if (ret) {
    dev_err(&pdev->dev, "failed to register the interlock input handler: %d", ret);
    goto failed_interlock_register;
  }

  /* Create /dev/glowforge LAST: the device becomes visible to userspace
   * only once everything behind it (regulator, IRQs, sysfs) is up, so
   * the unwind can never deregister a device someone already opened. */
  self->pulsedev.minor = MISC_DYNAMIC_MINOR;
  self->pulsedev.name = PULSE_DEVICE_NAME;
  self->pulsedev.fops = &pulsedev_fops;
  self->pulsedev.parent = &pdev->dev;
  ret = misc_register(&self->pulsedev);
  if (ret) {
    dev_err(&pdev->dev, "unable to register " PULSE_DEVICE_PATH);
    goto failed_pulsedev_register;
  }

#if !INITIAL_STATE_DISABLED
  /* Enable the steppers. */
  stepper_power_on(self);
#endif

#if INSTALL_PANIC_HANDLER
  /* Register panic handler */
  self->panic_notifier.notifier_call = cnc_panic_handler;
  atomic_notifier_chain_register(&panic_notifier_list, &self->panic_notifier);
#endif

  /* Start the free-running laser-safety sampler now that the GPIOs are up. */
  hrtimer_start(&self->laser_sample_timer, laser_sample_interval_ktime, HRTIMER_MODE_REL_SOFT);

  dev_info(&pdev->dev, "%s: done", __func__);
  return 0;

failed_pulsedev_register:
  cnc_interlock_unregister(&self->interlock);
failed_interlock_register:
  sysfs_remove_link(glowforge_kobj, CNC_GROUP_NAME);
failed_create_link:
  {
    /* NULL first: a fault IRQ's tasklet can call
     * cnc_notify_state_changed concurrently. */
    struct kernfs_node *state_node = self->state_attr_node;
    self->state_attr_node = NULL;
    sysfs_put(state_node);
  }
failed_get_dirent:
  sysfs_remove_group(&pdev->dev.kobj, &cnc_attr_group);
failed_create_group:
failed_register_fault_irqs:
failed_regulator_get:
failed_load_sdma_script:
failed_sdma_init:
  /* The channel must never keep a callback into this (devm-freed on
   * probe failure) driver data - e.g. across an -EPROBE_DEFER cycle. */
  if (self->sdmac) {
    sdma_set_channel_interrupt_callback(self->sdmac, NULL, NULL);
    sdma_put_channel(self->sdmac);
  }
  epit_stop(self->epit);
failed_epit_init:
  io_release_pwms(&self->laser_pwm, 1);
failed_pwm_init:
  io_release_gpios(self->gpios, NUM_GPIO_PINS);
failed_io_init:
  cnc_buffer_destroy(self);
failed_buffer_init:
  of_reserved_mem_device_release(self->dev);
  return ret;
}


void cnc_remove(struct platform_device *pdev)
{
  struct cnc *self = platform_get_drvdata(pdev);
  struct kernfs_node *state_node;
  if (!cnc_enabled) { return; }
  if (!self) { return; } /* probe never set drvdata (failed early) */
  dev_info(&pdev->dev, "%s: started", __func__);
  /* Remove every userspace surface FIRST, so no open() and no attribute
   * access can race the hardware teardown below (a concurrent cat used
   * to reach gpio_get_value on freed descriptors).
   * The link lives on the shared /sys/glowforge kobject, not the
   * device's; removing it from the wrong kobject left a stale link that
   * made every rebind fail with -EEXIST. */
  misc_deregister(&self->pulsedev);
  /* Detaching from the switch device leaves INTERLOCK_LATCH_RESET high. */
  cnc_interlock_unregister(&self->interlock);
  sysfs_remove_link(glowforge_kobj, CNC_GROUP_NAME);
  sysfs_remove_group(&pdev->dev.kobj, &cnc_attr_group);
  /* NULL first: the fault tasklet can call cnc_notify_state_changed
   * concurrently; then drop the dirent reference instead of leaking it. */
  state_node = self->state_attr_node;
  self->state_attr_node = NULL;
  sysfs_put(state_node);
  /* Stop the engine and its timers. */
  epit_stop(self->epit);
  sdma_set_channel_interrupt_callback(self->sdmac, NULL, NULL);
  hrtimer_cancel(&self->ramp_timer);
  hrtimer_cancel(&self->charge_pump_timer);
  hrtimer_cancel(&self->laser_sample_timer);
#if INSTALL_PANIC_HANDLER
  atomic_notifier_chain_unregister(&panic_notifier_list, &self->panic_notifier);
#endif
  /* Only now release the hardware. */
  io_release_pwms(&self->laser_pwm, 1);
  stepper_power_off(self);
  io_release_gpios(self->gpios, NUM_GPIO_PINS);
  cnc_buffer_destroy(self);
  of_reserved_mem_device_release(self->dev);
  tasklet_kill(&self->fault_tasklet);
  /* Last: nothing above may touch the engine once its clocks are released. */
  sdma_put_channel(self->sdmac);
  dev_info(&pdev->dev, "%s: done", __func__);
  return;
}
