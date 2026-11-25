#ifndef FAS_MOVE_TIMED_H
#define FAS_MOVE_TIMED_H

#include <stdint.h>

#include "fas_arch/result_codes.h"

// This header contains the core command generation logic of
// StepperMotion::moveTimed(). It is kept separate so it can be unit
// tested on the pc_based platform without any real hardware dependencies.
//
// The test provides its own StepperMotion/StepperQueue definitions and
// includes this header to compile the production moveTimed() body verbatim,
// the same pattern as fas_member/add_queue_entry.h.

// A direction change makes the queue driver inject at most one pause command
// per call (driver dependent; up to three pauses total across retries, e.g.
// ESP32 RMT). moveTimed() must therefore treat two slots as reserved for every
// timed move, so that a move is only admitted when the whole move plus the
// direction pauses fits into the queue. Otherwise a direction change could run
// the queue short between the separate pause/step appends and silently drop
// steps (Issue 370). The application should keep the number of queue commands
// a move generates well below QUEUE_LEN/2 so that splitting large moves on the
// application side stays feasible.
//
// A steps==0 pause uses queue_end.count_up XOR prepare_revert: false keeps
// DIR, true issues the pause in the opposite direction (a dir change).
//
// On a pause-injected return (AqeResultCode::DirChangePauseInjected or
// AqeResultCode::DirPin2msPauseAdded) the command was NOT enqueued and
// *actual_duration carries the ticks of the one injected pause. The
// application accumulates this value per retry and computes the drift only
// once the move is accepted (see API documentation).
static uint32_t timedDurationEntries(uint32_t duration) {
  return duration / UINT16_MAX + (duration % UINT16_MAX != 0);
}

MoveTimedResultCode StepperMotion::moveTimed(int16_t steps,
                                                uint32_t duration,
                                                uint32_t* actual_duration,
                                                bool start,
                                                bool prepare_revert) {
  MoveTimedResultCode ret_ok =
      isQueueEmpty() ? MOVE_TIMED_EMPTY : MOVE_TIMED_OK;
  if (actual_duration != NULL) {
    *actual_duration = 0;
  }
  if ((steps == 0) && (duration == 0)) {
    if (start) {
      AqeResultCode ret = addQueueEntry(NULL, true);
      if (ret != AQE_OK) {
        return tmrFrom(ret);
      }
    }
    return ret_ok;
  }
  uint8_t freeEntries = QUEUE_LEN - queueEntries();
  _queue()->_injected_pause_ticks = 0;
  bool count_up = _queue()->queue_end.count_up ^ prepare_revert;
  struct stepper_command_s cmd = {.ticks = 0, .steps = 0, .count_up = count_up};
  if (steps == 0) {
    const uint32_t entries = timedDurationEntries(duration);
    if (entries > QUEUE_LEN) {
      return MOVE_TIMED_TOO_LARGE_ERROR;
    }
    if (entries > freeEntries) {
      return MOVE_TIMED_BUSY;
    }
    while (duration > 0) {
      if (duration <= 65535) {
        // done using one command
        cmd.ticks = duration;
      } else if (duration >= 131072) {
        // need more than one command
        cmd.ticks = 65535;
      } else {
        // just use half of the duration now, and the other half in the next
        // cmd.
        cmd.ticks = duration >> 1;
      }
      AqeResultCode ret = addQueueEntry(&cmd, start);
      if (ret != AQE_OK) {
        if ((actual_duration != NULL) && aqeIsPauseInjected(ret)) {
          *actual_duration += _queue()->_injected_pause_ticks;
        }
        return tmrFrom(ret);
      }
      if (actual_duration != NULL) {
        *actual_duration += cmd.ticks;
      }
      duration -= cmd.ticks;
    }
    return ret_ok;
  }

  cmd.count_up = steps > 0;
  uint16_t steps_left = cmd.count_up ? steps : -(int32_t)steps;
  if (steps_left > QUEUE_LEN * 255) {
    return MOVE_TIMED_TOO_LARGE_ERROR;
  }

  // There are steps to execute
  // Let's first calculate the step rate
  uint32_t rate = duration;
  rate /= steps_left;
  if (rate > 65535) {
    const uint32_t cmds = steps_left * timedDurationEntries(rate);
    if (cmds >= QUEUE_LEN) {
      return MOVE_TIMED_TOO_LARGE_ERROR;
    }
    if (cmds > freeEntries) {
      return MOVE_TIMED_BUSY;
    }
    // Should fit into the queue.
    for (uint16_t steps_i = 0; steps_i < steps_left; steps_i++) {
      uint32_t this_duration = rate;
      cmd.steps = 1;
      while (this_duration) {
        if (this_duration >= 131072) {
          cmd.ticks = 65535;
        } else if (this_duration > 65535) {
          cmd.ticks = this_duration / 2;
        } else {
          cmd.ticks = this_duration;
        }
        this_duration -= cmd.ticks;

        AqeResultCode ret = addQueueEntry(&cmd, start);
        if (ret != AQE_OK) {
          if ((actual_duration != NULL) && aqeIsPauseInjected(ret)) {
            *actual_duration += _queue()->_injected_pause_ticks;
          }
          return tmrFrom(ret);
        }
        if (actual_duration != NULL) {
          *actual_duration += cmd.ticks;
        }
        // remaining are pauses
        cmd.steps = 0;
      }
    }
    return ret_ok;
  }
  // Now we need to run steps at "high" speed.
  if (steps_left > freeEntries * 255) {
    return MOVE_TIMED_BUSY;
  }
  // The steps should fit in
  cmd.ticks = rate;
  uint32_t expected_duration = rate;
  expected_duration *= steps_left;
  // duration must be larger than expected_duration
  uint16_t missing = duration - expected_duration;
  while (steps_left > 0) {
    if (steps_left > 510) {
      cmd.steps = 255;
    } else if (steps_left > 255) {
      cmd.steps = steps_left / 2;
    } else {
      cmd.steps = steps_left;
    }
    if (steps_left <= missing && cmd.ticks < UINT16_MAX) {
      // run the remaining steps bit slower to adjust for missing ticks
      cmd.ticks++;
      missing = 0;  // only increase once
    }
    AqeResultCode ret = addQueueEntry(&cmd, start);
    if (ret != AQE_OK) {
      if ((actual_duration != NULL) && aqeIsPauseInjected(ret)) {
        *actual_duration += _queue()->_injected_pause_ticks;
      }
      return tmrFrom(ret);
    }
    if (actual_duration != NULL) {
      uint32_t d = cmd.ticks;
      d *= cmd.steps;
      *actual_duration += d;
    }
    steps_left -= cmd.steps;
  }
  return ret_ok;
}

#endif  // FAS_MOVE_TIMED_H
