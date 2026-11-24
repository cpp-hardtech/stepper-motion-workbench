#include <inttypes.h>
#include <stdio.h>

#include "StepperMotion.h"
#include "fas_queue/stepper_queue.h"

void inject_fill_interrupt(int) {}
void noInterrupts() {}
void interrupts() {}

namespace {
int failures = 0;

void check(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    ++failures;
  }
}

struct Fixture {
  StepperMotionEngine engine;
  StepperMotion stepper;
  StepperQueue& queue = fas_queue[0];

  Fixture() {
    engine.init();
    stepper.init(&engine, 0, 0);
    stepper.setDirectionPin(1);
  }

  void prefill(unsigned entries) {
    const stepper_command_s step{3200, 1, true};
    for (unsigned i = 0; i < entries; ++i) {
      check(stepper.addQueueEntry(&step, false) == AQE_OK, "prefill accepted");
    }
  }

  MoveTimedResultCode request(int16_t steps, uint32_t ticks, uint32_t& actual,
                              bool start = false) {
    const auto result = stepper.moveTimed(steps, ticks, &actual, start);
    printf("moveTimed(%d, %" PRIu32 ", start=%d): rc=%d actual=%" PRIu32
           " entries=%u planned position=%" PRId32 " running=%d\n",
           steps, ticks, start, static_cast<int>(result), actual,
           stepper.queueEntries(), stepper.getPositionAfterCommandsCompleted(),
           stepper.isQueueRunning());
    return result;
  }
};

void start_only_reports_queue_result() {
  Fixture f;
  uint32_t actual = 0;
  check(f.request(0, 0, actual, true) ==
            MoveTimedResultCode::ErrorEmptyQueueToStart,
        "start-only call reports an empty queue");
  f.prefill(1);
  f.stepper.forceStop();
  check(f.request(0, 0, actual, true) == MOVE_TIMED_COMMANDS_SUSPENDED,
        "start-only call reports the admission latch");
  check(!f.stepper.isQueueRunning() && f.stepper.queueEntries() == 1,
        "a suspended start leaves the queued step untouched");
  f.stepper.resumeCommands();
  check(f.request(0, 0, actual, true) == MOVE_TIMED_OK &&
            f.stepper.isQueueRunning(),
        "start-only call starts a prepared queue");
}

void no_op_clears_stale_duration() {
  Fixture f;
  uint32_t actual = 12345;
  check(f.request(0, 0, actual) == MOVE_TIMED_EMPTY,
        "zero-duration no-op still reports an empty queue");
  check(actual == 0, "no-op reports zero elapsed ticks");
}

void smallest_signed_step_count_is_too_large() {
  Fixture f;
  uint32_t actual = 12345;
  check(f.request(INT16_MIN, 327680000, actual) == MOVE_TIMED_TOO_LARGE_ERROR,
        "32,768 reverse steps exceed the queue capacity");
  check(actual == 0 && f.stepper.isQueueEmpty(),
        "oversized reverse move leaves the queue untouched");
}

void maximum_duration_does_not_wrap_command_count() {
  Fixture f;
  uint32_t actual = 0;
  check(f.request(1, UINT32_MAX, actual) == MOVE_TIMED_TOO_LARGE_ERROR,
        "a very long step exceeds the queue capacity");
  check(actual == 0 && f.stepper.isQueueEmpty(),
        "oversized slow move appends neither step nor pauses");
}

void pause_that_needs_three_slots_does_not_partially_append() {
  Fixture f;
  f.prefill(14);
  uint32_t actual = 0;
  check(f.request(0, 131071, actual) == MOVE_TIMED_BUSY,
        "three-command dwell must wait when there are two slots");
  check(actual == 0 && f.stepper.queueEntries() == 14,
        "busy dwell leaves the existing queue untouched");
}

void slow_step_that_needs_three_slots_does_not_partially_append() {
  Fixture f;
  f.prefill(14);
  uint32_t actual = 0;
  check(f.request(1, 131071, actual) == MOVE_TIMED_BUSY,
        "three-command step must wait when there are two slots");
  check(actual == 0 && f.stepper.queueEntries() == 14 &&
            f.stepper.getPositionAfterCommandsCompleted() == 14,
        "busy step leaves the existing queue and its position untouched");
}

void maximum_period_with_remainder_does_not_wrap() {
  Fixture f;
  uint32_t actual = 0;
  check(f.request(256, 16777215, actual) == MOVE_TIMED_EMPTY,
        "maximum-period move accepts all its steps");
  unsigned pulses = 0;
  uint32_t queued_ticks = 0;
  for (uint8_t i = f.queue.read_idx; i != f.queue.next_write_idx; ++i) {
    const auto& command = f.queue.entry[i & QUEUE_LEN_MASK];
    pulses += command.steps;
    queued_ticks += (uint32_t)command.steps * command.ticks;
  }
  printf("maximum-period move: pulses=%u queued ticks=%" PRIu32 "\n", pulses,
         queued_ticks);
  check(pulses == 256 && actual == 16776960 && queued_ticks == actual,
        "all pulses use the largest representable period");
}
}  // namespace

int main() {
  start_only_reports_queue_result();
  no_op_clears_stale_duration();
  smallest_signed_step_count_is_too_large();
  maximum_duration_does_not_wrap_command_count();
  pause_that_needs_three_slots_does_not_partially_append();
  slow_step_that_needs_three_slots_does_not_partially_append();
  maximum_period_with_remainder_does_not_wrap();
  return failures != 0;
}
