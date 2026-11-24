#include <inttypes.h>
#include <stdio.h>

#include "StepperMotion.h"
#include "fas_queue/stepper_queue.h"

void inject_fill_interrupt(int) {}
void noInterrupts() {}
void interrupts() {}

class StepperMotionTest {
 public:
  static void fill(StepperMotion& stepper) { stepper.fill_queue(); }
};

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

  explicit Fixture(bool direction_pin = false) {
    engine.init();
    stepper.init(&engine, 0, 0);
    if (direction_pin) {
      stepper.setDirectionPin(1);
    }
    check(stepper.setSpeedInUs(1000) == 0, "configure speed");
    check(stepper.setAcceleration(10000) == 0, "configure acceleration");
  }

  int drain() {
    int pulses = 0;
    for (int i = 0; i < 100 && stepper.isRunning(); ++i) {
      StepperMotionTest::fill(stepper);
      while (!stepper.isQueueEmpty()) {
        const auto& command = queue.entry[queue.read_idx & QUEUE_LEN_MASK];
        pulses += command.countUp ? command.steps : -command.steps;
        ++queue.read_idx;
      }
      queue._isRunning = false;
    }
    check(!stepper.isRunning(), "short simulator move settles");
    return pulses;
  }
};

void reverse_without_direction_pin() {
  Fixture f;
  auto& s = f.stepper;
  check(s.move(40) == MOVE_OK, "forward move accepted without DIR");
  StepperMotionTest::fill(s);
  const int32_t pending_position = s.getPositionAfterCommandsCompleted();
  const unsigned entries = s.queueEntries();
  const auto result = s.runBackward();
  printf("runBackward without DIR: rc=%d planned=%" PRId32 " entries=%u\n",
         static_cast<int>(result), pending_position, entries);
  check(result == MOVE_ERR_NO_DIRECTION_PIN, "runBackward needs DIR");
  if (result == MOVE_ERR_NO_DIRECTION_PIN) {
    const int pulses = f.drain();
    printf("after rejection: pulses=%d position=%" PRId32 "\n", pulses,
           s.getCurrentPosition());
    check(pulses == 40 && s.getCurrentPosition() == 40,
          "rejected reverse preserves the forward move");
  }
}

void reverse_absolute_without_direction_pin() {
  Fixture f;
  auto& s = f.stepper;
  s.setCurrentPosition(20);
  const auto result = s.moveTo(19);
  printf("moveTo(19) without DIR from 20: rc=%d active=%d entries=%u\n",
         static_cast<int>(result), s.isRampGeneratorActive(), s.queueEntries());
  check(result == MOVE_ERR_NO_DIRECTION_PIN, "reverse absolute move needs DIR");
  check(!s.isRunning() && s.getCurrentPosition() == 20,
        "rejected absolute move leaves the axis idle");
}

void invalid_signed_acceleration_preserves_move() {
  Fixture f(true);
  auto& s = f.stepper;
  check(s.move(40) == MOVE_OK, "queue the move before invalid acceleration");
  const auto result = s.moveByAcceleration(INT32_MIN);
  printf("moveByAcceleration(INT32_MIN): rc=%d accel=%" PRIu32 "\n",
         static_cast<int>(result), s.getAcceleration());
  check(result == MOVE_ERR_ACCELERATION_OUT_OF_RANGE,
        "out-of-range acceleration rejected");
  check(s.getAcceleration() == 10000, "configured acceleration preserved");
  if (result == MOVE_ERR_ACCELERATION_OUT_OF_RANGE) {
    const int pulses = f.drain();
    printf("after rejection: pulses=%d position=%" PRId32 "\n", pulses,
           s.getCurrentPosition());
    check(pulses == 40 && s.getCurrentPosition() == 40,
          "rejected acceleration preserves the finite move");
  }
}

void overflowing_enable_delay_preserves_configuration() {
  Fixture f;
  auto& s = f.stepper;
  s.setAutoEnable(true);
  check(s.setDelayToEnable(500) == DELAY_OK, "valid enable delay accepted");
  const auto result = s.setDelayToEnable(268435456);
  s.forwardStep();
  const auto& first = f.queue.entry[f.queue.read_idx & QUEUE_LEN_MASK];
  printf("enable delay 268435456 us: rc=%d first steps=%u ticks=%u\n",
         static_cast<int>(result), first.steps, first.ticks);
  check(result == DELAY_TOO_HIGH, "overflowing enable delay rejected");
  check(first.steps == 0 && first.ticks == 8000,
        "first step still waits for the configured enable delay");
}

void slow_microsecond_limit() {
  Fixture f;
  auto& s = f.stepper;
  const int accepted = s.setSpeedInUs(268435455);
  const uint32_t selected = s.getSpeedInTicks();
  const int rejected = s.setSpeedInUs(268435456);
  printf("speed period 268435455/268435456 us: rc=%d/%d ticks=%" PRIu32
         " after rejection=%" PRIu32 "\n",
         accepted, rejected, selected, s.getSpeedInTicks());
  check(accepted == 0 && selected == 4294967280u,
        "last representable whole microsecond period is accepted");
  check(rejected == -1 && s.getSpeedInTicks() == selected,
        "overflowing period does not replace the configured speed");
}

void slow_millihertz_limit() {
  Fixture f;
  auto& s = f.stepper;
  const int accepted = s.setSpeedInMilliHz(4);
  const uint32_t selected = s.getSpeedInTicks();
  const int rejected = s.setSpeedInMilliHz(3);
  printf("speed 4/3 mHz: rc=%d/%d ticks=%" PRIu32
         " after rejection=%" PRIu32 "\n",
         accepted, rejected, selected, s.getSpeedInTicks());
  check(accepted == 0 && selected == 4000000000u,
        "four millihertz is representable at 16 MHz");
  check(rejected == -1 && s.getSpeedInTicks() == selected,
        "three millihertz exceeds the representable period");
}

void slow_configured_speed_rounding() {
  Fixture f;
  auto& s = f.stepper;
  check(s.setSpeedInTicks(3000000000u) == 0, "slow period configured");
  const uint32_t frequency = s.getSpeedInMilliHz();
  printf("configured period 3000000000 ticks: frequency=%" PRIu32 " mHz\n",
         frequency);
  check(frequency == 5, "millihertz rounding does not overflow");
}

void actual_and_maximum_speed_precision() {
  Fixture f(true);
  auto& s = f.stepper;
  s.setAbsoluteSpeedLimit(96);
  const stepper_command_s reverse = {8192, 2, false};
  check(s.addQueueEntry(&reverse) == AQE_OK, "queue two reverse steps");
  const int32_t actual = s.getCurrentSpeedInMilliHz();
  const uint32_t maximum = s.getMaxSpeedInMilliHz();
  printf("speed period 8192/96 ticks: actual=%" PRId32
         " maximum=%" PRIu32 " mHz\n", actual, maximum);
  check(actual == -1953125, "actual reverse frequency retains all digits");
  check(maximum == 166666666, "maximum frequency retains all digits");
}
}  // namespace

int main() {
  reverse_without_direction_pin();
  reverse_absolute_without_direction_pin();
  invalid_signed_acceleration_preserves_move();
  overflowing_enable_delay_preserves_configuration();
  slow_microsecond_limit();
  slow_millihertz_limit();
  slow_configured_speed_rounding();
  actual_and_maximum_speed_precision();
  return failures != 0;
}
