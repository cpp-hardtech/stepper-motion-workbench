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

  Fixture() {
    engine.init();
    stepper.init(&engine, 0, 0);
    stepper.setDirectionPin(1);
    check(stepper.setSpeedInUs(1000) == 0, "configure speed");
    check(stepper.setAcceleration(10000) == 0, "configure acceleration");
  }

  void raw(uint8_t steps) {
    const stepper_command_s command{3200, steps, true};
    check(stepper.addQueueEntry(&command, false) == AQE_OK, "queue raw steps");
  }

  int drain() {
    int pulses = 0;
    for (int i = 0; i < 150 && stepper.isRunning(); ++i) {
      StepperMotionTest::fill(stepper);
      while (!stepper.isQueueEmpty()) {
        const auto& command = queue.entry[queue.read_idx & QUEUE_LEN_MASK];
        pulses += command.countUp ? command.steps : -command.steps;
        ++queue.read_idx;
      }
      queue._isRunning = false;
    }
    check(!stepper.isRunning(), "simulator move settles");
    return pulses;
  }
};

void relative_move_extends_pending_absolute_target() {
  Fixture f;
  auto& s = f.stepper;
  check(s.moveTo(20) == MOVE_OK && s.move(10) == MOVE_OK,
        "absolute and relative requests accepted before refill");
  const int pulses = f.drain();
  printf("moveTo(20), move(10): target=%" PRId32 " position=%" PRId32
         " pulses=%d\n",
         s.targetPos(), s.getCurrentPosition(), pulses);
  check(pulses == 30 && s.getCurrentPosition() == 30,
        "relative move extends the accepted absolute target");
}

void first_relative_move_starts_from_queued_position() {
  Fixture f;
  auto& s = f.stepper;
  f.raw(10);
  check(s.move(5) == MOVE_OK, "first ramp may follow raw commands");
  const int pulses = f.drain();
  printf("raw(10), move(5): target=%" PRId32 " position=%" PRId32
         " pulses=%d\n",
         s.targetPos(), s.getCurrentPosition(), pulses);
  check(pulses == 15 && s.getCurrentPosition() == 15,
        "first relative ramp appends five steps after the queue tail");
}

void rebase_updates_positions_of_queued_commands() {
  Fixture f;
  auto& s = f.stepper;
  f.raw(2);
  f.raw(3);
  s.setCurrentPosition(1000);
  const int32_t rebased = s.getCurrentPosition();
  const int32_t tail = s.getPositionAfterCommandsCompleted();
  ++f.queue.read_idx;
  const int32_t next = s.getCurrentPosition();
  s.setPositionAfterCommandsCompleted(2005);
  const int32_t rebased_again = s.getCurrentPosition();
  printf("rebase raw(2), raw(3): current=%" PRId32 " tail=%" PRId32
         " next=%" PRId32 " after tail rebase=%" PRId32 "\n",
         rebased, tail, next, rebased_again);
  check(rebased == 1000 && tail == 1005 && next == 1002 && rebased_again == 2002,
        "coordinate shifts apply while queued commands remain");
}

void rebase_updates_pending_absolute_target() {
  Fixture f;
  auto& s = f.stepper;
  check(s.moveTo(20) == MOVE_OK, "absolute move accepted before coordinate shift");
  s.setPositionAfterCommandsCompleted(1000);
  const int pulses = f.drain();
  printf("moveTo(20), rebase(1000): target=%" PRId32 " position=%" PRId32
         " pulses=%d\n",
         s.targetPos(), s.getCurrentPosition(), pulses);
  check(pulses == 20 && s.getCurrentPosition() == 1020,
        "rebasing preserves planned travel before the first refill");
}

void half_range_move_plans_reverse_pulses() {
  Fixture f;
  auto& s = f.stepper;
  printf("start moveTo(INT32_MIN) from zero\n");
  check(s.moveTo(INT32_MIN) == MOVE_OK, "half-range reverse request accepted");
  StepperMotionTest::fill(s);
  const auto& first = f.queue.entry[f.queue.read_idx & QUEUE_LEN_MASK];
  printf("half-range move: target=%" PRId32 " countUp=%u steps=%u entries=%u\n",
         s.targetPos(), first.countUp, first.steps, s.queueEntries());
  check(s.targetPos() == INT32_MIN && first.steps > 0 && !first.countUp,
        "half-range request begins in the defined negative direction");
}

void queue_and_coordinate_shift_can_cross_signed_boundary() {
  Fixture f;
  auto& s = f.stepper;
  printf("set position INT32_MAX, queue two forward pulses\n");
  s.setCurrentPosition(INT32_MAX);
  f.raw(2);
  const int32_t wrapped_tail = s.getPositionAfterCommandsCompleted();
  printf("queue boundary: current=%" PRId32 " tail=%" PRId32 "\n",
         s.getCurrentPosition(), wrapped_tail);
  check(wrapped_tail == INT32_MIN + 1 && s.getCurrentPosition() == INT32_MAX,
        "queued position wraps while the current coordinate is unchanged");
  s.setCurrentPosition(INT32_MIN);
  printf("coordinate boundary: current=%" PRId32 " tail=%" PRId32 "\n",
         s.getCurrentPosition(), s.getPositionAfterCommandsCompleted());
  check(s.getCurrentPosition() == INT32_MIN &&
            s.getPositionAfterCommandsCompleted() == INT32_MIN + 2,
        "rebasing across the signed limit preserves the two queued steps");
}
}  // namespace

int main() {
  relative_move_extends_pending_absolute_target();
  first_relative_move_starts_from_queued_position();
  rebase_updates_positions_of_queued_commands();
  rebase_updates_pending_absolute_target();
  half_range_move_plans_reverse_pulses();
  queue_and_coordinate_shift_can_cross_signed_boundary();
  return failures != 0;
}
