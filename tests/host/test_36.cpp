#include <inttypes.h>
#include <stdio.h>

#include "StepperMotion.h"
#include "fas_queue/stepper_queue.h"

namespace {
StepperQueue* insert_after_sample = nullptr;
AqeResultCode insertion = AQE_OK;
int failures = 0;

void check(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    ++failures;
  }
}
}  // namespace

void inject_fill_interrupt(int mark) {
  if (mark == 0 && insert_after_sample != nullptr) {
    StepperQueue* queue = insert_after_sample;
    insert_after_sample = nullptr;
    const stepper_command_s command{3200, 3, true};
    insertion = queue->addQueueEntry(&command, false);
  }
}

void noInterrupts() {}
void interrupts() {}

int main() {
  StepperMotionEngine engine;
  engine.init();
  StepperMotion x;
  StepperMotion y;
  x.init(&engine, 0, 0);
  y.init(&engine, 1, 1);

  const stepper_command_s first{3200, 2, true};
  check(x.addQueueEntry(&first, false) == AQE_OK, "prefill first axis");
  const auto initial = x.getQueueStatus();
  insert_after_sample = &fas_queue[0];
  const unsigned previous_count = x.queueEntries();
  const int32_t later_position = x.getPositionAfterCommandsCompleted();
  const auto updated = x.getQueueStatus();
  const auto idle = y.getQueueStatus();
  printf("queue transition: snapshots=(%u,%" PRId32 ")->(%u,%" PRId32
         ") separate reads=(%u,%" PRId32 ") free=%u idle axis entries=%u\n",
         initial.entries, initial.position_after_commands, updated.entries,
         updated.position_after_commands, previous_count, later_position,
         updated.free_entries, idle.entries);
  check(insertion == AQE_OK && previous_count == 1 && later_position == 5,
        "a new command can land between separate diagnostics");
  check(initial.entries == 1 && initial.free_entries == QUEUE_LEN - 1 &&
            initial.position_after_commands == 2 && !initial.running,
        "prefill snapshot reports the first command and its position together");
  check(updated.entries == 2 && updated.free_entries == QUEUE_LEN - 2 &&
            updated.position_after_commands == 5 && !updated.running,
        "after enqueue both diagnostics include the second command");
  check(idle.entries == 0 && idle.free_entries == QUEUE_LEN &&
            idle.position_after_commands == 0 && !idle.running,
        "status belongs to the requested axis");

  check(x.addQueueEntry(nullptr, true) == AQE_OK, "start the prefilled queue");
  check(x.setSpeedInUs(1000) == 0 && x.setAcceleration(10000) == 0 &&
            x.move(20) == MOVE_OK,
        "start ramp admission");
  const auto running = x.getQueueStatus();
  x.forceStop();
  const auto stopped = x.getQueueStatus();
  const auto reread = x.getQueueStatus();
  const auto untouched_axis = y.getQueueStatus();
  printf("stop snapshots: before queue=%d ramp=%d cause=%u after suspended=%d"
         " cause=%u reread cause=%u other axis cause=%u\n",
         running.running, running.ramp_active,
         static_cast<unsigned>(running.stop_cause), stopped.commands_suspended,
         static_cast<unsigned>(stopped.stop_cause),
         static_cast<unsigned>(reread.stop_cause),
         static_cast<unsigned>(untouched_axis.stop_cause));
  check(running.running && running.ramp_active &&
            !running.commands_suspended && running.stop_cause == StepperStopCause::None,
        "snapshot reports active queue and ramp separately");
  check(stopped.commands_suspended &&
            stopped.stop_cause == StepperStopCause::ForceStop &&
            reread.stop_cause == StepperStopCause::ForceStop &&
            x.takeStopCause() == StepperStopCause::ForceStop,
        "status exposes the stop cause without consuming it");
  check(!untouched_axis.commands_suspended &&
            untouched_axis.stop_cause == StepperStopCause::None,
        "one axis stopping leaves the other axis status alone");
  return failures != 0;
}
