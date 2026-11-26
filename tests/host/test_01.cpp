#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "StepperMotion.h"
#include "fas_queue/stepper_queue.h"

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

void init_queue() {
  fas_queue[0]._initVars();
  fas_queue[1]._initVars();
}

void basic_test() {
  puts("basic_test...");
  init_queue();
  StepperMotion s = StepperMotion();
  s.init(NULL, 0, 0);
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  struct stepper_command_s cmd = {
      .ticks = 10000, .steps = 100, .count_up = true};
  AqeResultCode res = s.addQueueEntry(&cmd);
  assert(res == AQE_OK);
  assert(!s.isQueueEmpty());
}

void queue_full() {
  puts("queue_full...");
  init_queue();
  StepperMotion s = StepperMotion();
  s.init(NULL, 0, 0);
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());
  printf("Queue read/write = %d/%d\n", fas_queue[0].read_idx,
         fas_queue[0].next_write_idx);
  struct stepper_command_s cmd = {
      .ticks = 10000, .steps = 100, .count_up = true};
  for (int i = 0; i < QUEUE_LEN - 1; i++) {
    s.addQueueEntry(&cmd);
    assert(!s.isQueueEmpty());
    assert(!s.isQueueFull());
    printf("Queue read/write = %d/%d\n", fas_queue[0].read_idx,
           fas_queue[0].next_write_idx);
  }
  s.addQueueEntry(&cmd);
  printf("Queue read/write = %d/%d\n", fas_queue[0].read_idx,
         fas_queue[0].next_write_idx);
  assert(!s.isQueueEmpty());
  assert(s.isQueueFull());
  puts("...done");
}

void queue_out_of_range() {
  AqeResultCode res;

  puts("queue_out_of_range...");
  init_queue();
  StepperMotion s = StepperMotion();
  s.init(NULL, 0, 0);
  assert(s.isQueueEmpty());
  assert(0 == s.getCurrentPosition());
  assert(s.isQueueEmpty());
  assert(s.isQueueEmpty());

  uint16_t ticks = s.getMaxSpeedInTicks();
  ticks = ticks - 1;
  struct stepper_command_s cmd2 = {
      .ticks = ticks, .steps = 255, .count_up = true};

  res = s.addQueueEntry(&cmd2);
  test(res == AQE_ERROR_TICKS_TOO_LOW, "Too low ticks should trigger an error");
  assert(s.isQueueEmpty());

  struct stepper_command_s cmd3 = {
      .ticks = MIN_CMD_TICKS - 1, .steps = 1, .count_up = true};

  res = s.addQueueEntry(&cmd3);
  test(res == AQE_ERROR_TICKS_TOO_LOW,
       "Too short command time should trigger an error");
  assert(s.isQueueEmpty());
}

void end_pos_test() {
  init_queue();
  StepperMotion s = StepperMotion();
  s.init(NULL, 0, 0);
  assert(0 == s.getPositionAfterCommandsCompleted());
  struct stepper_command_s cmd = {.ticks = 65535, .steps = 1, .count_up = true};

  assert(AQE_OK == s.addQueueEntry(&cmd));
  assert(1 == s.getPositionAfterCommandsCompleted());
}

void queued_time_test() {
  init_queue();
  StepperMotion s;
  s.init(NULL, 0, 0);
  test(s.ticksInQueue() == 0 && !s.hasTicksInQueue(1),
       "An empty queue has no pending time");

  constexpr uint16_t command_ticks = MIN_CMD_TICKS;
  constexpr uint32_t move_ticks = 2UL * command_ticks;
  const struct stepper_command_s move = {command_ticks, 2, true};
  const struct stepper_command_s pause = {command_ticks, 0, true};
  assert(s.addQueueEntry(&move, false) == AQE_OK);
  assert(!s.isQueueRunning());
  assert(s.queueEntries() == 1);
  const uint32_t buffered_ticks = s.ticksInQueue();
  const bool has_move_ticks = s.hasTicksInQueue(move_ticks);
  printf("Prefilled move: %lu ticks, reaches threshold: %d\n",
         (unsigned long)buffered_ticks, has_move_ticks);
  test(buffered_ticks == move_ticks, "Prefilled head contributes its full time");
  test(has_move_ticks, "Prefilled head satisfies the duration threshold");

  assert(s.addQueueEntry(&pause, false) == AQE_OK);
  const uint32_t total_ticks = move_ticks + command_ticks;
  test(s.ticksInQueue() == total_ticks && s.hasTicksInQueue(total_ticks),
       "A prefilled move and pause both contribute time");
  assert(s.addQueueEntry(NULL, true) == AQE_OK);
  assert(s.isQueueRunning());
  test(s.ticksInQueue() == command_ticks && s.hasTicksInQueue(command_ticks) &&
           !s.hasTicksInQueue(command_ticks + 1),
       "Once running, only commands behind the active head contribute time");
}

void pending_queue_tail_test() {
  init_queue();
  StepperMotion x;
  StepperMotion y;
  x.init(nullptr, 0, 0);
  y.init(nullptr, 1, 1);
  x.setDirectionPin(4);
  y.setDirectionPin(5);

  const stepper_command_s move{3200, 2, false};
  const stepper_command_s pause{6400, 0, false};
  stepper_command_s tail{4800, 7, true};
  const bool empty_has_tail = x.getLastPendingCommand(tail);
  const bool empty_preserves_tail =
      tail.ticks == 4800 && tail.steps == 7 && tail.count_up;

  fas_queue[0].read_idx = UINT8_MAX;
  fas_queue[0].next_write_idx = UINT8_MAX;
  assert(x.addQueueEntry(&move, false) == AQE_OK);
  const bool wrapped_has_tail = x.getLastPendingCommand(tail);
  const bool wrapped_matches_move =
      tail.ticks == move.ticks && tail.steps == move.steps && !tail.count_up;
  assert(x.addQueueEntry(&pause, false) == AQE_OK);
  assert(y.addQueueEntry(&pause, false) == AQE_OK);
  assert(y.addQueueEntry(&move, false) == AQE_OK);
  const auto x_status = x.getQueueStatus();
  const auto y_status = y.getQueueStatus();
  printf("Prefilled move/pause vs pause/move: entries %u/%u, position %ld/%ld,"
         " pending ticks %lu/%lu, ramp periods %lu/%lu\n",
         x_status.entries, y_status.entries,
         (long)x_status.position_after_commands,
         (long)y_status.position_after_commands, (unsigned long)x.ticksInQueue(),
         (unsigned long)y.ticksInQueue(),
         (unsigned long)x.getPeriodInTicksAfterCommandsCompleted(),
         (unsigned long)y.getPeriodInTicksAfterCommandsCompleted());

  test(!empty_has_tail && empty_preserves_tail,
       "Empty queue leaves the caller's command untouched");
  test(wrapped_has_tail && wrapped_matches_move,
       "Prefilled move is visible across the write index wrap");
  test(x.getLastPendingCommand(tail) && tail.ticks == pause.ticks &&
           tail.steps == 0 && !tail.count_up,
       "Idle queue tail preserves the pause duration and direction");
  test(y.getLastPendingCommand(tail) && tail.ticks == move.ticks &&
           tail.steps == move.steps && !tail.count_up,
       "Identical queue totals can have a different tail command");

  assert(x.addQueueEntry(nullptr, true) == AQE_OK);
  test(x.getLastPendingCommand(tail) && tail.ticks == pause.ticks &&
           tail.steps == 0 && !tail.count_up,
       "Running queue reports its last waiting command");
  // Complete the head move in the host queue. The remaining pause is active.
  ++fas_queue[0].read_idx;
  test(!x.getLastPendingCommand(tail) && tail.ticks == pause.ticks &&
           tail.steps == 0 && !tail.count_up,
       "The active command alone is not pending and leaves output untouched");
}

int main() {
  // assert(sizeof(struct queue_entry) == 6);
  basic_test();
  queue_out_of_range();
  queue_full();
  end_pos_test();
  queued_time_test();
  pending_queue_tail_test();
  printf("TEST_01 PASSED\n");
}
