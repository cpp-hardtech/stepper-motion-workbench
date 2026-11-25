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

int main() {
  // assert(sizeof(struct queue_entry) == 6);
  basic_test();
  queue_out_of_range();
  queue_full();
  end_pos_test();
  queued_time_test();
  printf("TEST_01 PASSED\n");
}
