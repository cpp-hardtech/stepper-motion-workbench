#include <inttypes.h>
#include <stdio.h>

#include "FasTimed.h"
#include "naxis_sim_port.h"

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

template <uint16_t Horizon = 4>
struct Fixture {
  TestStepperMotionEngine engine;
  SimPort x{80, 64};
  SimPort y{80, 64};
  FasTimed<2, Horizon, SimPort, TestStepperMotionEngine> plan{engine};

  Fixture() {
    x.setAcceleration(100000000);
    y.setAcceleration(100000000);
    check(plan.addAxis(0, &x) && plan.addAxis(1, &y), "register both axes");
    plan.syncFromSteppers();
  }
};

void absolute_waypoints_use_the_planned_coordinates() {
  Fixture<> f;
  const int32_t origin[2] = {100, -50};
  const int32_t first[2] = {104, -52};
  const int32_t next[2] = {106, -53};
  const int32_t overflow[2] = {INT32_MIN, INT32_MAX};
  f.plan.setCurrentPosition(origin);
  const auto a = f.plan.addWaypoint(first, 48000);
  const auto b = f.plan.addWaypoint(next, 24000);
  const auto invalid = f.plan.addWaypoint(overflow, 48000);
  f.plan.pump();
  f.x.drain();
  f.y.drain();
  printf("absolute targets: rc=%d/%d/%d planned=(%" PRId32 ",%" PRId32
         ") pulses=(%" PRId32 ",%" PRId32 ") ticks=(%" PRIu32 ",%" PRIu32 ")\n",
         static_cast<int>(a), static_cast<int>(b), static_cast<int>(invalid),
         f.plan.plannedPosition(0), f.plan.plannedPosition(1), f.x.position(),
         f.y.position(), f.x.clock(), f.y.clock());
  check(a == TimedAdd::Ok && b == TimedAdd::Ok && invalid == TimedAdd::Rejected,
        "waypoints accept in-range travel and reject excessive deltas");
  check(f.plan.plannedPosition(0) == 106 && f.plan.plannedPosition(1) == -53 &&
            f.x.position() == 6 && f.y.position() == -3 &&
            f.x.clock() == 72000 && f.y.clock() == 72000,
        "waypoints emit displacement from the committed coordinates");
}

void exact_ticks_are_never_changed() {
  Fixture<> f;
  const int16_t steps[2] = {10, 0};
  const auto invalid = f.plan.addDeltaExact(steps, 20001);
  const auto ticks_after_rejection = f.plan.actualTicks();
  const auto position_after_rejection = f.plan.plannedPosition(0);
  const auto exact = f.plan.addDeltaExact(steps, 20000);
  f.plan.pump();
  f.x.drain();
  f.y.drain();
  printf("exact ticks 20001/20000: rc=%d/%d after rejection=%" PRIu32
         " position=%" PRId32 " emitted=(%" PRIu32 ",%" PRIu32 ")\n",
         static_cast<int>(invalid), static_cast<int>(exact), ticks_after_rejection,
         position_after_rejection, f.x.clock(), f.y.clock());
  check(invalid == TimedAdd::TimingNotAchievable && ticks_after_rejection == 0 &&
            position_after_rejection == 0,
        "an unrepresentable exact span leaves the plan untouched");
  check(exact == TimedAdd::Ok && f.x.clock() == 20000 && f.y.clock() == 20000,
        "a representable exact span keeps both axes on the requested clock");
}

void long_dwell_is_admitted_as_a_unit() {
  Fixture<2> f;
  check(f.plan.addDwellTicks(3200) == TimedAdd::Ok, "one dwell fits in the horizon");
  const auto busy = f.plan.addDwellTicks(100000);
  const auto planned_while_busy = f.plan.actualTicks();
  f.plan.pump();
  f.x.drain();
  f.y.drain();
  f.plan.pump();
  const auto accepted = f.plan.addDwellTicks(100000);
  f.plan.pump();
  f.x.drain();
  f.y.drain();
  printf("100000-tick dwell: rc=%d/%d busy planned=%" PRIu32
         " total clocks=(%" PRIu32 ",%" PRIu32 ")\n",
         static_cast<int>(busy), static_cast<int>(accepted), planned_while_busy,
         f.x.clock(), f.y.clock());
  check(busy == TimedAdd::Rejected && planned_while_busy == 3200,
        "a dwell that exceeds available horizon commits no partial chunks");
  check(accepted == TimedAdd::Ok && f.x.clock() == 103200 &&
            f.y.clock() == 103200 && f.x.position() == 0 && f.y.position() == 0,
        "a long dwell emits its full shared duration and no pulses");
}

void time_accounting_tracks_long_streams() {
  Fixture<> f;
  const int16_t zero[2] = {0, 0};
  uint64_t emitted = 0;
  for (unsigned i = 0; i < 65538; ++i) {
    if (f.plan.addDelta(zero, 65535) != TimedAdd::Ok) {
      check(false, "streamed dwell is accepted");
      return;
    }
    f.plan.pump();
    emitted += f.x.drain();
    f.y.drain();
    f.plan.pump();
  }
  printf("streamed time: emitted=%" PRIu64 " full=%" PRIu64
         " legacy=%" PRIu32 " ticks\n",
         emitted, f.plan.actualTicks64(), f.plan.actualTicks());
  check(emitted == 4295032830ULL && f.plan.actualTicks64() == emitted &&
            f.plan.actualTicks() == 65534,
        "wide tick reporting preserves the clock beyond the legacy range");
}
}  // namespace

int main() {
  absolute_waypoints_use_the_planned_coordinates();
  exact_ticks_are_never_changed();
  long_dwell_is_admitted_as_a_unit();
  time_accounting_tracks_long_streams();
  return failures != 0;
}
