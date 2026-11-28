# Stepper Motion Workbench

Stepper Motion Workbench is an embedded C++ library for driving stepper drivers
through STEP and DIR signals. It queues pulse commands ahead of the timer
peripherals and provides acceleration ramps, timed segments, and coordinated
multi-axis movement.

## Host build

On Linux with GCC or Clang, install CMake 3.20 or newer and a C++17 compiler.

```sh
git clone https://github.com/cpp-hardtech/stepper-motion-workbench.git
cd stepper-motion-workbench
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Run `./build/tests/host/test_29 --command-csv` from the repository root to add
`test_29.commands.csv` to the simulated square trace. Each row records the
start and end of a paired command in 16 MHz ticks, signed step deltas, and both
end positions.

The tests use a simulated stepper queue. They exercise ramp calculations,
position tracking, timed moves, coordinated paths, and the ESP32 I2S and RMT
pulse encoders without connecting a motor. To run the timed-move example
against the simulator and see its output, use
`ctest --test-dir build -V -R '^test_29$'`.

## ESP32 quick start

The [ESP-IDF example](firmware/esp32/) builds against this checkout as a local
component. Use an ESP-IDF 5.x environment, version 5.3 or newer, with the
`esp32` target tools installed.

```sh
cd firmware/esp32
idf.py set-target esp32
idf.py build
idf.py -p PORT flash monitor
```

The example commands 200 steps with GPIO 17 for STEP, GPIO 18 for DIR, and
GPIO 26 for active-low ENABLE. Adjust the pins and enable polarity in
[main.cpp](firmware/esp32/main/main.cpp) for your board and driver. Connect the
signals and common ground to a compatible external stepper driver. The serial
monitor reports the commanded position from the step queue.

To use the library in an existing ESP-IDF application, add the checkout to
`EXTRA_COMPONENT_DIRS` before including the ESP-IDF project CMake file. In an
Arduino project, install this checkout as a local library named
`StepperMotionWorkbench` and open [UsageExample](examples/UsageExample/UsageExample.ino).

## API

- [StepperMotionEngine](src/StepperMotionEngine.h) binds a STEP output to a
  driver queue and manages steppers.
- [StepperMotion](src/StepperMotion.h) sets speed and acceleration, starts
  point-to-point and continuous moves, and queues step or pause commands.
  `getLastPendingCommand()` reads the period, step count, and direction of
  the tail command still waiting in the software queue.
- [FasTimed](src/FasTimed.h) queues coordinated deltas, absolute waypoints,
  and shared dwells. Use `addDeltaExact()` or `addWaypointExact()` to require exact
  tick durations, and `actualTicks64()` to report the committed duration of long
  streams. [FasNAxis](src/FasNAxis.h) plans coordinated motion along waypoints.
- [Examples](examples/) include single-axis, timed, and multi-axis sketches.
  Backend availability and usable pins depend on the board and SDK.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

MIT. See [LICENSE](LICENSE)
