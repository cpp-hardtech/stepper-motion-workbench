# Contributing

Stepper Motion Workbench welcomes fixes and improvements to its motion
planner, command queue, drivers, and examples.

1. Create a topic branch from the default branch and keep the pull request
   focused on one change.
2. Follow the surrounding C++ style. Add a host test for changes to planning
   or queue behavior when that behavior can be simulated.
3. Run the host build and CTest commands in [README.md](README.md).
4. For changes to a hardware backend, state the target and SDK version. For
   hardware runs, include the driver and board, test inputs, and observed
   pulse timing or position.

The project is distributed under [LICENSE](LICENSE). Keep the source and
third-party attributions in [NOTICE](NOTICE) intact.
