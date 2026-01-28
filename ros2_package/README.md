# RoboOrchard Ros2 Packages

## Build

We use [colcon](https://colcon.readthedocs.io/en/released/) as the build system. Please refer to the [colcon build documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) for more information.

To build all packages:

```bash
colcon build
```

You can also call make to build all packages:

```bash
make build build_args=""
```

Remember to source the workspace:

```bash
source install/setup.bash
```

## Test

Please refer to the [colcon test documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html) for more information.

To run all tests:

```bash
colcon test --event-handlers console_cohesion+
```

To see the test results:

```bash
colcon test-result --all --verbose
```

All the above commands can be called via make too. Please refer to the Makefile for more information.

TUDO:

* Add test configuration according to <https://colcon.readthedocs.io/en/released/reference/verb/test.html>
