# Deploying to the robot

Build for your controller first:

| Controller | Build | Java architecture directory |
| --- | --- | --- |
| roboRIO 1, roboRIO 2 | `scripts/build.sh roborio` | `linux-arm` |
| Systemcore, arm64 coprocessors | `scripts/build.sh systemcore` | `linux-aarch64` |
| Desktop simulation (Linux x86-64) | `scripts/build.sh host` | `linux-amd64` |

The files are in `~/.frc-soar/build/<target>/dist`. Each library contains Soar,
so there is nothing else to install on the robot.

Put your agent (`robot.soar`) wherever your language deploys files, and pass
its full path as the productions path.

## Java

1. Copy `soarbridge.jar` into your project, for example `libs/soarbridge.jar`.
2. Add it to `build.gradle`:

    ```groovy
    dependencies {
        implementation files('libs/soarbridge.jar')
    }
    ```

3. Put `robot.soar` in `src/main/deploy/`. It is deployed with the robot code,
   and `Filesystem.getDeployDirectory() + "/robot.soar"` finds it.

The jar contains the native library for the architecture it was built for.
When the JVM cannot find `libsoarbridge_jni.so` on `java.library.path`, it
extracts the library from the jar to a temporary file and loads it. A jar built
with `roborio` only works on the roboRIO, and one built with `systemcore` only
works on ARM64.

!!! tip "Simulation"
    The jar from `scripts/build.sh host` works in desktop simulation on Linux
    x86-64. Use a different jar for simulation than for the robot, or put
    `libsoarbridge_jni.so` for the other architecture on `java.library.path`.

## C++

1. Add `include/soarbridge/` to your include path.
2. Link `libsoarbridge.so` and deploy it next to your robot program so it can be
   loaded at run time.
3. Put `robot.soar` in `src/main/deploy/` and load it with
   `frc::filesystem::GetDeployDirectory() + "/robot.soar"`.

The library does not depend on WPILib. `WpilibAdapter.h` is an optional,
header-only helper that does. See [The fixed schema](../guide/fixed-schema.md).

## Python (RobotPy)

1. Copy `python/soarbridge.*.so` from the build output into your robot project
   directory, next to `robot.py`. RobotPy deploys the whole project directory.
2. Copy `robot.soar` there as well. On the robot the project lives in
   `/home/lvuser/py/`, so use `/home/lvuser/py/robot.soar`.

The module must be built for the Python version on the robot. See
[Cross-compiling notes](building.md#cross-compiling-notes).

## Real-time priority

On the robot, other threads compete with the Soar thread for the CPU. To keep
latency predictable, give the Soar thread a SCHED_FIFO priority that is
**lower** than the robot loop's:

=== "Java"

    ```java
    // productions, decisionsPerInput, debuggerPort, realtimePriority
    soar = new SoarBridge(path, 1, 0, 10);
    ```

=== "C++"

    ```cpp
    soarbridge::SoarHardwareInterface soar{{.productionsPath = path, .realtimePriority = 10}};
    ```

=== "Python"

    ```python
    soar = soarbridge.SoarHardwareInterface(productions_path=path, realtime_priority=10)
    ```

If the process is not allowed to change priority, the setting is ignored and
the thread keeps normal scheduling.
