# Building

The bridge is built on **Linux or WSL**. You need `g++`, `make` and `python3`
with `venv` from your system. Everything else (CMake, Ninja, SWIG, pybind11, the
JDK and the cross compilers) is installed under `~/.frc-soar`, not system-wide.

## 1. Get the sources

Clone the repository, then clone Soar into `Soar/` at its root. Soar is compiled
into the bridge statically.

```bash
git clone https://github.com/Suave101/FRC-Soar.git
```

```bash
cd FRC-Soar
```

```bash
git clone https://github.com/SoarGroup/Soar.git Soar
```

To keep Soar somewhere else, pass `-DSOAR_ROOT=/path/to/Soar` to CMake.

## 2. Install the tools

Download the WPILib 2027 Linux installer archive and pass it to the setup
script:

```bash
scripts/setup.sh WPILib_Linux-x64-2027.0.0-alpha-8.tar.gz
```

This creates a Python venv with CMake, Ninja, SWIG and pybind11. From the WPILib
archive it also unpacks the JDK, the Systemcore (ARM64) cross toolchain and the
WPILib C++ headers. The headers are used to check that `WpilibAdapter.h`
compiles.

Building for the roboRIO also needs its toolchain, which is not part of WPILib
2027. Add `--roborio` to download it (opensdk v2025-2, GCC 12.1, about 125 MB):

```bash
scripts/setup.sh --roborio WPILib_Linux-x64-2027.0.0-alpha-8.tar.gz
```

The setup script can be run again safely. Steps that are already done are
skipped.

| Variable | Default | Meaning |
| --- | --- | --- |
| `SOARBRIDGE_TOOLS` | `~/.frc-soar` | Where tools are installed |
| `SOARBRIDGE_BUILD_DIR` | `$SOARBRIDGE_TOOLS/build` | Where build trees go |
| `ROBORIO_TOOLCHAIN` | — | Root of an existing WPILib 2025/2026 roboRIO toolchain, used instead of downloading one |
| `SYSTEMCORE_TOOLCHAIN` | — | Root of an ARM64 toolchain to use instead of the WPILib one |
| `JAVA_HOME` | `$SOARBRIDGE_TOOLS/jdk` | JDK for the Java bindings |

## 3. Build

```bash
scripts/build.sh host
```

| Target | Builds | Runs tests |
| --- | --- | --- |
| `host` | x86-64 C++ library, JNI library + jar, Python module | yes |
| `asan` | C++ only, with AddressSanitizer and LeakSanitizer | yes |
| `systemcore` | ARM64, for Systemcore (WPILib 2027) and arm64 coprocessors | no |
| `roborio` | ARM32, for roboRIO 1 and roboRIO 2 | no |

Extra arguments after the target are passed to CMake, for example
`scripts/build.sh host -DSOARBRIDGE_BUILD_PYTHON=OFF`.

## Output

Artifacts land in `~/.frc-soar/build/<target>/dist`:

| File | Use |
| --- | --- |
| `libsoarbridge.so` | C++ robot projects |
| `libsoarbridge_jni.so` + `soarbridge.jar` | Java robot projects. The jar also contains the `.so` |
| `python/soarbridge.*.so` | RobotPy projects |

Each `.so` contains Soar and SQLite, with their symbols hidden so they cannot
clash with other libraries in the robot program.

## CMake options

| Option | Default | Meaning |
| --- | --- | --- |
| `SOAR_ROOT` | `./Soar` | Soar source checkout |
| `SOARBRIDGE_BUILD_JAVA` | `ON` | Build the JNI library and jar. Skipped with a warning if SWIG or a JDK is missing |
| `SOARBRIDGE_BUILD_PYTHON` | `ON` | Build the pybind11 module |
| `SOARBRIDGE_BUILD_TESTS` | `ON` | Build the tests |
| `SOARBRIDGE_SANITIZE` | `OFF` | Build with AddressSanitizer and LeakSanitizer |
| `WPILIB_INCLUDE_DIRS` | — | WPILib header directories. When set, `WpilibAdapter.h` is compile-checked |
| `SOARBRIDGE_JAVA_HOME` | `$JAVA_HOME` | JDK used for `jni.h`, `javac` and `jar` |
| `SOARBRIDGE_PYTHON_INCLUDE_DIR` | — | Target Python headers, for cross compiling the Python module |
| `SOARBRIDGE_PYTHON_EXT_SUFFIX` | — | Target Python extension suffix, for cross compiling |

## Cross-compiling notes

**roboRIO (ARM32).** The roboRIO 1 and roboRIO 2 are both 32-bit ARMv7
(Cortex-A9, soft-float ABI) and use the same binary. It must be built with the
WPILib roboRIO toolchain (`arm-frc<year>-linux-gnueabi-g++`). A generic
`arm-linux-gnueabihf` compiler produces incompatible code. The jar stores the
library under `native/linux-arm/`, which matches the roboRIO JVM's `os.arch`.

**Systemcore (ARM64).** Uses the `aarch64-systemcore2027-linux-gnu` toolchain
from the WPILib 2027 installer, or a distro `aarch64-linux-gnu-g++`.

**Python.** Cross-compiling the Python module needs the target's Python
headers and extension suffix:

```bash
scripts/build.sh systemcore -DSOARBRIDGE_PYTHON_INCLUDE_DIR=/path/to/include/python3.x -DSOARBRIDGE_PYTHON_EXT_SUFFIX=.cpython-3x-aarch64-linux-gnu.so
```

Without them the Python module is skipped for that target.

## Tests

`host` and `asan` run the test suite with CTest:

- `tests/bridge_test.cpp`: the C++ API end to end.
- `tests/BridgeTest.java`: the Java API.
- `tests/test_python.py`: the Python module.

The tests run real agents (`agents/default.soar`, `tests/generic_io.soar` and
`tests/bad_commands.soar`) against the bridge.
