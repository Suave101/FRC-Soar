#!/usr/bin/env bash
# Configure, build and (for native targets) test the bridge.
#
#   scripts/build.sh host         native build + tests
#   scripts/build.sh asan         native build with AddressSanitizer/LeakSanitizer + tests
#   scripts/build.sh roborio      ARM32 cross build (roboRIO 1 and 2)
#   scripts/build.sh systemcore   ARM64 cross build (Systemcore / arm64 coprocessors)
#
# Tools and toolchains are looked up under $SOARBRIDGE_TOOLS (default
# ~/.frc-soar); scripts/setup.sh populates it. Build trees go to
# $SOARBRIDGE_BUILD_DIR (default $SOARBRIDGE_TOOLS/build).
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
TOOLS=${SOARBRIDGE_TOOLS:-$HOME/.frc-soar}
TARGET=${1:-host}
BUILD=${SOARBRIDGE_BUILD_DIR:-$TOOLS/build}/$TARGET
shift || true

if [ -d "$TOOLS/venv/bin" ]; then PATH="$TOOLS/venv/bin:$PATH"; fi
if [ -z "${JAVA_HOME:-}" ] && [ -d "$TOOLS/jdk" ]; then export JAVA_HOME="$TOOLS/jdk"; fi

args=(-G Ninja -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release)
if command -v pybind11-config >/dev/null; then
  args+=("-Dpybind11_DIR=$(pybind11-config --cmakedir)")
fi

run_tests=1
case "$TARGET" in
  host)
    # Explicit so a cached OFF from an earlier configure does not stick.
    args+=(-DSOARBRIDGE_BUILD_JAVA=ON -DSOARBRIDGE_BUILD_PYTHON=ON)
    if [ -d "$TOOLS/wpilib-2027-headers" ]; then
      args+=("-DWPILIB_INCLUDE_DIRS=$(printf '%s;' "$TOOLS"/wpilib-2027-headers/*)")
    fi ;;
  asan)
    args+=(-DSOARBRIDGE_SANITIZE=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
           -DSOARBRIDGE_BUILD_JAVA=OFF -DSOARBRIDGE_BUILD_PYTHON=OFF) ;;
  roborio)
    args+=("-DCMAKE_TOOLCHAIN_FILE=$ROOT/cmake/toolchains/roborio-arm32.cmake")
    run_tests=0 ;;
  systemcore)
    args+=("-DCMAKE_TOOLCHAIN_FILE=$ROOT/cmake/toolchains/systemcore-arm64.cmake")
    run_tests=0 ;;
  *) echo "unknown target '$TARGET' (host, asan, roborio, systemcore)" >&2; exit 2 ;;
esac

cmake "${args[@]}" "$@"
cmake --build "$BUILD"
if [ "$run_tests" = 1 ]; then
  ASAN_OPTIONS=detect_leaks=1 LSAN_OPTIONS="suppressions=$ROOT/tests/lsan.supp:print_suppressions=1" ctest --test-dir "$BUILD" --output-on-failure
else
  echo "Cross build complete; artifacts:"
  file "$BUILD"/dist/*.so* "$BUILD"/dist/python/*.so 2>/dev/null || ls -l "$BUILD/dist"
fi
echo "Output: $BUILD/dist"
