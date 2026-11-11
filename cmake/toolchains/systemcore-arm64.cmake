# ARM64 (AArch64) for Systemcore (WPILib 2027+) and arm64 coprocessors.
#
# Uses the WPILib Systemcore toolchain (aarch64-systemcore<year>-linux-gnu-g++),
# falling back to a distro aarch64-linux-gnu cross compiler.
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(_frc_env SYSTEMCORE_TOOLCHAIN)
set(_frc_roots
  "$ENV{HOME}/.frc-soar/wpilib-2027/systemcore"
  "$ENV{HOME}/wpilib/2027/systemcore"
  "$ENV{HOME}/wpilib/2027_alpha1/systemcore"
  "/usr")
set(_frc_prefixes "aarch64-systemcore*-linux-gnu" "aarch64-linux-gnu")
include("${CMAKE_CURRENT_LIST_DIR}/frc-common.cmake")
