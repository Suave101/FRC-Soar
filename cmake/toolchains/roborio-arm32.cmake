# ARM32 (ARMv7 Cortex-A9, soft-float ABI) for the NI roboRIO 1 and roboRIO 2.
# Both roboRIO generations use this same 32-bit target ("linuxathena").
#
# Uses the WPILib roboRIO toolchain (arm-frc<year>-linux-gnueabi-g++). A
# generic arm-linux-gnueabihf compiler is NOT usable: the roboRIO is not
# hard-float.
set(CMAKE_SYSTEM_PROCESSOR arm)
set(_frc_env ROBORIO_TOOLCHAIN)
set(_frc_roots
  "$ENV{HOME}/.frc-soar/roborio"
  "$ENV{HOME}/wpilib/2026/roborio"
  "$ENV{HOME}/wpilib/2025/roborio"
  "$ENV{HOME}/wpilib/2024/roborio")
set(_frc_prefixes "arm-frc*-linux-gnueabi" "arm-nilrt-linux-gnueabi")
# Silence GCC's "parameter passing ... changed in GCC 7.1" notes; everything on
# the roboRIO is built with GCC >= 7, so the old ABI never comes into play.
set(CMAKE_CXX_FLAGS_INIT "-Wno-psabi")
include("${CMAKE_CURRENT_LIST_DIR}/frc-common.cmake")
