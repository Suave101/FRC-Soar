#!/usr/bin/env bash
# One-time setup of build tools under $SOARBRIDGE_TOOLS (default ~/.frc-soar).
# Needs only g++, make and python3 with venv from the system; nothing is
# installed system-wide.
#
#   scripts/setup.sh [--roborio] [path/to/WPILib_Linux-x64-2027.*.tar.gz]
#
# With a WPILib 2027 installer archive it also unpacks the JDK, the Systemcore
# (ARM64) cross toolchain and the WPILib C++ headers from it.
#
# The roboRIO (ARM32) toolchain is not part of WPILib 2027. --roborio downloads
# it (~125 MB) from https://github.com/wpilibsuite/opensdk/releases into
# $SOARBRIDGE_TOOLS/roborio. Alternatively point ROBORIO_TOOLCHAIN at an
# existing WPILib 2025/2026 roborio toolchain.
set -euo pipefail

TOOLS=${SOARBRIDGE_TOOLS:-$HOME/.frc-soar}
WANT_ROBORIO=0
if [ "${1:-}" = "--roborio" ]; then WANT_ROBORIO=1; shift; fi
ARCHIVE=${1:-}
mkdir -p "$TOOLS"

ROBORIO_TGZ=cortexa9_vfpv3-roborio-academic-2025-x86_64-linux-gnu-Toolchain-12.1.0.tgz
ROBORIO_URL=https://github.com/wpilibsuite/opensdk/releases/download/v2025-2/$ROBORIO_TGZ
if [ "$WANT_ROBORIO" = 1 ] && ! ls "$TOOLS"/roborio/bin/arm-frc*-linux-gnueabi-g++ >/dev/null 2>&1; then
  mkdir -p "$TOOLS/downloads/roborio"
  tgz="$TOOLS/downloads/roborio/$ROBORIO_TGZ"
  [ -s "$tgz" ] || curl -fL -o "$tgz" "$ROBORIO_URL"
  rm -rf "$TOOLS/roborio" && mkdir -p "$TOOLS/roborio"
  # Archive layout: ./roborio-academic/{bin,lib,...}
  tar xzf "$tgz" -C "$TOOLS/roborio" --strip-components=2
fi

if [ ! -x "$TOOLS/venv/bin/cmake" ]; then
  python3 -m venv "$TOOLS/venv"
  "$TOOLS/venv/bin/pip" install --quiet cmake ninja swig pybind11
fi

if [ -n "$ARCHIVE" ]; then
  W="$TOOLS/wpilib-2027"
  if [ ! -f "$W/.extracted" ]; then
    mkdir -p "$W"
    tar xzf "$ARCHIVE" --wildcards '*artifacts.tar.gz' -O | tar xzf - -C "$W"
    touch "$W/.extracted"
  fi
  ln -sfn "$W/jdk" "$TOOLS/jdk"

  H="$TOOLS/wpilib-2027-headers"
  for lib in wpiutil ntcore hal wpilibc wpimath wpinet datalog telemetry; do
    zip=$(find "$W/maven/org/wpilib/$lib" -name '*headers.zip' | head -1)
    if [ -n "$zip" ] && [ ! -d "$H/$lib" ]; then
      mkdir -p "$H/$lib"
      python3 -I -c 'import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])' "$zip" "$H/$lib"
    fi
  done
fi

echo "Tools ready in $TOOLS"
"$TOOLS/venv/bin/cmake" --version | head -1
"$TOOLS/venv/bin/swig" -version | grep Version
[ -x "$TOOLS/jdk/bin/javac" ] && "$TOOLS/jdk/bin/javac" -version || echo "No JDK in $TOOLS/jdk (set JAVA_HOME to build the Java bindings)"
