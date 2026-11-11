# Shared logic for the FRC cross toolchain files. Expects:
#   _frc_env        environment variable naming the toolchain root
#   _frc_roots      default toolchain roots to search (globs allowed)
#   _frc_prefixes   compiler name globs, e.g. arm-frc*-linux-gnueabi

set(CMAKE_SYSTEM_NAME Linux)

if(DEFINED ENV{${_frc_env}})
  set(_frc_roots "$ENV{${_frc_env}}")
endif()

set(_frc_gxx "")
foreach(_root ${_frc_roots})
  foreach(_prefix ${_frc_prefixes})
    file(GLOB _found "${_root}/bin/${_prefix}-g++")
    if(_found)
      list(SORT _found ORDER DESCENDING)  # newest season first
      list(GET _found 0 _frc_gxx)
      break()
    endif()
  endforeach()
  if(_frc_gxx)
    break()
  endif()
endforeach()

if(NOT _frc_gxx)
  message(FATAL_ERROR
    "No ${_frc_prefixes} cross compiler found under: ${_frc_roots}\n"
    "Install the WPILib toolchain (scripts/setup.sh) or set the ${_frc_env} "
    "environment variable to the toolchain root (the directory containing bin/).")
endif()

string(REGEX REPLACE "g\\+\\+$" "gcc" _frc_gcc "${_frc_gxx}")
set(CMAKE_C_COMPILER "${_frc_gcc}")
set(CMAKE_CXX_COMPILER "${_frc_gxx}")

# Headers and libraries come from the toolchain's sysroot only; build tools
# (swig, javac, ...) come from the host.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
