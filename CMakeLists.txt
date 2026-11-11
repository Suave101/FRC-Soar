cmake_minimum_required(VERSION 3.20)
project(soarbridge VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SOARBRIDGE_BUILD_JAVA "Build the SWIG/JNI library and soarbridge.jar" ON)
option(SOARBRIDGE_BUILD_PYTHON "Build the pybind11 Python module" ON)
option(SOARBRIDGE_BUILD_TESTS "Build tests" ON)
option(SOARBRIDGE_SANITIZE "Build everything with AddressSanitizer + LeakSanitizer" OFF)
set(WPILIB_INCLUDE_DIRS "" CACHE STRING
  "WPILib header directories; when set, the WPILib adapter header is compile-checked")

if(SOARBRIDGE_SANITIZE)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=address)
endif()

include(cmake/SoarKernel.cmake)

set(SOARBRIDGE_OUT "${CMAKE_BINARY_DIR}/dist")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${SOARBRIDGE_OUT}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${SOARBRIDGE_OUT}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${SOARBRIDGE_OUT}")

# --- C++ core -----------------------------------------------------------------
# A static core that each language binding absorbs, so every binding ships as a
# single self-contained .so with no libSoar/libsoarbridge to deploy beside it.
add_library(soarbridge_core STATIC src/SoarHardwareInterface.cpp)
target_include_directories(soarbridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(soarbridge_core PRIVATE soar_static)
set_target_properties(soarbridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(soarbridge_core PRIVATE -Wall -Wextra)
endif()

# Shared library for C++ robot projects.
add_library(soarbridge SHARED src/SoarHardwareInterface.cpp)
target_include_directories(soarbridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(soarbridge PRIVATE soar_static)
set_target_properties(soarbridge PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 0)

# Keep Soar and SQLite symbols out of the dynamic symbol tables so they cannot
# clash with other libraries loaded into the robot program.
function(soarbridge_hide_static_symbols target)
  if(UNIX AND NOT APPLE)
    target_link_options(${target} PRIVATE "LINKER:--exclude-libs,ALL")
  endif()
endfunction()
soarbridge_hide_static_symbols(soarbridge)

if(WPILIB_INCLUDE_DIRS)
  add_library(soarbridge_wpilib_check OBJECT src/WpilibAdapterCheck.cpp)
  target_include_directories(soarbridge_wpilib_check PRIVATE include ${WPILIB_INCLUDE_DIRS})
  # WPILib 2027 headers require C++23; the bridge's own headers need only C++17.
  target_compile_features(soarbridge_wpilib_check PRIVATE cxx_std_23)
endif()

if(SOARBRIDGE_BUILD_JAVA)
  add_subdirectory(java)
endif()
if(SOARBRIDGE_BUILD_PYTHON)
  add_subdirectory(python)
endif()
if(SOARBRIDGE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

include(GNUInstallDirs)
install(TARGETS soarbridge LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY agents/ DESTINATION share/soarbridge/agents)
