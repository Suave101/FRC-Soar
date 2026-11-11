# pybind11 extension module `soarbridge`.
#
# Native builds use the host interpreter. For a cross build, point
# SOARBRIDGE_PYTHON_INCLUDE_DIR at the *target* Python's headers and set
# SOARBRIDGE_PYTHON_EXT_SUFFIX (e.g. .cpython-313-aarch64-linux-gnu.so); Linux
# extension modules do not link libpython, so headers are all that is needed.

if(CMAKE_CROSSCOMPILING)
  set(SOARBRIDGE_PYTHON_INCLUDE_DIR "" CACHE PATH "Target Python include directory")
  set(SOARBRIDGE_PYTHON_EXT_SUFFIX ".so" CACHE STRING "Target Python extension suffix")
  if(NOT EXISTS "${SOARBRIDGE_PYTHON_INCLUDE_DIR}/Python.h")
    message(STATUS "soarbridge: Python module skipped for this cross build "
      "(set SOARBRIDGE_PYTHON_INCLUDE_DIR to the target Python headers)")
    return()
  endif()
  set(PYBIND11_NOPYTHON ON)  # headers only; do not look for a host Python
else()
  set(PYBIND11_FINDPYTHON ON)
endif()

find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
  message(WARNING "soarbridge: Python module skipped (pybind11 not found; "
    "pip install pybind11 and pass -Dpybind11_DIR=$(pybind11-config --cmakedir))")
  return()
endif()

if(CMAKE_CROSSCOMPILING)
  add_library(soarbridge_py MODULE soarbridge_py.cpp)
  target_include_directories(soarbridge_py PRIVATE "${SOARBRIDGE_PYTHON_INCLUDE_DIR}")
  target_link_libraries(soarbridge_py PRIVATE pybind11::headers)
  set_target_properties(soarbridge_py PROPERTIES
    PREFIX "" SUFFIX "${SOARBRIDGE_PYTHON_EXT_SUFFIX}" CXX_VISIBILITY_PRESET hidden)
else()
  pybind11_add_module(soarbridge_py MODULE soarbridge_py.cpp)
  set(Python3_EXECUTABLE "${Python_EXECUTABLE}" PARENT_SCOPE)
endif()

set_target_properties(soarbridge_py PROPERTIES
  OUTPUT_NAME soarbridge
  LIBRARY_OUTPUT_DIRECTORY "${SOARBRIDGE_OUT}/python")
target_link_libraries(soarbridge_py PRIVATE soarbridge_core)
soarbridge_hide_static_symbols(soarbridge_py)

install(TARGETS soarbridge_py LIBRARY DESTINATION python)
