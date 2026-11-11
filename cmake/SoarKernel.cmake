# Builds the Soar kernel + SML client as a static, position-independent library
# straight from the Soar source tree. Soar's own CMakeLists is not used because
# it requires a system SQLite and Tcl, neither of which exists in the FRC cross
# sysroots; Soar ships the SQLite amalgamation, so that is compiled in instead.

set(SOAR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Soar" CACHE PATH "Soar source checkout")
if(NOT EXISTS "${SOAR_ROOT}/Core/SoarKernel/SoarKernel.cxx")
  message(FATAL_ERROR "Soar sources not found at ${SOAR_ROOT}. "
    "Clone https://github.com/SoarGroup/Soar there or set -DSOAR_ROOT=...")
endif()

add_library(soar_static STATIC
  ${SOAR_ROOT}/Core/ClientSML/ClientSML.cxx
  ${SOAR_ROOT}/Core/ConnectionSML/ConnectionSML.cxx
  ${SOAR_ROOT}/Core/ElementXML/ElementXML.cxx
  ${SOAR_ROOT}/Core/KernelSML/KernelSML.cxx
  ${SOAR_ROOT}/Core/SoarKernel/SoarKernel.cxx
  ${SOAR_ROOT}/Core/shared/shared.cxx
  ${SOAR_ROOT}/Core/CLI/CommandLineInterface.cxx
  ${SOAR_ROOT}/Core/SoarKernel/sqlite/sqlite3.c
)

set(_soar_kernel_dirs
  debug_code decision_process episodic_memory explanation_based_chunking
  explanation_memory interface output_manager parsing reinforcement_learning
  semantic_memory shared soar_representation visualizer)
set(_soar_private_includes
  ${SOAR_ROOT}/Core/CLI/src
  ${SOAR_ROOT}/Core/KernelSML/src)
foreach(_dir ${_soar_kernel_dirs})
  list(APPEND _soar_private_includes ${SOAR_ROOT}/Core/SoarKernel/src/${_dir})
endforeach()

# Only the SML client API is exposed to code that links against soar_static.
target_include_directories(soar_static
  PUBLIC
    ${SOAR_ROOT}/Core/ClientSML/src
    ${SOAR_ROOT}/Core/ConnectionSML/src
    ${SOAR_ROOT}/Core/ElementXML/src
    ${SOAR_ROOT}/Core/shared
  PRIVATE
    ${_soar_private_includes})

target_compile_definitions(soar_static
  PUBLIC STATIC_LINKED
  PRIVATE NO_SVS SQLITE_THREADSAFE=1 SQLITE_OMIT_LOAD_EXTENSION)
target_compile_features(soar_static PUBLIC cxx_std_17)
set_target_properties(soar_static PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(soar_static PRIVATE -w)  # third-party code
endif()

find_package(Threads REQUIRED)
target_link_libraries(soar_static PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
