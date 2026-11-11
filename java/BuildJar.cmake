# Compiles the generated + hand-written Java sources and packs them with the
# native library into one jar. Run with cmake -P; see java/CMakeLists.txt.

file(GLOB sources "${GENERATED_DIR}/*.java" "${SOURCE_DIR}/*.java")
file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}/${NATIVE_DIR}")

# Java 11 bytecode runs on every JDK WPILib has shipped since 2019.
execute_process(
  COMMAND "${JAVAC}" --release 11 -Xlint:-options -nowarn -d "${WORK_DIR}" ${sources}
  RESULT_VARIABLE result)
if(result)
  message(FATAL_ERROR "javac failed")
endif()

file(COPY "${NATIVE_LIB}" DESTINATION "${WORK_DIR}/${NATIVE_DIR}")

execute_process(
  COMMAND "${JAR}" cf "${OUTPUT}" -C "${WORK_DIR}" .
  RESULT_VARIABLE result)
if(result)
  message(FATAL_ERROR "jar failed")
endif()
