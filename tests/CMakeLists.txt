add_executable(bridge_test bridge_test.cpp)
target_link_libraries(bridge_test PRIVATE soarbridge_core)

set(_agents "${CMAKE_SOURCE_DIR}/agents")
set(_tests "${CMAKE_CURRENT_SOURCE_DIR}")

# Tests execute target binaries, so they only run on native builds.
if(NOT CMAKE_CROSSCOMPILING)
  add_test(NAME bridge_cpp COMMAND bridge_test ${_agents} ${_tests})

  if(TARGET soarbridge_py)
    add_test(NAME bridge_python
      COMMAND ${Python3_EXECUTABLE} ${_tests}/test_python.py ${_agents})
    set_tests_properties(bridge_python PROPERTIES
      ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:soarbridge_py>")
  endif()

  if(TARGET soarbridge_jar)
    set(_test_classes "${CMAKE_CURRENT_BINARY_DIR}/java-classes")
    add_custom_command(OUTPUT ${_test_classes}/BridgeTest.class
      COMMAND ${Java_JAVAC_EXECUTABLE} -cp ${SOARBRIDGE_JAR} -d ${_test_classes}
              ${_tests}/BridgeTest.java
      DEPENDS soarbridge_jar ${_tests}/BridgeTest.java)
    add_custom_target(bridge_java_test ALL DEPENDS ${_test_classes}/BridgeTest.class)
    # No java.library.path: the test must load the native library from the jar.
    add_test(NAME bridge_java
      COMMAND ${Java_JAVA_EXECUTABLE} -cp "${SOARBRIDGE_JAR}:${_test_classes}"
              BridgeTest ${_agents})
  endif()
endif()
