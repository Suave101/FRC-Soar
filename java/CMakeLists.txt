# JNI library (SWIG-generated) and soarbridge.jar.
#
# Only the JDK's headers and javac are needed, so this also works when cross
# compiling: any Linux JDK's jni.h is valid for every Linux target.

set(SOARBRIDGE_JAVA_HOME "$ENV{JAVA_HOME}" CACHE PATH "JDK used for jni.h, javac and jar")
find_program(SWIG_EXECUTABLE swig)
find_program(Java_JAVAC_EXECUTABLE javac HINTS "${SOARBRIDGE_JAVA_HOME}/bin")
find_program(Java_JAR_EXECUTABLE jar HINTS "${SOARBRIDGE_JAVA_HOME}/bin")
find_program(Java_JAVA_EXECUTABLE java HINTS "${SOARBRIDGE_JAVA_HOME}/bin")
if(NOT SOARBRIDGE_JAVA_HOME AND Java_JAVAC_EXECUTABLE)
  get_filename_component(_javac "${Java_JAVAC_EXECUTABLE}" REALPATH)
  get_filename_component(_bin "${_javac}" DIRECTORY)
  get_filename_component(SOARBRIDGE_JAVA_HOME "${_bin}" DIRECTORY)
endif()

if(NOT SWIG_EXECUTABLE OR NOT Java_JAVAC_EXECUTABLE OR NOT Java_JAR_EXECUTABLE
   OR NOT EXISTS "${SOARBRIDGE_JAVA_HOME}/include/jni.h")
  message(WARNING "soarbridge: Java bindings skipped (need swig and a JDK; "
    "set JAVA_HOME or -DSOARBRIDGE_JAVA_HOME=...)")
  return()
endif()

set(_gen "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(_gen_java "${_gen}/frc/soar")
set(_wrap "${_gen}/soarbridge_wrap.cxx")
set(_headers
  ${CMAKE_SOURCE_DIR}/include/soarbridge/RobotIO.h
  ${CMAKE_SOURCE_DIR}/include/soarbridge/SoarHardwareInterface.h)

add_custom_command(OUTPUT ${_wrap}
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${_gen}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${_gen_java}
  COMMAND ${SWIG_EXECUTABLE} -c++ -java -package frc.soar
          -I${CMAKE_SOURCE_DIR}/include -outdir ${_gen_java} -o ${_wrap}
          ${CMAKE_CURRENT_SOURCE_DIR}/soarbridge.i
  DEPENDS soarbridge.i ${_headers}
  COMMENT "SWIG: generating JNI wrapper"
  VERBATIM)

add_library(soarbridge_jni SHARED ${_wrap})
target_include_directories(soarbridge_jni PRIVATE
  "${SOARBRIDGE_JAVA_HOME}/include" "${SOARBRIDGE_JAVA_HOME}/include/linux")
target_link_libraries(soarbridge_jni PRIVATE soarbridge_core)
soarbridge_hide_static_symbols(soarbridge_jni)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # SWIG's generated code type-puns jlong <-> pointer.
  target_compile_options(soarbridge_jni PRIVATE -fno-strict-aliasing)
endif()

# Resource directory inside the jar must match NativeLoader.platform().
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
  set(_java_arch amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(_java_arch aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
  set(_java_arch arm)
else()
  set(_java_arch "${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(SOARBRIDGE_JAR "${SOARBRIDGE_OUT}/soarbridge.jar" CACHE INTERNAL "")
file(GLOB _java_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/frc/soar/*.java)
add_custom_command(OUTPUT ${SOARBRIDGE_JAR}
  COMMAND ${CMAKE_COMMAND}
          -DJAVAC=${Java_JAVAC_EXECUTABLE} -DJAR=${Java_JAR_EXECUTABLE}
          -DGENERATED_DIR=${_gen_java} -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src/frc/soar
          -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/jar
          -DNATIVE_LIB=$<TARGET_FILE:soarbridge_jni> -DNATIVE_DIR=native/linux-${_java_arch}
          -DOUTPUT=${SOARBRIDGE_JAR}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/BuildJar.cmake
  DEPENDS soarbridge_jni ${_java_sources} BuildJar.cmake
  COMMENT "Building soarbridge.jar"
  VERBATIM)
add_custom_target(soarbridge_jar ALL DEPENDS ${SOARBRIDGE_JAR})

set(Java_JAVAC_EXECUTABLE "${Java_JAVAC_EXECUTABLE}" PARENT_SCOPE)
set(Java_JAVA_EXECUTABLE "${Java_JAVA_EXECUTABLE}" PARENT_SCOPE)

install(TARGETS soarbridge_jni LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${SOARBRIDGE_JAR} DESTINATION share/java)
