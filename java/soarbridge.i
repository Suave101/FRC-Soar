// SWIG interface for the JNI layer. The generated classes are an
// implementation detail; robot code uses frc.soar.SoarBridge.
%module SoarBridgeNative

%include "enums.swg"
%include "std_string.i"
%include "std_vector.i"
%javaconst(1);

%{
#include "soarbridge/SoarHardwareInterface.h"
%}

// Load the native library the first time any generated class is touched.
%pragma(java) jniclasscode=%{
  static {
    NativeLoader.load();
  }
%}

// Java method naming: SetInputs -> setInputs.
%rename("%(lowercamelcase)s", %$isfunction) "";

// Command dispatch is done in Java (SoarBridge) so no JNI upcalls are needed.
%ignore soarbridge::CommandHandler;
%ignore soarbridge::SoarHardwareInterface::DispatchCommands;

// Generic commands are copied into frc.soar.SoarCommand by name, so the
// params map itself is not wrapped; Command.paramNames()/getParam() reach it.
%ignore soarbridge::Command::params;
// Constants named Float/String would shadow java.lang inside the enum.
%rename(INT) soarbridge::ValueType::Int;
%rename(FLOAT) soarbridge::ValueType::Float;
%rename(STRING) soarbridge::ValueType::String;
%ignore soarbridge::Value::AsNumber;
%ignore soarbridge::Value::AsBool;
%ignore soarbridge::Value::operator==;
%ignore soarbridge::Value::operator!=;

%template(CommandVector) std::vector<soarbridge::Command>;
%template(StringVector) std::vector<std::string>;

%include "soarbridge/RobotIO.h"
%include "soarbridge/SoarHardwareInterface.h"
