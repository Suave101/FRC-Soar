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

%template(CommandVector) std::vector<soarbridge::Command>;

%include "soarbridge/RobotIO.h"
%include "soarbridge/SoarHardwareInterface.h"
