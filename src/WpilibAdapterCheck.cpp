// Compiled only when WPILIB_INCLUDE_DIRS is set: verifies WpilibAdapter.h
// against the installed WPILib headers. Not part of any library.
#include "soarbridge/WpilibAdapter.h"

namespace {
[[maybe_unused]] void Instantiate(soarbridge::RobotInputs& in) {
  soarbridge::wpilib::ReadMatchState(in);
  soarbridge::wpilib::ReadDrive(in, soarbridge::wpilib::Pose2d(),
                                soarbridge::wpilib::ChassisVelocity());
}
}  // namespace
