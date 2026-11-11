// Fills RobotInputs from WPILib objects. Header-only and optional: the bridge
// library itself does not depend on WPILib, so include this only from robot
// code that already links wpilibc.
//
// Works with both API generations:
//   - WPILib 2027+ (Systemcore): wpi:: namespace, .hpp headers
//   - WPILib 2020-2026 (roboRIO): frc:: namespace, .h headers
//
//   soarbridge::RobotInputs in;
//   soarbridge::wpilib::ReadMatchState(in);
//   soarbridge::wpilib::ReadDrive(in, drive.GetPose(), drive.GetFieldVelocity());
//   in.hasPiece = !beamBreak.Get();
//   soar.SetInputs(in);
#pragma once

#include "soarbridge/RobotIO.h"

#if __has_include("wpi/driverstation/MatchState.hpp")

#include "wpi/driverstation/MatchState.hpp"
#include "wpi/driverstation/RobotState.hpp"
#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/kinematics/ChassisVelocities.hpp"

namespace soarbridge::wpilib {

using Pose2d = wpi::math::Pose2d;
using ChassisVelocity = wpi::math::ChassisVelocities;

// ^match.time, ^match.mode and ^match.alliance from the Driver Station.
// The alliance is left unchanged while the Driver Station has not reported one.
inline void ReadMatchState(RobotInputs& in) {
  in.matchTime = wpi::MatchState::GetMatchTime().value();
  if (wpi::RobotState::IsDisabled()) {
    in.mode = MatchMode::Disabled;
  } else if (wpi::RobotState::IsAutonomous()) {
    in.mode = MatchMode::Autonomous;
  } else if (wpi::RobotState::IsTeleop()) {
    in.mode = MatchMode::Teleop;
  } else {
    in.mode = MatchMode::Test;  // "utility" mode in 2027
  }
  if (auto alliance = wpi::MatchState::GetAlliance()) {
    in.alliance = *alliance == wpi::Alliance::RED ? Alliance::Red : Alliance::Blue;
  }
}

}  // namespace soarbridge::wpilib

#elif __has_include(<frc/DriverStation.h>)

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

namespace soarbridge::wpilib {

using Pose2d = frc::Pose2d;
using ChassisVelocity = frc::ChassisSpeeds;

// ^match.time, ^match.mode and ^match.alliance from the Driver Station.
// The alliance is left unchanged while the Driver Station has not reported one.
inline void ReadMatchState(RobotInputs& in) {
  in.matchTime = frc::DriverStation::GetMatchTime().value();
  if (frc::DriverStation::IsDisabled()) {
    in.mode = MatchMode::Disabled;
  } else if (frc::DriverStation::IsAutonomous()) {
    in.mode = MatchMode::Autonomous;
  } else if (frc::DriverStation::IsTeleop()) {
    in.mode = MatchMode::Teleop;
  } else {
    in.mode = MatchMode::Test;
  }
  if (auto alliance = frc::DriverStation::GetAlliance()) {
    in.alliance = *alliance == frc::DriverStation::kRed ? Alliance::Red : Alliance::Blue;
  }
}

}  // namespace soarbridge::wpilib

#else
#error "WpilibAdapter.h needs WPILib headers on the include path"
#endif

namespace soarbridge::wpilib {

// ^drive.pose.* and ^drive.velocity.* from odometry. Pass field-relative
// velocities if the agent reasons in field coordinates.
inline void ReadDrive(RobotInputs& in, const Pose2d& pose, const ChassisVelocity& velocity) {
  in.poseX = pose.X().value();
  in.poseY = pose.Y().value();
  in.heading = pose.Rotation().Degrees().value();
  in.velocityX = velocity.vx.value();
  in.velocityY = velocity.vy.value();
}

}  // namespace soarbridge::wpilib
