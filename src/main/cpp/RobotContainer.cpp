// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <subzero/frc2/command/EmptyCommand.h>
#include <frc2/command/RunCommand.h>
#include <subzero/utils/InputUtils.h>

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        subzero::InputUtils::DeadzoneAxes axes = subzero::InputUtils::CalculateCircularDeadzone(
            m_driverController.GetLeftX(), m_driverController.GetLeftY(),
            OIConstants::kDriveDeadband);

        subzero::ITurnToTarget *turnToTarget = nullptr;

        m_drive.Drive(
            -units::meters_per_second_t{axes.y},
            -units::meters_per_second_t{axes.x},
            -units::radians_per_second_t{frc::ApplyDeadband(m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true, DriveConstants::kLoopTime, turnToTarget);
      },
      {&m_drive}));

  RegisterAutos();
}

void RobotContainer::RegisterAutos()
{
  // Create pathplanner named commands here, setup the auto factory

  m_autoChooser.Initialize();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  auto autoType = m_autoChooser.GetSelectedValue();
  autoCommand = m_autoFactory.GetAuto(autoType);

  return autoCommand.get();
}

void RobotContainer::ResetPose()
{
  m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void RobotContainer::DisableSubsystems()
{
  // Call .DisablePid() on any singleAxisSubsystems here
}

void RobotContainer::Initialize()
{
  // Call OnInit() on any singleAxisSubsystems here
}

void RobotContainer::StopMotors()
{
  // Forcibly stop any flywheels here
}

units::degree_t RobotContainer::CurveRotation(double sensitivity, double val,
                                              double inMin, double inMax,
                                              double outMin, double outMax)
{
  auto normalizedValue =
      subzero::TurnToPose::NormalizeScalar(val, inMin, inMax, -1.0, 1.0);
  double cubedVal = std::clamp(pow(normalizedValue, 3), -1.0, 1.0);
  double rampedVal = std::clamp(
      (sensitivity * (cubedVal) + (1.0 - sensitivity) * val), -1.0, 1.0);
  double summedVal = cubedVal + rampedVal;
  return units::degree_t(
      subzero::TurnToPose::NormalizeScalar(summedVal, -1.0, 1.0, outMin, outMax));
}

void RobotContainer::Periodic()
{
  m_turnToPose.Update();

  // publish to nt
  frc::SmartDashboard::PutData("Mechanism 2d", &m_mech);

  // Here is where you manage your state for auto scoring, auto aiming, etc
}

std::optional<frc::Rotation2d> RobotContainer::GetRotationTargetOverride()
{
  auto targetPose = m_turnToPose.GetTargetPose();

  // if you're attempting to auto acquire a game piece
  // get the angle via m_turnToPose.GetTargetHeading()
  // return the angle
  // else
  return std::nullopt;
}

void RobotContainer::ToggleAimbot()
{
  m_aimbotEnabled = !m_aimbotEnabled;
}

void RobotContainer::ToggleAutoScoring()
{
  m_autoScoringEnabled = !m_aimbotEnabled;
}
