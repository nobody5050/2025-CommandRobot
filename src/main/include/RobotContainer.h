// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismObject2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <subzero/target/TurnToPose.h>
#include <subzero/frc/smartdashboard/TaggedChooser.cpp>
#include <subzero/autonomous/AutoFactory.h>
#include <subzero/frc2/command/EmptyCommand.h>

#include "subsystems/DriveSubsystem.h"
// #include "subsystems/ElevatorSubsystem.h"

#include "Constants.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  void ResetPose();
  void DisableSubsystems();
  void Initialize();
  void StopMotors();
  void Periodic();
  units::degree_t CurveRotation(double sensitivity, double val, double inMin, double inMax, double outMin, double outMax);

  frc2::Command *GetAutonomousCommand();

  // Default command to run in auto, does *nothing*
  // frc2::CommandPtr autoCommand = subzero::EmptyCommand().ToPtr();
  frc2::CommandPtr autoCommand = frc2::InstantCommand([] {}).ToPtr();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OIConstants::kDriverControllerPort};

  // Dummy, stolen from 2024-CommandRobot
  photon::PhotonPoseEstimator poseFront{
      // layout
      VisionConstants::kTagLayout,
      // strategy
      VisionConstants::kPoseStrategy,
      // offsets
      VisionConstants::kRobotToCam};

  photon::PhotonCamera frontCam{VisionConstants::kFrontCamera};

  std::vector<subzero::PhotonVisionEstimators::PhotonCameraEstimator> poseCameras{
      subzero::PhotonVisionEstimators::PhotonCameraEstimator(poseFront, frontCam)};

  subzero::TurnToPose m_turnToPose{
      {// Rotation constraints
       frc::TrapezoidProfile<units::radians>::Constraints{
           TurnToPoseConstants::kProfileVelocity,
           TurnToPoseConstants::kProfileAcceleration},
       // Turn P
       TurnToPoseConstants::kTurnP,
       // Turn I
       TurnToPoseConstants::kTurnI,
       // Turn D
       TurnToPoseConstants::kTurnD,
       // Translation P
       TurnToPoseConstants::kTurnTranslationP,
       // Translation I
       TurnToPoseConstants::kTurnTranslationI,
       // Translation D
       TurnToPoseConstants::kTurnTranslationD,
       // Pose tolerance
       TurnToPoseConstants::kPoseTolerance},
      [this]
      { return m_drive.GetPose(); },
      [this]
      { return m_drive.GetField(); }};

  // Tagged sendablechooser for autonomous selection
  subzero::TaggedChooser<AutoConstants::AutoType> m_autoChooser{
      AutoConstants::kChooserEntries, AutoConstants::kChooserGroups, "Auto Selector"};

  // Factory for getting the command from pathplanner based on the auto chosen
  subzero::AutoFactory<AutoConstants::AutoType> m_autoFactory{
      AutoConstants::kPpAutos};

  // Mechanism 2d is used for visualizing different parts of the robot
  frc::Mechanism2d m_mech{1, 1};

  // Elevator
  frc::MechanismRoot2d *elevatorRoot = m_mech.GetRoot("Elevator Root", MechanismConstants::kArmRootX, MechanismConstants::kArmRootY);
  frc::MechanismLigament2d *elevatorLigament = elevatorRoot->Append<frc::MechanismLigament2d>("Elevator Ligament", MechanismConstants::kArmPostX, MechanismConstants::kArmPostAngle);

  // The robot's subsystems are defined here...
  subzero::PhotonVisionEstimators m_vision{poseCameras, VisionConstants::kSingleTagStdDevs, VisionConstants::kMultiTagStdDevs};
  DriveSubsystem m_drive{&m_vision};

  // CURRENTLY BROKEN DUE TO REV API CHANGES DO NOT UNCOMMENT
  // ElevatorSubsystem m_elevator{(frc::MechanismObject2d *)m_mech.GetRoot(
  //     "Climber Left", MechanismConstants::kArmRootX,
  //     MechanismConstants::kArmRootY)};

  void ConfigureBindings();

  bool m_aimbotEnabled = false;
  bool m_autoScoringEnabled = false;
  bool m_shouldAim = false;
  void ToggleAimbot();
  void ToggleAutoScoring();
  std::optional<frc::Rotation2d> GetRotationTargetOverride();

  void RegisterAutos();
};
