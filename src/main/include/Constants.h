// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #define USING_SYSID

// Don't define as TEST_SWERVE_BOT if not using the testing swerve robot
// #define TEST_SWERVE_BOT

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <rev/SparkFlex.h>
#include <subzero/motor/PidMotorController.h>

#include "constants/DriveConstants.h"
#include "constants/OIConstants.h"
#include "constants/VisionConstants.h"
#include "constants/CANConstants.h"
#include "constants/TurnToPoseConstants.h"
#include "constants/ModuleConstants.h"
#include "constants/AutoConstants.h"
#include "constants/MechanismConstants.h"
#include "constants/ElevatorConstants.h"

using SparkMaxController =
    subzero::PidMotorController<rev::spark::SparkMax, rev::spark::SparkClosedLoopController,
                                rev::spark::SparkRelativeEncoder,
                                rev::spark::SparkAbsoluteEncoder>;
using SparkFlexController =
    subzero::PidMotorController<rev::spark::SparkFlex, rev::spark::SparkClosedLoopController,
                                rev::spark::SparkRelativeEncoder,
                                rev::spark::SparkAbsoluteEncoder>;