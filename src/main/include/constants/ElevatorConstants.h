#pragma once

#include <subzero/constants/ColorConstants.h>
#include <subzero/singleaxis/ISingleAxisSubsystem.h>
#include <units/length.h>
#include <units/velocity.h>

namespace ElevatorConstants
{
    constexpr int kElevatorMotorId = 10;

    constexpr double kElevatorSetP = 40.0;
    constexpr double kElevatorSetI = 0.0;
    constexpr double kElevatorSetD = 0.0;
    constexpr double kElevatorSetIZone = 0.0;
    constexpr double kElevatorSetFF = 0.0;

    constexpr units::inch_t kMinElevatorDistance = 0_in;
    constexpr units::inch_t kMaxElevatorDistance = 36_in;
    constexpr units::inch_t kElevatorExtensionPosition = 20_in;
    constexpr units::inch_t kElevatorRetractPosition = 1_in;
    constexpr units::inch_t kInPerRotation = 1_in / 23.1;
    constexpr units::inch_t kElevatorTolerance = 0.5_in;

    constexpr double kElevatorStepSize = 1.0;
    constexpr double kElevatorHomingSpeed = 1.0;
    constexpr int kTicksPerMotorRotation = 42;

    constexpr units::feet_per_second_t kElevatorExtensionSpeed = 1_fps;
    constexpr double kElevatorVelocityScalar = 1.0;

    static const subzero::SingleAxisMechanism kElevatorMechanism = {
        12_in,
        90_deg,
        6.0,
        subzero::ColorConstants::kAcidGreen};
} // namespace ElevatorConstants