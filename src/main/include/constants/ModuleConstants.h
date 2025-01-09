#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>

#include <numbers>

namespace ModuleConstants
{
    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    constexpr bool kTurningEncoderInverted = true;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    constexpr int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    constexpr double kDrivingMotorFreeSpeedRps =
        5676.0 / 60; // NEO free speed is 5676 RPM
    constexpr units::meter_t kWheelDiameter = 0.0762_m;
    constexpr units::meter_t kWheelCircumference =
        kWheelDiameter * std::numbers::pi;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    constexpr double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    constexpr double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
        kDrivingMotorReduction;

    constexpr double kDrivingEncoderPositionFactor =
        (kWheelDiameter.value() * std::numbers::pi) /
        kDrivingMotorReduction; // meters
    constexpr double kDrivingEncoderVelocityFactor =
        ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
        60.0; // meters per second

    constexpr double kTurningEncoderPositionFactor =
        (2 * std::numbers::pi); // radians
    constexpr double kTurningEncoderVelocityFactor =
        (2 * std::numbers::pi) / 60.0; // radians per second

    constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
    constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
        units::radian_t{kTurningEncoderPositionFactor};

    namespace Configs
    {
        class MAXSwerveModule
        {
        public:
            static rev::spark::SparkMaxConfig &DrivingConfig()
            {
                static rev::spark::SparkMaxConfig drivingConfig{};

                // Use module constants to calculate conversion factors and feed forward
                // gain.
                double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                                       std::numbers::pi /
                                       ModuleConstants::kDrivingMotorReduction;
                double drivingVelocityFeedForward =
                    1 / ModuleConstants::kDriveWheelFreeSpeedRps;

                drivingConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
                    .SmartCurrentLimit(50);
                drivingConfig.encoder
                    .PositionConversionFactor(drivingFactor)         // meters
                    .VelocityConversionFactor(drivingFactor / 60.0); // meters per second
                drivingConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .Pid(0.04, 0, 0)
                    .VelocityFF(drivingVelocityFeedForward)
                    .OutputRange(-1, 1);

                return drivingConfig;
            }

            static rev::spark::SparkMaxConfig &TurningConfig()
            {
                static rev::spark::SparkMaxConfig turningConfig{};

                // Use module constants to calculate conversion factor
                double turningFactor = 2 * std::numbers::pi;

                turningConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
                    .SmartCurrentLimit(20);
                turningConfig
                    .absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the
                    // opposite direction of the steering motor in the MAXSwerve Module.
                    .Inverted(true)
                    .PositionConversionFactor(turningFactor)         // radians
                    .VelocityConversionFactor(turningFactor / 60.0); // radians per second
                turningConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .Pid(1, 0, 0)
                    .OutputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the
                    // PID controller to go through 0 to get to the setpoint i.e. going
                    // from 350 degrees to 10 degrees will go through 0 rather than the
                    // other direction which is a longer route.
                    .PositionWrappingEnabled(true)
                    .PositionWrappingInputRange(0, turningFactor);

                return turningConfig;
            }
        };
    } // namespace Configs

} // namespace ModuleConstants