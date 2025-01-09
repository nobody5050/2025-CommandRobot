// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/MAXSwerveModule.h"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation2d.h>

#include <numbers>

#include "Constants.h"

using namespace ModuleConstants;

MAXSwerveModule::MAXSwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset)
    : m_drivingSpark(drivingCANId, rev::spark::SparkMax::MotorType::kBrushless),
      m_turningSpark(turningCANId,
                     rev::spark::SparkMax::MotorType::kBrushless)
{
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.

    // ! MAY CAUSE BABY AUTO !

    m_drivingSpark.Configure(ModuleConstants::Configs::MAXSwerveModule::DrivingConfig(), rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_turningSpark.Configure(ModuleConstants::Configs::MAXSwerveModule::DrivingConfig(), rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    if (frc::RobotBase::IsReal())
    {
        m_desiredState.angle = frc::Rotation2d(
            units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
    }

    m_drivingEncoder.SetPosition(0);
}

frc::SwerveModuleState MAXSwerveModule::GetState() const
{
    if (!frc::RobotBase::IsReal())
    {
        return GetSimState();
    }

    return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
            units::radian_t{m_turningAbsoluteEncoder.GetPosition() -
                            m_chassisAngularOffset}};
}

frc::SwerveModuleState MAXSwerveModule::GetSimState() const
{
    return frc::SwerveModuleState{
        m_simDriveEncoderVelocity,
        frc::Rotation2d{
            units::radian_t{m_simCurrentAngle.value() - m_chassisAngularOffset}}};
}
frc::SwerveModulePosition MAXSwerveModule::GetPosition() const
{
    if (!frc::RobotBase::IsReal())
    {
        return GetSimPosition();
    }

    return {units::meter_t{m_drivingEncoder.GetPosition()},
            units::radian_t{m_turningAbsoluteEncoder.GetPosition() -
                            m_chassisAngularOffset}};
}

frc::SwerveModulePosition MAXSwerveModule::GetSimPosition() const
{
    return frc::SwerveModulePosition{
        m_simDriveEncoderPosition,
        frc::Rotation2d{
            units::radian_t{m_simCurrentAngle.value() - m_chassisAngularOffset}}};
}

frc::Rotation2d MAXSwerveModule::GetRotation() const
{
    if (!frc::RobotBase::IsReal())
    {
        return frc::Rotation2d{m_simCurrentAngle};
    }
    return frc::Rotation2d{
        units::radian_t{m_turningAbsoluteEncoder.GetPosition()}};
}

void MAXSwerveModule::SetDesiredState(
    const frc::SwerveModuleState &desiredState)
{
    // Apply chassis angular offset to the desired state.
    frc::SwerveModuleState correctedDesiredState{};
    correctedDesiredState.speed = desiredState.speed;
    correctedDesiredState.angle =
        desiredState.angle +
        frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

    // Optimize the reference state to avoid spinning further than 90 degrees.
    frc::SwerveModuleState optimizedDesiredState{
        frc::SwerveModuleState::Optimize(correctedDesiredState, GetRotation())};

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingClosedLoopController.SetReference(
        static_cast<double>(optimizedDesiredState.speed),
        rev::spark::SparkMax::ControlType::kVelocity);
    m_turningClosedLoopController.SetReference(
        optimizedDesiredState.angle.Radians().value(),
        rev::spark::SparkMax::ControlType::kPosition);

    m_desiredState = desiredState;

    if (!frc::RobotBase::IsReal())
    {
        simUpdateDrivePosition(optimizedDesiredState);
        m_simCurrentAngle = optimizedDesiredState.angle.Radians();
    }
}

void MAXSwerveModule::SetMotorVoltage(MotorType type, units::volt_t voltage)
{
    if (type == MotorType::DriveMotor)
    {
        m_drivingSpark.SetVoltage(voltage);
        return;
    }
    m_turningSpark.SetVoltage(voltage);
}

double MAXSwerveModule::Get(MotorType type)
{
    if (type == MotorType::DriveMotor)
    {
        return m_drivingSpark.Get();
    }
    return m_turningSpark.Get();
}

double MAXSwerveModule::GetDistance(MotorType type)
{
    if (type == MotorType::DriveMotor)
    {
        return m_drivingEncoder.GetPosition();
    }
    return m_turningAbsoluteEncoder.GetPosition();
}

double MAXSwerveModule::GetRate(MotorType type)
{
    if (type == MotorType::DriveMotor)
    {
        return m_drivingEncoder.GetVelocity();
    }
    return m_turningAbsoluteEncoder.GetVelocity();
}

void MAXSwerveModule::simUpdateDrivePosition(
    const frc::SwerveModuleState &desiredState)
{
    m_simDriveEncoderVelocity = desiredState.speed;
    m_simDriveEncoderPosition += units::meter_t{
        m_simDriveEncoderVelocity.value() / DriveConstants::kLoopsPerSecond};
}

void MAXSwerveModule::ResetEncoders() { m_drivingEncoder.SetPosition(0); }