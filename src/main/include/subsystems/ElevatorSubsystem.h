#pragma once

#include <frc/RobotBase.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>
#include <subzero/motor/IPidMotorController.h>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.h>

#include <memory>
#include <string>

#include "Constants.h"
#include <subzero/motor/SimPidMotorController.h>

class ElevatorSubsystem : public LinearSingleAxisSubsystem<IPidMotorController>
{
public:
    explicit ElevatorSubsystem(frc::MechanismObject2d *node = nullptr)
        : LinearSingleAxisSubsystem<IPidMotorController>{
              "Elevator",
              frc::RobotBase::IsReal()
                  ? dynamic_cast<IPidMotorController &>(elevatorController)
                  : dynamic_cast<IPidMotorController &>(simElevatorController),
              {ElevatorConstants::kMinElevatorDistance,
               ElevatorConstants::kMaxElevatorDistance,
               ElevatorConstants::kInPerRotation,
               std::nullopt,
               ElevatorConstants::kElevatorExtensionSpeed,
               ElevatorConstants::kElevatorVelocityScalar,
               ElevatorConstants::kElevatorTolerance,
               std::nullopt,
               std::nullopt,
               false,
               ElevatorConstants::kElevatorMechanism,
               [](units::meter_t from)
               {
                   return std::to_string(from.convert<units::inch>().value()) + "inches";
               },
               []
               {
                   return false;
               },
               AutoConstants::kLinearAxisConstraints},
              node} {}

private:
    rev::spark::SparkMax m_motor{ElevatorConstants::kElevatorMotorId, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController m_pidController = m_motor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
    PidSettings m_elevatorPidSettings = {
        ElevatorConstants::kElevatorSetP, ElevatorConstants::kElevatorSetI,
        ElevatorConstants::kElevatorSetD, ElevatorConstants::kElevatorSetIZone,
        ElevatorConstants::kElevatorSetFF};
    SparkMaxController elevatorController{
        "Elevator Motor",
        m_motor,
        m_encoder,
        m_pidController,
        m_elevatorPidSettings,
        nullptr,
        5676_rpm};
    subzero::SimPidMotorController simElevatorController{
        "Sim Elevator Motor", m_elevatorPidSettings, 5676_rpm};
};