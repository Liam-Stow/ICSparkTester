#include "RobotContainer.h"
#include <frc2/command/Commands.h>

#include "subsystems/Arm.h"
#include "subsystems/Elevator.h"
#include "subsystems/Feeder.h"
#include "subsystems/Flywheel.h"
#include "subsystems/Turret.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Arm
  _armController.A().WhileTrue(Arm::GetInstance().MaxMotionTo(0_deg));
  _armController.B().WhileTrue(Arm::GetInstance().WPIProfileTo(90_deg));
  _armController.X().WhileTrue(Arm::GetInstance().PIDTo(180_deg));
  _armController.Y().WhileTrue(Arm::GetInstance().DriveWithDutyCycle(-1));

  // Elevator
  _elevatorController.A().WhileTrue(Elevator::GetInstance().MaxMotionTo(1_m));
  _elevatorController.B().WhileTrue(Elevator::GetInstance().WPIProfileTo(0_m));
  _elevatorController.X().WhileTrue(Elevator::GetInstance().PIDTo(0.5_m));
  _elevatorController.Y().WhileTrue(Elevator::GetInstance().DriveWithDutyCycle(-1));

  // Feeder
  // _feederController.A().WhileTrue(Feeder::GetInstance().FeedIn());
  // _feederController.B().WhileTrue(Feeder::GetInstance().FeedOut());

  // Flywheel
  // _flywheelController.A().WhileTrue(Flywheel::GetInstance().SpinUpTo(1000_rpm));
  // _flywheelController.B().WhileTrue(Flywheel::GetInstance().MaxMotionTo(1000_rpm));

  // Turret
  _turretController.A().WhileTrue(Turret::GetInstance().MaxMotionTo(0_deg));
  _turretController.B().WhileTrue(Turret::GetInstance().WPIProfileTo(90_deg));
  _turretController.X().WhileTrue(Turret::GetInstance().PIDTo(180_deg));
  _turretController.Y().WhileTrue(Turret::GetInstance().DriveWithDutyCycle(-1));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
