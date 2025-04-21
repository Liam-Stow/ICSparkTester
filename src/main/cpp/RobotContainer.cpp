#include "RobotContainer.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Arm
  _armController.A().WhileTrue(_arm.MaxMotionTo(0_deg));
  _armController.B().WhileTrue(_arm.WPIProfileTo(90_deg));
  _armController.X().WhileTrue(_arm.PIDTo(180_deg));
  _armController.Y().WhileTrue(_arm.DriveWithDutyCycle(-1));

  // Elevator
  _elevatorController.A().WhileTrue(_elevator.MaxMotionTo(0.5_m));
  _elevatorController.B().WhileTrue(_elevator.WPIProfileTo(0_m));
  _elevatorController.X().WhileTrue(_elevator.PIDTo(0.5_m));
  _elevatorController.Y().WhileTrue(_elevator.DriveWithDutyCycle(0.5));

  // Feeder
  _feederController.A().WhileTrue(_feeder.FeedIn());
  _feederController.B().WhileTrue(_feeder.FeedOut());

  // Flywheel
  _flywheelController.A().WhileTrue(_flywheel.SpinAt(1000_rpm));
  _flywheelController.B().WhileTrue(_flywheel.SpinAt(-1000_rpm));

  // Turret
  _turretController.A().WhileTrue(_turret.MaxMotionTo(0_deg));
  _turretController.B().WhileTrue(_turret.WPIProfileTo(90_deg));
  _turretController.X().WhileTrue(_turret.PIDTo(180_deg));
  _turretController.Y().WhileTrue(_turret.DriveWithDutyCycle(-1));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
