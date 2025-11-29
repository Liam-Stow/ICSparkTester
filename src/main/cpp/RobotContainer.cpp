#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include "frc/DriverStation.h"

RobotContainer::RobotContainer() {
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Arm
  _armController.A().OnTrue(_arm.MaxMotionTo(0_deg));
  _armController.B().OnTrue(_arm.WPIProfileTo(90_deg));
  _armController.X().OnTrue(_arm.PIDTo(180_deg));
  _armController.Y().OnTrue(_arm.DriveWithDutyCycle(-1));

  // Elevator
  _elevatorController.A().OnTrue(_elevator.MaxMotionTo(0.5_m));
  _elevatorController.B().OnTrue(_elevator.WPIProfileTo(0_m));
  _elevatorController.X().OnTrue(_elevator.PIDTo(0.5_m));
  _elevatorController.Y().OnTrue(_elevator.DriveWithDutyCycle(0.5));

  // Feeder
  _feederController.A().OnTrue(_feeder.FeedIn());
  _feederController.B().OnTrue(_feeder.FeedOut());

  // Flywheel
  _flywheelController.A().OnTrue(_flywheel.SpinAt(1000_rpm));
  _flywheelController.B().OnTrue(_flywheel.SpinAt(-1000_rpm));

  // Turret
  _turretController.A().OnTrue(_turret.MaxMotionTo(0_deg));
  _turretController.B().OnTrue(_turret.WPIProfileTo(90_deg));
  _turretController.X().OnTrue(_turret.PIDTo(180_deg));
  _turretController.Y().OnTrue(_turret.DriveWithDutyCycle(-1));

}
