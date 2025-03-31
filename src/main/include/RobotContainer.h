#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

  frc2::CommandXboxController _armController{0};
  frc2::CommandXboxController _elevatorController{1};
  frc2::CommandXboxController _feederController{2};
  frc2::CommandXboxController _flywheelController{3};
  frc2::CommandXboxController _turretController{4};
};
