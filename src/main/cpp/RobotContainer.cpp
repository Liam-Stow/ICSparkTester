#include "RobotContainer.h"
#include <frc2/command/Commands.h>

#include "subsystems/Arm.h"
#include "subsystems/Turret.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _controller.A().WhileTrue(Turret::GetInstance().MaxMotionTo(0_deg));
  _controller.B().WhileTrue(Turret::GetInstance().WPIProfileTo(90_deg));
  _controller.X().WhileTrue(Turret::GetInstance().PIDTo(180_deg));
  _controller.Y().WhileTrue(Turret::GetInstance().DriveWithDutyCycle(-1));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
