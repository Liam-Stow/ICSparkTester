#include "RobotContainer.h"
#include <frc2/command/Commands.h>

#include "subsystems/Arm.h"
#include "subsystems/Turret.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _controller.A().WhileTrue(Arm::GetInstance().MaxMotionTo(0_deg));
  _controller.B().WhileTrue(Arm::GetInstance().WPIProfileTo(90_deg));
  _controller.X().WhileTrue(Arm::GetInstance().PIDTo(180_deg));
  _controller.Y().WhileTrue(Arm::GetInstance().DriveWithDutyCycle(-1));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
