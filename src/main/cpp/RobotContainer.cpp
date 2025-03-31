#include "RobotContainer.h"
#include <frc2/command/Commands.h>

#include "subsystems/Arm.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _controller.A().WhileTrue(Arm::GetInstance().DriveWithDutyCycle(0.8));
  _controller.B().WhileTrue(Arm::GetInstance().DriveWithDutyCycle(-0.5));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
