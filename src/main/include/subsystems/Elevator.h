#pragma once

#include <frc2/command/SubsystemBase.h>

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  void Periodic() override;

 private:
};
