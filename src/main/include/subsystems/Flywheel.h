#pragma once

#include <frc2/command/SubsystemBase.h>

class Flywheel : public frc2::SubsystemBase {
 public:
  Flywheel();

  void Periodic() override;

 private:
};
