#pragma once

#include <frc2/command/SubsystemBase.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  void Periodic() override;

 private:
};
