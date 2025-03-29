#pragma once

#include <frc2/command/SubsystemBase.h>

class Turret : public frc2::SubsystemBase {
 public:
  Turret();

  void Periodic() override;

 private:
};
