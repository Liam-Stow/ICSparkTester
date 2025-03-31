#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include "utilities/ICSparkFlex.h"
#include "Constants.h"

class Feeder : public frc2::SubsystemBase {
 public:
  Feeder();
  static Feeder& GetInstance() { static Feeder inst; return inst; }

  void Periodic() override;
  void SimulationPeriodic() override;

  // Commands
  frc2::CommandPtr FeedIn();
  frc2::CommandPtr FeedOut();

 private:
  // Electronics
  ICSparkFlex _motor{canid::ELEVATOR_MOTOR};
};
