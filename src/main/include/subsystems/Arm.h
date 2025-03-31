#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include "utilities/ICSparkFlex.h"
#include "Constants.h"

class Arm : public frc2::SubsystemBase {
 public:
  Arm();
  static Arm& GetInstance() { static Arm inst; return inst; }

  void Periodic() override;
  void SimulationPeriodic() override;

  // Commands
  frc2::CommandPtr MaxMotionTo(units::turn_t angle);
  frc2::CommandPtr WPIProfileTo(units::turn_t angle);
  frc2::CommandPtr PIDTo(units::turn_t angle);
  frc2::CommandPtr DriveWithDutyCycle(double dutyCycle);

 private:
  // Electronics
  ICSparkFlex _motor{canid::ARM_MOTOR};

  // Physical Constants
  static constexpr double GEARING = 30;
  static constexpr units::kilogram_t MASS = 1_kg;
  static constexpr units::meter_t LENGTH = 1_m;
  static constexpr units::turn_t MIN_ANGLE = 0_deg;
  static constexpr units::turn_t MAX_ANGLE = 180_deg;
  static constexpr units::turn_t STARTING_ANGLE = 0_deg;
  static constexpr units::kilogram_square_meter_t MOI =
      frc::sim::SingleJointedArmSim::EstimateMOI(LENGTH, MASS);

  // Simulation
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  static constexpr bool SIMULATE_GRAVITY = true;
  frc::sim::SingleJointedArmSim _sim{
      MOTOR_MODEL, GEARING, MOI, LENGTH, MIN_ANGLE, MAX_ANGLE, SIMULATE_GRAVITY, STARTING_ANGLE};
};
