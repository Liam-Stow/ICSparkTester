#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "utilities/ICSparkFlex.h"
#include "Constants.h"

class Flywheel : public frc2::SubsystemBase {
 public:
  Flywheel();
  static Flywheel& GetInstance() { static Flywheel inst; return inst; }

  void Periodic() override;
  void SimulationPeriodic() override;
  
  // Commands
  frc2::CommandPtr SpinAt(units::turns_per_second_t velocity);
  frc2::CommandPtr SpinAt(double dutyCycle);

  // Getters
  units::turns_per_second_t GetVelocity();
  double GetMotorDutyCycle();

 private:
  // Electronics
  ICSparkFlex _motor{canid::FLYWHEEL_MOTOR};

  // Simulation
  static constexpr double GEARING = 1.0;
  static constexpr units::kilogram_square_meter_t MOI = 0.05_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<1,1,1> _flywheelSystem = frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _sim{_flywheelSystem, MOTOR_MODEL};
  units::turn_t _simPosition{0};
};
