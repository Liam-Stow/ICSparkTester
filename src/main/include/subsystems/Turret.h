#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include "utilities/ICSparkFlex.h"
#include "Constants.h"

class Turret : public frc2::SubsystemBase {
 public:
  Turret();
  static Turret& GetInstance() { static Turret inst; return inst; }

  void Periodic() override;
  void SimulationPeriodic() override;

  // Commands
  frc2::CommandPtr MaxMotionTo(units::turn_t angle);
  frc2::CommandPtr WPIProfileTo(units::turn_t angle);
  frc2::CommandPtr PIDTo(units::turn_t angle);

 private:
  // Electronics
  ICSparkFlex _turretMotor{canid::TURRET_MOTOR};

  // Constants
  static constexpr double GEARING = 20;
  static constexpr units::kilogram_square_meter_t MOI = 0.05_kg_sq_m;

  // Simulation
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2,1,2> _turretSystem = frc::LinearSystemId::DCMotorSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::DCMotorSim _turretSim{_turretSystem, MOTOR_MODEL};
};
