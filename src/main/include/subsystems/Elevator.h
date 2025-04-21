#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <numbers>
#include "utilities/ICSparkFlex.h"
#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  static Elevator& GetInstance() { static Elevator inst; return inst; }

  void Periodic() override;
  void SimulationPeriodic() override;
  units::meter_t TurnsToHeight(units::turn_t turns);
  units::turn_t HeightToTurns(units::meter_t height);
  units::meters_per_second_t RPMToMPS(units::revolutions_per_minute_t rpm);
  units::revolutions_per_minute_t MPSToRPM(units::meters_per_second_t mps);
  units::meter_t GetHeight();
  double GetMotorDutyCycle();

  // Commands
  frc2::CommandPtr MaxMotionTo(units::meter_t height);
  frc2::CommandPtr WPIProfileTo(units::meter_t height);
  frc2::CommandPtr PIDTo(units::meter_t height);
  frc2::CommandPtr DriveWithDutyCycle(double dutyCycle);

 private:
  // Electronics
  ICSparkFlex _motor{canid::ELEVATOR_MOTOR};

  // Constants
  static constexpr double GEARING = 10;
  static constexpr units::kilogram_t CARRIAGE_MASS = 1_kg;
  static constexpr units::meter_t PULLEY_RADIUS = 2_in;
  static constexpr units::meter_t PULLEY_CIRCUMFERENCE = 2 * std::numbers::pi * PULLEY_RADIUS;
  static constexpr units::meter_t MAX_HEIGHT = 1_m;
  static constexpr units::meter_t MIN_HEIGHT = 0_m;
  static constexpr units::meter_t STARTING_HEIGHT = 0.0_m;

  // Simulation
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  static constexpr bool SIMULATE_GRAVITY = true;
  frc::sim::ElevatorSim _sim{MOTOR_MODEL, GEARING,    CARRIAGE_MASS,    PULLEY_RADIUS,
                             MIN_HEIGHT,  MAX_HEIGHT, SIMULATE_GRAVITY, STARTING_HEIGHT};
};
