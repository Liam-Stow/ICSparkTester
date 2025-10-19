#include "subsystems/Turret.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_acceleration.h>

Turret::Turret() {
  frc::SmartDashboard::PutData("Turret", &_motor);

  ICSparkConfig config;
  config.encoder.positionConversionFactor = 1.0 / GEARING;
  config.encoder.velocityConversionFactor = 1.0 / GEARING;
  config.closedLoop.slots[0].p = 1;
  config.closedLoop.slots[0].maxMotion.maxVelocity = 30_rpm;
  config.closedLoop.slots[0].maxMotion.maxAcceleration = 500_rev_per_m_per_s;
  config.smartCurrentStallLimit = 100_A;
  config.closedLoop.slots[0].feedforward.velocity = 0.036_V / 1_rpm;
  _motor.OverwriteConfig(config);
};

void Turret::Periodic() {
  _motor.UpdateControls();
}

void Turret::SimulationPeriodic() {
    _turretSim.SetInputVoltage(_motor.CalcSimVoltage());
    _turretSim.Update(20_ms);
    _motor.IterateSim(_turretSim.GetAngularVelocity(), _turretSim.GetAngularPosition());
}

frc2::CommandPtr Turret::MaxMotionTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetMaxMotionTarget(angle);
  });
}

frc2::CommandPtr Turret::WPIProfileTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetMotionProfileTarget(angle);
  });
}

frc2::CommandPtr Turret::PIDTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetPositionTarget(angle);
  });
}

frc2::CommandPtr Turret::DriveWithDutyCycle(double dutyCycle) {
  return StartEnd([this, dutyCycle] { _motor.SetDutyCycle(dutyCycle); },
                  [this] { _motor.StopMotor(); });
}

units::turn_t Turret::GetPosition() {
  return _motor.GetPosition();
}

double Turret::GetMotorDutyCycle() {
  return _motor.GetDutyCycle();
}