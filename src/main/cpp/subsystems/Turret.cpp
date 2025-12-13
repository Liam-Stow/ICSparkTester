#include "subsystems/Turret.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_acceleration.h>
#include <rev/config/SparkBaseConfig.h>

Turret::Turret() {
  frc::SmartDashboard::PutData("Turret", &_motor);

  rev::spark::SparkBaseConfig config;
  config.SmartCurrentLimit(100); // amps
  config.encoder.PositionConversionFactor(1.0 / GEARING);
  config.encoder.VelocityConversionFactor(1.0 / GEARING);
  
  // WPILIb motion profile config in slot 0
  config.closedLoop.P(1.0, rev::spark::ClosedLoopSlot::kSlot0);
  config.closedLoop.maxMotion.CruiseVelocity(30, rev::spark::ClosedLoopSlot::kSlot0);
  config.closedLoop.maxMotion.MaxAcceleration(500, rev::spark::ClosedLoopSlot::kSlot0);
  config.closedLoop.feedForward.kV(0.036, rev::spark::ClosedLoopSlot::kSlot0);
  
  // MAXMotion config in slot 1
  config.closedLoop.P(1.0, rev::spark::ClosedLoopSlot::kSlot1);
  config.closedLoop.maxMotion.CruiseVelocity(30, rev::spark::ClosedLoopSlot::kSlot1);
  config.closedLoop.maxMotion.MaxAcceleration(500, rev::spark::ClosedLoopSlot::kSlot1);
  config.closedLoop.feedForward.kV(0.0, rev::spark::ClosedLoopSlot::kSlot1);

  _motor.OverwriteConfig(config);
};

void Turret::Periodic() {
  _motor.UpdateMotionProfile();
  _motor.CheckAlerts();
}

void Turret::SimulationPeriodic() {
    _turretSim.SetInputVoltage(_motor.CalcSimVoltage());
    _turretSim.Update(20_ms);
    _motor.IterateSim(_turretSim.GetAngularVelocity(), _turretSim.GetAngularPosition());
}

frc2::CommandPtr Turret::MaxMotionTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetMaxMotionTarget(angle, 0_V, rev::spark::ClosedLoopSlot::kSlot1);
  });
}

frc2::CommandPtr Turret::WPIProfileTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetMotionProfileTarget(angle, 0_V, rev::spark::ClosedLoopSlot::kSlot0);
  });
}

frc2::CommandPtr Turret::PIDTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetPositionTarget(angle, 0_V, rev::spark::ClosedLoopSlot::kSlot0);
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