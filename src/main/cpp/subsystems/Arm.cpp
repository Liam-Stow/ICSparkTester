#include "subsystems/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkBaseConfig.h>

Arm::Arm() {
  frc::SmartDashboard::PutData("Arm", &_motor);

  rev::spark::SparkBaseConfig config;
  config.SmartCurrentLimit(100); // amps
  config.encoder.PositionConversionFactor(1.0 / GEARING);
  config.encoder.VelocityConversionFactor(1.0 / GEARING);
  config.closedLoop.P(2.0);
  config.closedLoop.maxMotion.CruiseVelocity(300); // rpm
  config.closedLoop.maxMotion.MaxAcceleration(100); // rev/s^2
  config.closedLoop.feedForward.kCos(0.07); // V
  config.closedLoop.feedForward.kV(0.06); // V per rpm
  config.closedLoop.feedForward.kA(0.005); // V per rev/s^2
  _motor.OverwriteConfig(config);
};

void Arm::Periodic() {
  _motor.UpdateControls();
}

void Arm::SimulationPeriodic() {
  _sim.SetInputVoltage(_motor.CalcSimVoltage());
  _sim.Update(20_ms);
  _motor.IterateSim(_sim.GetVelocity(), _sim.GetAngle());
}

frc2::CommandPtr Arm::MaxMotionTo(units::turn_t angle) { 
  return RunOnce([this, angle] {
    _motor.SetMaxMotionTarget(angle);
  });
}

frc2::CommandPtr Arm::WPIProfileTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetMotionProfileTarget(angle);
  });
}

frc2::CommandPtr Arm::PIDTo(units::turn_t angle) {
  return RunOnce([this, angle] {
    _motor.SetPositionTarget(angle);
  });
}

frc2::CommandPtr Arm::DriveWithDutyCycle(double dutyCycle) {
  return StartEnd([this, dutyCycle] { _motor.SetDutyCycle(dutyCycle); },
                  [this] { _motor.StopMotor(); });
}

units::turn_t Arm::GetPosition() {
  return _motor.GetPosition();
}

units::turns_per_second_t Arm::GetVelocity() {
  return _motor.GetVelocity();
}

double Arm::GetMotorDutyCycle() {
  return _motor.GetDutyCycle();
}