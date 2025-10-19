#include "subsystems/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/ICSparkConfig.h"

Arm::Arm() {
  frc::SmartDashboard::PutData("Arm", &_motor);

  ICSparkConfig config;
  config.encoder.positionConversionFactor = 1.0 / GEARING;
  config.encoder.velocityConversionFactor = 1.0 / GEARING;
  config.closedLoop.slots[0].p = 1;
  config.closedLoop.slots[0].maxMotion.maxVelocity = 300_rpm;
  config.closedLoop.slots[0].maxMotion.maxAcceleration = 100_rev_per_m_per_s;
  config.smartCurrentStallLimit = 100_A;
  config.closedLoop.slots[0].feedforward.rotationalGravity = 0.6_V;
  config.closedLoop.slots[0].feedforward.velocity = 0.049_V / 1_rpm;
  config.closedLoop.slots[0].feedforward.acceleration = 0.003_V / 1_rev_per_m_per_s;
  _motor.OverwriteConfig(config);

  _motor.SetPosition(STARTING_ANGLE);
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