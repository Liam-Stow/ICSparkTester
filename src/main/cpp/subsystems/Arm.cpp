#include "subsystems/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/ICSparkConfig.h"

Arm::Arm() {
  frc::SmartDashboard::PutData("Arm", &_motor);

  ICSparkConfig config;
  config.encoder.positionConversionFactor = 1.0 / GEARING;
  config.encoder.velocityConversionFactor = 1.0 / GEARING;
  config.closedLoop.slots[0].p = kP;
  config.closedLoop.slots[0].maxMotion.maxVelocity = 1_tps;
  config.closedLoop.slots[0].maxMotion.maxAcceleration = 1_tr_per_s_sq;
  config.smartCurrentStallLimit = 100_A;
  _motor.OverwriteConfig(config);

  _motor.SetFeedforwardGains(kS, kG, true, kV, kA);

  // _motor.SetPosition(STARTING_ANGLE);
};

void Arm::Periodic() {
  // _motor.UpdateControls();
}

void Arm::SimulationPeriodic() {
  _sim.SetInputVoltage(_motor.CalcSimVoltage());
  _sim.Update(20_ms);
  _motor.IterateSim(_sim.GetVelocity(), _sim.GetAngle());
  frc::SmartDashboard::PutNumber("Sim Arm Position", _sim.GetAngle().convert<units::turns>().value());
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