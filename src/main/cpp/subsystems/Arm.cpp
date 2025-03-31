#include "subsystems/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/ICSparkConfig.h"

Arm::Arm() {
  frc::SmartDashboard::PutData("Arm", &_motor);

  ICSparkConfig config;
  config.encoder.positionConversionFactor = 1.0 / GEARING;
  config.encoder.velocityConversionFactor = 1.0 / GEARING;
  config.closedLoop.slots[0].p = 0.4;
  config.closedLoop.slots[0].maxMotion.maxVelocity = 30_tps;
  config.closedLoop.slots[0].maxMotion.maxAcceleration = 200_tr_per_s_sq;
  config.smartCurrentStallLimit = 100_A;
  _motor.OverwriteConfig(config);

  _motor.SetFeedforwardGains(0.0_V, 0.6_V, true, 0.047_V / 1_rpm);

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