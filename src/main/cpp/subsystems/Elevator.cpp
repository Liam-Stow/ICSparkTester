#include "subsystems/Elevator.h"
#include <frc/smartdashboard/SmartDashboard.h>

Elevator::Elevator() {
    frc::SmartDashboard::PutData("Elevator", &_motor);

    ICSparkConfig config;
    config.encoder.positionConversionFactor = 1.0 / GEARING;
    config.encoder.velocityConversionFactor = 1.0 / GEARING;
    config.closedLoop.slots[0].p = 0.1;
    config.closedLoop.slots[0].maxMotion.maxVelocity = 30_rpm;
    config.closedLoop.slots[0].maxMotion.maxAcceleration = 200_rev_per_m_per_s;
    config.smartCurrentStallLimit = 100_A;
    config.feedforward.linearGravity = 0.16_V;
    config.feedforward.velocity = 0.02_V / 1_rpm;
    _motor.OverwriteConfig(config);
};

void Elevator::Periodic() {
    _motor.UpdateControls();
}

void Elevator::SimulationPeriodic() {
    _sim.SetInputVoltage(_motor.CalcSimVoltage());
    _sim.Update(20_ms);
    _motor.IterateSim(MPSToRPM(_sim.GetVelocity()), HeightToTurns(_sim.GetPosition()));
}

units::meter_t Elevator::TurnsToHeight(units::turn_t turns) { 
    return turns.value() * PULLEY_CIRCUMFERENCE;
}

units::turn_t Elevator::HeightToTurns(units::meter_t height) { 
    return (height.value() / PULLEY_CIRCUMFERENCE.value()) * 1_tr;
}

units::meters_per_second_t Elevator::RPMToMPS(units::revolutions_per_minute_t rpm) {
    return rpm.value() * PULLEY_CIRCUMFERENCE / 1_min;
}

units::revolutions_per_minute_t Elevator::MPSToRPM(units::meters_per_second_t mps) {
    return (mps.value() / PULLEY_CIRCUMFERENCE.value()) * 1_tps;
}

frc2::CommandPtr Elevator::MaxMotionTo(units::meter_t height) {
    return RunOnce([this, height] {
        _motor.SetMaxMotionTarget(HeightToTurns(height));
    });
}

frc2::CommandPtr Elevator::WPIProfileTo(units::meter_t height) {
    return RunOnce([this, height] {
        _motor.SetMotionProfileTarget(HeightToTurns(height));
    });
}

frc2::CommandPtr Elevator::PIDTo(units::meter_t height) {
    return RunOnce([this, height] {
        _motor.SetPositionTarget(HeightToTurns(height));
    });
}

frc2::CommandPtr Elevator::DriveWithDutyCycle(double dutyCycle) {
  return StartEnd([this, dutyCycle] { _motor.SetDutyCycle(dutyCycle); },
                  [this] { _motor.StopMotor(); });
}
