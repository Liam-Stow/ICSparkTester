#include "subsystems/Elevator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkBaseConfig.h>

Elevator::Elevator() {
    frc::SmartDashboard::PutData("Elevator", &_motor);

    rev::spark::SparkBaseConfig config;
    config.SmartCurrentLimit(100); // amps
    config.encoder.PositionConversionFactor(1.0 / GEARING);
    config.encoder.VelocityConversionFactor(1.0 / GEARING);
    config.closedLoop.P(1.0);
    config.closedLoop.maxMotion.CruiseVelocity(500); // rpm
    config.closedLoop.maxMotion.MaxAcceleration(500); // rev/s^2
    config.closedLoop.feedForward.kCos(0.16); // V
    config.closedLoop.feedForward.kV(0.02); // V per rpm
    _motor.OverwriteConfig(config);
};

void Elevator::Periodic() {
    _motor.UpdateMotionProfile();
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

units::meter_t Elevator::GetHeight() {
  return TurnsToHeight(_motor.GetPosition());
}

double Elevator::GetMotorDutyCycle() {
  return _motor.GetDutyCycle();
}