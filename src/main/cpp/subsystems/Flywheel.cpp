#include "subsystems/Flywheel.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkBaseConfig.h>

Flywheel::Flywheel() {
    frc::SmartDashboard::PutData("Flywheel", &_motor);
    
    rev::spark::SparkBaseConfig config;
    config.SmartCurrentLimit(200); // amps
    config.closedLoop.P(0.1);
    config.closedLoop.feedForward.kV(0.0018); // V per rpm
    _motor.OverwriteConfig(config);
};

void Flywheel::Periodic() {
    _motor.UpdateMotionProfile();
    _motor.CheckAlerts();
}

void Flywheel::SimulationPeriodic() {
    _sim.SetInputVoltage(_motor.CalcSimVoltage());
    _sim.Update(20_ms);
    auto velocity = _sim.GetAngularVelocity();
    _simPosition += units::turn_t{velocity * 20_ms};
    _motor.IterateSim(velocity, _simPosition);
}

frc2::CommandPtr Flywheel::SpinAt(units::turns_per_second_t velocity) {
    return StartEnd([this, velocity] { _motor.SetVelocityTarget(velocity); },
                    [this] { _motor.StopMotor(); });
}

frc2::CommandPtr Flywheel::SpinAt(double dutyCycle) {
    return StartEnd([this, dutyCycle] { _motor.SetDutyCycle(dutyCycle); },
                    [this] { _motor.StopMotor(); });
}

units::turns_per_second_t Flywheel::GetVelocity() {
    return _motor.GetVelocity();
}

double Flywheel::GetMotorDutyCycle() {
    return _motor.GetDutyCycle();
}