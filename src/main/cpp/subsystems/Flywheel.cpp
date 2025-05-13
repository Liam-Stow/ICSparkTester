#include "subsystems/Flywheel.h"
#include <frc/smartdashboard/SmartDashboard.h>

Flywheel::Flywheel() {
    frc::SmartDashboard::PutData("Flywheel", &_motor);
    
    ICSparkConfig config;
    config.smartCurrentStallLimit = 200_A;
    config.closedLoop.slots[0].p = 0.1;
    config.feedforward.velocity = 0.0018_V / 1_rpm;
    _motor.OverwriteConfig(config);
};

void Flywheel::Periodic() {
    _motor.UpdateControls();
}

void Flywheel::SimulationPeriodic() {
    _sim.SetInputVoltage(_motor.CalcSimVoltage());
    _sim.Update(20_ms);
    _motor.IterateSim(_sim.GetAngularVelocity());
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