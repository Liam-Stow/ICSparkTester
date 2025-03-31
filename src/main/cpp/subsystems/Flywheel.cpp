#include "subsystems/Flywheel.h"
#include <frc/smartdashboard/SmartDashboard.h>

Flywheel::Flywheel() {
    frc::SmartDashboard::PutData("Flywheel", &_motor);
    
    ICSparkConfig config;
    config.smartCurrentStallLimit = 100_A;
    config.closedLoop.slots[0].p = 0.1;
    _motor.OverwriteConfig(config);

    _motor.SetFeedforwardVelocity(0.0018_V / 1_rpm);
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