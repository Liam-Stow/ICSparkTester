#include "subsystems/Feeder.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkBaseConfig.h>

Feeder::Feeder() {
    frc::SmartDashboard::PutData("Feeder", &_motor);
    
    rev::spark::SparkBaseConfig config;
    config.SmartCurrentLimit(100); // amps
    _motor.OverwriteConfig(config);
};

void Feeder::Periodic() {}

void Feeder::SimulationPeriodic() {
    auto velocity = _motor.CalcSimVoltage() / 12_V * 5700_rpm;
    auto position = _motor.GetPosition() + velocity * 20_ms;
    _motor.IterateSim(velocity, position);
}

frc2::CommandPtr Feeder::FeedIn() {
    return StartEnd([this] { _motor.SetDutyCycle(0.5); },
                    [this] { _motor.StopMotor(); });
}

frc2::CommandPtr Feeder::FeedOut() {
    return StartEnd([this] { _motor.SetDutyCycle(-0.5); },
                    [this] { _motor.StopMotor(); });
}

double Feeder::GetMotorDutyCycle() {
  return _motor.Get();
}