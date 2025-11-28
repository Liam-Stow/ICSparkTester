#include "utilities/ICSpark.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>
#include <rev/ClosedLoopTypes.h>

ICSpark::ICSpark(rev::spark::SparkBase* spark, rev::spark::SparkRelativeEncoder& inbuiltEncoder)
    : _spark(spark), _encoder(inbuiltEncoder), _simSpark(spark, &_vortexModel) {
}

void ICSpark::InitSendable(wpi::SendableBuilder& builder) {
  // clang-format off
  //----------------------- Label ------------------------ Getter ------------------------------------------------------------------------------------------------- Setter -------------------------------------------------
  builder.AddDoubleProperty("Position",                   [&] { return GetPosition().value(); },                                                                    nullptr);
  builder.AddDoubleProperty("Velocity",                   [&] { return GetVelocity().value(); },                                                                    nullptr);
  builder.AddDoubleProperty("Duty Cycle",                 [&] { return GetDutyCycle(); },                                                                           nullptr);
  builder.AddDoubleProperty("Voltage",                    [&] { return GetMotorVoltage().value(); },                                                                nullptr);
  builder.AddDoubleProperty("Current",                    [&] { return GetStatorCurrent().value(); },                                                               nullptr);
  builder.AddDoubleProperty("Temperature",                [&] { return _spark->GetMotorTemperature(); },                                                            nullptr);
  builder.AddDoubleProperty("Position conversion factor", [&] { return _configCache.encoder.positionConversionFactor.value_or(1); },                                nullptr);
  builder.AddDoubleProperty("Velocity conversion factor", [&] { return _configCache.encoder.velocityConversionFactor.value_or(1); },                                nullptr);
  builder.AddDoubleProperty("Position Target",            [&] { return _positionTarget.value(); },                                                                  [&](double targ) { SetPositionTarget(targ*1_tr); });
  builder.AddDoubleProperty("Velocity Target",            [&] { return _velocityTarget.value(); },                                                                  [&](double targ) { SetVelocityTarget(targ*1_tps); });
  builder.AddDoubleProperty("Profile Position Target",    [&] { return _latestMotionTarget.position.value(); },                                                     [&](double targ) { SetMotionProfileTarget(targ*1_tr); });
  builder.AddDoubleProperty("Profile Velocity Target",    [&] { return _latestMotionTarget.velocity.convert<units::revolutions_per_minute>().value(); },            nullptr);
  builder.AddDoubleProperty("Gains/Active Slot",          [&] { return _activeClosedLoopSlot; },                                                                    nullptr);
  builder.AddDoubleProperty("Gains/FB P Gain",            [&] { return GetActiveSlotConfig().p.value_or(0); },                                                      [&](double P) { TuneFeedbackProportional(P); });
  builder.AddDoubleProperty("Gains/FB I Gain",            [&] { return GetActiveSlotConfig().i.value_or(0); },                                                      [&](double I) { TuneFeedbackIntegral(I); });
  builder.AddDoubleProperty("Gains/FB D Gain",            [&] { return GetActiveSlotConfig().d.value_or(0); },                                                      [&](double D) { TuneFeedbackDerivative(D); });
  builder.AddDoubleProperty("Gains/FF S Gain",            [&] { return GetActiveSlotConfig().feedforward.staticFriction.value_or(0_V).value(); },                   [&](double S) { TuneFeedforwardStaticFriction(S*1_V); });
  builder.AddDoubleProperty("Gains/FF Linear G Gain",     [&] { return GetActiveSlotConfig().feedforward.linearGravity.value_or(0_V).value(); },                    [&](double lG) { TuneFeedforwardLinearGravity(lG*1_V); });
  builder.AddDoubleProperty("Gains/FF Rotational G Gain", [&] { return GetActiveSlotConfig().feedforward.rotationalGravity.value_or(0_V).value(); },                [&](double rG) { TuneFeedforwardRotationalGravity(rG*1_V); });
  builder.AddDoubleProperty("Gains/FF V Gain",            [&] { return GetActiveSlotConfig().feedforward.velocity.value_or(0_V/1_rpm).value(); },                   [&](double V) { TuneFeedforwardVelocity(ICSparkConfig::VoltsPerRpm{V}); });
  builder.AddDoubleProperty("Gains/FF A Gain",            [&] { return GetActiveSlotConfig().feedforward.acceleration.value_or(0_V/1_rev_per_m_per_s).value(); },   [&](double A) { TuneFeedforwardAcceleration(ICSparkConfig::VoltsPerRpmPerS{A}); });
  builder.AddDoubleProperty("Motion Config/Max vel",      [&] { return GetActiveSlotConfig().maxMotion.maxVelocity.value_or(0_rpm).value(); },                      [&](double vel) { SetMotionMaxVel(vel*1_rpm); });
  builder.AddDoubleProperty("Motion Config/Max accel",    [&] { return GetActiveSlotConfig().maxMotion.maxAcceleration.value_or(0_rev_per_m_per_s).value(); },      [&](double accel) { SetMotionMaxAccel(accel*1_rev_per_m_per_s); });
  // clang-format on
}

rev::REVLibError ICSpark::Configure(ICSparkConfig& config,
                                    rev::spark::SparkBase::ResetMode resetMode,
                                    rev::spark::SparkBase::PersistMode persistMode, bool async) {
  // Update my cached config to match new config.
  if (resetMode == rev::spark::SparkBase::ResetMode::kResetSafeParameters) {
    _configCache = config;
  } else {
    _configCache.Adjust(config);
  }

  // Match wpilib motion profile config to max motion config
  _motionProfile = frc::TrapezoidProfile<units::turns>{
      {GetActiveSlotConfig().maxMotion.maxVelocity.value_or(0_tps),
       GetActiveSlotConfig().maxMotion.maxAcceleration.value_or(0_tr_per_s_sq)}};

  // Run the configuration and return any errors
  rev::spark::SparkBaseConfig revConfig;
  _configCache.FillREVConfig(revConfig);
  if (async) {
    return _spark->ConfigureAsync(revConfig, resetMode, persistMode);
  } else {
    return _spark->Configure(revConfig, resetMode, persistMode);
  }
}

rev::REVLibError ICSpark::AdjustConfig(ICSparkConfig& config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                   rev::spark::SparkBase::PersistMode::kPersistParameters);
};

rev::REVLibError ICSpark::AdjustConfigNoPersist(ICSparkConfig& config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                   rev::spark::SparkBase::PersistMode::kNoPersistParameters);
};

rev::REVLibError ICSpark::OverwriteConfig(ICSparkConfig& config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                   rev::spark::SparkBase::PersistMode::kPersistParameters);
};

ICSparkConfig ICSpark::GetCachedConfig() const {
  return _configCache;
}

void ICSpark::SetPosition(units::turn_t position) {
  _encoder.SetPosition(position.value());
}

void ICSpark::SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward,
                                rev::spark::ClosedLoopSlot slot) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kPosition;
  _activeClosedLoopSlot = slot;

  _sparkPidController.SetSetpoint(
      target.value(), rev::spark::SparkLowLevel::ControlType::kPosition, slot,
      _arbFeedForward.value() + CalculateFeedforward(_positionTarget, 0_tps).value());
}

void ICSpark::SetMaxMotionTarget(units::turn_t target, units::volt_t arbFeedForward,
                                 rev::spark::ClosedLoopSlot slot) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _controlType = ControlType::kMaxMotion;
  _activeClosedLoopSlot = slot;
  _latestMotionTarget = {
      units::turn_t{_sparkPidController.GetMAXMotionSetpointPosition()},
      units::revolutions_per_minute_t{_sparkPidController.GetMAXMotionSetpointVelocity()}};

  _sparkPidController.SetSetpoint(_latestMotionTarget.position.value(),
                                  rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                  _activeClosedLoopSlot, _arbFeedForward.value());
}

void ICSpark::SetMotionProfileTarget(units::turn_t target, units::volt_t arbFeedForward,
                                     rev::spark::ClosedLoopSlot slot) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {GetPosition(), GetVelocity()};
  _controlType = ControlType::kMotionProfile;
  _activeClosedLoopSlot = slot;

  UpdateControls();
}

void ICSpark::SetVelocityTarget(units::revolutions_per_minute_t target,
                                units::volt_t arbFeedForward, rev::spark::ClosedLoopSlot slot) {
  _velocityTarget = target;
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kVelocity;
  _activeClosedLoopSlot = slot;

  _sparkPidController.SetSetpoint(
      target.value(), rev::spark::SparkLowLevel::ControlType::kVelocity, slot,
      _arbFeedForward.value() + CalculateFeedforward(0_tr, _velocityTarget).value());
}

void ICSpark::SetDutyCycle(double speed) {
  _velocityTarget = units::revolutions_per_minute_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = 0_V;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kDutyCycle;

  _sparkPidController.SetSetpoint(speed, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
}

void ICSpark::SetVoltage(units::volt_t output) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = output;
  _arbFeedForward = 0_V;
  _latestMotionTarget = {0_tr, 0_rpm};
  _controlType = ControlType::kVoltage;

  _sparkPidController.SetSetpoint(output.value(), rev::spark::SparkLowLevel::ControlType::kVoltage);
}

void ICSpark::UpdateControls(units::second_t loopTime) {
  switch (GetControlType()) {
    case ControlType::kMotionProfile:
      // In motion profile mode, we use the prev target state as the "current state"
      // and the sparkPIDController uses the next target state as its goal.
      auto prevVelTarget = _latestMotionTarget.velocity;
      _latestMotionTarget = CalcNextMotionTarget(_latestMotionTarget, _positionTarget, loopTime);
      auto accelTarget = (_latestMotionTarget.velocity - prevVelTarget) / loopTime;
      auto modelFeedForward = CalculateFeedforward(_latestMotionTarget.position,
                                                   _latestMotionTarget.velocity, accelTarget);
      _sparkPidController.SetSetpoint(_latestMotionTarget.position.value(), GetREVControlType(),
                                      _activeClosedLoopSlot,
                                      modelFeedForward.value() + _arbFeedForward.value());
      return;
    case ControlType::kMaxMotion: {
      // The built-in spark logic takes care of max motion.
      // just set the motion target to whatever max moton says for logging, then return.
      _latestMotionTarget = {
          units::turn_t{_sparkPidController.GetMAXMotionSetpointPosition()},
          units::revolutions_per_minute_t{_sparkPidController.GetMAXMotionSetpointVelocity()}};
      return;
    }
    default:
      return;
  }
}

units::volt_t ICSpark::CalculateFeedforward(units::turn_t pos, units::revolutions_per_minute_t vel,
                                            units::revolutions_per_minute_per_second_t accel) {
  auto kS = GetActiveSlotConfig().feedforward.staticFriction.value_or(0_V);
  auto kLG = GetActiveSlotConfig().feedforward.linearGravity.value_or(0_V);
  auto kRG = GetActiveSlotConfig().feedforward.rotationalGravity.value_or(0_V);
  auto kV = GetActiveSlotConfig().feedforward.velocity.value_or(0_V / 1_rpm);
  auto kA = GetActiveSlotConfig().feedforward.acceleration.value_or(0_V / 1_rev_per_m_per_s);

  return kS * wpi::sgn(vel) + kLG + kRG * units::math::cos(pos) + kV * vel + kA * accel;
}

rev::spark::SparkLowLevel::ControlType ICSpark::GetREVControlType() {
  auto controlType = GetControlType();
  if (controlType == ControlType::kMotionProfile) {
    return rev::spark::SparkLowLevel::ControlType::kPosition;
  } else {
    return (rev::spark::SparkLowLevel::ControlType)controlType;
  }
}

void ICSpark::SetMotionMaxVel(units::revolutions_per_minute_t maxVelocity) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].maxMotion.maxVelocity = maxVelocity;
  AdjustConfig(config);
}

void ICSpark::SetMotionMaxAccel(units::revolutions_per_minute_per_second_t maxAcceleration) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].maxMotion.maxAcceleration = maxAcceleration;
  AdjustConfig(config);
}

ICSparkConfig ICSpark::UseAbsoluteEncoder(units::turn_t zeroOffset) {
  _encoder.UseAbsolute(_spark->GetAbsoluteEncoder());

  // Create a config to help the user to setup an abolute encoder
  ICSparkConfig config;
  config.absoluteEncoder.averageDepth = 128;
  config.absoluteEncoder.zeroOffset = zeroOffset;
  config.signals.absoluteEncoderPositionAlwaysOn = true;
  config.signals.absoluteEncoderPositionPeriodMs = 10_ms;
  config.signals.absoluteEncoderVelocityAlwaysOn = true;
  config.signals.absoluteEncoderVelocityPeriodMs = 10_ms;
  config.closedLoop.feedbackSensor = rev::spark::kAbsoluteEncoder;
  return config;
}

void ICSpark::TuneFeedbackProportional(double P) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].p = P;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedbackIntegral(double I) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].i = I;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedbackDerivative(double D) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].d = D;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardStaticFriction(units::volt_t S) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].feedforward.staticFriction = S;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardLinearGravity(units::volt_t linearG) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].feedforward.linearGravity = linearG;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardRotationalGravity(units::volt_t rotationalG) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].feedforward.rotationalGravity = rotationalG;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardVelocity(ICSparkConfig::VoltsPerRpm V) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].feedforward.velocity = V;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardAcceleration(ICSparkConfig::VoltsPerRpmPerS A) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].feedforward.acceleration = A;
  AdjustConfigNoPersist(config);
}

void ICSpark::TuneFeedforwardCosineRatio(double ratio) {
  ICSparkConfig config;
  config.closedLoop.slots[_activeClosedLoopSlot].feedforward.cosineRatio = ratio;
  AdjustConfigNoPersist(config);
}

units::revolutions_per_minute_t ICSpark::GetVelocity() {
  return units::revolutions_per_minute_t{_encoder.GetVelocity()};
}

double ICSpark::GetDutyCycle() const {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return _simSpark.GetAppliedOutput();
  } else {
    return _spark->GetAppliedOutput();
  }
}

units::volt_t ICSpark::GetMotorVoltage() {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return CalcSimVoltage();
  } else {
    return _spark->GetAppliedOutput() * _spark->GetBusVoltage() * 1_V;
  }
}

units::ampere_t ICSpark::GetStatorCurrent() {
  return _spark->GetOutputCurrent() * 1_A;
}

units::volt_t ICSpark::CalcSimVoltage() {
  return _simSpark.GetAppliedOutput() * frc::RobotController::GetBatteryVoltage();
}

void ICSpark::IterateSim(units::revolutions_per_minute_t velocity,
                         std::optional<units::turn_t> position) {
  const units::second_t dt = 20_ms;
  const units::volt_t batteryVoltage = frc::sim::RoboRioSim::GetVInVoltage();
  _simSpark.iterate(velocity.value(), batteryVoltage.value(), dt.value());

  // REV's spark sim can work without explicitly telling it the positon of the mechanism,
  // but we find that it falls out of sync with the physics model if we don't set it.
  if (position.has_value()) {
    _simSpark.SetPosition(position.value().value());
  }
}

bool ICSpark::InMotionMode() {
  return GetControlType() == ControlType::kMotionProfile ||
         GetControlType() == ControlType::kMaxMotion;
}

ICSparkConfig::ClosedLoopSlotConfig& ICSpark::GetActiveSlotConfig() {
  return _configCache.closedLoop.slots[(int)_activeClosedLoopSlot];
}

ICSpark::MPState ICSpark::CalcNextMotionTarget(MPState current, units::turn_t goalPosition,
                                               units::second_t lookahead) {
  units::turn_t error = units::math::abs(goalPosition - current.position);
  units::turn_t tolerance = GetActiveSlotConfig().maxMotion.allowedClosedLoopError.value_or(0_tr);
  if (error < tolerance) {
    return MPState{current.position, 0_rpm};
  }

  return _motionProfile.Calculate(lookahead, current, {goalPosition, 0_rpm});
}
