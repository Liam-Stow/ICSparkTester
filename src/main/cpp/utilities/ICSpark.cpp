#include "utilities/ICSpark.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>
#include <rev/ClosedLoopSlot.h>

ICSpark::ICSpark(rev::spark::SparkBase* spark, rev::spark::SparkRelativeEncoder& inbuiltEncoder)
    : _spark(spark),
      _encoder(inbuiltEncoder),
      _simSpark(spark, &_vortexModel) {
  // Apply the IC default configuration.
  // Includes setting vel conversion factor to use revs per sec not revs per min and a safeish
  // current limit.
  ICSparkConfig defaultConfig;
  AdjustConfig(defaultConfig);
}

void ICSpark::InitSendable(wpi::SendableBuilder& builder) {
  // clang-format off
  //----------------------- Label ------------------------ Getter ------------------------------------------------ Setter -------------------------------------------------
  builder.AddDoubleProperty("Position",                   [&] { return GetPosition().value(); },                  nullptr);
  builder.AddDoubleProperty("Velocity",                   [&] { return GetVelocity().value(); },                  nullptr);
  builder.AddDoubleProperty("Voltage",                    [&] { return GetMotorVoltage().value(); },              nullptr);
  builder.AddDoubleProperty("Current",                    [&] { return GetStatorCurrent().value(); },             nullptr);
  builder.AddDoubleProperty("Temperature",                [&] { return _spark->GetMotorTemperature(); },          nullptr);
  builder.AddDoubleProperty("Position conversion factor", [&] { return _configCache.encoder.positionConversionFactor.value_or(1); }, nullptr);
  builder.AddDoubleProperty("Velocity conversion factor", [&] { return _configCache.encoder.velocityConversionFactor.value_or(1); }, nullptr);
  builder.AddDoubleProperty("Position Target",            [&] { return _positionTarget.value(); },                [&](double targ) { SetPositionTarget(targ*1_tr); });
  builder.AddDoubleProperty("Velocity Target",            [&] { return _velocityTarget.value(); },                [&](double targ) { SetVelocityTarget(targ*1_tps); });
  builder.AddDoubleProperty("Profile Position Target",    [&] { return _latestMotionTarget.position.value(); },   [&](double targ) { SetMotionProfileTarget(targ*1_tr); });
  builder.AddDoubleProperty("Profile Velocity Target",    [&] { return _latestMotionTarget.velocity.value(); },   nullptr);
  builder.AddDoubleProperty("Gains/FB P Gain",            [&] { return _rioPidController.GetP(); },               [&](double P) { SetFeedbackProportional(P); });
  builder.AddDoubleProperty("Gains/FB I Gain",            [&] { return _rioPidController.GetI(); },               [&](double I) { SetFeedbackIntegral(I); });
  builder.AddDoubleProperty("Gains/FB D Gain",            [&] { return _rioPidController.GetD(); },               [&](double D) { SetFeedbackDerivative(D); });
  builder.AddDoubleProperty("Gains/FF S Gain",            [&] { return _feedforwardStaticFriction.value(); },     [&](double S) { SetFeedforwardStaticFriction(S*1_V); });
  builder.AddDoubleProperty("Gains/FF V Gain",            [&] { return _feedforwardVelocity.value(); },           [&](double V) { SetFeedforwardVelocity(VoltsPerRpm{V}); });
  builder.AddDoubleProperty("Gains/FF A Gain",            [&] { return _feedforwardAcceleration.value(); },       [&](double A) { SetFeedforwardAcceleration(VoltsPerRpmPerS{A}); });
  builder.AddDoubleProperty("Gains/FF Linear G Gain",     [&] { return _feedforwardLinearGravity.value(); },      [&](double lG) { SetFeedforwardLinearGravity(lG*1_V); });
  builder.AddDoubleProperty("Gains/FF Rotational G Gain", [&] { return _feedforwardRotationalGravity.value(); },  [&](double rG) { SetFeedforwardRotationalGravity(rG*1_V); });
  
  builder.AddDoubleProperty("Motion Config/Max vel",      
    [&] { return _configCache.closedLoop.slots[0].maxMotion.maxVelocity.value_or(0_rpm).value(); },              
    [&](double vel) { SetMotionMaxVel(vel*1_rpm); });
  builder.AddDoubleProperty("Motion Config/Max accel",
    [&] { return _configCache.closedLoop.slots[0].maxMotion.maxAcceleration.value_or(0_rev_per_m_per_s).value(); },
    [&](double accel) { SetMotionMaxAccel(accel*1_rev_per_m_per_s); });
  // clang-format on
}

rev::REVLibError ICSpark::Configure(ICSparkConfig& config,
                                    rev::spark::SparkBase::ResetMode resetMode,
                                    rev::spark::SparkBase::PersistMode persistMode) {
  // Update my cached config to match new config.
  if (resetMode == rev::spark::SparkBase::ResetMode::kResetSafeParameters) {
    _configCache = config;
  } else {
    _configCache.Adjust(config);
  }

  // Motion profile config for sim and feed forward model
  auto& slot0 = _configCache.closedLoop.slots[0];
  _motionProfile = frc::TrapezoidProfile<units::turns>{
      {slot0.maxMotion.maxVelocity.value_or(0_tps),
       slot0.maxMotion.maxAcceleration.value_or(0_tr_per_s_sq)}};

  // Wrapping settings for sim
  if (_configCache.closedLoop.positionWrappingEnabled.value_or(false)) {
    _rioPidController.EnableContinuousInput(
        _configCache.closedLoop.positionWrappingMinInput.value_or(0_tr).value(),
        _configCache.closedLoop.positionWrappingMaxInput.value_or(0_tr).value());
  } else {
    _rioPidController.DisableContinuousInput();
  }

  // Set the rio pid gains to match the spark config
  _rioPidController.SetPID(slot0.p.value_or(0), slot0.i.value_or(0), slot0.d.value_or(0));

  // Run the configuration and return any errors
  rev::spark::SparkBaseConfig revConfig;
  _configCache.FillREVConfig(revConfig);
  return _spark->Configure(revConfig, resetMode, persistMode);
}

rev::REVLibError ICSpark::AdjustConfig(ICSparkConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);
};

rev::REVLibError ICSpark::AdjustConfigNoPersist(ICSparkConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kNoPersistParameters);
};

rev::REVLibError ICSpark::OverwriteConfig(ICSparkConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);
};

void ICSpark::SetPosition(units::turn_t position) {
  _encoder.SetPosition(position.value());
}

void ICSpark::SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestModelFeedForward = CalculateFeedforward(target, 0_tps);
  _controlType = ControlType::kPosition;

  _sparkPidController.SetReference(target.value(),
                                   rev::spark::SparkLowLevel::ControlType::kPosition,
                                   rev::spark::ClosedLoopSlot::kSlot0,
                                   _arbFeedForward.value() + _latestModelFeedForward.value());
}

void ICSpark::SetMaxMotionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {GetPosition(), GetVelocity()};
  _controlType = ControlType::kMaxMotion;

  UpdateControls();
}

void ICSpark::SetMotionProfileTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::revolutions_per_minute_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {GetPosition(), GetVelocity()};
  _controlType = ControlType::kMotionProfile;

  UpdateControls();
}

void ICSpark::SetVelocityTarget(units::revolutions_per_minute_t target, units::volt_t arbFeedForward) {
  _velocityTarget = target;
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestModelFeedForward = CalculateFeedforward(0_tr, _velocityTarget);
  _controlType = ControlType::kVelocity;

  _sparkPidController.SetReference(target.value(),
                                   rev::spark::SparkLowLevel::ControlType::kVelocity,
                                   rev::spark::ClosedLoopSlot::kSlot0,
                                   _arbFeedForward.value() + _latestModelFeedForward.value());
}

void ICSpark::SetDutyCycle(double speed) {
  _velocityTarget = units::revolutions_per_minute_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = 0_V;
  _latestModelFeedForward = 0_V;
  _controlType = ControlType::kDutyCycle;

  _sparkPidController.SetReference(speed, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
}

void ICSpark::SetVoltage(units::volt_t output) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = output;
  _arbFeedForward = 0_V;
  _controlType = ControlType::kVoltage;

  _sparkPidController.SetReference(output.value(), rev::spark::SparkLowLevel::ControlType::kVoltage);
}

void ICSpark::UpdateControls(units::second_t loopTime) {
  auto prevVelTarget = _latestMotionTarget.velocity;
  double sparkTarget = 0;

  switch (GetControlType()) {
    case ControlType::kMotionProfile:
      // In motion profile mode, we use the prev target state as the "current state"
      // and the sparkPIDController uses the next target state as its goal.
      _latestMotionTarget = CalcNextMotionTarget(_latestMotionTarget, _positionTarget, loopTime);
      sparkTarget = _latestMotionTarget.position.value();
      break;
    case ControlType::kMaxMotion: {
      // In Max Motion mode, we use the true, sensed current state state as the "current state"
      // and the sparkPIDController uses the overall target as its goal.
      MPState currentState = {GetPosition(), GetVelocity()};
      _latestMotionTarget = CalcNextMotionTarget(currentState, _positionTarget, loopTime);
      sparkTarget = _positionTarget.value();
      break;
    }
    case ControlType::kPosition:
      sparkTarget = _positionTarget.value();
      break;
    case ControlType::kVelocity:
      sparkTarget = _velocityTarget.value();
      break;
    default:
      return;
  }

  auto accelTarget = (_latestMotionTarget.velocity - prevVelTarget) / loopTime;
  _latestModelFeedForward =
      CalculateFeedforward(_latestMotionTarget.position, _latestMotionTarget.velocity, accelTarget);
  units::volt_t feedforward = _arbFeedForward + _latestModelFeedForward;
  _sparkPidController.SetReference(sparkTarget, GetREVControlType(), rev::spark::ClosedLoopSlot::kSlot0, feedforward.value());
}

units::volt_t ICSpark::CalculateFeedforward(units::turn_t pos, units::revolutions_per_minute_t vel,
                                            units::revolutions_per_minute_per_second_t accel) {
  return _feedforwardStaticFriction * wpi::sgn(vel) + _feedforwardLinearGravity +
         _feedforwardRotationalGravity * units::math::cos(pos) + _feedforwardVelocity * vel +
         _feedforwardAcceleration * accel;
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
  config.closedLoop.slots[0].maxMotion.maxVelocity = maxVelocity;
  AdjustConfig(config);
}

void ICSpark::SetMotionMaxAccel(units::revolutions_per_minute_per_second_t maxAcceleration) {
  ICSparkConfig config;
  config.closedLoop.slots[0].maxMotion.maxAcceleration = maxAcceleration;
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
  config.signals.absoluteEncoderVelocityAlwaysOn =true;
  config.signals.absoluteEncoderVelocityPeriodMs = 10_ms;
  config.closedLoop.feedbackSensor = rev::spark::ClosedLoopConfig::kAbsoluteEncoder;
  return config;
}

void ICSpark::SetFeedbackProportional(double P) {
  ICSparkConfig config;
  config.closedLoop.slots[0].p = P;
  AdjustConfig(config);
}

void ICSpark::SetFeedbackIntegral(double I) {
  ICSparkConfig config;
  config.closedLoop.slots[0].i = I;
  AdjustConfig(config);
}

void ICSpark::SetFeedbackDerivative(double D) {
  ICSparkConfig config;
  config.closedLoop.slots[0].d = D;
  AdjustConfig(config);
}

void ICSpark::SetFeedforwardGains(units::volt_t S, units::volt_t G, bool gravityIsRotational,
                                  VoltsPerRpm V, VoltsPerRpmPerS A) {
  SetFeedforwardStaticFriction(S);
  SetFeedforwardVelocity(V);
  SetFeedforwardAcceleration(A);
  if (gravityIsRotational) {
    SetFeedforwardRotationalGravity(G);
  } else {
    SetFeedforwardLinearGravity(G);
  }
}

void ICSpark::SetFeedforwardStaticFriction(units::volt_t S) {
  _feedforwardStaticFriction = S;
}

void ICSpark::SetFeedforwardLinearGravity(units::volt_t linearG) {
  _feedforwardLinearGravity = linearG;
}

void ICSpark::SetFeedforwardRotationalGravity(units::volt_t rotationalG) {
  _feedforwardRotationalGravity = rotationalG;
}

void ICSpark::SetFeedforwardVelocity(VoltsPerRpm V) {
  _feedforwardVelocity = V;
}

void ICSpark::SetFeedforwardAcceleration(VoltsPerRpmPerS A) {
  _feedforwardAcceleration = A;
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
    return _spark->GetAppliedOutput() * _spark->GetBusVoltage()*1_V;
  }
}

units::ampere_t ICSpark::GetStatorCurrent() {
  return _spark->GetOutputCurrent() * 1_A;
}

units::volt_t ICSpark::CalcSimVoltage() {
  return _simSpark.GetAppliedOutput() * frc::RobotController::GetBatteryVoltage();
}

void ICSpark::IterateSim(units::revolutions_per_minute_t velocity, units::turn_t position) {
  const units::second_t dt = 20_ms;
  const units::volt_t batteryVoltage = frc::sim::RoboRioSim::GetVInVoltage();
  _simSpark.iterate(velocity.value(), batteryVoltage.value(), dt.value());
  // REV's spark sim can work without explicitly telling it the positon of the mechanism,
  // but we find that it falls out of sync with the physics model if we don't set it.
  _simSpark.SetPosition(position.value());
}

bool ICSpark::InMotionMode() {
  return GetControlType() == ControlType::kMotionProfile ||
         GetControlType() == ControlType::kMaxMotion;
}

ICSpark::MPState ICSpark::CalcNextMotionTarget(MPState current, units::turn_t goalPosition,
                                               units::second_t lookahead) {
  units::turn_t error = units::math::abs(goalPosition - GetPosition());
  units::turn_t tolerance =
      _configCache.closedLoop.slots[0].maxMotion.allowedClosedLoopError.value_or(0_tr);
  if (error < tolerance) {
    return MPState{GetPosition(), 0_rpm};
  }

  return _motionProfile.Calculate(
      lookahead, current,
      {goalPosition, units::revolutions_per_minute_t{0}});
}
