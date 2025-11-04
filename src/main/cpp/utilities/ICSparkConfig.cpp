#include "utilities/ICSparkConfig.h"

// Implementation for AbsoluteEncoderConfig
void ICSparkConfig::AbsoluteEncoderConfig::Fill(rev::spark::AbsoluteEncoderConfig& config) const {
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  ABSOLUTE_ENCODER_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::AbsoluteEncoderConfig::Adjust(const AbsoluteEncoderConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  ABSOLUTE_ENCODER_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for AnalogSensorConfig
void ICSparkConfig::AnalogSensorConfig::Fill(rev::spark::AnalogSensorConfig& config) const {
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  ANALOG_SENSOR_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::AnalogSensorConfig::Adjust(const AnalogSensorConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  ANALOG_SENSOR_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for FeedforwardConfig
void ICSparkConfig::FeedforwardConfig::Fill(rev::spark::FeedForwardConfig& config,
                                            rev::spark::ClosedLoopSlot slotID) const {
#define FILL_MEMBER(name, type, rev_call) \
  if (name)                               \
    config.rev_call;
  FEEDFORWARD_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::FeedforwardConfig::Adjust(const FeedforwardConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  FEEDFORWARD_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for MAXMotionConfig
void ICSparkConfig::MAXMotionConfig::Fill(rev::spark::MAXMotionConfig& config,
                                          rev::spark::ClosedLoopSlot slotID) const {
#define FILL_MEMBER(name, type, rev_call) \
  if (name)                               \
    config.rev_call;
  MAX_MOTION_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::MAXMotionConfig::Adjust(const MAXMotionConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  MAX_MOTION_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for ClosedLoopSlotConfig
void ICSparkConfig::ClosedLoopSlotConfig::Fill(rev::spark::ClosedLoopConfig& config) const {
  // Fill simple PID members
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  CLOSED_LOOP_SLOT_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER

  // Fill nested structs
  feedforward.Fill(config.feedForward, slotID);
  maxMotion.Fill(config.maxMotion, slotID);
}
void ICSparkConfig::ClosedLoopSlotConfig::Adjust(const ClosedLoopSlotConfig& other) {
  // Adjust simple PID members
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  CLOSED_LOOP_SLOT_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER

  // Adjust nested structs
  feedforward.Adjust(other.feedforward);
  maxMotion.Adjust(other.maxMotion);
}

// Implementation for ClosedLoopConfig
void ICSparkConfig::ClosedLoopConfig::Fill(rev::spark::ClosedLoopConfig& config) const {
  // Fill top-level members
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  CLOSED_LOOP_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER

  // Fill all slots
  for (const auto& slot : slots) {
    slot.Fill(config);
  }
}
void ICSparkConfig::ClosedLoopConfig::Adjust(const ClosedLoopConfig& other) {
  // Adjust top-level members
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  CLOSED_LOOP_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER

  // Adjust all slots
  for (size_t i = 0; i < slots.size(); ++i) {
    slots[i].Adjust(other.slots[i]);
  }
}

// Implementation for EncoderConfig
void ICSparkConfig::EncoderConfig::Fill(rev::spark::EncoderConfig& config) const {
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  ENCODER_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::EncoderConfig::Adjust(const EncoderConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  ENCODER_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for LimitSwitchConfig
void ICSparkConfig::LimitSwitchConfig::Fill(rev::spark::LimitSwitchConfig& config) const {
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  LIMIT_SWITCH_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::LimitSwitchConfig::Adjust(const LimitSwitchConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  LIMIT_SWITCH_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for SignalsConfig
void ICSparkConfig::SignalsConfig::Fill(rev::spark::SignalsConfig& config) const {
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  SIGNALS_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::SignalsConfig::Adjust(const SignalsConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  SIGNALS_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for SoftLimitConfig
void ICSparkConfig::SoftLimitConfig::Fill(rev::spark::SoftLimitConfig& config) const {
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  SOFT_LIMIT_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER
}
void ICSparkConfig::SoftLimitConfig::Adjust(const SoftLimitConfig& other) {
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  SOFT_LIMIT_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER
}

// Implementation for ICSparkConfig (Top Level)
void ICSparkConfig::FillREVConfig(rev::spark::SparkBaseConfig& config) const {
  // --- Handle simple top-level members ---
  #define FILL_MEMBER(name, type, rev_call) if (name) config.rev_call;
  IC_SPARK_CONFIG_PARAMS(FILL_MEMBER)
  #undef FILL_MEMBER

  // --- Handle special cases manually ---

  // Smart Current Limit (multiple overloads)
  if (smartCurrentStallLimit && smartCurrentfreeLimit && smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value(), smartCurrentfreeLimit->value(),
                             smartCurrentVelocityLimit->value());
  if (smartCurrentStallLimit && !smartCurrentfreeLimit && !smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value());
  if (smartCurrentStallLimit && smartCurrentfreeLimit && !smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value(), smartCurrentfreeLimit->value());
  
  // Secondary Current Limit
  if (secondaryCurrentLimit && secondaryCurrentLimitChopCycles)
    config.SecondaryCurrentLimit(secondaryCurrentLimit->value(), *secondaryCurrentLimitChopCycles);

  // Voltage Compensation (has a disable case)
  if (voltageCompensationNominalVoltage)
    config.VoltageCompensation(voltageCompensationNominalVoltage->value());
  else
    config.DisableVoltageCompensation();

  // Follow (multiple overloads)
  if (followCanId && followInverted)
    config.Follow(*followCanId, *followInverted);
  if (followCanId && !followInverted)
    config.Follow(*followCanId);

  // --- Fill all sub-structs ---
  absoluteEncoder.Fill(config.absoluteEncoder);
  analogSensor.Fill(config.analogSensor);
  closedLoop.Fill(config.closedLoop);
  encoder.Fill(config.encoder);
  limitSwitch.Fill(config.limitSwitch);
  signals.Fill(config.signals);
  softLimit.Fill(config.softLimit);
}

void ICSparkConfig::Adjust(const ICSparkConfig& other) {
  // Adjust simple top-level members
  #define ADJUST_MEMBER(name, ...) UpdateOptional(name, other.name);
  IC_SPARK_CONFIG_PARAMS(ADJUST_MEMBER)
  #undef ADJUST_MEMBER

  // Adjust special cases manually (multi-parameter configs)
  UpdateOptional(smartCurrentStallLimit, other.smartCurrentStallLimit);
  UpdateOptional(smartCurrentfreeLimit, other.smartCurrentfreeLimit);
  UpdateOptional(smartCurrentVelocityLimit, other.smartCurrentVelocityLimit);
  UpdateOptional(secondaryCurrentLimit, other.secondaryCurrentLimit);
  UpdateOptional(secondaryCurrentLimitChopCycles, other.secondaryCurrentLimitChopCycles);
  UpdateOptional(followCanId, other.followCanId);
  UpdateOptional(followInverted, other.followInverted);

  // Adjust all sub-structs
  absoluteEncoder.Adjust(other.absoluteEncoder);
  analogSensor.Adjust(other.analogSensor);
  closedLoop.Adjust(other.closedLoop);
  encoder.Adjust(other.encoder);
  limitSwitch.Adjust(other.limitSwitch);
  signals.Adjust(other.signals);
  softLimit.Adjust(other.softLimit);
}