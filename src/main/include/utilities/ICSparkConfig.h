#pragma once

#include <units/current.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <optional>
#include <array>
#include <rev/config/SparkBaseConfig.h>

struct ICSparkConfig {
  using VoltsPerRpm = units::unit_t<units::compound_unit<
      units::volts, units::inverse<units::revolutions_per_minute>>>;
  using VoltsPerRpmPerS = units::unit_t<units::compound_unit<
      units::volts, units::inverse<units::revolutions_per_minute_per_second>>>;

  // Top level configs
  std::optional<bool> inverted;
  std::optional<rev::spark::SparkBaseConfig::IdleMode> idleMode = std::nullopt;
  std::optional<units::ampere_t> smartCurrentStallLimit = std::nullopt;
  std::optional<units::ampere_t> smartCurrentFreeLimit = std::nullopt;
  std::optional<units::revolutions_per_minute_t> smartCurrentVelocityLimit = std::nullopt;
  std::optional<units::ampere_t> secondaryCurrentLimit = std::nullopt;
  std::optional<int> secondaryCurrentLimitChopCycles = std::nullopt;
  std::optional<units::second_t> openLoopRampRate = std::nullopt;
  std::optional<units::second_t> closedLoopRampRate = std::nullopt;
  std::optional<units::volt_t> voltageCompensationNominalVoltage = std::nullopt;
  std::optional<bool> voltageCompensationEnabled = std::nullopt;
  std::optional<int> followCanId = std::nullopt;
  std::optional<bool> followInverted = std::nullopt;

  // Absolute Encoder
  struct {
    std::optional<bool> inverted = std::nullopt;
    std::optional<double> positionConversionFactor = std::nullopt;
    std::optional<double> velocityConversionFactor = std::nullopt;
    std::optional<units::turn_t> zeroOffset = std::nullopt;
    std::optional<int> averageDepth = std::nullopt;
    std::optional<units::microsecond_t> startPulseUs = std::nullopt;
    std::optional<units::microsecond_t> endPulseUs = std::nullopt;
    std::optional<bool> zeroCentered = std::nullopt;
  } absoluteEncoder;

  // Analog sensor
  struct {
    std::optional<bool> inverted = std::nullopt;
    std::optional<double> positionConversionFactor = std::nullopt;
    std::optional<double> velocityConversionFactor = std::nullopt;
  } analogSensor;

  // Closed loop
  struct ClosedLoopSlotConfig {
    rev::spark::ClosedLoopSlot slotID;
    std::optional<double> p = std::nullopt; 
    std::optional<double> i = std::nullopt;
    std::optional<double> d = std::nullopt;
    std::optional<double> dFilter = std::nullopt;
    std::optional<double> iZone = std::nullopt;
    std::optional<double> iMaxAccum = std::nullopt;
    std::optional<double> minOutput = std::nullopt;
    std::optional<double> maxOutput = std::nullopt;
    struct {
      std::optional<VoltsPerRpm> velocity = std::nullopt;             // kV
      std::optional<VoltsPerRpmPerS> acceleration = std::nullopt;     // kA
      std::optional<units::volt_t> staticFriction = std::nullopt;     // kS
      std::optional<units::volt_t> linearGravity = std::nullopt;      // kG
      std::optional<units::volt_t> rotationalGravity = std::nullopt;  // kCos
      std::optional<double> cosineRatio = std::nullopt;               // kCosRatio
    } feedforward = {};
    struct {
      std::optional<units::revolutions_per_minute_t> maxVelocity = std::nullopt;
      std::optional<units::revolutions_per_minute_per_second_t> maxAcceleration = std::nullopt;
      std::optional<units::turn_t> allowedClosedLoopError = std::nullopt;
      std::optional<rev::spark::MAXMotionConfig::MAXMotionPositionMode> positionMode = std::nullopt;
    } maxMotion = {};
  };
  struct {
    std::optional<bool> positionWrappingEnabled = std::nullopt;
    std::optional<units::turn_t> positionWrappingMinInput = std::nullopt;
    std::optional<units::turn_t> positionWrappingMaxInput = std::nullopt;
    std::optional<rev::spark::FeedbackSensor> feedbackSensor = std::nullopt;

    std::array<ClosedLoopSlotConfig, 4> slots = {
        {{rev::spark::kSlot0}, {rev::spark::kSlot1}, {rev::spark::kSlot2}, {rev::spark::kSlot3}}};
  } closedLoop;

  // Encoder
  struct {
    std::optional<bool> inverted = std::nullopt;
    std::optional<double> positionConversionFactor = std::nullopt;
    std::optional<double> velocityConversionFactor = std::nullopt;
    std::optional<int> quadratureAverageDepth = std::nullopt;
    std::optional<units::millisecond_t> quadratureMeasurementPeriod = std::nullopt;
    std::optional<int> uvwAverageDepth = std::nullopt;
    std::optional<units::millisecond_t> uvwMeasurementPeriod = std::nullopt;
  } encoder;

  // Limit switch
  struct {
    std::optional<bool> sparkMaxDataPortLimitSwitchMode = std::nullopt;
    std::optional<double> forwardLimitSwitchPosition = std::nullopt;
    std::optional<rev::spark::LimitSwitchConfig::Behavior> forwardLimitSwitchTriggerBehavior = std::nullopt;
    std::optional<rev::spark::LimitSwitchConfig::Type> forwardLimitSwitchType = std::nullopt;
    std::optional<double> reverseLimitSwitchPosition = std::nullopt;
    std::optional<rev::spark::LimitSwitchConfig::Behavior> reverseLimitSwitchTriggerBehavior = std::nullopt;
    std::optional<rev::spark::LimitSwitchConfig::Type> reverseLimitSwitchType = std::nullopt;
    std::optional<rev::spark::FeedbackSensor> positionSensor = std::nullopt;
  } limitSwitch;

  // Signals
  struct {
    std::optional<units::millisecond_t> appliedOutputPeriodMs = std::nullopt;
    std::optional<units::millisecond_t> busVoltagePeriodMs = std::nullopt;
    std::optional<units::millisecond_t> outputCurrentPeriodMs = std::nullopt;
    std::optional<units::millisecond_t> motorTemperaturePeriodMs = std::nullopt;
    std::optional<units::millisecond_t> limitsPeriodMs = std::nullopt;
    std::optional<units::millisecond_t> faultsPeriodMs = std::nullopt;
    std::optional<bool> faultsAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> warningsPeriodMs = std::nullopt;
    std::optional<bool> warningsAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> primaryEncoderVelocityPeriodMs = std::nullopt;
    std::optional<bool> primaryEncoderVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> primaryEncoderPositionPeriodMs = std::nullopt;
    std::optional<bool> primaryEncoderPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> analogVoltagePeriodMs = std::nullopt;
    std::optional<bool> analogVoltageAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> analogVelocityPeriodMs = std::nullopt;
    std::optional<bool> analogVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> analogPositionPeriodMs = std::nullopt;
    std::optional<bool> analogPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> externalOrAltEncoderVelocity = std::nullopt;
    std::optional<bool> externalOrAltEncoderVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> externalOrAltEncoderPosition = std::nullopt;
    std::optional<bool> externalOrAltEncoderPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> absoluteEncoderVelocityPeriodMs = std::nullopt;
    std::optional<bool> absoluteEncoderVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> absoluteEncoderPositionPeriodMs = std::nullopt;
    std::optional<bool> absoluteEncoderPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> iAccumulationPeriodMs = std::nullopt;
    std::optional<bool> iAccumulationAlwaysOn = std::nullopt;
  } signals;

  // Soft limit
  struct {
    std::optional<units::turn_t> forwardSoftLimit = std::nullopt;
    std::optional<bool> forwardSoftLimitEnabled = std::nullopt;
    std::optional<units::turn_t> reverseSoftLimit = std::nullopt;
    std::optional<bool> reverseSoftLimitEnabled = std::nullopt;
  } softLimit;

  /**
   * Apply this config to a REV SparkBaseConfig.
   */
  void FillREVConfig(rev::spark::SparkBaseConfig& config) const;

  /**
   * Edit the settings in this config to match the settings in a provided other
   * config.
   * Paramters that don't have a value in other will be ignored.
   * 
   * @param other The other config to pull settings from.
   */
  void Adjust(const ICSparkConfig& other);
};
