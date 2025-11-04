#pragma once

#include <rev/config/SparkBaseConfig.h>
#include <rev/config/MaxMotionConfig.h>
#include <units/time.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <optional>
#include <array>

/**
 * @brief Helper template to merge std::optional values.
 * If the source has a value, it updates the target.
 */
template <typename T>
void UpdateOptional(std::optional<T>& target, const std::optional<T>& source) {
  if (source) {
    target = source;
  }
}

// We have to map configs to types and REV function calls, use an X-Macro Parameter List.
// This lets us easily iterate over all parameters in a config without duplicating code.
// Here, X is a placeholder for another macro that can be given later to run functionality on each
// grouping of (member_name, member_type, and rev_call)
// clang-format off
#define ABSOLUTE_ENCODER_CONFIG_PARAMS(X) \
  X(inverted,                             bool,                                     Inverted(*inverted)) \
  X(positionConversionFactor,             double,                                   PositionConversionFactor(*positionConversionFactor)) \
  X(velocityConversionFactor,             double,                                   VelocityConversionFactor(*velocityConversionFactor)) \
  X(zeroOffset,                           units::turn_t,                            ZeroOffset(zeroOffset->value())) \
  X(averageDepth,                         uint32_t,                                 AverageDepth(*averageDepth)) \
  X(startPulseUs,                         units::microsecond_t,                     StartPulseUs(startPulseUs->value())) \
  X(endPulseUs,                           units::microsecond_t,                     EndPulseUs(endPulseUs->value())) \
  X(zeroCentered,                         bool,                                     ZeroCentered(*zeroCentered))

#define ANALOG_SENSOR_CONFIG_PARAMS(X) \
  X(inverted,                             bool,                                     Inverted(*inverted)) \
  X(positionConversionFactor,             double,                                   PositionConversionFactor(*positionConversionFactor)) \
  X(velocityConversionFactor,             double,                                   VelocityConversionFactor(*velocityConversionFactor))

#define FEEDFORWARD_CONFIG_PARAMS(X) \
  X(velocity,                             ICSparkConfig::VoltsPerRpm,               kV(velocity->value(), slotID)) \
  X(acceleration,                         ICSparkConfig::VoltsPerRpmPerS,           kA(acceleration->value(), slotID)) \
  X(staticFriction,                       units::volt_t,                            kS(staticFriction->value(), slotID)) \
  X(linearGravity,                        units::volt_t,                            kG(linearGravity->value(), slotID)) \
  X(rotationalGravity,                    units::volt_t,                            kCos(rotationalGravity->value(), slotID)) \
  X(cosineRatio,                          double,                                   kCosRatio(*cosineRatio, slotID))

#define MAX_MOTION_CONFIG_PARAMS(X) \
  X(maxVelocity,                          units::revolutions_per_minute_t,                      CruiseVelocity(maxVelocity->value(), slotID)) \
  X(maxAcceleration,                      units::revolutions_per_minute_per_second_t,           MaxAcceleration(maxAcceleration->value(), slotID)) \
  X(allowedClosedLoopError,               units::turn_t,                                        AllowedProfileError(allowedClosedLoopError->value(), slotID)) \
  X(positionMode,                         rev::spark::MAXMotionConfig::MAXMotionPositionMode,   PositionMode(*positionMode, slotID))

#define CLOSED_LOOP_SLOT_CONFIG_PARAMS(X) \
  X(p,                                    double,                                   P(*p, slotID)) \
  X(i,                                    double,                                   I(*i, slotID)) \
  X(d,                                    double,                                   D(*d, slotID)) \
  X(dFilter,                              double,                                   DFilter(*dFilter, slotID)) \
  X(iZone,                                double,                                   IZone(*iZone, slotID)) \
  X(minOutput,                            double,                                   MinOutput(*minOutput, slotID)) \
  X(maxOutput,                            double,                                   MaxOutput(*maxOutput, slotID)) \
  X(iMaxAccum,                            double,                                   IMaxAccum(*iMaxAccum, slotID))

#define CLOSED_LOOP_CONFIG_PARAMS(X) \
  X(positionWrappingEnabled,              bool,                                     PositionWrappingEnabled(*positionWrappingEnabled)) \
  X(positionWrappingMinInput,             units::turn_t,                            PositionWrappingMinInput(positionWrappingMinInput->value())) \
  X(positionWrappingMaxInput,             units::turn_t,                            PositionWrappingMaxInput(positionWrappingMaxInput->value())) \
  X(feedbackSensor,                       rev::spark::FeedbackSensor,               SetFeedbackSensor(*feedbackSensor))

#define ENCODER_CONFIG_PARAMS(X) \
  X(inverted,                             bool,                                     Inverted(*inverted)) \
  X(positionConversionFactor,             double,                                   PositionConversionFactor(*positionConversionFactor)) \
  X(velocityConversionFactor,             double,                                   VelocityConversionFactor(*velocityConversionFactor)) \
  X(quadratureAverageDepth,               uint32_t,                                 QuadratureAverageDepth(*quadratureAverageDepth)) \
  X(quadratureMeasurementPeriod,          units::millisecond_t,                     QuadratureMeasurementPeriod(quadratureMeasurementPeriod->value())) \
  X(uvwAverageDepth,                      uint32_t,                                 UvwAverageDepth(*uvwAverageDepth)) \
  X(uvwMeasurementPeriod,                 units::millisecond_t,                     UvwMeasurementPeriod(uvwMeasurementPeriod->value()))

#define LIMIT_SWITCH_CONFIG_PARAMS(X) \
  X(forwardLimitSwitchPosition,           double,                                   ForwardLimitSwitchPosition(*forwardLimitSwitchPosition)) \
  X(forwardLimitSwitchTriggerBehavior,    rev::spark::LimitSwitchConfig::Behavior,  ForwardLimitSwitchTriggerBehavior(*forwardLimitSwitchTriggerBehavior)) \
  X(forwardLimitSwitchType,               rev::spark::LimitSwitchConfig::Type,      ForwardLimitSwitchType(*forwardLimitSwitchType)) \
  X(reverseLimitSwitchPosition,           double,                                   ReverseLimitSwitchPosition(*reverseLimitSwitchPosition)) \
  X(reverseLimitSwitchTriggerBehavior,    rev::spark::LimitSwitchConfig::Behavior,  ReverseLimitSwitchTriggerBehavior(*reverseLimitSwitchTriggerBehavior)) \
  X(reverseLimitSwitchType,               rev::spark::LimitSwitchConfig::Type,      ReverseLimitSwitchType(*reverseLimitSwitchType)) \
  X(positionSensor,                       rev::spark::FeedbackSensor,               LimitSwitchPositionSensor(*positionSensor))

#define SIGNALS_CONFIG_PARAMS(X) \
  X(appliedOutputPeriodMs,                units::millisecond_t,                     AppliedOutputPeriodMs(appliedOutputPeriodMs->value())) \
  X(busVoltagePeriodMs,                   units::millisecond_t,                     BusVoltagePeriodMs(busVoltagePeriodMs->value())) \
  X(outputCurrentPeriodMs,                units::millisecond_t,                     OutputCurrentPeriodMs(outputCurrentPeriodMs->value())) \
  X(motorTemperaturePeriodMs,             units::millisecond_t,                     MotorTemperaturePeriodMs(motorTemperaturePeriodMs->value())) \
  X(limitsPeriodMs,                       units::millisecond_t,                     LimitsPeriodMs(limitsPeriodMs->value())) \
  X(faultsPeriodMs,                       units::millisecond_t,                     FaultsPeriodMs(faultsPeriodMs->value())) \
  X(faultsAlwaysOn,                       bool,                                     FaultsAlwaysOn(*faultsAlwaysOn)) \
  X(warningsPeriodMs,                     units::millisecond_t,                     WarningsPeriodMs(warningsPeriodMs->value())) \
  X(warningsAlwaysOn,                     bool,                                     WarningsAlwaysOn(*warningsAlwaysOn)) \
  X(primaryEncoderVelocityPeriodMs,       units::millisecond_t,                     PrimaryEncoderVelocityPeriodMs(primaryEncoderVelocityPeriodMs->value())) \
  X(primaryEncoderVelocityAlwaysOn,       bool,                                     PrimaryEncoderVelocityAlwaysOn(*primaryEncoderVelocityAlwaysOn)) \
  X(primaryEncoderPositionPeriodMs,       units::millisecond_t,                     PrimaryEncoderPositionPeriodMs(primaryEncoderPositionPeriodMs->value())) \
  X(primaryEncoderPositionAlwaysOn,       bool,                                     PrimaryEncoderPositionAlwaysOn(*primaryEncoderPositionAlwaysOn)) \
  X(analogVoltagePeriodMs,                units::millisecond_t,                     AnalogVoltagePeriodMs(analogVoltagePeriodMs->value())) \
  X(analogVoltageAlwaysOn,                bool,                                     AnalogVoltageAlwaysOn(*analogVoltageAlwaysOn)) \
  X(analogVelocityPeriodMs,               units::millisecond_t,                     AnalogVelocityPeriodMs(analogVelocityPeriodMs->value())) \
  X(analogVelocityAlwaysOn,               bool,                                     AnalogVelocityAlwaysOn(*analogVelocityAlwaysOn)) \
  X(analogPositionPeriodMs,               units::millisecond_t,                     AnalogPositionPeriodMs(analogPositionPeriodMs->value())) \
  X(analogPositionAlwaysOn,               bool,                                     AnalogPositionAlwaysOn(*analogPositionAlwaysOn)) \
  X(externalOrAltEncoderVelocity,         units::millisecond_t,                     ExternalOrAltEncoderVelocity(externalOrAltEncoderVelocity->value())) \
  X(externalOrAltEncoderVelocityAlwaysOn, bool,                                     ExternalOrAltEncoderVelocityAlwaysOn(*externalOrAltEncoderVelocityAlwaysOn)) \
  X(externalOrAltEncoderPosition,         units::millisecond_t,                     ExternalOrAltEncoderPosition(externalOrAltEncoderPosition->value())) \
  X(externalOrAltEncoderPositionAlwaysOn, bool,                                     ExternalOrAltEncoderPositionAlwaysOn(*externalOrAltEncoderPositionAlwaysOn)) \
  X(absoluteEncoderVelocityPeriodMs,      units::millisecond_t,                     AbsoluteEncoderVelocityPeriodMs(absoluteEncoderVelocityPeriodMs->value())) \
  X(absoluteEncoderVelocityAlwaysOn,      bool,                                     AbsoluteEncoderVelocityAlwaysOn(*absoluteEncoderVelocityAlwaysOn)) \
  X(absoluteEncoderPositionPeriodMs,      units::millisecond_t,                     AbsoluteEncoderPositionPeriodMs(absoluteEncoderPositionPeriodMs->value())) \
  X(absoluteEncoderPositionAlwaysOn,      bool,                                     AbsoluteEncoderPositionAlwaysOn(*absoluteEncoderPositionAlwaysOn)) \
  X(iAccumulationPeriodMs,                units::millisecond_t,                     IAccumulationPeriodMs(iAccumulationPeriodMs->value())) \
  X(iAccumulationAlwaysOn,                bool,                                     IAccumulationAlwaysOn(*iAccumulationAlwaysOn))

#define SOFT_LIMIT_CONFIG_PARAMS(X) \
  X(forwardSoftLimit,                     units::turn_t,                            ForwardSoftLimit(forwardSoftLimit->value())) \
  X(forwardSoftLimitEnabled,              bool,                                     ForwardSoftLimitEnabled(*forwardSoftLimitEnabled)) \
  X(reverseSoftLimit,                     units::turn_t,                            ReverseSoftLimit(reverseSoftLimit->value())) \
  X(reverseSoftLimitEnabled,              bool,                                     ReverseSoftLimitEnabled(*reverseSoftLimitEnabled))

#define IC_SPARK_CONFIG_PARAMS(X) \
  X(inverted,                             bool,                                     Inverted(*inverted)) \
  X(idleMode,                             rev::spark::SparkBaseConfig::IdleMode,    SetIdleMode(*idleMode)) \
  X(openLoopRampRate,                     units::second_t,                          OpenLoopRampRate(openLoopRampRate->value())) \
  X(closedLoopRampRate,                   units::second_t,                          ClosedLoopRampRate(closedLoopRampRate->value())) \
  X(voltageCompensationNominalVoltage,    units::volt_t,                            VoltageCompensation(voltageCompensationNominalVoltage->value()) )
// clang-format on

// Config Struct Definitions
struct ICSparkConfig {
  // Aliases for feedforward units
  using VoltsPerRpm = units::unit_t<
      units::compound_unit<units::volts, units::inverse<units::revolutions_per_minute>>>;
  using VoltsPerRpmPerS = units::unit_t<
      units::compound_unit<units::volts, units::inverse<units::revolutions_per_minute_per_second>>>;

  struct AbsoluteEncoderConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
    ABSOLUTE_ENCODER_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::AbsoluteEncoderConfig& config) const;
    void Adjust(const AbsoluteEncoderConfig& other);
  };

  struct AnalogSensorConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
    ANALOG_SENSOR_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::AnalogSensorConfig& config) const;
    void Adjust(const AnalogSensorConfig& other);
  };

  struct FeedforwardConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name = std::nullopt;
    FEEDFORWARD_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::FeedForwardConfig& config, rev::spark::ClosedLoopSlot slotID) const;
    void Adjust(const FeedforwardConfig& other);
  };

  struct MAXMotionConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name = std::nullopt;
    MAX_MOTION_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::MAXMotionConfig& config, rev::spark::ClosedLoopSlot slotID) const;
    void Adjust(const MAXMotionConfig& other);
  };

  struct ClosedLoopSlotConfig {
    rev::spark::ClosedLoopSlot slotID;
    FeedforwardConfig feedforward = {};
    MAXMotionConfig maxMotion = {};

    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name = std::nullopt;
    CLOSED_LOOP_SLOT_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::ClosedLoopConfig& config) const;
    void Adjust(const ClosedLoopSlotConfig& other);
  };

  struct ClosedLoopConfig {
    std::array<ClosedLoopSlotConfig, 4> slots = {
        {{rev::spark::kSlot0}, {rev::spark::kSlot1}, {rev::spark::kSlot2}, {rev::spark::kSlot3}}};

    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
    CLOSED_LOOP_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::ClosedLoopConfig& config) const;
    void Adjust(const ClosedLoopConfig& other);
  };

  struct EncoderConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
    ENCODER_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::EncoderConfig& config) const;
    void Adjust(const EncoderConfig& other);
  };

  struct LimitSwitchConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
    LIMIT_SWITCH_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::LimitSwitchConfig& config) const;
    void Adjust(const LimitSwitchConfig& other);
  };

  struct SignalsConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
    SIGNALS_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::SignalsConfig& config) const;
    void Adjust(const SignalsConfig& other);
  };

  struct SoftLimitConfig {
    #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
    SOFT_LIMIT_CONFIG_PARAMS(DECLARE_MEMBER)
    #undef DECLARE_MEMBER

    void Fill(rev::spark::SoftLimitConfig& config) const;
    void Adjust(const SoftLimitConfig& other);
  };

  // Sub-structs
  AbsoluteEncoderConfig absoluteEncoder;
  AnalogSensorConfig analogSensor;
  ClosedLoopConfig closedLoop;
  EncoderConfig encoder;
  LimitSwitchConfig limitSwitch;
  SignalsConfig signals;
  SoftLimitConfig softLimit;

  // Simple top-level members
  #define DECLARE_MEMBER(name, type, ...) std::optional<type> name;
  IC_SPARK_CONFIG_PARAMS(DECLARE_MEMBER)
  #undef DECLARE_MEMBER

  // Manually defined members (special cases)
  std::optional<units::ampere_t> smartCurrentStallLimit;
  std::optional<units::ampere_t> smartCurrentfreeLimit;
  std::optional<units::revolutions_per_minute_t> smartCurrentVelocityLimit;
  std::optional<units::ampere_t> secondaryCurrentLimit;
  std::optional<uint32_t> secondaryCurrentLimitChopCycles;
  std::optional<int> followCanId;
  std::optional<bool> followInverted;

  /**
   * @brief Converts this readable ICSparkConfig into a write-only REV SparkBaseConfig.
   */
  void FillREVConfig(rev::spark::SparkBaseConfig& config) const;

  /**
   * @brief Merges another config into this one. Any values present in 'other'
   * will overwrite values in this config.
   */
  void Adjust(const ICSparkConfig& other);
};