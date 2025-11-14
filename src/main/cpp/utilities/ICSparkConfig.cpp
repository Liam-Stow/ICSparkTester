#include "utilities/ICSparkConfig.h"
#include <tuple>

namespace {
template <typename T>
void UpdateOptional(std::optional<T>& target, const std::optional<T>& source) {
  if (source) {
    target = source;
  }
}
}  // anonymous namespace

void ICSparkConfig::FillREVConfig(rev::spark::SparkBaseConfig& config) const {
  if (inverted)
    config.Inverted(*inverted);
  if (idleMode)
    config.SetIdleMode(*idleMode);
  if (smartCurrentStallLimit && smartCurrentFreeLimit && smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value(), smartCurrentFreeLimit->value(),
                             smartCurrentVelocityLimit->value());
  if (smartCurrentStallLimit && !smartCurrentFreeLimit && !smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value());
  if (smartCurrentStallLimit && smartCurrentFreeLimit && !smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value(), smartCurrentFreeLimit->value());
  if (smartCurrentStallLimit && !smartCurrentFreeLimit && smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value(), 0, smartCurrentVelocityLimit->value());
  if (secondaryCurrentLimit && secondaryCurrentLimitChopCycles)
    config.SecondaryCurrentLimit(secondaryCurrentLimit->value(), *secondaryCurrentLimitChopCycles);
  if (secondaryCurrentLimit && !secondaryCurrentLimitChopCycles)
    config.SecondaryCurrentLimit(secondaryCurrentLimit->value());
  if (openLoopRampRate)
    config.OpenLoopRampRate(openLoopRampRate->value());
  if (closedLoopRampRate)
    config.ClosedLoopRampRate(closedLoopRampRate->value());
  if (voltageCompensationEnabled.value_or(false))
    config.VoltageCompensation(voltageCompensationNominalVoltage.value_or(0_V).value());
  else
    config.DisableVoltageCompensation();
  if (followCanId)
    config.Follow(*followCanId, followInverted.value_or(false));

  // Absolute Encoder
  if (absoluteEncoder.inverted)
    config.absoluteEncoder.Inverted(*absoluteEncoder.inverted);
  if (absoluteEncoder.positionConversionFactor)
    config.absoluteEncoder.PositionConversionFactor(*absoluteEncoder.positionConversionFactor);
  if (absoluteEncoder.velocityConversionFactor)
    config.absoluteEncoder.VelocityConversionFactor(*absoluteEncoder.velocityConversionFactor);
  if (absoluteEncoder.zeroOffset)
    config.absoluteEncoder.ZeroOffset(absoluteEncoder.zeroOffset->value());
  if (absoluteEncoder.averageDepth)
    config.absoluteEncoder.AverageDepth(*absoluteEncoder.averageDepth);
  if (absoluteEncoder.startPulseUs)
    config.absoluteEncoder.StartPulseUs(absoluteEncoder.startPulseUs->value());
  if (absoluteEncoder.endPulseUs)
    config.absoluteEncoder.EndPulseUs(absoluteEncoder.endPulseUs->value());
  if (absoluteEncoder.zeroCentered)
    config.absoluteEncoder.ZeroCentered(*absoluteEncoder.zeroCentered);

  // Analog sensor
  if (analogSensor.inverted)
    config.analogSensor.Inverted(*analogSensor.inverted);
  if (analogSensor.positionConversionFactor)
    config.analogSensor.PositionConversionFactor(*analogSensor.positionConversionFactor);
  if (analogSensor.velocityConversionFactor)
    config.analogSensor.VelocityConversionFactor(*analogSensor.velocityConversionFactor);

  // Closed loop general
  if (closedLoop.positionWrappingEnabled)
    config.closedLoop.PositionWrappingEnabled(*closedLoop.positionWrappingEnabled);
  if (closedLoop.positionWrappingMinInput)
    config.closedLoop.PositionWrappingMinInput(closedLoop.positionWrappingMinInput->value());
  if (closedLoop.positionWrappingMaxInput)
    config.closedLoop.PositionWrappingMaxInput(closedLoop.positionWrappingMaxInput->value());
  if (closedLoop.feedbackSensor)
    config.closedLoop.SetFeedbackSensor(*closedLoop.feedbackSensor);

  // Closed loop - Slots
  for (auto& slot : closedLoop.slots) {
    if (slot.p)
      config.closedLoop.P(*slot.p, slot.slotID);
    if (slot.i)
      config.closedLoop.I(*slot.i, slot.slotID);
    if (slot.d)
      config.closedLoop.D(*slot.d, slot.slotID);
    if (slot.dFilter)
      config.closedLoop.DFilter(*slot.dFilter, slot.slotID);
    if (slot.iZone)
      config.closedLoop.IZone(*slot.iZone, slot.slotID);
    if (slot.minOutput)
      config.closedLoop.MinOutput(*slot.minOutput, slot.slotID);
    if (slot.maxOutput)
      config.closedLoop.MaxOutput(*slot.maxOutput, slot.slotID);
    if (slot.iMaxAccum)
      config.closedLoop.IMaxAccum(*slot.iMaxAccum, slot.slotID);
    if (slot.feedforward.velocity)
      config.closedLoop.feedForward.kV(slot.feedforward.velocity->value(), slot.slotID);
    if (slot.feedforward.acceleration)
      config.closedLoop.feedForward.kA(slot.feedforward.acceleration->value(), slot.slotID);
    if (slot.feedforward.staticFriction)
      config.closedLoop.feedForward.kS(slot.feedforward.staticFriction->value(), slot.slotID);
    if (slot.feedforward.linearGravity)
      config.closedLoop.feedForward.kG(slot.feedforward.linearGravity->value(), slot.slotID);
    if (slot.feedforward.rotationalGravity)
      config.closedLoop.feedForward.kCos(slot.feedforward.rotationalGravity->value(), slot.slotID);
    if (slot.feedforward.cosineRatio)
      config.closedLoop.feedForward.kCosRatio(*slot.feedforward.cosineRatio, slot.slotID);
    if (slot.maxMotion.maxVelocity)
      config.closedLoop.maxMotion.CruiseVelocity(slot.maxMotion.maxVelocity->value(), slot.slotID);
    if (slot.maxMotion.maxAcceleration)
      config.closedLoop.maxMotion.MaxAcceleration(slot.maxMotion.maxAcceleration->value(),
                                                  slot.slotID);
    if (slot.maxMotion.allowedClosedLoopError)
      config.closedLoop.maxMotion.AllowedProfileError(
          slot.maxMotion.allowedClosedLoopError->value(), slot.slotID);
    if (slot.maxMotion.positionMode)
      config.closedLoop.maxMotion.PositionMode(*slot.maxMotion.positionMode, slot.slotID);
  }

  // Encoder
  if (encoder.inverted)
    config.encoder.Inverted(*encoder.inverted);
  if (encoder.positionConversionFactor)
    config.encoder.PositionConversionFactor(*encoder.positionConversionFactor);
  if (encoder.velocityConversionFactor)
    config.encoder.VelocityConversionFactor(*encoder.velocityConversionFactor);
  if (encoder.quadratureAverageDepth)
    config.encoder.QuadratureAverageDepth(*encoder.quadratureAverageDepth);
  if (encoder.quadratureMeasurementPeriod)
    config.encoder.QuadratureMeasurementPeriod(encoder.quadratureMeasurementPeriod->value());
  if (encoder.uvwAverageDepth)
    config.encoder.UvwAverageDepth(*encoder.uvwAverageDepth);
  if (encoder.uvwMeasurementPeriod)
    config.encoder.UvwMeasurementPeriod(encoder.uvwMeasurementPeriod->value());

  // Limit switch
  if (limitSwitch.sparkMaxDataPortLimitSwitchMode.value_or(false))
    config.limitSwitch.SetSparkMaxDataPortConfig();
  if (limitSwitch.positionSensor)
    config.limitSwitch.LimitSwitchPositionSensor(*limitSwitch.positionSensor);
  if (limitSwitch.forwardLimitSwitchPosition)
    config.limitSwitch.ForwardLimitSwitchPosition(*limitSwitch.forwardLimitSwitchPosition);
  if (limitSwitch.forwardLimitSwitchTriggerBehavior)
    config.limitSwitch.ForwardLimitSwitchTriggerBehavior(
        *limitSwitch.forwardLimitSwitchTriggerBehavior);
  if (limitSwitch.forwardLimitSwitchType)
    config.limitSwitch.ForwardLimitSwitchType(*limitSwitch.forwardLimitSwitchType);
  if (limitSwitch.reverseLimitSwitchPosition)
    config.limitSwitch.ReverseLimitSwitchPosition(*limitSwitch.reverseLimitSwitchPosition);
  if (limitSwitch.reverseLimitSwitchTriggerBehavior)
    config.limitSwitch.ReverseLimitSwitchTriggerBehavior(
        *limitSwitch.reverseLimitSwitchTriggerBehavior);
  if (limitSwitch.reverseLimitSwitchType)
    config.limitSwitch.ReverseLimitSwitchType(*limitSwitch.reverseLimitSwitchType);

  // Signals
  if (signals.appliedOutputPeriodMs)
    config.signals.AppliedOutputPeriodMs(signals.appliedOutputPeriodMs->value());
  if (signals.busVoltagePeriodMs)
    config.signals.BusVoltagePeriodMs(signals.busVoltagePeriodMs->value());
  if (signals.outputCurrentPeriodMs)
    config.signals.OutputCurrentPeriodMs(signals.outputCurrentPeriodMs->value());
  if (signals.motorTemperaturePeriodMs)
    config.signals.MotorTemperaturePeriodMs(signals.motorTemperaturePeriodMs->value());
  if (signals.limitsPeriodMs)
    config.signals.LimitsPeriodMs(signals.limitsPeriodMs->value());
  if (signals.faultsPeriodMs)
    config.signals.FaultsPeriodMs(signals.faultsPeriodMs->value());
  if (signals.faultsAlwaysOn)
    config.signals.FaultsAlwaysOn(*signals.faultsAlwaysOn);
  if (signals.warningsPeriodMs)
    config.signals.WarningsPeriodMs(signals.warningsPeriodMs->value());
  if (signals.warningsAlwaysOn)
    config.signals.WarningsAlwaysOn(*signals.warningsAlwaysOn);
  if (signals.primaryEncoderVelocityPeriodMs)
    config.signals.PrimaryEncoderVelocityPeriodMs(signals.primaryEncoderVelocityPeriodMs->value());
  if (signals.primaryEncoderVelocityAlwaysOn)
    config.signals.PrimaryEncoderVelocityAlwaysOn(*signals.primaryEncoderVelocityAlwaysOn);
  if (signals.primaryEncoderPositionPeriodMs)
    config.signals.PrimaryEncoderPositionPeriodMs(signals.primaryEncoderPositionPeriodMs->value());
  if (signals.primaryEncoderPositionAlwaysOn)
    config.signals.PrimaryEncoderPositionAlwaysOn(*signals.primaryEncoderPositionAlwaysOn);
  if (signals.analogVoltagePeriodMs)
    config.signals.AnalogVoltagePeriodMs(signals.analogVoltagePeriodMs->value());
  if (signals.analogVoltageAlwaysOn)
    config.signals.AnalogVoltageAlwaysOn(*signals.analogVoltageAlwaysOn);
  if (signals.analogVelocityPeriodMs)
    config.signals.AnalogVelocityPeriodMs(signals.analogVelocityPeriodMs->value());
  if (signals.analogVelocityAlwaysOn)
    config.signals.AnalogVelocityAlwaysOn(*signals.analogVelocityAlwaysOn);
  if (signals.analogPositionPeriodMs)
    config.signals.AnalogPositionPeriodMs(signals.analogPositionPeriodMs->value());
  if (signals.analogPositionAlwaysOn)
    config.signals.AnalogPositionAlwaysOn(*signals.analogPositionAlwaysOn);
  if (signals.externalOrAltEncoderVelocity)
    config.signals.ExternalOrAltEncoderVelocity(signals.externalOrAltEncoderVelocity->value());
  if (signals.externalOrAltEncoderVelocityAlwaysOn)
    config.signals.ExternalOrAltEncoderVelocityAlwaysOn(
        *signals.externalOrAltEncoderVelocityAlwaysOn);
  if (signals.externalOrAltEncoderPosition)
    config.signals.ExternalOrAltEncoderPosition(signals.externalOrAltEncoderPosition->value());
  if (signals.externalOrAltEncoderPositionAlwaysOn)
    config.signals.ExternalOrAltEncoderPositionAlwaysOn(
        *signals.externalOrAltEncoderPositionAlwaysOn);
  if (signals.absoluteEncoderVelocityPeriodMs)
    config.signals.AbsoluteEncoderVelocityPeriodMs(
        signals.absoluteEncoderVelocityPeriodMs->value());
  if (signals.absoluteEncoderVelocityAlwaysOn)
    config.signals.AbsoluteEncoderVelocityAlwaysOn(*signals.absoluteEncoderVelocityAlwaysOn);
  if (signals.absoluteEncoderPositionPeriodMs)
    config.signals.AbsoluteEncoderPositionPeriodMs(
        signals.absoluteEncoderPositionPeriodMs->value());
  if (signals.absoluteEncoderPositionAlwaysOn)
    config.signals.AbsoluteEncoderPositionAlwaysOn(*signals.absoluteEncoderPositionAlwaysOn);
  if (signals.iAccumulationPeriodMs)
    config.signals.IAccumulationPeriodMs(signals.iAccumulationPeriodMs->value());
  if (signals.iAccumulationAlwaysOn)
    config.signals.IAccumulationAlwaysOn(*signals.iAccumulationAlwaysOn);

  // Soft limit
  if (softLimit.forwardSoftLimit)
    config.softLimit.ForwardSoftLimit(softLimit.forwardSoftLimit->value());
  if (softLimit.forwardSoftLimitEnabled)
    config.softLimit.ForwardSoftLimitEnabled(*softLimit.forwardSoftLimitEnabled);
  if (softLimit.reverseSoftLimit)
    config.softLimit.ReverseSoftLimit(softLimit.reverseSoftLimit->value());
  if (softLimit.reverseSoftLimitEnabled)
    config.softLimit.ReverseSoftLimitEnabled(*softLimit.reverseSoftLimitEnabled);
}

void ICSparkConfig::Adjust(const ICSparkConfig& other) {
  // Top level configs
  UpdateOptional(inverted, other.inverted);
  UpdateOptional(idleMode, other.idleMode);
  UpdateOptional(smartCurrentStallLimit, other.smartCurrentStallLimit);
  UpdateOptional(smartCurrentFreeLimit, other.smartCurrentFreeLimit);
  UpdateOptional(smartCurrentVelocityLimit, other.smartCurrentVelocityLimit);
  UpdateOptional(secondaryCurrentLimit, other.secondaryCurrentLimit);
  UpdateOptional(secondaryCurrentLimitChopCycles, other.secondaryCurrentLimitChopCycles);
  UpdateOptional(openLoopRampRate, other.openLoopRampRate);
  UpdateOptional(closedLoopRampRate, other.closedLoopRampRate);
  UpdateOptional(voltageCompensationNominalVoltage, other.voltageCompensationNominalVoltage);
  UpdateOptional(followCanId, other.followCanId);
  UpdateOptional(followInverted, other.followInverted);

  // Absolute Encoder
  UpdateOptional(absoluteEncoder.inverted, other.absoluteEncoder.inverted);
  UpdateOptional(absoluteEncoder.positionConversionFactor,
                 other.absoluteEncoder.positionConversionFactor);
  UpdateOptional(absoluteEncoder.velocityConversionFactor,
                 other.absoluteEncoder.velocityConversionFactor);
  UpdateOptional(absoluteEncoder.zeroOffset, other.absoluteEncoder.zeroOffset);
  UpdateOptional(absoluteEncoder.averageDepth, other.absoluteEncoder.averageDepth);
  UpdateOptional(absoluteEncoder.startPulseUs, other.absoluteEncoder.startPulseUs);
  UpdateOptional(absoluteEncoder.endPulseUs, other.absoluteEncoder.endPulseUs);
  UpdateOptional(absoluteEncoder.zeroCentered, other.absoluteEncoder.zeroCentered);

  // Analog sensor
  UpdateOptional(analogSensor.inverted, other.analogSensor.inverted);
  UpdateOptional(analogSensor.positionConversionFactor,
                 other.analogSensor.positionConversionFactor);
  UpdateOptional(analogSensor.velocityConversionFactor,
                 other.analogSensor.velocityConversionFactor);

  // Closed loop global settings
  UpdateOptional(closedLoop.positionWrappingEnabled, other.closedLoop.positionWrappingEnabled);
  UpdateOptional(closedLoop.positionWrappingMinInput, other.closedLoop.positionWrappingMinInput);
  UpdateOptional(closedLoop.positionWrappingMaxInput, other.closedLoop.positionWrappingMaxInput);
  UpdateOptional(closedLoop.feedbackSensor, other.closedLoop.feedbackSensor);

  // Helper lambda to merge a closed-loop slot
  auto MergeSlot = [](ClosedLoopSlotConfig& targetSlot, const ClosedLoopSlotConfig& sourceSlot) {
    UpdateOptional(targetSlot.p, sourceSlot.p);
    UpdateOptional(targetSlot.i, sourceSlot.i);
    UpdateOptional(targetSlot.d, sourceSlot.d);
    UpdateOptional(targetSlot.dFilter, sourceSlot.dFilter);
    UpdateOptional(targetSlot.iZone, sourceSlot.iZone);
    UpdateOptional(targetSlot.minOutput, sourceSlot.minOutput);
    UpdateOptional(targetSlot.maxOutput, sourceSlot.maxOutput);
    UpdateOptional(targetSlot.iMaxAccum, sourceSlot.iMaxAccum);
    UpdateOptional(targetSlot.feedforward.velocity, sourceSlot.feedforward.velocity);
    UpdateOptional(targetSlot.feedforward.acceleration, sourceSlot.feedforward.acceleration);
    UpdateOptional(targetSlot.feedforward.staticFriction, sourceSlot.feedforward.staticFriction);
    UpdateOptional(targetSlot.feedforward.linearGravity, sourceSlot.feedforward.linearGravity);
    UpdateOptional(targetSlot.feedforward.rotationalGravity,
                   sourceSlot.feedforward.rotationalGravity);
    UpdateOptional(targetSlot.feedforward.cosineRatio, sourceSlot.feedforward.cosineRatio);
    UpdateOptional(targetSlot.maxMotion.maxVelocity, sourceSlot.maxMotion.maxVelocity);
    UpdateOptional(targetSlot.maxMotion.maxAcceleration, sourceSlot.maxMotion.maxAcceleration);
    UpdateOptional(targetSlot.maxMotion.allowedClosedLoopError,
                   sourceSlot.maxMotion.allowedClosedLoopError);
    UpdateOptional(targetSlot.maxMotion.positionMode, sourceSlot.maxMotion.positionMode);
  };

  MergeSlot(closedLoop.slots[0], other.closedLoop.slots[0]);
  MergeSlot(closedLoop.slots[1], other.closedLoop.slots[1]);
  MergeSlot(closedLoop.slots[2], other.closedLoop.slots[2]);
  MergeSlot(closedLoop.slots[3], other.closedLoop.slots[3]);

  // Encoder
  UpdateOptional(encoder.inverted, other.encoder.inverted);
  UpdateOptional(encoder.positionConversionFactor, other.encoder.positionConversionFactor);
  UpdateOptional(encoder.velocityConversionFactor, other.encoder.velocityConversionFactor);
  UpdateOptional(encoder.quadratureAverageDepth, other.encoder.quadratureAverageDepth);
  UpdateOptional(encoder.quadratureMeasurementPeriod, other.encoder.quadratureMeasurementPeriod);
  UpdateOptional(encoder.uvwAverageDepth, other.encoder.uvwAverageDepth);
  UpdateOptional(encoder.uvwMeasurementPeriod, other.encoder.uvwMeasurementPeriod);

  // Limit switch
  UpdateOptional(limitSwitch.forwardLimitSwitchPosition,
                 other.limitSwitch.forwardLimitSwitchPosition);
  UpdateOptional(limitSwitch.forwardLimitSwitchTriggerBehavior,
                 other.limitSwitch.forwardLimitSwitchTriggerBehavior);
  UpdateOptional(limitSwitch.forwardLimitSwitchType, other.limitSwitch.forwardLimitSwitchType);
  UpdateOptional(limitSwitch.reverseLimitSwitchPosition,
                 other.limitSwitch.reverseLimitSwitchPosition);
  UpdateOptional(limitSwitch.reverseLimitSwitchTriggerBehavior,
                 other.limitSwitch.reverseLimitSwitchTriggerBehavior);
  UpdateOptional(limitSwitch.reverseLimitSwitchType, other.limitSwitch.reverseLimitSwitchType);
  UpdateOptional(limitSwitch.positionSensor, other.limitSwitch.positionSensor);

  // Signals
  UpdateOptional(signals.appliedOutputPeriodMs, other.signals.appliedOutputPeriodMs);
  UpdateOptional(signals.busVoltagePeriodMs, other.signals.busVoltagePeriodMs);
  UpdateOptional(signals.outputCurrentPeriodMs, other.signals.outputCurrentPeriodMs);
  UpdateOptional(signals.motorTemperaturePeriodMs, other.signals.motorTemperaturePeriodMs);
  UpdateOptional(signals.limitsPeriodMs, other.signals.limitsPeriodMs);
  UpdateOptional(signals.faultsPeriodMs, other.signals.faultsPeriodMs);
  UpdateOptional(signals.faultsAlwaysOn, other.signals.faultsAlwaysOn);
  UpdateOptional(signals.warningsPeriodMs, other.signals.warningsPeriodMs);
  UpdateOptional(signals.warningsAlwaysOn, other.signals.warningsAlwaysOn);
  UpdateOptional(signals.primaryEncoderVelocityPeriodMs,
                 other.signals.primaryEncoderVelocityPeriodMs);
  UpdateOptional(signals.primaryEncoderVelocityAlwaysOn,
                 other.signals.primaryEncoderVelocityAlwaysOn);
  UpdateOptional(signals.primaryEncoderPositionPeriodMs,
                 other.signals.primaryEncoderPositionPeriodMs);
  UpdateOptional(signals.primaryEncoderPositionAlwaysOn,
                 other.signals.primaryEncoderPositionAlwaysOn);
  UpdateOptional(signals.analogVoltagePeriodMs, other.signals.analogVoltagePeriodMs);
  UpdateOptional(signals.analogVoltageAlwaysOn, other.signals.analogVoltageAlwaysOn);
  UpdateOptional(signals.analogVelocityPeriodMs, other.signals.analogVelocityPeriodMs);
  UpdateOptional(signals.analogVelocityAlwaysOn, other.signals.analogVelocityAlwaysOn);
  UpdateOptional(signals.analogPositionPeriodMs, other.signals.analogPositionPeriodMs);
  UpdateOptional(signals.analogPositionAlwaysOn, other.signals.analogPositionAlwaysOn);
  UpdateOptional(signals.externalOrAltEncoderVelocity, other.signals.externalOrAltEncoderVelocity);
  UpdateOptional(signals.externalOrAltEncoderVelocityAlwaysOn,
                 other.signals.externalOrAltEncoderVelocityAlwaysOn);
  UpdateOptional(signals.externalOrAltEncoderPosition, other.signals.externalOrAltEncoderPosition);
  UpdateOptional(signals.externalOrAltEncoderPositionAlwaysOn,
                 other.signals.externalOrAltEncoderPositionAlwaysOn);
  UpdateOptional(signals.absoluteEncoderVelocityPeriodMs,
                 other.signals.absoluteEncoderVelocityPeriodMs);
  UpdateOptional(signals.absoluteEncoderVelocityAlwaysOn,
                 other.signals.absoluteEncoderVelocityAlwaysOn);
  UpdateOptional(signals.absoluteEncoderPositionPeriodMs,
                 other.signals.absoluteEncoderPositionPeriodMs);
  UpdateOptional(signals.absoluteEncoderPositionAlwaysOn,
                 other.signals.absoluteEncoderPositionAlwaysOn);
  UpdateOptional(signals.iAccumulationPeriodMs, other.signals.iAccumulationPeriodMs);
  UpdateOptional(signals.iAccumulationAlwaysOn, other.signals.iAccumulationAlwaysOn);

  // Soft limit
  UpdateOptional(softLimit.forwardSoftLimit, other.softLimit.forwardSoftLimit);
  UpdateOptional(softLimit.forwardSoftLimitEnabled, other.softLimit.forwardSoftLimitEnabled);
  UpdateOptional(softLimit.reverseSoftLimit, other.softLimit.reverseSoftLimit);
  UpdateOptional(softLimit.reverseSoftLimitEnabled, other.softLimit.reverseSoftLimitEnabled);
}