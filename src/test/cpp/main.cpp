#include <gtest/gtest.h>
#include <hal/HALBase.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/CommandPtr.h>
#include <units/time.h>

#include "subsystems/Arm.h"
#include "subsystems/Elevator.h"
#include "subsystems/Feeder.h"
#include "subsystems/Flywheel.h"
#include "subsystems/Turret.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

void SimCmdScheduler(units::second_t duration = 20_ms) {
  int loops = (duration.value() * 1000) / 20;
  for (int i = 0; i <= loops; i++) {
    frc2::CommandScheduler::GetInstance().Run();
  }
}

void ExpectNearAngle(units::turn_t expected, units::turn_t actual,
                     units::turn_t tolerance = 0.005_tr) {
  EXPECT_NEAR(expected.value(), actual.value(), tolerance.value());
}
void ExpectNearAngleVel(units::turns_per_second_t expected, units::turns_per_second_t actual,
                        units::turns_per_second_t tolerance = 0.1_tps) {
  EXPECT_NEAR(expected.value(), actual.value(), tolerance.value());
}
void ExpectNearLinear(units::meter_t expected, units::meter_t actual,
                      units::meter_t tolerance = 0.01_m) {
  EXPECT_NEAR(expected.value(), actual.value(), tolerance.value());
}
void ExpectNearLinearVelocity(units::meters_per_second_t expected,
                              units::meters_per_second_t actual,
                              units::meters_per_second_t tolerance = 0.1_mps) {
  EXPECT_NEAR(expected.value(), actual.value(), tolerance.value());
}

// ARM TESTS
class ArmTest : public testing::Test {};

TEST_F(ArmTest, wpiProfile) {
  Arm arm;
  auto target = 90_deg;
  auto cmd = arm.WPIProfileTo(target);
  cmd.Schedule();

  SimCmdScheduler(5_s);
  ExpectNearAngle(target, arm.GetPosition());
}

TEST_F(ArmTest, pid) {
  Arm arm;
  units::turn_t target = 90_deg;
  auto cmd = arm.PIDTo(target);
  cmd.Schedule();

  auto duration = 0.5_s;
  int loops = (duration.value() * 1000) / 20;
  for (int i = 0; i <= loops; i++) {
    frc2::CommandScheduler::GetInstance().Run();
    std::cout << "Arm Position: " << arm.GetPosition().value() << " turns\n";
    std::cout << "Arm Velocity: " << arm.GetVelocity().value() << " turns/sec\n";
    std::cout << "Arm Duty Cycle: " << arm.GetMotorDutyCycle() << "\n";
    std::cout << "-------------------------\n";
  }

  EXPECT_NEAR(target.value(), arm.GetPosition().value(), 0.005);
}

// ELEVATOR TESTS
class ElevatorTest : public testing::Test {
 protected:
  Elevator elevator;
};

TEST_F(ElevatorTest, wpiProfile) {
  auto target = 0.5_m;
  auto cmd = elevator.WPIProfileTo(target);
  cmd.Schedule();
  SimCmdScheduler(5_s);
  ExpectNearLinear(target, elevator.GetHeight());
}

TEST_F(ElevatorTest, pid) {
  auto target = 0.5_m;
  auto cmd = elevator.PIDTo(target);
  cmd.Schedule();
  SimCmdScheduler(2_s);
  ExpectNearLinear(target, elevator.GetHeight());
}

TEST_F(ElevatorTest, driveWithDutyCycle) {
  auto target = 0.5;
  auto cmd = elevator.DriveWithDutyCycle(target);
  cmd.Schedule();
  SimCmdScheduler(
      0.5_s);  // give it some time to accelerate so the current limit isn't affecting dutycycle
  EXPECT_NEAR(target, elevator.GetMotorDutyCycle(), 0.0001);
}

// FLYWHEEL TESTS
class FlywheelTest : public testing::Test {
 protected:
  Flywheel flywheel;
};

TEST_F(FlywheelTest, spinAt) {
  auto target = 3000_rpm;
  auto cmd = flywheel.SpinAt(target);
  cmd.Schedule();
  SimCmdScheduler(10_s);
  ExpectNearAngleVel(target, flywheel.GetVelocity());
}

TEST_F(FlywheelTest, spinAtDutyCycle) {
  auto target = 0.5;
  auto cmd = flywheel.SpinAt(target);
  cmd.Schedule();
  SimCmdScheduler(
      0.5_s);  // give it some time to accelerate so the current limit isn't affecting dutycycle
  EXPECT_NEAR(target, flywheel.GetMotorDutyCycle(), 0.0001);
}

// TURRET TESTS
class TurretTest : public testing::Test {
 protected:
  Turret turret;
};

TEST_F(TurretTest, wpiProfile) {
  auto target = 90_deg;
  auto cmd = turret.WPIProfileTo(target);
  cmd.Schedule();
  SimCmdScheduler(5_s);
  ExpectNearAngle(target, turret.GetPosition());
}

TEST_F(TurretTest, pid) {
  auto target = 90_deg;
  auto cmd = turret.PIDTo(target);
  cmd.Schedule();
  SimCmdScheduler(2_s);
  ExpectNearAngle(target, turret.GetPosition());
}

TEST_F(TurretTest, driveWithDutyCycle) {
  auto target = 0.5;
  auto cmd = turret.DriveWithDutyCycle(target);
  cmd.Schedule();
  SimCmdScheduler(
      0.5_s);  // give it some time to accelerate so the current limit isn't affecting dutycycle
  EXPECT_NEAR(target, turret.GetMotorDutyCycle(), 0.005);
}

// FEEDER TESTS
class FeederTest : public testing::Test {
 protected:
  Feeder feeder;
};

TEST_F(FeederTest, feedIn) {
  auto cmd = feeder.FeedIn();
  cmd.Schedule();
  SimCmdScheduler(
      0.5_s);  // give it some time to accelerate so the current limit isn't affecting dutycycle
  EXPECT_GT(feeder.GetMotorDutyCycle(), 0);
}

TEST_F(FeederTest, feedOut) {
  auto cmd = feeder.FeedOut();
  cmd.Schedule();
  SimCmdScheduler(
      0.5_s);  // give it some time to accelerate so the current limit isn't affecting dutycycle
  EXPECT_LT(feeder.GetMotorDutyCycle(), 0);
}
