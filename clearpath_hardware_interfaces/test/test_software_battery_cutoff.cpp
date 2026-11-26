#include <chrono>

#include "clearpath_hardware_interfaces/common/software_battery_cutoff.hpp"
#include "gtest/gtest.h"

using software_battery_cutoff::BatteryCutoffMonitor;
using software_battery_cutoff::VoltageThresholds;
using sensor_msgs::msg::BatteryState;

TEST(SoftwareBatteryCutoff, SelectsVoltageThresholdsByBatteryTechnology)
{
  VoltageThresholds thresholds{};

  ASSERT_TRUE(software_battery_cutoff::get_voltage_thresholds(
    BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA, thresholds));
  EXPECT_FLOAT_EQ(23.0F, thresholds.warning);
  EXPECT_FLOAT_EQ(22.0F, thresholds.cutoff);

  ASSERT_TRUE(software_battery_cutoff::get_voltage_thresholds(
    BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE, thresholds));
  EXPECT_FLOAT_EQ(24.0F, thresholds.warning);
  EXPECT_FLOAT_EQ(23.0F, thresholds.cutoff);

  EXPECT_FALSE(software_battery_cutoff::get_voltage_thresholds(
    BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN, thresholds));
}

TEST(SoftwareBatteryCutoff, WarnsAfterThirtySecondsBelowVoltageThreshold)
{
  BatteryCutoffMonitor monitor;
  BatteryState state;
  state.present = true;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA;
  state.voltage = 22.5F;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(BatteryCutoffMonitor::Decision::NORMAL, monitor.update(state, start, true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(29), true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(30), true));

  state.voltage = 23.5F;
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(31), true));

  state.voltage = 22.5F;
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(32), true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(61), true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(62), true));
}

TEST(SoftwareBatteryCutoff, RequiresThirtyContinuousSecondsBelowVoltageCutoff)
{
  BatteryCutoffMonitor monitor;
  BatteryState state;
  state.present = true;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA;
  state.voltage = 21.5F;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::LOW,
    monitor.update(state, start, true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::LOW,
    monitor.update(state, start + std::chrono::seconds(29), true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::SHUTDOWN,
    monitor.update(state, start + std::chrono::seconds(30), true));

  state.voltage = 22.5F;
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(31), true));

  state.voltage = 21.5F;
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(32), true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(61), true));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::SHUTDOWN,
    monitor.update(state, start + std::chrono::seconds(62), true));
}

TEST(SoftwareBatteryCutoff, AppliesStateOfChargePolicyAcrossPlatforms)
{
  BatteryCutoffMonitor monitor;
  BatteryState state;
  state.present = true;
  const auto start = std::chrono::steady_clock::time_point{};

  state.percentage = 0.12F;
  EXPECT_EQ(BatteryCutoffMonitor::Decision::NORMAL, monitor.update(state, start, false));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(29), false));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(30), false));

  state.percentage = 0.08F;
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(31), false));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(60), false));
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::SHUTDOWN,
    monitor.update(state, start + std::chrono::seconds(61), false));
}

TEST(SoftwareBatteryCutoff, IgnoresMissingOrUnsupportedBattery)
{
  BatteryCutoffMonitor monitor;
  BatteryState state;
  state.present = false;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA;
  state.voltage = 21.0F;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(BatteryCutoffMonitor::Decision::IGNORE, monitor.update(state, start, true));

  state.present = true;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  EXPECT_EQ(
    BatteryCutoffMonitor::Decision::IGNORE,
    monitor.update(state, start + std::chrono::seconds(1), true));
}
