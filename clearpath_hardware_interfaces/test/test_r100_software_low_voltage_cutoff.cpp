#include <chrono>

#include "clearpath_hardware_interfaces/r100/software_low_voltage_cutoff.hpp"
#include "gtest/gtest.h"

using r100_sw_low_voltage_cutoff::BatteryVoltageMonitor;
using r100_sw_low_voltage_cutoff::VoltageThresholds;
using sensor_msgs::msg::BatteryState;

TEST(R100SoftwareLowVoltageCutoff, SelectsThresholdsByBatteryTechnology)
{
  VoltageThresholds thresholds{};

  ASSERT_TRUE(r100_sw_low_voltage_cutoff::get_voltage_thresholds(
    BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA, thresholds));
  EXPECT_FLOAT_EQ(23.0F, thresholds.warning);
  EXPECT_FLOAT_EQ(22.0F, thresholds.cutoff);

  ASSERT_TRUE(r100_sw_low_voltage_cutoff::get_voltage_thresholds(
    BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE, thresholds));
  EXPECT_FLOAT_EQ(24.0F, thresholds.warning);
  EXPECT_FLOAT_EQ(23.0F, thresholds.cutoff);

  EXPECT_FALSE(r100_sw_low_voltage_cutoff::get_voltage_thresholds(
    BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN, thresholds));
}

TEST(R100SoftwareLowVoltageCutoff, WarnsAfterThirtySecondsBelowWarningThreshold)
{
  BatteryVoltageMonitor monitor;
  BatteryState state;
  state.present = true;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA;
  state.voltage = 22.5F;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(BatteryVoltageMonitor::Decision::NORMAL, monitor.update(state, start));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(29)));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(30)));

  state.voltage = 23.5F;
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(31)));

  state.voltage = 22.5F;
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(32)));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::NORMAL,
    monitor.update(state, start + std::chrono::seconds(61)));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(62)));
}

TEST(R100SoftwareLowVoltageCutoff, RequiresThirtyContinuousSecondsBelowCutoff)
{
  BatteryVoltageMonitor monitor;
  BatteryState state;
  state.present = true;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA;
  state.voltage = 21.5F;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::LOW,
    monitor.update(state, start));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::LOW,
    monitor.update(state, start + std::chrono::seconds(29)));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::SHUTDOWN,
    monitor.update(state, start + std::chrono::seconds(30)));

  state.voltage = 22.5F;
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(31)));

  state.voltage = 21.5F;
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(32)));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::WARNING,
    monitor.update(state, start + std::chrono::seconds(61)));
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::SHUTDOWN,
    monitor.update(state, start + std::chrono::seconds(62)));
}

TEST(R100SoftwareLowVoltageCutoff, IgnoresMissingOrUnsupportedBattery)
{
  BatteryVoltageMonitor monitor;
  BatteryState state;
  state.present = false;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA;
  state.voltage = 21.0F;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(BatteryVoltageMonitor::Decision::IGNORE, monitor.update(state, start));

  state.present = true;
  state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  EXPECT_EQ(
    BatteryVoltageMonitor::Decision::IGNORE,
    monitor.update(state, start + std::chrono::seconds(1)));
}
