// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "battery.h"
#include <gtest/gtest.h>
#include <uORB/Publication.hpp>

class ConfigurableBattery : public Battery
{
public:
	using Battery::Battery;
	using Battery::updateParams;
};

class BatteryTest : public ::testing::Test
{
protected:
	hrt_abstime now{1000000};
	uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};

	void integer(const char *name, int32_t value)
	{
		ASSERT_EQ(param_set_no_notification(param_find(name), &value), 0) << name;
	}

	void profile(int mode = battery_soc_status_s::MODE_SHADOW)
	{
		integer("BAT1_SOC_MODE", mode);
		set("BAT1_CAPACITY", 4000.f);
		set("BAT1_R_INTERNAL", 0.f);
		set("BAT_SOC_SETTLE", 1.f);

		for (unsigned i = 0; i < BatterySocEstimator::PROFILE_POINTS; ++i) {
			char name[17];
			snprintf(name, sizeof(name), "BAT1_SOC_V%u", i);
			const float soc = i / static_cast<float>(BatterySocEstimator::PROFILE_POINTS - 1);
			set(name, 3.3f + 0.9f * soc * soc);
		}
	}

	void set(const char *name, float value)
	{
		ASSERT_EQ(param_set_no_notification(param_find(name), &value), 0) << name;
	}

	void SetUp() override
	{
		param_control_autosave(false); // Functional tests do not start the autosave work queue.
		param_reset_all();
		const int32_t cells = 4;
		ASSERT_EQ(param_set_no_notification(param_find("BAT1_N_CELLS"), &cells), 0);
		set("BAT1_V_EMPTY", 3.5f);
		set("BAT1_V_CHARGED", 4.2f);
		set("BAT1_R_INTERNAL", 0.02f);
		set("BAT1_CAPACITY", 0.f);
		set("BAT1_I_OVERWRITE", 0.f);
	}

	void step(Battery &battery, float voltage, float current, unsigned count = 1, bool armed = false, bool fresh_vehicle_status = true)
	{
		for (unsigned i = 0; i < count; ++i) {
			if (fresh_vehicle_status) {
				vehicle_status_s status{};
				status.timestamp = now;
				status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
				vehicle_status_pub.publish(status);
			}

			battery.updateVoltage(voltage);
			battery.updateCurrent(current);
			battery.updateBatteryStatus(now);
			now += 50000;
		}
	}
};

TEST_F(BatteryTest, CompensatedVoltageUsesSocFilterAndConfiguredResistance)
{
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f, 60);
	const auto status = battery.getBatteryStatus();
	EXPECT_FLOAT_EQ(status.voltage_v, 14.6f);
	EXPECT_NEAR(status.voltage_v_compensated, 15.4f, 0.0001f);
	EXPECT_NEAR(status.remaining, 0.5f, 0.0001f);
	// The diagnostic OCV estimate uses estimated resistance, not our override.
	EXPECT_GT(fabsf(status.voltage_v_compensated - status.ocv_estimate_filtered), 0.1f);

	step(battery, 14.6f, 20.f);
	const auto filtered = battery.getBatteryStatus();
	EXPECT_GT(filtered.voltage_v_compensated, status.voltage_v_compensated);
	EXPECT_LT(filtered.voltage_v_compensated, 16.2f); // No unfiltered jump to V + I*R.
	EXPECT_NEAR(filtered.remaining, (filtered.voltage_v_compensated / 4.f - 3.5f) / 0.7f, 0.0001f);
}

TEST_F(BatteryTest, CompensatedVoltageUsesEstimatedResistanceWhenSelected)
{
	set("BAT1_R_INTERNAL", -1.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f, 60);
	const auto status = battery.getBatteryStatus();
	EXPECT_GT(status.voltage_v_compensated, status.voltage_v);
	EXPECT_NEAR(status.voltage_v_compensated, status.ocv_estimate_filtered, 0.0001f);
	EXPECT_NEAR(status.remaining, (status.voltage_v_compensated / 4.f - 3.5f) / 0.7f, 0.0001f);
}

TEST_F(BatteryTest, MissingBatteryInitializationOrCellCountDoesNotPublishAnEstimate)
{
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().voltage_v_compensated, 0.f);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().voltage_v_compensated, 0.f);
	step(battery, 14.6f, 10.f, 60);
	EXPECT_GT(battery.getBatteryStatus().voltage_v_compensated, 0.f);
	battery.setConnected(false);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().voltage_v_compensated, 0.f);

	const int32_t cells = 0;
	ASSERT_EQ(param_set_no_notification(param_find("BAT1_N_CELLS"), &cells), 0);
	Battery unknown_cells(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	unknown_cells.setConnected(true);
	step(unknown_cells, 14.6f, 10.f, 60);
	EXPECT_FLOAT_EQ(unknown_cells.getBatteryStatus().voltage_v_compensated, 0.f);
}

TEST_F(BatteryTest, PartiallyChargedBatteryStartsFromVoltageEvenWithKnownCapacity)
{
	set("BAT1_R_INTERNAL", 0.f);
	set("BAT1_CAPACITY", 4000.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 15.4f, 0.f, 60);
	const auto status = battery.getBatteryStatus();
	EXPECT_FLOAT_EQ(status.discharged_mah, 0.f);
	EXPECT_NEAR(status.remaining, 0.5f, 0.0001f);
	step(battery, 15.4f, 8.f, 20);
	const auto discharged = battery.getBatteryStatus();
	EXPECT_GT(discharged.discharged_mah, 0.f);
	EXPECT_LT(discharged.remaining, status.remaining);
	EXPECT_LT(discharged.remaining, 1.f - discharged.discharged_mah / 4000.f);
}

TEST_F(BatteryTest, NoLoadVoltageAndWarningsContinueUsingTheSameSocFilter)
{
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f, 60);
	const float before = battery.getBatteryStatus().voltage_v_compensated;
	step(battery, 14.6f, 0.f);
	EXPECT_LT(battery.getBatteryStatus().voltage_v_compensated, before);
	step(battery, 13.f, 0.f, 200);
	const auto empty = battery.getBatteryStatus();
	EXPECT_LT(empty.voltage_v_compensated, 14.f);
	EXPECT_FLOAT_EQ(empty.remaining, 0.f);
	EXPECT_EQ(empty.warning, battery_status_s::WARNING_EMERGENCY);
}

TEST_F(BatteryTest, ShadowModePreservesLegacyRemainingAndWarnings)
{
	profile();
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 0.f, 100);
	const auto status = battery.getBatteryStatus();
	const auto diagnostics = battery.getSocEstimatorStatus();
	ASSERT_TRUE(diagnostics.initialized);
	EXPECT_FALSE(diagnostics.active);
	EXPECT_FLOAT_EQ(status.remaining, diagnostics.legacy_soc);
	EXPECT_EQ(status.warning, battery.determineWarning(diagnostics.legacy_soc));
	EXPECT_NEAR(diagnostics.soc, 0.5f, 1e-5f);
	EXPECT_GT(diagnostics.soc - diagnostics.legacy_soc, 0.4f);
}

TEST_F(BatteryTest, ActiveModeStartsAfterNormalSensorSettlingWithoutLowCurrentOrRelaxation)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	set("BAT_SOC_SETTLE", 120.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 10.f, 40);
	EXPECT_FALSE(battery.getSocEstimatorStatus().active);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, -1.f);
	step(battery, 14.1f, 10.f, 3);
	const auto initial = battery.getBatteryStatus();
	ASSERT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_TRUE(battery.getSocEstimatorStatus().initialized);
	EXPECT_TRUE(battery.getSocEstimatorStatus().charge_history_valid);
	EXPECT_FALSE(battery.getSocEstimatorStatus().correction_eligible);
	EXPECT_NEAR(initial.remaining, 0.5f, 0.0001f);

	for (unsigned i = 0; i < 200; ++i) {
		step(battery, 16.8f, i % 2 == 0 ? 3.f : 10.f, 1, i >= 10);
	}

	const auto diagnostics = battery.getSocEstimatorStatus();
	EXPECT_TRUE(diagnostics.active);
	EXPECT_TRUE(diagnostics.charge_history_valid);
	EXPECT_FALSE(diagnostics.correction_eligible);
	EXPECT_NEAR(battery.getBatteryStatus().remaining,
		    initial.remaining - (diagnostics.discharged_mah - initial.discharged_mah) / 4000.f, 1e-5f);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, diagnostics.soc);
}

TEST_F(BatteryTest, FirstQualifiedRestCorrectsTheStartupEstimateGradually)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 10.f, 60);
	ASSERT_TRUE(battery.getSocEstimatorStatus().active);
	ASSERT_TRUE(battery.getSocEstimatorStatus().charge_history_valid);
	const float startup_soc = battery.getBatteryStatus().remaining;
	const float pure_coulomb_soc = battery.getSocEstimatorStatus().coulomb_soc;
	step(battery, 15.225f, 0.f, 100); // The synthetic profile maps this relaxed voltage to 75%.
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_TRUE(battery.getSocEstimatorStatus().charge_history_valid);
	EXPECT_TRUE(battery.getSocEstimatorStatus().correction_eligible);
	EXPECT_GT(battery.getBatteryStatus().remaining, startup_soc);
	EXPECT_LT(battery.getBatteryStatus().remaining, startup_soc + 0.01f);
	EXPECT_FLOAT_EQ(battery.getSocEstimatorStatus().coulomb_soc, pure_coulomb_soc);
}

TEST_F(BatteryTest, ActiveModeCanStartWhileAlreadyArmedWithoutRelaxation)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	set("BAT_SOC_SETTLE", 120.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 5.f, 43, true);
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_TRUE(battery.getSocEstimatorStatus().charge_history_valid);
	EXPECT_FALSE(battery.getSocEstimatorStatus().correction_eligible);
	EXPECT_NEAR(battery.getBatteryStatus().remaining, 0.5f, 0.0001f);
}

TEST_F(BatteryTest, UnconfiguredOrInvalidActiveProfileReportsUnavailableSoc)
{
	integer("BAT1_SOC_MODE", battery_soc_status_s::MODE_ACTIVE);
	set("BAT1_CAPACITY", 4000.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.f, 0.f, 100);
	EXPECT_FALSE(battery.getSocEstimatorStatus().profile_valid);
	EXPECT_FALSE(battery.getSocEstimatorStatus().active);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, -1.f);
	EXPECT_TRUE(std::isnan(battery.getBatteryStatus().time_remaining_s));
}

TEST_F(BatteryTest, InvalidVoltageProtectionBlocksNewSocSelection)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	set("BAT1_V_CRIT", 3.5f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 0.f, 100);
	EXPECT_FALSE(battery.getSocEstimatorStatus().protection_valid);
	EXPECT_FALSE(battery.getSocEstimatorStatus().active);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, -1.f);
}

TEST_F(BatteryTest, IndependentTerminalVoltageWarningReachesExistingFailsafeField)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	set("BAT1_CAPACITY", 100000.f); // Deliberately incorrect, dangerous overestimate of usable capacity.
	set("BAT1_R_INTERNAL", 0.05f); // Strong compensation must not hide low terminal voltage.
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 0.f, 100);
	ASSERT_TRUE(battery.getSocEstimatorStatus().active);
	step(battery, 12.4f, 20.f, 200, true);
	const auto status = battery.getBatteryStatus();
	const auto diagnostics = battery.getSocEstimatorStatus();
	EXPECT_GT(status.remaining, 0.99f);
	EXPECT_GT(status.voltage_v_compensated, 16.f);
	EXPECT_EQ(diagnostics.soc_warning, battery_status_s::WARNING_NONE);
	EXPECT_EQ(diagnostics.voltage_warning, battery_status_s::WARNING_EMERGENCY);
	EXPECT_EQ(status.warning, battery_status_s::WARNING_EMERGENCY);
}

TEST_F(BatteryTest, ActiveModeArmingCyclePreservesChargeAndSoc)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 0.f, 100);
	const float initial_discharge = battery.getBatteryStatus().discharged_mah;
	step(battery, 14.1f, 10.f, 100, true);
	const auto flying = battery.getBatteryStatus();
	EXPECT_NEAR(flying.remaining, 0.5f - (flying.discharged_mah - initial_discharge) / 4000.f, 1e-5f);
	step(battery, 16.8f, 0.f);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, flying.remaining);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().discharged_mah, flying.discharged_mah);
	EXPECT_FALSE(battery.getSocEstimatorStatus().correction_eligible);
}

TEST_F(BatteryTest, DisconnectResetsMeasuredDischargeAndInitializesReplacementFromRelaxedVoltage)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 0.f, 100);
	step(battery, 16.8f, 10.f, 100, true);
	ASSERT_GT(battery.getBatteryStatus().discharged_mah, 0.f);
	battery.setConnected(false);
	step(battery, 0.f, 0.f);
	EXPECT_FALSE(battery.getSocEstimatorStatus().initialized);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().discharged_mah, 0.f);
	battery.setConnected(true);
	step(battery, 14.1f, 0.f, 100);
	EXPECT_NEAR(battery.getBatteryStatus().remaining, 0.5f, 1e-5f);
	EXPECT_FALSE(battery.getSocEstimatorStatus().full_charge_confirmed);
}

TEST_F(BatteryTest, MissingCurrentDoesNotRepeatThePreviousDischargeIncrement)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 0.f, 100);
	step(battery, 16.8f, 10.f, 100, true);
	const auto before = battery.getBatteryStatus();
	step(battery, 16.8f, -1.f, 100, true);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().discharged_mah, before.discharged_mah);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, before.remaining);
	step(battery, 16.8f, NAN, 100, true);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().discharged_mah, before.discharged_mah);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, before.remaining);
	EXPECT_FALSE(battery.getSocEstimatorStatus().charge_history_valid);
}

TEST_F(BatteryTest, MissingVoltageCannotQualifyUsingAnOldFilteredMeasurement)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 0.f, 10);
	step(battery, NAN, 0.f, 100);
	EXPECT_FALSE(battery.getSocEstimatorStatus().initialized);
	EXPECT_FALSE(battery.getSocEstimatorStatus().active);
	step(battery, 14.1f, 0.f, 200);
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_TRUE(std::isfinite(battery.getBatteryStatus().remaining));
}

TEST_F(BatteryTest, StaleVehicleStatusAndIdleCurrentOverwriteCannotQualifyRelaxation)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 10.f);
	now += 3000000; // The last vehicle heartbeat expires before low-current settling begins.
	step(battery, 16.8f, 0.f, 100, false, false);
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_TRUE(battery.getSocEstimatorStatus().charge_history_valid);
	EXPECT_FALSE(battery.getSocEstimatorStatus().correction_eligible);
	set("BAT1_I_OVERWRITE", 1.f);
	ConfigurableBattery overwritten(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	overwritten.setConnected(true);
	step(overwritten, 16.8f, 0.f, 100);
	EXPECT_FALSE(overwritten.getSocEstimatorStatus().active);
}

TEST_F(BatteryTest, ExternalSocIsNeverReplacedOrGivenNewVoltageWarnings)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_EXTERNAL);
	battery.setConnected(true);
	battery.setStateOfCharge(0.8f);
	step(battery, 12.f, 20.f, 200, true);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, 0.8f);
	EXPECT_EQ(battery.getBatteryStatus().warning, battery_status_s::WARNING_NONE);
	EXPECT_TRUE(battery.getSocEstimatorStatus().external_soc);
	EXPECT_FALSE(battery.getSocEstimatorStatus().active);
}

TEST_F(BatteryTest, ModeAndProfileChangesAreDeferredWhileArmed)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	ConfigurableBattery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 0.f, 100);
	step(battery, 16.8f, 10.f, 100, true);
	integer("BAT1_SOC_MODE", battery_soc_status_s::MODE_DISABLED);
	set("BAT1_CAPACITY", 1000.f);
	battery.updateParams();
	step(battery, 16.8f, 10.f, 10, true);
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_EQ(battery.getSocEstimatorStatus().mode, battery_soc_status_s::MODE_ACTIVE);
	step(battery, 16.8f, 0.f);
	EXPECT_FALSE(battery.getSocEstimatorStatus().active);
	EXPECT_EQ(battery.getSocEstimatorStatus().mode, battery_soc_status_s::MODE_DISABLED);
}

TEST_F(BatteryTest, SwitchingBackToShadowUsesLegacyOutputAndRetainsComparisonHistory)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	ConfigurableBattery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 0.f, 100);
	ASSERT_TRUE(battery.getSocEstimatorStatus().active);
	integer("BAT1_SOC_MODE", battery_soc_status_s::MODE_SHADOW);
	battery.updateParams();
	step(battery, 14.1f, 0.f);
	EXPECT_FALSE(battery.getSocEstimatorStatus().active);
	EXPECT_TRUE(battery.getSocEstimatorStatus().initialized);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, battery.getSocEstimatorStatus().legacy_soc);
}

TEST_F(BatteryTest, DiagnosticPublicationCarriesBatteryIdentityAndMeasurementTime)
{
	profile();
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 0.f, 100);
	uORB::Subscription diagnostics_sub{ORB_ID(battery_soc_status)};
	battery.publishBatteryStatus(battery.getBatteryStatus());
	battery_soc_status_s diagnostics{};
	ASSERT_TRUE(diagnostics_sub.copy(&diagnostics));
	EXPECT_EQ(diagnostics.battery_id, 1);
	EXPECT_EQ(diagnostics.source, battery_status_s::SOURCE_POWER_MODULE);
	EXPECT_EQ(diagnostics.timestamp_sample, now - 50000);
	EXPECT_EQ(diagnostics.mode, battery_soc_status_s::MODE_SHADOW);
	EXPECT_NEAR(diagnostics.soc, 0.5f, 1e-5f);
	EXPECT_FALSE(diagnostics.active);
}

TEST_F(BatteryTest, MissingStartupCurrentStillProvidesADefinedVoltageSoc)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, NAN, 100);
	const auto status = battery.getBatteryStatus();
	EXPECT_TRUE(std::isfinite(status.remaining));
	EXPECT_NEAR(status.voltage_v_compensated, 16.8f, 0.001f);
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_TRUE(battery.getSocEstimatorStatus().initialized);
	EXPECT_FLOAT_EQ(status.remaining, 1.f);
	EXPECT_FALSE(battery.getSocEstimatorStatus().charge_history_valid);
	step(battery, 14.1f, 0.f, 200);
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
}

TEST_F(BatteryTest, DefaultFlightCorrectionsQualifyAtHighCurrentInShadowMode)
{
	profile();
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 0.f, 100);
	step(battery, 15.f, 20.f, 200, true);
	EXPECT_FALSE(battery.getSocEstimatorStatus().correction_eligible);
	step(battery, 15.f, 20.f, 200, true);
	const auto diagnostics = battery.getSocEstimatorStatus();
	EXPECT_TRUE(diagnostics.correction_eligible);
	EXPECT_LT(diagnostics.soc, diagnostics.coulomb_soc);
	EXPECT_LT(diagnostics.coulomb_soc - diagnostics.soc, 0.004f);
	EXPECT_LT(diagnostics.legacy_soc, diagnostics.soc - 0.1f);
	EXPECT_FALSE(diagnostics.active);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, diagnostics.legacy_soc);
}

TEST_F(BatteryTest, ConfiguredFlightSettlingAndRateLimitReachActiveOutput)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	set("BAT_SOC_F_SETT", 3.f);
	set("BAT_SOC_F_RATE", 6.f);
	set("BAT_SOC_F_DV", 0.f);
	set("BAT_SOC_TC", 1.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 16.8f, 0.f, 100);
	step(battery, 13.2f, 20.f, 61, true);
	EXPECT_TRUE(battery.getSocEstimatorStatus().correction_eligible);
	step(battery, 13.2f, 20.f, 1340, true);
	const auto diagnostics = battery.getSocEstimatorStatus();
	EXPECT_NEAR(diagnostics.coulomb_soc - diagnostics.soc, 0.067f, 0.0005f);
	EXPECT_NEAR(battery.getBatteryStatus().discharged_mah, 20.f * 70.05f / 3.6f, 0.1f);
	EXPECT_TRUE(diagnostics.active);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, diagnostics.soc);
}

TEST_F(BatteryTest, ConfiguredFlightPowerSpanAndVoltageMarginAreRespected)
{
	profile(battery_soc_status_s::MODE_ACTIVE);
	set("BAT_SOC_P_STAB", 10.f);
	set("BAT_SOC_F_DV", 0.1f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.1f, 0.f, 100);

	for (unsigned i = 0; i < 400; ++i) {
		step(battery, 14.f, i % 2 == 0 ? 18.f : 22.f, 1, true);
	}

	EXPECT_FALSE(battery.getSocEstimatorStatus().correction_eligible);
	step(battery, 14.f, 20.f, 400, true);
	EXPECT_TRUE(battery.getSocEstimatorStatus().correction_eligible);
	EXPECT_NEAR(battery.getSocEstimatorStatus().soc, battery.getSocEstimatorStatus().coulomb_soc, 1e-6f);
}
