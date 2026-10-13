// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "BatterySocEstimator.hpp"
#include <gtest/gtest.h>
#include <uORB/topics/battery_status.h>

class BatterySocEstimatorTest : public ::testing::Test
{
protected:
	BatterySocEstimator estimator;
	BatterySocEstimator::Config config{};
	uint64_t now{1000000};
	float discharged{0.f};

	void SetUp() override
	{
		config.capacity_mah = 4000.f;
		config.cell_count = 4;
		config.settling_time = 2.f;

		for (unsigned i = 0; i < BatterySocEstimator::PROFILE_POINTS; ++i) {
			const float soc = i / static_cast<float>(BatterySocEstimator::PROFILE_POINTS - 1);
			config.voltage[i] = 3.3f + 0.9f * soc * soc; // Synthetic nonlinear fixture, not a LiPo calibration.
		}

		estimator.configure(config);
	}

	void sample(float current = 0.f, float voltage = 3.525f, bool armed = false, bool state_valid = true,
		    uint64_t interval = 100000, float terminal = NAN)
	{
		if (std::isfinite(current) && current >= 0.f) {
			discharged += current * (interval * 1e-6f) / 3.6f;
		}

		estimator.update(now, true, armed, state_valid, current, voltage,
				 std::isfinite(terminal) ? terminal : voltage, discharged);
		now += interval;
	}

	void rest(float voltage = 3.525f, unsigned count = 31)
	{
		for (unsigned i = 0; i < count; ++i) { sample(0.f, voltage); }
	}
};

TEST_F(BatterySocEstimatorTest, NonlinearInterpolationAndEndpoints)
{
	ASSERT_TRUE(estimator.profileValid());
	EXPECT_FLOAT_EQ(estimator.lookup(3.f), 0.f);
	EXPECT_FLOAT_EQ(estimator.lookup(5.f), 1.f);
	EXPECT_NEAR(estimator.lookup(config.voltage[20]), 0.5f, 1e-6f);
	EXPECT_NEAR(estimator.lookup((config.voltage[20] + config.voltage[21]) / 2.f), 0.5125f, 1e-6f);
	EXPECT_NEAR(estimator.lookup(config.voltage[39]), 0.975f, 1e-6f);
	EXPECT_TRUE(std::isnan(estimator.lookup(NAN)));
}

TEST_F(BatterySocEstimatorTest, OldTwentyOnePointProfileCannotActivateNewEstimator)
{
	for (unsigned i = 0; i < BatterySocEstimator::PROFILE_POINTS; ++i) {
		const float old_soc = i / 20.f;
		config.voltage[i] = i < 21 ? 3.3f + 0.9f * old_soc * old_soc : 0.f;
	}

	estimator.configure(config);
	EXPECT_FALSE(estimator.profileValid());
	rest();
	EXPECT_FALSE(estimator.initialized());
}

TEST_F(BatterySocEstimatorTest, RejectsIncompleteNonmonotonicOrNonfiniteProfile)
{
	for (float bad : {0.f, NAN, INFINITY, 6.f}) {
		auto invalid = config;
		invalid.voltage[10] = bad;
		estimator.configure(invalid);
		EXPECT_FALSE(estimator.profileValid());
		EXPECT_TRUE(std::isnan(estimator.lookup(3.8f)));
	}

	config.voltage[20] = config.voltage[19];
	EXPECT_TRUE(estimator.configure(config));
	EXPECT_FALSE(estimator.profileValid());
}

TEST_F(BatterySocEstimatorTest, RequiresCapacityCellsAndCorrectionConfiguration)
{
	config.capacity_mah = 0.f;
	estimator.configure(config);
	EXPECT_FALSE(estimator.profileValid());
	config.capacity_mah = 4000.f;
	config.cell_count = 0;
	estimator.configure(config);
	EXPECT_FALSE(estimator.profileValid());
	config.cell_count = 4;
	config.correction_time_constant = NAN;
	estimator.configure(config);
	EXPECT_FALSE(estimator.profileValid());
}

TEST_F(BatterySocEstimatorTest, PartialStartupIsInitializedFromVoltageBeforeRelaxation)
{
	sample();
	EXPECT_NEAR(estimator.soc(), 0.5f, 1e-5f);
	EXPECT_TRUE(estimator.initialized());
	EXPECT_TRUE(estimator.chargeHistoryValid());
	EXPECT_FALSE(estimator.correctionEligible());
	rest();
	EXPECT_TRUE(estimator.initialized());
	EXPECT_TRUE(estimator.chargeHistoryValid());
	EXPECT_FALSE(estimator.fullChargeConfirmed());
	EXPECT_NEAR(estimator.soc(), 0.5f, 1e-5f);
}

TEST_F(BatterySocEstimatorTest, LoadedStartupTracksChargeWithoutWaitingForRelaxation)
{
	config.settling_time = 120.f;
	estimator.configure(config);
	sample(5.f);
	ASSERT_TRUE(estimator.initialized());
	const float initial_soc = estimator.soc();
	const float initial_discharge = discharged;

	for (unsigned i = 0; i < 600; ++i) {
		// Variable idle loads, followed by immediate arming, never meet the relaxation gates.
		sample(i % 2 == 0 ? 3.f : 10.f, config.voltage[30], i >= 10);
	}

	EXPECT_NEAR(estimator.soc(), initial_soc - (discharged - initial_discharge) / config.capacity_mah, 1e-5f);
	EXPECT_NEAR(estimator.soc(), estimator.coulombSoc(), 1e-5f);
	EXPECT_TRUE(estimator.chargeHistoryValid());
	EXPECT_FALSE(estimator.correctionEligible());
	EXPECT_FALSE(estimator.fullChargeConfirmed());
}

TEST_F(BatterySocEstimatorTest, MissingStartupCurrentStillInitializesFromVoltage)
{
	for (float invalid : {-1.f, NAN, INFINITY}) {
		estimator.reset();
		sample(invalid);
		EXPECT_TRUE(estimator.initialized());
		EXPECT_NEAR(estimator.soc(), 0.5f, 1e-5f);
		EXPECT_FALSE(estimator.chargeHistoryValid());
		EXPECT_FALSE(estimator.correctionEligible());
	}
}

TEST_F(BatterySocEstimatorTest, MissingStartupVoltageCannotInventAStartingSoc)
{
	estimator.update(now, true, false, true, 5.f, NAN, NAN, discharged);
	now += 100000;
	EXPECT_FALSE(estimator.initialized());
	sample(5.f);
	EXPECT_TRUE(estimator.initialized());
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, FullChargeRequiresRelaxationNotJustHighVoltage)
{
	sample(20.f, 4.2f);
	EXPECT_FALSE(estimator.fullChargeConfirmed());
	EXPECT_TRUE(estimator.chargeHistoryValid());
	rest(4.2f);
	EXPECT_TRUE(estimator.fullChargeConfirmed());
	EXPECT_FLOAT_EQ(estimator.soc(), 1.f);
}

TEST_F(BatterySocEstimatorTest, HighLoadUsesOnlyCoulombCounting)
{
	rest();
	const float baseline_discharge = discharged;

	for (unsigned i = 0; i < 360; ++i) { sample(10.f, 3.1f); }

	EXPECT_FALSE(estimator.correctionEligible());
	EXPECT_NEAR(estimator.soc(), 0.5f - (discharged - baseline_discharge) / config.capacity_mah, 1e-5f);
	EXPECT_NEAR(estimator.soc(), estimator.coulombSoc(), 1e-5f);
}

TEST_F(BatterySocEstimatorTest, CorrectionsCanIncreaseSocWithoutACoulombUpperClamp)
{
	rest();
	const float pure = estimator.coulombSoc();
	rest(config.voltage[30], 3001);
	EXPECT_NEAR(estimator.soc(), 0.75f + (0.5f - 0.75f) * expf(-300.1f / 300.f), 0.0002f);
	EXPECT_GT(estimator.soc(), pure);
	EXPECT_FLOAT_EQ(estimator.coulombSoc(), pure);
	EXPECT_FLOAT_EQ(discharged, 0.f);
}

TEST_F(BatterySocEstimatorTest, CorrectionsCanDecreaseSocGradually)
{
	rest();
	sample(0.f, config.voltage[10]);
	EXPECT_GT(estimator.soc(), 0.499f);
	rest(config.voltage[10], 3000);
	EXPECT_NEAR(estimator.soc(), 0.25f + 0.25f * expf(-300.1f / 300.f), 0.0002f);
	EXPECT_FLOAT_EQ(estimator.coulombSoc(), 0.5f);
}

TEST_F(BatterySocEstimatorTest, UpdateFrequencyDoesNotChangeCorrectionOrChargeIntegration)
{
	float result[3] {};
	unsigned k = 0;

	for (uint64_t interval : {10000ULL, 100000ULL, 1000000ULL}) {
		BatterySocEstimator other;
		other.configure(config);
		uint64_t timestamp = 1000000;
		other.update(timestamp, true, false, true, 0.f, config.voltage[20], config.voltage[20], 0.f);

		for (uint64_t elapsed = interval; elapsed <= 3000000; elapsed += interval) {
			other.update(timestamp + elapsed, true, false, true, 0.f, config.voltage[20], config.voltage[20], 0.f);
		}

		for (uint64_t elapsed = interval; elapsed <= 300000000; elapsed += interval) {
			const float mah = (elapsed * 1e-6f) / 3.6f;
			other.update(timestamp + 3000000 + elapsed, true, false, true, 1.f, config.voltage[30], config.voltage[30], mah);
		}

		result[k++] = other.soc();
		EXPECT_NEAR(other.coulombSoc(), 0.5f - 300.f / 3.6f / 4000.f, 0.0005f);
	}

	EXPECT_NEAR(result[0], result[1], 0.0005f);
	EXPECT_NEAR(result[1], result[2], 0.0005f);
}

TEST_F(BatterySocEstimatorTest, SlowCurrentDriftRestartsQualification)
{
	config.settling_time = 10.f;
	estimator.configure(config);

	for (unsigned i = 0; i < 100; ++i) { sample(i * 0.02f); }

	EXPECT_TRUE(estimator.initialized());
	EXPECT_TRUE(estimator.chargeHistoryValid());
	EXPECT_FALSE(estimator.correctionEligible());
	rest(3.525f, 110);
	EXPECT_TRUE(estimator.initialized());
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, LoadChangesRequireANewSettlingPeriod)
{
	rest();
	sample(20.f);
	rest(3.525f, 10);
	EXPECT_FALSE(estimator.correctionEligible());
	rest();
	EXPECT_TRUE(estimator.correctionEligible());
}

TEST_F(BatterySocEstimatorTest, ArmedCorrectionCanBeDisabledExplicitly)
{
	ASSERT_TRUE(config.correction_in_flight);
	config.correction_in_flight = false;
	estimator.configure(config);
	rest();
	const float before = estimator.soc();

	for (unsigned i = 0; i < 400; ++i) { sample(0.f, config.voltage[30], true); }

	EXPECT_FLOAT_EQ(estimator.soc(), before);
	EXPECT_FALSE(estimator.correctionEligible());
	config.correction_in_flight = true;
	estimator.configure(config);

	for (unsigned i = 0; i < 110; ++i) { sample(0.f, config.voltage[30], true); }

	EXPECT_TRUE(estimator.initialized());
	EXPECT_TRUE(estimator.correctionEligible());
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, ArmingAndDisarmingDoNotResetSocOrChargeHistory)
{
	rest();

	for (unsigned i = 0; i < 100; ++i) { sample(10.f, 3.2f, true); }

	const float before = estimator.soc();
	sample(0.f, 4.2f, false);
	EXPECT_FLOAT_EQ(estimator.soc(), before);
	EXPECT_TRUE(estimator.initialized());
	EXPECT_FALSE(estimator.correctionEligible());
}

TEST_F(BatterySocEstimatorTest, UnknownVehicleStatePreventsCorrectionsButAllowsStartup)
{
	for (unsigned i = 0; i < 40; ++i) { sample(0.f, 4.2f, false, false); }

	EXPECT_TRUE(estimator.initialized());
	EXPECT_FALSE(estimator.correctionEligible());
	EXPECT_FALSE(estimator.fullChargeConfirmed());
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, InvalidCurrentCannotQualifyVoltageOrPoisonSoc)
{
	rest();
	const float before = estimator.soc();

	for (float invalid : {-1.f, NAN, INFINITY}) {
		sample(invalid, 4.2f);
		EXPECT_FLOAT_EQ(estimator.soc(), before);
		EXPECT_FALSE(estimator.chargeHistoryValid());
		EXPECT_FALSE(estimator.correctionEligible());
	}

	rest();
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, MissingVoltageStillCountsMeasuredChargeAndCannotInitialize)
{
	rest();
	const float baseline = discharged;

	for (unsigned i = 0; i < 100; ++i) {
		now += 100000;
		discharged += 10.f * 0.1f / 3.6f;
		estimator.update(now, true, false, true, 10.f, NAN, NAN, discharged);
	}

	EXPECT_NEAR(estimator.soc(), 0.5f - (discharged - baseline) / config.capacity_mah, 1e-5f);
	EXPECT_FALSE(estimator.correctionEligible());
}

TEST_F(BatterySocEstimatorTest, LongSampleGapInvalidatesChargeHistoryAndRestartsSettling)
{
	rest();
	now += 10000000;
	sample();
	EXPECT_FALSE(estimator.chargeHistoryValid());
	EXPECT_FALSE(estimator.correctionEligible());
	rest();
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, DuplicateAndBackwardTimestampsDoNotIntegrate)
{
	rest();
	const float before = estimator.soc();
	estimator.update(now - 100000, true, false, true, 10.f, 4.2f, 4.2f, 1000.f);
	estimator.update(now - 200000, true, false, true, 10.f, 4.2f, 4.2f, 1000.f);
	EXPECT_FLOAT_EQ(estimator.soc(), before);
}

TEST_F(BatterySocEstimatorTest, DischargeCounterResetCannotIncreaseSoc)
{
	rest();

	for (unsigned i = 0; i < 100; ++i) { sample(10.f); }

	const float before = estimator.soc();
	discharged = 0.f;
	sample();
	EXPECT_FLOAT_EQ(estimator.soc(), before);
	EXPECT_FALSE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, DisconnectAndRebootDoNotReuseChargeHistory)
{
	rest(4.2f);
	estimator.update(now, false, false, true, 0.f, 4.2f, 4.2f, discharged);
	EXPECT_FALSE(estimator.initialized());
	EXPECT_TRUE(std::isnan(estimator.soc()));
	rest();
	EXPECT_NEAR(estimator.soc(), 0.5f, 1e-5f);
	BatterySocEstimator rebooted;
	rebooted.configure(config);
	EXPECT_FALSE(rebooted.initialized());
	EXPECT_TRUE(std::isnan(rebooted.soc()));
}

TEST_F(BatterySocEstimatorTest, VoltageProtectionIsIndependentOfSocAndCapacity)
{
	rest(4.2f);

	for (unsigned i = 0; i < 100; ++i) { sample(20.f, 4.2f, true, true, 100000, 3.1f); }

	EXPECT_GT(estimator.soc(), 0.98f);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_EMERGENCY);
	config.capacity_mah = 0.f;
	estimator.configure(config);

	for (unsigned i = 0; i < 100; ++i) { sample(20.f, 4.2f, true, true, 100000, 3.1f); }

	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_EMERGENCY);
}

TEST_F(BatterySocEstimatorTest, BriefVoltageDropIsFilteredAndDebounced)
{
	rest(4.2f);
	sample(20.f, 4.2f, true, true, 100000, 3.f);
	rest(4.2f);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_NONE);
}

TEST_F(BatterySocEstimatorTest, VoltageWarningHasHysteresisAndRecoveryDebounce)
{
	rest(3.35f, 100);
	ASSERT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_CRITICAL);
	rest(3.43f, 100);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_CRITICAL);
	rest(4.2f, 5);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_CRITICAL);
	rest(4.2f, 100);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_NONE);
}

TEST_F(BatterySocEstimatorTest, InvalidVoltageDoesNotClearWarningOrQualifyRelaxation)
{
	rest(3.1f, 100);
	estimator.update(now, true, false, true, 0.f, NAN, NAN, discharged);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_EMERGENCY);
	EXPECT_FALSE(estimator.correctionEligible());
}

TEST_F(BatterySocEstimatorTest, VoltageDebounceCannotSpanAMissingSampleInterval)
{
	config.voltage_filter_time_constant = 0.01f;
	estimator.configure(config);
	rest(3.1f, 10);
	now += 10000000;
	sample(0.f, 3.1f);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_NONE);
	rest(3.1f, 10);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_NONE);
	rest(3.1f, 30);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_EMERGENCY);
}

TEST_F(BatterySocEstimatorTest, InvalidProtectionThresholdsAreRejected)
{
	config.voltage_critical = config.voltage_low;
	estimator.configure(config);
	EXPECT_FALSE(estimator.protectionValid());
	rest(3.1f, 100);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_NONE);
}

TEST_F(BatterySocEstimatorTest, UnchangedConfigurationDoesNotResetSoc)
{
	rest();
	EXPECT_FALSE(estimator.configure(config));
	EXPECT_TRUE(estimator.initialized());
	EXPECT_NEAR(estimator.soc(), 0.5f, 1e-5f);
	config.capacity_mah *= 0.8f;
	EXPECT_TRUE(estimator.configure(config));
	EXPECT_FALSE(estimator.initialized());
}

TEST_F(BatterySocEstimatorTest, CalibrationChangesAndSyntheticCurrentInvalidateChargeInitialization)
{
	rest();
	config.cell_resistance = 0.01f;
	EXPECT_TRUE(estimator.configure(config));
	EXPECT_FALSE(estimator.initialized());
	config.current_overwrite = true;
	estimator.configure(config);
	EXPECT_FALSE(estimator.profileValid());
	EXPECT_TRUE(estimator.protectionValid());
}

TEST_F(BatterySocEstimatorTest, MissingVoltageBreaksAnOtherwiseShortDebounceInterval)
{
	config.voltage_filter_time_constant = 0.01f;
	estimator.configure(config);
	rest(3.1f, 10);
	estimator.update(now, true, false, true, 0.f, NAN, NAN, discharged);
	now += 1500000; // Less than the sample-gap limit, but voltage was absent.
	sample(0.f, 3.1f);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_NONE);
	rest(3.1f, 10);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_NONE);
	rest(3.1f, 30);
	EXPECT_EQ(estimator.voltageWarning(), battery_status_s::WARNING_EMERGENCY);
}

TEST_F(BatterySocEstimatorTest, VariableFlightLoadsTrackKnownPartialCapacityDespitePolarization)
{
	config.correction_in_flight = false; // Pure coulomb-counting reference for loaded-voltage error.
	estimator.configure(config);
	rest(config.voltage[32]); // Independently known 80% start.
	float true_soc = 0.8f;

	for (float current : {5.f, 20.f, 8.f, 30.f, 12.f}) {
		for (unsigned second = 0; second < 60; ++second) {
			true_soc -= current / (3.6f * 4000.f);
			const float ocv = 3.3f + 0.9f * true_soc * true_soc;
			const float polarization = 0.05f;
			const float terminal = ocv - current * 0.005f - polarization;
			sample(current, ocv - polarization, true, true, 1000000, terminal);
		}
	}

	EXPECT_NEAR(true_soc, 0.4875f, 0.0001f);
	EXPECT_NEAR(estimator.soc(), true_soc, 0.001f);
	EXPECT_GT(fabsf(estimator.voltageSoc() - true_soc), 0.05f);
	EXPECT_FALSE(estimator.correctionEligible());
}

TEST_F(BatterySocEstimatorTest, SemiStableFlightPowerQualifiesWithoutLowCurrentOrLongRelaxation)
{
	config.settling_time = 120.f;
	estimator.configure(config);

	for (unsigned i = 0; i < 100; ++i) {
		sample(i % 2 == 0 ? 18.f : 22.f, 3.8f, true);
		EXPECT_FALSE(estimator.correctionEligible());
	}

	sample(20.f, 3.8f, true);
	EXPECT_TRUE(estimator.correctionEligible());
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, FlightQualificationUsesElectricalPowerRatherThanCurrentAlone)
{
	config.flight_power_stability = 0.05f;
	estimator.configure(config);

	for (unsigned i = 0; i < 110; ++i) {
		const float terminal = 3.1f + i * 0.009f;
		sample(100.f / (terminal * config.cell_count), 3.8f, true, true, 100000, terminal);
	}

	EXPECT_TRUE(estimator.correctionEligible());
	estimator.reset();

	for (unsigned i = 0; i < 200; ++i) {
		sample(10.f, 3.8f, true, true, 100000, i % 2 == 0 ? 3.1f : 4.1f);
		EXPECT_FALSE(estimator.correctionEligible()); // Constant current, varying terminal power.
	}
}

TEST_F(BatterySocEstimatorTest, FlightLoadStepsAndGradualRampsRestartQualification)
{
	for (unsigned i = 0; i < 110; ++i) { sample(20.f, 3.8f, true); }

	ASSERT_TRUE(estimator.correctionEligible());
	sample(40.f, 3.8f, true);
	EXPECT_FALSE(estimator.correctionEligible());

	for (unsigned i = 0; i < 99; ++i) { sample(40.f, 3.8f, true); }

	EXPECT_FALSE(estimator.correctionEligible());
	sample(40.f, 3.8f, true);
	EXPECT_TRUE(estimator.correctionEligible());
	estimator.reset();

	for (unsigned i = 0; i < 300; ++i) {
		sample(10.f * expf(i * 0.005f), 3.8f, true);
		EXPECT_FALSE(estimator.correctionEligible()); // Small steps, but >30% span within 10 s.
	}
}

TEST_F(BatterySocEstimatorTest, FlightVoltageMarginRejectsSmallErrorsInBothDirections)
{
	for (float error : {-0.015f, 0.015f}) {
		estimator.reset();
		discharged = 0.f;
		sample(0.f, config.voltage[32]);

		for (unsigned second = 0; second < 180; ++second) {
			const float true_soc = 0.8f - (discharged + 10.f / 3.6f) / config.capacity_mah;
			const float ocv = 3.3f + 0.9f * true_soc * true_soc;
			sample(10.f, ocv + error, true, true, 1000000, ocv - 0.05f);
			EXPECT_NEAR(estimator.soc(), estimator.coulombSoc(), 1e-6f);
		}

		EXPECT_TRUE(estimator.correctionEligible());
	}
}

TEST_F(BatterySocEstimatorTest, FlightCorrectionsAreSymmetricallyRateLimitedWithoutLimitingDischarge)
{
	for (float direction : {-1.f, 1.f}) {
		estimator.reset();
		discharged = 0.f;
		sample();
		float previous_correction = 0.f;

		for (unsigned second = 0; second <= 70; ++second) {
			sample(20.f, direction < 0.f ? 3.f : 4.3f, true, true, 1000000, 3.7f);
			const float correction = estimator.soc() - estimator.coulombSoc();
			EXPECT_LE(fabsf(correction - previous_correction), config.flight_correction_rate + 1e-6f);
			previous_correction = correction;
		}

		EXPECT_NEAR(estimator.soc() - estimator.coulombSoc(), direction * 0.02f, 1e-5f);
		EXPECT_NEAR(estimator.coulombSoc(), 0.5f - discharged / config.capacity_mah, 1e-5f);
		EXPECT_GT(0.5f - estimator.coulombSoc(), 0.09f); // Measured discharge exceeds the voltage rate cap.
		EXPECT_FALSE(estimator.fullChargeConfirmed()); // Loaded full voltage is not relaxed confirmation.
	}
}

TEST_F(BatterySocEstimatorTest, FlightVoltageCorrectionTimeConstantStillAppliesBelowRateLimit)
{
	config.flight_voltage_margin = 0.f;
	estimator.configure(config);
	sample();

	for (unsigned second = 0; second <= 310; ++second) {
		sample(0.f, config.voltage[22], true, true, 1000000);
	}

	EXPECT_NEAR(estimator.soc(), 0.55f - 0.05f * expf(-300.f / config.correction_time_constant), 1e-5f);
}

TEST_F(BatterySocEstimatorTest, BriefLoadSagCannotPullSocDown)
{
	config.flight_voltage_margin = 0.1f;
	estimator.configure(config);
	sample(0.f, config.voltage[24]);

	for (unsigned second = 0; second < 20; ++second) { sample(10.f, config.voltage[24], true, true, 1000000); }

	ASSERT_TRUE(estimator.correctionEligible());
	sample(40.f, 3.f, true, true, 1000000);
	EXPECT_FALSE(estimator.correctionEligible());
	EXPECT_NEAR(estimator.soc(), estimator.coulombSoc(), 1e-6f);

	for (unsigned second = 0; second < 10; ++second) {
		sample(10.f, config.voltage[24], true, true, 1000000);
		EXPECT_FALSE(estimator.correctionEligible());
		EXPECT_NEAR(estimator.soc(), estimator.coulombSoc(), 1e-6f);
	}
}

TEST_F(BatterySocEstimatorTest, MissingFlightMeasurementsNeverCauseAFullVoltageReseed)
{
	for (unsigned failure = 0; failure < 3; ++failure) {
		estimator.reset();
		discharged = 0.f;
		sample(0.f, config.voltage[32]);
		sample(10.f, 3.8f, true);

		if (failure == 0) {
			sample(NAN, 3.3f, true);

		} else if (failure == 1) {
			now += 10000000;
			sample(10.f, 3.3f, true);

		} else {
			discharged = 0.f;
			sample(0.f, 3.3f, true);
		}

		ASSERT_FALSE(estimator.chargeHistoryValid());
		ASSERT_FALSE(estimator.correctionEligible());

		for (unsigned second = 0; second < 30; ++second) { sample(10.f, 3.3f, true, true, 1000000); }

		EXPECT_TRUE(estimator.correctionEligible());
		EXPECT_FALSE(estimator.chargeHistoryValid());
		EXPECT_GT(estimator.soc(), 0.75f);
		EXPECT_LT(estimator.soc(), estimator.coulombSoc());
		EXPECT_LT(estimator.coulombSoc() - estimator.soc(), 0.01f);
		rest();
		EXPECT_TRUE(estimator.chargeHistoryValid());
		EXPECT_NEAR(estimator.soc(), 0.5f, 1e-5f); // Only disarmed relaxation can reanchor missing history.
	}
}

TEST_F(BatterySocEstimatorTest, FlightQualificationCannotCarryAcrossArmingStateOrStaleVehicleStatus)
{
	rest();
	ASSERT_TRUE(estimator.correctionEligible());
	sample(0.f, 3.525f, true);
	EXPECT_FALSE(estimator.correctionEligible());

	for (unsigned i = 0; i < 110; ++i) { sample(0.f, 3.525f, true); }

	ASSERT_TRUE(estimator.correctionEligible());
	sample(0.f, 3.525f, true, false);
	EXPECT_FALSE(estimator.correctionEligible());

	for (unsigned i = 0; i < 100; ++i) { sample(0.f, 3.525f, true); }

	EXPECT_FALSE(estimator.correctionEligible());
	sample(0.f, 3.525f, true);
	EXPECT_TRUE(estimator.correctionEligible());
	sample();
	EXPECT_FALSE(estimator.correctionEligible());
	EXPECT_NEAR(estimator.soc(), 0.5f, 1e-6f);
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, FlightCorrectionAndQualificationAreConsistentAcrossUpdateRates)
{
	float result[3] {};
	unsigned k = 0;

	for (uint64_t interval : {10000ULL, 100000ULL, 1000000ULL}) {
		BatterySocEstimator other;
		other.configure(config);
		other.update(1000000, true, false, true, 0.f, config.voltage[32], config.voltage[32], 0.f);
		other.update(2000000, true, true, true, 10.f, config.voltage[20], 3.5f, 0.f);

		for (uint64_t elapsed = interval; elapsed <= 180000000; elapsed += interval) {
			other.update(2000000 + elapsed, true, true, true, 10.f, config.voltage[20], 3.5f,
				     10.f * (elapsed * 1e-6f) / 3.6f);
		}

		result[k++] = other.soc();
		EXPECT_NEAR(other.coulombSoc(), 0.675f, 0.0005f);
		EXPECT_NEAR(other.coulombSoc() - other.soc(), 170.f * config.flight_correction_rate, 0.001f);
	}

	EXPECT_NEAR(result[0], result[1], 0.001f);
	EXPECT_NEAR(result[1], result[2], 0.001f);
}

TEST_F(BatterySocEstimatorTest, FlightCorrectionsReduceCapacityErrorWithVariableCruiseLoad)
{
	// Synthetic 20% capacity loss, representing ageing or lower usable capacity under load.
	// This tests adaptation, not accuracy of a particular battery's voltage model.
	sample(0.f, config.voltage[40]);
	float true_soc = 1.f;
	unsigned qualified_seconds = 0;

	for (unsigned second = 0; second < 480; ++second) {
		const float current = 18.f + (second % 5); // 18..22 A, with brief manoeuvres.
		const float flight_current = second % 60 < 3 ? 35.f : current;
		true_soc -= flight_current / (3.6f * 3200.f);
		const float ocv = 3.3f + 0.9f * true_soc * true_soc;
		sample(flight_current, ocv, true, true, 1000000, ocv - flight_current * 0.005f);
		qualified_seconds += estimator.correctionEligible();
	}

	EXPECT_GT(qualified_seconds, 350u);
	EXPECT_LT(true_soc, 0.2f);
	EXPECT_GT(estimator.coulombSoc() - true_soc, 0.15f);
	EXPECT_LT(fabsf(estimator.soc() - true_soc), fabsf(estimator.coulombSoc() - true_soc) - 0.05f);
	EXPECT_GT(estimator.soc(), true_soc); // Deliberately slow corrections need not fully catch up in one flight.
	EXPECT_TRUE(estimator.chargeHistoryValid());
}

TEST_F(BatterySocEstimatorTest, InvalidFlightCorrectionConfigurationCannotActivateProfile)
{
	for (float invalid : {NAN, -1.f, INFINITY}) {
		auto bad = config;
		bad.flight_voltage_margin = invalid;
		estimator.configure(bad);
		EXPECT_FALSE(estimator.profileValid());
		bad = config;
		bad.flight_power_stability = invalid;
		estimator.configure(bad);
		EXPECT_FALSE(estimator.profileValid());
		bad = config;
		bad.flight_settling_time = invalid;
		estimator.configure(bad);
		EXPECT_FALSE(estimator.profileValid());
		bad = config;
		bad.flight_correction_rate = invalid;
		estimator.configure(bad);
		EXPECT_FALSE(estimator.profileValid());
	}
}
