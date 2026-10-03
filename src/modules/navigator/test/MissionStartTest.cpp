/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <memory>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/Publication.hpp>
#include "navigator.h"
#include "mission.h"

extern "C" int dataman_main(int argc, char *argv[]);

class MissionStartPeer : public Mission
{
public:
	explicit MissionStartPeer(Navigator *navigator) : Mission(navigator) {}

	void seed(int index)
	{
		_mission = {};
		_mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		_mission.mission_id = 1234;
		_mission.count = 3;
		_mission.current_seq = index;
		_mission.land_start_index = _mission.land_index = -1;
		_mission.timestamp = hrt_absolute_time();

		for (unsigned i = 0; i < _mission.count; ++i) {
			mission_item_s item{};
			item.nav_cmd = i == 0 ? NAV_CMD_TAKEOFF : (i == 1 ? NAV_CMD_DO_VTOL_TRANSITION : NAV_CMD_WAYPOINT);
			item.lat = 47.;
			item.lon = 8.;
			item.altitude = 430.48f;
			item.autocontinue = true;

			if (i == 1) { item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW; }

			ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, i,
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}

		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_MISSION_STATE, 0,
						      reinterpret_cast<uint8_t *>(&_mission), sizeof(_mission)));
		_mission_pub.publish(_mission);
		mission_s ignored{};
		_mission_sub.update(&ignored);
		_mission_checked = true;
		_is_current_planned_mission_item_valid = true;
		_navigator->get_mission_result()->valid = true;
	}

	void activate()
	{
		_vehicle_status_sub.update();
		_land_detected_sub.update();
		_global_pos_sub.update();
		MissionBase::on_activation();
	}

	void advanceToTransition() { setMissionIndex(1); }
	void exhaustLeadingJump()
	{
		mission_item_s item{};
		item.nav_cmd = NAV_CMD_DO_JUMP;
		item.do_jump_mission_index = 2;
		item.do_jump_repeat_count = item.do_jump_current_count = 1;
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0, 0,
						      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		_dataman_cache.invalidate();
	}
	int index() const { return _mission.current_seq; }
	int command() const { return _mission_item.nav_cmd; }
	bool climbing() const { return _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB; }
	int checkpoint()
	{
		mission_s saved{};
		EXPECT_TRUE(_dataman_client.readSync(DM_KEY_MISSION_STATE, 0,
						     reinterpret_cast<uint8_t *>(&saved), sizeof(saved)));
		return saved.current_seq;
	}
};

class MissionStartTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();
		char name[] = "dataman", start[] = "start", ram[] = "-r";
		char *argv[] = {name, start, ram};
		ASSERT_EQ(dataman_main(3, argv), 0);
	}

	static void TearDownTestSuite()
	{
		char name[] = "dataman", stop[] = "stop";
		char *argv[] = {name, stop};
		dataman_main(2, argv);
		px4::WorkQueueManagerStop();
	}

	void SetUp() override
	{
		_navigator = std::make_unique<Navigator>();
		mission = std::make_unique<MissionStartPeer>(_navigator.get());
		auto &home = *_navigator->get_home_position();
		home.lat = 47.; home.lon = 8.; home.alt = 400.f;
		home.valid_alt = home.valid_hpos = true;
		setVehicle(false, true);
		mission->seed(1);
		mission->on_inactive();
	}

	void setVehicle(bool armed, bool landed, float height = 0.f)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.is_vtol = true;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		*_navigator->get_vstatus() = status;
		_vehicle.publish(status);
		vehicle_land_detected_s land{};
		land.timestamp = status.timestamp;
		land.landed = landed;
		*_navigator->get_land_detected() = land;
		_land.publish(land);
		vehicle_global_position_s position{};
		position.timestamp = status.timestamp;
		position.lat = 47.; position.lon = 8.; position.alt = 400.f + height;
		*_navigator->get_global_position() = position;
		_position.publish(position);
	}

	std::unique_ptr<Navigator> _navigator;
	std::unique_ptr<MissionStartPeer> mission;
	uORB::Publication<vehicle_status_s> _vehicle{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _position{ORB_ID(vehicle_global_position)};
};

TEST_F(MissionStartTest, BootedCheckpointStartsAtTakeoffWithoutErasingCheckpoint)
{
	setVehicle(true, true);
	mission->activate();
	EXPECT_EQ(mission->index(), 0);
	EXPECT_TRUE(mission->climbing());
	EXPECT_EQ(mission->command(), NAV_CMD_TAKEOFF);
	EXPECT_EQ(mission->checkpoint(), 1);
}

TEST_F(MissionStartTest, DisarmRearmRestartsInterruptedMission)
{
	setVehicle(true, true);
	mission->activate();
	mission->advanceToTransition();
	mission->on_inactivation();
	setVehicle(false, true);
	mission->on_inactive();
	setVehicle(true, true);
	mission->activate();
	EXPECT_EQ(mission->index(), 0);
	EXPECT_EQ(mission->command(), NAV_CMD_TAKEOFF);
}

TEST_F(MissionStartTest, SameFlightPauseRetainsProgress)
{
	setVehicle(true, false, 30.48f);
	mission->activate();
	mission->advanceToTransition();
	mission->on_inactivation();
	mission->on_inactive();
	mission->activate();
	EXPECT_EQ(mission->index(), 1);
}

TEST_F(MissionStartTest, ExplicitAirborneResumeHonorsUnchangedCheckpointIndex)
{
	setVehicle(true, false, 30.48f);
	ASSERT_TRUE(mission->set_current_mission_index(1));
	mission->activate();
	EXPECT_EQ(mission->index(), 1);
}

TEST_F(MissionStartTest, ExplicitResumeBelowTargetClimbsBeforeTransition)
{
	setVehicle(true, false, 5.f);
	ASSERT_TRUE(mission->set_current_mission_index(1));
	mission->activate();
	EXPECT_EQ(mission->index(), 1);
	EXPECT_TRUE(mission->climbing());
	EXPECT_EQ(mission->command(), NAV_CMD_LOITER_TO_ALT);
}

TEST_F(MissionStartTest, GroundResumeRejectedEvenIfIndexUnchanged)
{
	EXPECT_FALSE(mission->set_current_mission_index(1));
	setVehicle(true, true);
	EXPECT_FALSE(mission->set_current_mission_index(1));
	mission->activate();
	EXPECT_EQ(mission->index(), 0);
}

TEST_F(MissionStartTest, ExplicitResumeDoesNotSurviveAnotherDisarm)
{
	setVehicle(true, false, 30.48f);
	ASSERT_TRUE(mission->set_current_mission_index(1));
	setVehicle(false, true);
	mission->on_inactive();
	setVehicle(true, true);
	mission->activate();
	EXPECT_EQ(mission->index(), 0);
}

TEST_F(MissionStartTest, InvalidIndexDoesNotAuthorizeResume)
{
	setVehicle(true, false, 30.48f);
	EXPECT_FALSE(mission->set_current_mission_index(3));
	mission->activate();
	EXPECT_EQ(mission->index(), 0);
}

TEST_F(MissionStartTest, ExplicitGroundStartAtZeroStillWorks)
{
	ASSERT_TRUE(mission->set_current_mission_index(0));
	setVehicle(true, true);
	mission->activate();
	EXPECT_EQ(mission->index(), 0);
	EXPECT_EQ(mission->command(), NAV_CMD_TAKEOFF);
}

TEST_F(MissionStartTest, ExplicitRestartResetsLoopsBeforeResolvingFirstItem)
{
	mission->exhaustLeadingJump();
	setVehicle(true, false, 30.48f);
	ASSERT_TRUE(mission->set_current_mission_index(0));
	EXPECT_EQ(mission->index(), 2);
}
