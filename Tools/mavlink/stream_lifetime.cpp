// SPDX-License-Identifier: BSD-3-Clause
// Minimal dependencies for test_stream_lifetime.py's production method bodies.
#include <containers/List.hpp>
#include <containers/LockGuard.hpp>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <future>
#include <mutex>
#include <thread>

#define PX4_DEBUG(...) do {} while (0)
#define PX4_WARN(...) do {} while (0)
#define PX4_OK 0
#define PX4_ERROR -1
#define OK 0

static pthread_mutex_t mavlink_channel_send_mutexes[1];
static std::mutex gate_mutex;
static std::condition_variable gate_cv;
static bool entered = false;
static bool released = false;
static std::atomic<bool> block_request{false};
static std::atomic<bool> block_lookup{false};
static int reported_interval;

static void pause_reader()
{
	std::unique_lock<std::mutex> lock(gate_mutex);
	entered = true;
	gate_cv.notify_all();
	gate_cv.wait(lock, [] { return released; });
}

class MavlinkStream : public ListNode<MavlinkStream *>
{
public:
	virtual ~MavlinkStream() = default;
	virtual const char *get_name() { return "TEST"; }
	virtual uint16_t get_id()
	{
		if (block_lookup.exchange(false)) { pause_reader(); }

		return _id;
	}
	virtual bool request_message(float, float, float, float, float, float)
	{
		// Real stream sends re-enter this mutex through the MAVLink helpers.
		LockGuard send_guard{mavlink_channel_send_mutexes[0]};

		if (block_request.exchange(false)) { pause_reader(); }

		// ASan catches this access if configure_stream deleted us while paused.
		_interval++;
		return _id == 42;
	}
	void set_interval(int value) { _interval = value; }
	int get_interval() { return _interval; }
	unsigned get_size() { return 12; }
	bool const_rate() { return false; }
private:
	uint16_t _id{42};
	int _interval{1000000};
};

class Mavlink
{
public:
	int configure_stream(const char *, float);
	void display_status_streams();
	void lock_send() { pthread_mutex_lock(&mavlink_channel_send_mutexes[_instance_id]); }
	void unlock_send() { pthread_mutex_unlock(&mavlink_channel_send_mutexes[_instance_id]); }
	List<MavlinkStream *> &get_streams() { return _streams; }
	int get_channel() { return 0; }
	void configure_stream_threadsafe(const char *name, float rate)
	{
		// The real handoff waits for the sender. It must not hold its lock.
		auto sender = std::async(std::launch::async, [ = ] { return configure_stream(name, rate); });
		assert(sender.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
		assert(sender.get() == OK);
	}
private:
	int _instance_id{0};
	float _rate_mult{1.f};
	List<MavlinkStream *> _streams;
};

class MavlinkReceiver
{
public:
	explicit MavlinkReceiver(Mavlink &mavlink) : _mavlink(mavlink) {}
	uint8_t handle_request_message_command(uint16_t, float, float, float, float, float, float);
	void get_message_interval(int);
private:
	Mavlink &_mavlink;
};

struct vehicle_command_ack_s {
	static constexpr uint8_t VEHICLE_CMD_RESULT_ACCEPTED = 0;
	static constexpr uint8_t VEHICLE_CMD_RESULT_DENIED = 2;
};

static const char *get_stream_name(uint16_t id) { return id == 42 ? "TEST" : nullptr; }
static MavlinkStream *create_mavlink_stream(const char *, Mavlink *) { return new MavlinkStream(); }
static void mavlink_msg_message_interval_send(int, int, int interval) { reported_interval = interval; }

#include "stream_methods.inc"

int main()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&mavlink_channel_send_mutexes[0], &attr);
	pthread_mutexattr_destroy(&attr);

	Mavlink mavlink;
	MavlinkReceiver receiver(mavlink);

	// Stop at both the virtual lookup and the actual send, then try deletion.
	for (int scenario = 0; scenario < 3; ++scenario) {
		assert(mavlink.configure_stream("TEST", 1.f) == OK);
		entered = released = false;
		block_request = scenario == 0;
		block_lookup = scenario != 0;
		auto reader = std::async(std::launch::async, [&] {
			if (scenario == 2)
			{
				receiver.get_message_interval(42);

			} else
			{
				assert(receiver.handle_request_message_command(42, 0, 0, 0, 0, 0, 0) == 0);
			}
		});
		{
			std::unique_lock<std::mutex> lock(gate_mutex);
			assert(gate_cv.wait_for(lock, std::chrono::seconds(2), [] { return entered; }));
		}
		std::promise<void> deleting;
		auto writer = std::async(std::launch::async, [&] {
			deleting.set_value();
			return mavlink.configure_stream("TEST", 0.f);
		});
		deleting.get_future().wait();
		const bool deleted_while_in_use = writer.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready;
		{
			std::lock_guard<std::mutex> lock(gate_mutex);
			released = true;
		}
		gate_cv.notify_all();
		reader.get();
		assert(writer.get() == OK);
		assert(!deleted_while_in_use);
		assert(mavlink.get_streams().empty());
	}

	// Missing-stream handoff and recursive send locking must still work.
	assert(receiver.handle_request_message_command(42, 0, 0, 0, 0, 0, 0) == 0);
	receiver.get_message_interval(42);
	assert(reported_interval == 1);
	mavlink.display_status_streams();
	assert(mavlink.configure_stream("TEST", 0.f) == OK);
	assert(receiver.handle_request_message_command(99, 0, 0, 0, 0, 0, 0) == 2);
	pthread_mutex_destroy(&mavlink_channel_send_mutexes[0]);
	puts("MAVLink stream lifetime: request, lookup, interval and sender handoff passed");
}
