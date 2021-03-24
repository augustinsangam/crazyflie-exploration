#ifndef EXPLORATION_STATEMACHINE_HPP
#define EXPLORATION_STATEMACHINE_HPP

#include "WallFollowing.hpp"
#include "median_filter.hpp"
#include "types.hpp"
#include <array>
#include <cstdint>

#if EXPLORATION_METHOD == 1
#	include "exploration/WallFollowing.hpp"
#elif EXPLORATION_METHOD == 2
#	include "exploration/WallFollowingWithAvoid.hpp"
#elif EXPLORATION_METHOD == 3
#	include "exploration/SGBA.hpp"
#endif

namespace exploration {

class StateMachine {
public:
	void init();
	void step();
	void p2p_callback_handler(P2PPacket *p);
	StateMachine()
#if EXPLORATION_METHOD == 1
	    : exploration_controller_()
#endif
	{
	}

private:
	static constexpr float nominal_height = 0.3F;

	uint8_t my_id;
	struct MedianFilterFloat medFilt, medFilt_2, medFilt_3;
	float rssi_angle;
	float front_range, right_range, left_range, back_range, up_range;
	uint8_t rssi_beacon;
	uint64_t takeoffdelaytime = 0;
	std::array<std::uint64_t, 9> time_array_other_drones{0};
	std::array<std::uint8_t, 9> rssi_array_other_drones{150, 150, 150, 150, 150,
	                                                    150, 150, 150, 150};
	std::array<float, 9> rssi_angle_array_other_drones{500.0F};
	bool correctly_initialized = false;
	bool keep_flying = false, taken_off = false;
	int state = 0;

#if EXPLORATION_METHOD != 1
	uint64_t radioSendBroadcastTime = 0;
	P2PPacket p_reply;
#endif

#if EXPLORATION_METHOD == 1
	bool outbound = true;
#endif

#if EXPLORATION_METHOD == 1
	WallFollowing exploration_controller_;
#elif EXPLORATION_METHOD == 2
	WallFollowingWithAvoid exploration_controller_;
#elif EXPLORATION_METHOD == 3
	SGBA exploration_controller_;
	int state_wf_;
	static constexpr bool outbound_{true};
#endif
};

} // namespace exploration

#endif /* EXPLORATION_STATEMACHINE_HPP */
