#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <cstdint>

#include "median_filter.hpp"
#include "p2p.hpp"
#include "sgba.hpp"
#include "stabilizer_types.hpp"
#include "wall_following.hpp"
#include "wall_following_with_avoid.hpp"

namespace exploration {

class StateMachine {
public:
	void startup();
	void iteration_loop();
	void p2p_callback_handler(P2PPacket *p);

private:
	bool keep_flying_ = false;
	float height_;
	bool taken_off_ = false;
	float nominal_height_ = 0.3;
	uint8_t rssi_inter_;
	uint8_t rssi_inter_filtered_;
	uint8_t rssi_inter_closest_;
	float rssi_angle_inter_ext_;
	float rssi_angle_inter_closest_;
	uint8_t rssi_beacon_;
	uint8_t rssi_beacon_filtered_;
	uint8_t id_inter_ext_;
	exploration::setpoint_t setpoint_BG_;
	float vel_x_cmd_, vel_y_cmd_, vel_w_cmd_;
	float heading_rad_;
	float right_range_;
	float front_range_;
	float left_range_;
	float up_range_;
	float back_range_;
	float rssi_angle_;
	int state_;
#if EXPLORATION_METHOD == 3
	int state_wf_;
#endif
	float up_range_filtered_;
	int varid_;
	bool on_the_ground_ = true;
	bool correctly_initialized_;
	uint8_t rssi_array_other_drones_[9] = {150, 150, 150, 150, 150};
	uint64_t time_array_other_drones_[9] = {0};
	float rssi_angle_array_other_drones_[9] = {500.0f};
	uint8_t id_inter_closest_ = 100;

	MedianFilterFloat medFilt;
	MedianFilterFloat medFilt_2;
	MedianFilterFloat medFilt_3;
	uint8_t my_id;
	P2PPacket p_reply;

#if EXPLORATION_METHOD != 1
	uint64_t radioSendBroadcastTime = 0;
#endif

	uint64_t takeoffdelaytime = 0;

#if EXPLORATION_METHOD == 3
	bool outbound = true;
#endif

#if EXPLORATION_METHOD == 1
	WallFollowing exploration_controller_;
#elif EXPLORATION_METHOD == 2
	WallFollowingWithAvoid exploration_controller_;
#elif EXPLORATION_METHOD == 3
	Sgba exploration_controller_;
#endif
};

} // namespace exploration
#endif /* STATE_MACHINE_HPP */
