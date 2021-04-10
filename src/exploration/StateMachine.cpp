#include "exploration/StateMachine.hpp"
#include "exploration/DroneState.hpp"
#include "exploration/SetPoint.hpp"
#include "math_supp.hpp"
#include "porting.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>

#define STATE_MACHINE_COMMANDER_PRI 3

namespace exploration {

void StateMachine::init() {
	explorer_.init();
	init_median_filter_f(&medFilt, 5);
	init_median_filter_f(&medFilt_2, 5);
	init_median_filter_f(&medFilt_3, 13);
	auto address = porting_->config_block_radio_address();
	my_id = static_cast<uint8_t>(address & 0x00000000FFU);

#if EXPLORATION_METHOD != 1
	p_reply.port = 0x00;                                           // NOLINT
	p_reply.data[0] = my_id;                                       // NOLINT
	std::memcpy(&p_reply.data[1], &rssi_angle, sizeof rssi_angle); // NOLINT
	p_reply.size = 5;
#endif

#if EXPLORATION_METHOD == 1 // wall following
	exploration_controller_.init(0.4F, 0.5, 1);
#elif EXPLORATION_METHOD == 2 // wallfollowing with avoid
	if (my_id % 2 == 1) {
		exploration_controller_.init(0.4F, 0.5, -1);
	} else {
		exploration_controller_.init(0.4F, 0.5, 1);
	}
#elif EXPLORATION_METHOD == 3 // Swarm Gradient Bug Algorithm
	porting_->kalman_estimated_pos(&origin_);
	if (my_id == 4 || my_id == 8) {
		exploration_controller_.init(0.35F, 0.5, -0.8F, origin_.x, origin_.y);
	} else if (my_id == 2 || my_id == 6) {
		exploration_controller_.init(0.35F, 0.5, 0.8F, origin_.x, origin_.y);
	} else if (my_id == 3 || my_id == 7) {
		exploration_controller_.init(0.35F, 0.5, -2.4F, origin_.x, origin_.y);
	} else if (my_id == 5 || my_id == 9) {
		exploration_controller_.init(0.35F, 0.5, 2.4F, origin_.x, origin_.y);
	} else {
		exploration_controller_.init(0.35F, 0.5, 0.8F, origin_.x, origin_.y);
	}
#endif

	porting_->system_wait_start();
	porting_->delay_ms(EXPLORATION_DRONE_INITIALISATION_DELAY);
}

void StateMachine::set_state(DroneState state) {
	if (state == state_) {
		return;
	}

	state_ = state;
	porting_->debug_print("New state: %s\n", drone_state_to_name(state));
}

void StateMachine::step() {
	// Handle transitions
	auto next_state = state_;

	auto height = porting_->kalman_state_z();
	auto batteryLevel = porting_->get_battery_level();

	/* Stay in state crashed if we crashed earlier */
	auto droneCrashed =
	    next_state == DroneState::crashed || porting_->kalman_crashed();

	// checking init of multiranger and flowdeck
	auto sensors_work =
	    porting_->deck_bc_multiranger() != 0 && porting_->deck_bc_flow2() != 0;

	/* Detect above obstacles only when not on the ground*/
	auto obstacleOnTop =
	    next_state != DroneState::onTheGround && porting_->range_up() < 0.2F;

	/* Battery too low -> Emergency stop */
	auto batteryTooLow =
	    next_state != DroneState::onTheGround && batteryLevel < 0.3F;

	if (droneCrashed || !sensors_work) {
		next_state = DroneState::crashed;
	} else if (obstacleOnTop || batteryTooLow) {
		next_state = DroneState::landing;
	}

	switch (next_state) {
	case DroneState::onTheGround:
		break;
	case DroneState::crashed:
		// Nothing to do. Can't get out of this state
		porting_->debug_print("Crashed!\n");
		break;
	case DroneState::takingOff:
		if (height > nominal_height) {
			next_state = should_start_mission_ ? DroneState::exploring
			                                   : DroneState::standBy;
			should_start_mission_ = false;
		}
		break;
	case DroneState::landing:
		if (height < 0.1F) {
			next_state = DroneState::onTheGround;
		}
		break;
	case DroneState::exploring:
		/* Drone should stop exploring when battery below 60% to make sur o have
		 * enough power to come back */
		if (batteryLevel < 0.6F) {
			next_state = DroneState::returningToBase;
		}
		break;
	case DroneState::standBy:
		break;
	case DroneState::returningToBase: {
		// Landing if drone is enough close to starting point
		point_t pos;
		porting_->kalman_estimated_pos(&pos);
		auto dx = pos.x - origin_.x;
		auto dy = pos.y - origin_.y;
		auto distance_squared = dx * dx + dy * dy;
		constexpr auto distance = 0.5F;
		if (distance_squared < distance * distance) {
			next_state = DroneState::landing;
		}
		break;
	}
	}

	set_state(next_state);

	// Handle states
	setpoint_t sp{};
	switch (state_) {
	case DroneState::onTheGround:
		SetPoint::shut_off_engines(&sp);
		break;
	case DroneState::takingOff:
		SetPoint::take_off(&sp, 0.5F);
		break;
	case DroneState::landing:
		SetPoint::land(&sp, 0.2F);
		break;
	case DroneState::crashed:
		SetPoint::shut_off_engines(&sp);
		break;
	case DroneState::exploring:
		step_exploring(&sp, true);
		break;
	case DroneState::standBy:
		SetPoint::hover(&sp, 0.3F);
		break;
	case DroneState::returningToBase:
		step_exploring(&sp, false);
		break;
	}

	porting_->commander_set_point(&sp, STATE_MACHINE_COMMANDER_PRI);
}

void StateMachine::step_exploring(setpoint_t *setpoint_BG, bool outbound) {
	// For every 1 second, reset the RSSI value to high if it hasn't been
	// received for a while
	for (uint8_t it = 0; it < 9; it++) {
		if (porting::timestamp_us() >=
		    time_array_other_drones.at(it) + 1000 * 1000) {
			time_array_other_drones.at(it) =
			    porting::timestamp_us() + 1000 * 1000 + 1;
			rssi_array_other_drones.at(it) = 150;
			rssi_angle_array_other_drones.at(it) = 500.0F;
		}
	}

	// get RSSI, id and angle of closests crazyflie.
	auto *begin = rssi_array_other_drones.begin();
	auto id_inter_closest = static_cast<std::size_t>(
	    std::min_element(begin, rssi_array_other_drones.end()) - begin);
	auto rssi_inter_closest = rssi_array_other_drones.at(id_inter_closest);

#if EXPLORATION_METHOD == 3
	auto rssi_angle_inter_closest =
	    rssi_angle_array_other_drones.at(id_inter_closest);
#endif

	auto rssi_inter_filtered = static_cast<uint8_t>(update_median_filter_f(
	    &medFilt_2, static_cast<float>(rssi_inter_closest)));

	// get current height and heading
	// auto height = porting_->kalman_state_z();
	float heading_deg = porting_->stabilizer_yaw();
	auto heading_rad = deg_to_rad(heading_deg);

	// Select which laser range sensor readings to use
	front_range = porting_->range_front();
	right_range = porting_->range_right();
	left_range = porting_->range_left();
	back_range = porting_->range_back();
	up_range = porting_->range_up();

	// Get position estimate of kalman filter
	point_t pos;
	porting_->kalman_estimated_pos(&pos);

	// Filtere uprange, since it sometimes gives a low spike that
	auto up_range_filtered = update_median_filter_f(&medFilt, up_range);
	if (up_range_filtered < 0.05F) {
		up_range_filtered = up_range;
	}

	/*
	 * If the flight is given a OK
	 *  and the crazyflie has taken off
	 *   then perform exploration_state machine
	 */
	float vel_w_cmd = 0;
	float vel_x_cmd;
	float vel_y_cmd;

	SetPoint::hover(setpoint_BG, nominal_height);

#if EXPLORATION_METHOD == 1
	exploration_controller_.controller(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd,
	                                   front_range, right_range, heading_rad,
	                                   1);
#elif EXPLORATION_METHOD == 2
	if (id_inter_closest > my_id) {
		rssi_inter_filtered = 140;
	}

	exploration_controller_.controller(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd,
	                                   front_range, left_range, right_range,
	                                   heading_rad, rssi_inter_filtered);
#elif EXPLORATION_METHOD == 3
	bool priority = false;
	priority = id_inter_closest > my_id;
	exploration_controller_.controller(
	    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, front_range,
	    left_range, right_range, back_range, heading_rad, pos.x, pos.y,
	    rssi_inter_filtered, rssi_angle_inter_closest, priority, outbound);

	std::memcpy(&p_reply.data[1], &rssi_angle, // NOLINT
	            sizeof rssi_angle);
#endif

	// convert yaw rate commands to degrees
	float vel_w_cmd_convert = rad_to_deg(vel_w_cmd);

	// Convert relative commands to world commands (not necessary
	// anymore)
	/*float psi = heading_rad;
	float vel_x_cmd_convert =  cosf(-psi) * vel_x_cmd + sinf(-psi) *
	vel_y_cmd; float vel_y_cmd_convert = -sinf(-psi) * vel_x_cmd +
	cosf(-psi) * vel_y_cmd;*/
	// float vel_y_cmd_convert = -1 * vel_y_cmd;
	SetPoint::vel_command(setpoint_BG, vel_x_cmd, vel_y_cmd, vel_w_cmd_convert,
	                      nominal_height);
	// on_the_ground = false;

#if EXPLORATION_METHOD != 1
	if (porting::timestamp_us() >= radioSendBroadcastTime + 1000 * 500) {
		porting_->radiolink_broadcast_packet(&p_reply);
		radioSendBroadcastTime = porting::timestamp_us();
	}

#endif
}

void StateMachine::take_off_robot() { state_ = DroneState::takingOff; }

void StateMachine::land_robot() { state_ = DroneState::landing; }

void StateMachine::start_mission() {
	takeoffdelaytime = porting::timestamp_us();
	should_start_mission_ = true;
	state_ = DroneState::takingOff;
}

void StateMachine::end_mission() { state_ = DroneState::landing; }

void StateMachine::return_to_base() { state_ = DroneState::returningToBase; }

void StateMachine::p2p_callback_handler(P2PPacket *p) {
	auto id_inter_ext = p->data[0]; // NOLINT

	auto rssi_inter = p->rssi;
	float rssi_angle_inter_ext;
	std::memcpy(&rssi_angle_inter_ext, &p->data[1], // NOLINT
	            sizeof p->data[1]);                 // NOLINT

	rssi_array_other_drones.at(id_inter_ext) = rssi_inter;
	time_array_other_drones.at(id_inter_ext) = porting::timestamp_us();
	rssi_angle_array_other_drones.at(id_inter_ext) = rssi_angle_inter_ext;
}

} // namespace exploration
