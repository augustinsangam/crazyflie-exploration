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

	porting_->system_wait_start();
	porting_->delay_ms(EXPLORATION_DRONE_INITIALISATION_DELAY);
}

void StateMachine::set_state(DroneState state) {
	state_ = state;
	porting_->debug_print("New state: %d\n", static_cast<int>(state) + 1);
}

void StateMachine::step() {
	// Handle transitions

	auto height = porting_->kalman_state_z();
	auto batteryLevel = porting_->get_battery_level();

	switch (state_) {
	case DroneState::onTheGround:
	case DroneState::crashed: {
		// Nothing to do. Can't get out of this state
		break;
	}
	case DroneState::takingOff:
		if (batteryLevel < 0.3F) {
			set_state(DroneState::landing);
		} else if (height > nominal_height) {
			set_state(DroneState::standBy);
		}
		break;
	case DroneState::landing: {
		if (height < 0.1F) {
			set_state(DroneState::onTheGround);
		}
		break;
	}
	case DroneState::exploring: {
		if (batteryLevel < 0.3F) {
			set_state(DroneState::landing);
		} else if (batteryLevel < 0.6F) {
			set_state(DroneState::returningToBase);
		}
		bool droneCrashed = porting_->range_front() < 0.2F && height < 0.1F;
		if (droneCrashed) {
			// set_state(DroneState::crashed);
		}
		bool obstacleOnTop = porting_->range_up() < 0.2F;
		if (obstacleOnTop) {
			set_state(DroneState::landing);
		}
		break;
	}
	case DroneState::standBy:
	case DroneState::returningToBase: {
		if (batteryLevel < 0.3F) {
			set_state(DroneState::landing);
		}
		auto droneCrashed = porting_->range_front() < 0.2F && height < 0.1F;
		if (droneCrashed) {
			// set_state(DroneState::crashed);
		}
		auto obstacleOnTop = porting_->range_up() < 0.2F;
		if (obstacleOnTop) {
			set_state(DroneState::landing);
		}
		break;
	}
	}

	// Handle states
	setpoint_t setpoint_BG;
	memset(&setpoint_BG, 0, sizeof setpoint_BG);
	switch (state_) {
	case DroneState::onTheGround:
		step_on_the_ground(&setpoint_BG);
		break;
	case DroneState::takingOff:
		step_taking_off(&setpoint_BG);
		break;
	case DroneState::landing:
		step_landing(&setpoint_BG);
		break;
	case DroneState::crashed:
		step_crashed(&setpoint_BG);
		break;
	case DroneState::exploring:
		keep_flying = true;
		step_exploring(&setpoint_BG);
		break;
	case DroneState::standBy:
		step_standby(&setpoint_BG);
		break;
	case DroneState::returningToBase:
		step_returning_to_base(&setpoint_BG);
		break;
	}
	porting_->commander_set_point(&setpoint_BG, STATE_MACHINE_COMMANDER_PRI);
}

void StateMachine::step_on_the_ground(setpoint_t *sp) { SetPoint::shut_off_engines(sp); }

void StateMachine::step_taking_off(setpoint_t *sp) {
	auto height = porting_->kalman_state_z();
	if (height < nominal_height) {
		SetPoint::take_off(sp, 0.5F);
	}
}

void StateMachine::step_landing(setpoint_t *sp) {
	auto height = porting_->kalman_state_z();
	if (height > 0.1F) {
		SetPoint::land(sp, 0.2F);
	}
}

void StateMachine::step_crashed(setpoint_t *sp) {
	// TODO ()
	SetPoint::shut_off_engines(sp);
}

void StateMachine::step_standby(setpoint_t *sp) { SetPoint::hover(sp, 0.3F); }

void StateMachine::step_returning_to_base(setpoint_t *sp) {
	// TODO ()
}

void StateMachine::step_exploring(setpoint_t *setpoint_BG) {
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

	// checking init of multiranger and flowdeck
	uint8_t multiranger_isinit = porting_->deck_bc_multiranger();
	uint8_t flowdeck_isinit = porting_->deck_bc_flow2();

	// get current height and heading
	auto height = porting_->kalman_state_z();
	float heading_deg = porting_->stabilizer_yaw();
	auto heading_rad = deg_to_rad(heading_deg);

#if EXPLORATION_METHOD == 3
	rssi_beacon = porting_->radio_rssi();
	auto rssi_beacon_filtered = static_cast<uint8_t>(
	    update_median_filter_f(&medFilt_3, static_cast<float>(rssi_beacon)));
#endif

	// Select which laser range sensor readings to use
	if (multiranger_isinit != 0) {
		front_range = porting_->range_front();
		right_range = porting_->range_right();
		left_range = porting_->range_left();
		back_range = porting_->range_back();
		up_range = porting_->range_up();
	}

	// Get position estimate of kalman filter
	point_t pos;
	porting_->kalman_estimated_pos(&pos);

	// Filtere uprange, since it sometimes gives a low spike that
	auto up_range_filtered = update_median_filter_f(&medFilt, up_range);
	if (up_range_filtered < 0.05F) {
		up_range_filtered = up_range;
	}

	if (flowdeck_isinit != 0 && multiranger_isinit != 0) {
		correctly_initialized = true;
	}

#if EXPLORATION_METHOD == 3
	uint8_t rssi_beacon_threshold = 41;
	if (keep_flying &&
	    (!correctly_initialized || up_range < 0.2F ||
	     (!outbound_ && rssi_beacon_filtered < rssi_beacon_threshold))) {
		keep_flying = false;
	}
#else
	if (keep_flying && (!correctly_initialized || up_range < 0.2F)) {
		keep_flying = false;
	}
#endif

	exploration_state = 0;

	// Main flying code
	if (keep_flying) {
		if (taken_off) {
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
			exploration_state = exploration_controller_.controller(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, right_range,
			    heading_rad, 1);
#elif EXPLORATION_METHOD == 2
			if (id_inter_closest > my_id) {
				rssi_inter_filtered = 140;
			}

			exploration_state = exploration_controller_.controller(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, left_range,
			    right_range, heading_rad, rssi_inter_filtered);
#elif EXPLORATION_METHOD == 3
			bool priority = false;
			priority = id_inter_closest > my_id;
			exploration_state = exploration_controller_.controller(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, &state_wf_,
			    front_range, left_range, right_range, back_range, heading_rad,
			    pos.x, pos.y, rssi_beacon_filtered, rssi_inter_filtered,
			    rssi_angle_inter_closest, priority, outbound_);

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
			SetPoint::vel_command(setpoint_BG, vel_x_cmd, vel_y_cmd,
			                      vel_w_cmd_convert, nominal_height);
			// on_the_ground = false;
		} else {
			/*
			 * If the flight is given a OK
			 *  but the crazyflie  has not taken off
			 *   then take off
			 */
			if (porting::timestamp_us() >=
			    takeoffdelaytime + 1000 * 1000 * my_id) {

				SetPoint::take_off(setpoint_BG, nominal_height);
				if (height > nominal_height) {
					taken_off = true;

#if EXPLORATION_METHOD == 1 // wall following
					// exploration_controller_.init(0.4F, 0.5F, 1);
					exploration_controller_.init(0.4F, 0.5, 1);
#elif EXPLORATION_METHOD == 2 // wallfollowing with avoid
					if (my_id % 2 == 1) {
						exploration_controller_.init(0.4F, 0.5, -1);
					} else {
						exploration_controller_.init(0.4F, 0.5, 1);
					}
#elif EXPLORATION_METHOD == 3 // Swarm Gradient Bug Algorithm
					if (my_id == 4 || my_id == 8) {
						exploration_controller_.init(0.4F, 0.5, -0.8F);
					} else if (my_id == 2 || my_id == 6) {
						exploration_controller_.init(0.4F, 0.5, 0.8F);
					} else if (my_id == 3 || my_id == 7) {
						exploration_controller_.init(0.4F, 0.5, -2.4F);
					} else if (my_id == 5 || my_id == 9) {
						exploration_controller_.init(0.4F, 0.5, 2.4F);
					} else {
						exploration_controller_.init(0.4F, 0.5, 0.8F);
					}
#endif
				}
				// on_the_ground = false;
			} else {
				SetPoint::shut_off_engines(setpoint_BG);
				taken_off = false;
			}
		}
	} else {
		if (taken_off) {
			/*
			 * If the flight is given a not OK
			 *  but the crazyflie  has already taken off
			 *   then land
			 */
			SetPoint::land(setpoint_BG, 0.2F);
			if (height < 0.1F) {
				SetPoint::shut_off_engines(setpoint_BG);
				taken_off = false;
			}
			// on_the_ground = false;

		} else {

			/*
			 * If the flight is given a not OK
			 *  and crazyflie has landed
			 *   then keep engines off
			 */
			SetPoint::shut_off_engines(setpoint_BG);
			takeoffdelaytime = porting::timestamp_us();
			// on_the_ground = true;
		}
	}

#if EXPLORATION_METHOD != 1
	if (porting::timestamp_us() >= radioSendBroadcastTime + 1000 * 500) {
		porting_->radiolink_broadcast_packet(&p_reply);
		radioSendBroadcastTime = porting::timestamp_us();
	}

#endif
}

void StateMachine::take_off_robot() { state_ = DroneState::takingOff; }

void StateMachine::land_robot() { state_ = DroneState::landing; }

void StateMachine::start_mission() { state_ = DroneState::exploring; }

void StateMachine::end_mission() { state_ = DroneState::landing; }

void StateMachine::return_to_base() { state_ = DroneState::returningToBase; }

void StateMachine::p2p_callback_handler(P2PPacket *p) {
	auto id_inter_ext = p->data[0]; // NOLINT

	if (id_inter_ext == 0x64) {
		rssi_beacon = p->rssi;
	} else {
		auto rssi_inter = p->rssi;
		float rssi_angle_inter_ext;
		std::memcpy(&rssi_angle_inter_ext, &p->data[1], // NOLINT
		            sizeof p->data[1]);                 // NOLINT

		rssi_array_other_drones.at(id_inter_ext) = rssi_inter;
		time_array_other_drones.at(id_inter_ext) = porting::timestamp_us();
		rssi_angle_array_other_drones.at(id_inter_ext) = rssi_angle_inter_ext;
	}
}

} // namespace exploration
