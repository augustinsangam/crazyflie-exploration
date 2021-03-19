#include <cerrno>
#include <cmath>
#include <cstring>

#include "exploration/state_machine.hpp"
#include "math_supp.hpp"
#include "porting/porting.hpp"

#define STATE_MACHINE_COMMANDER_PRI 3

static void take_off(exploration::setpoint_t *sp, float velocity) {
	sp->mode.x = exploration::modeVelocity;
	sp->mode.y = exploration::modeVelocity;
	sp->mode.z = exploration::modeVelocity;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->velocity.z = velocity;
	sp->mode.yaw = exploration::modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void land(exploration::setpoint_t *sp, float velocity) {
	sp->mode.x = exploration::modeVelocity;
	sp->mode.y = exploration::modeVelocity;
	sp->mode.z = exploration::modeVelocity;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->velocity.z = -velocity;
	sp->mode.yaw = exploration::modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void hover(exploration::setpoint_t *sp, float height) {
	sp->mode.x = exploration::modeVelocity;
	sp->mode.y = exploration::modeVelocity;
	sp->mode.z = exploration::modeAbs;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->position.z = height;
	sp->mode.yaw = exploration::modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void vel_command(exploration::setpoint_t *sp, float vel_x, float vel_y,
                        float yaw_rate, float height) {
	sp->mode.x = exploration::modeVelocity;
	sp->mode.y = exploration::modeVelocity;
	sp->mode.z = exploration::modeAbs;
	sp->velocity.x = vel_x;
	sp->velocity.y = vel_y;
	sp->position.z = height;
	sp->mode.yaw = exploration::modeVelocity;
	sp->attitudeRate.yaw = yaw_rate;
	sp->velocity_body = true;
}

static void shut_off_engines(exploration::setpoint_t *sp) {
	sp->mode.x = exploration::modeDisable;
	sp->mode.y = exploration::modeDisable;
	sp->mode.z = exploration::modeDisable;
	sp->mode.yaw = exploration::modeDisable;
}

static int32_t find_minimum(uint8_t a[], int32_t n) {
	int32_t c, min, index;

	min = a[0];
	index = 0;

	for (c = 1; c < n; c++) {
		if (a[c] < min) {
			index = c;
			min = a[c];
		}
	}

	return index;
}

void exploration::StateMachine::startup() {
	init_median_filter_f(&medFilt, 5);
	init_median_filter_f(&medFilt_2, 5);
	init_median_filter_f(&medFilt_3, 13);
	uint64_t address = porting::config_block_get_radio_address();
	my_id = (uint8_t)((address)&0x00000000ff);
	p_reply.port = 0x00;
	p_reply.data[0] = my_id;
	memcpy(&p_reply.data[1], &rssi_angle_, sizeof(float));
	p_reply.size = 5;

	porting::system_wait_start();
	porting::ticks_delay(porting::ms_to_ticks(3000));
}

void exploration::StateMachine::iteration_loop() {
	porting::ticks_delay(TICKS_PER_FSM_LOOP);
	// For every 1 second, reset the RSSI value to high if it hasn't been
	// received for a while
	for (uint8_t it = 0; it < 9; it++)
		if (porting::us_timestamp() >=
		    time_array_other_drones_[it] + 1000 * 1000) {
			time_array_other_drones_[it] =
			    porting::us_timestamp() + 1000 * 1000 + 1;
			rssi_array_other_drones_[it] = 150;
			rssi_angle_array_other_drones_[it] = 500.0f;
		}

	// get RSSI, id and angle of closests crazyflie.
	id_inter_closest_ = (uint8_t)find_minimum(rssi_array_other_drones_, 9);
	rssi_inter_closest_ = rssi_array_other_drones_[id_inter_closest_];
	rssi_angle_inter_closest_ =
	    rssi_angle_array_other_drones_[id_inter_closest_];

	rssi_inter_filtered_ =
	    (uint8_t)update_median_filter_f(&medFilt_2, (float)rssi_inter_closest_);

	// checking init of multiranger and flowdeck
	uint8_t multiranger_isinit = porting::get_deck_bc_multiranger();
	uint8_t flowdeck_isinit = porting::get_deck_bc_flow2();

	// get current height and heading
	height_ = porting::get_kalman_state_z();
	float heading_deg = porting::get_stabilizer_yaw();
	heading_rad_ = deg_to_rag(heading_deg);

	// t RSSI of beacon
	rssi_beacon_ = porting::get_radio_rssi();
	rssi_beacon_filtered_ =
	    (uint8_t)update_median_filter_f(&medFilt_3, (float)rssi_beacon_);

	// Select which laser range sensor readings to use
	if (multiranger_isinit) {
		front_range_ = porting::get_front_range() / 1000.0f;
		right_range_ = porting::get_right_range() / 1000.0f;
		left_range_ = porting::get_left_range() / 1000.0f;
		back_range_ = porting::get_back_range() / 1000.0f;
		up_range_ = porting::get_up_range() / 1000.0f;
	}

	// Get position estimate of kalman filter
	exploration::point_t pos;
	porting::estimator_kalman_get_estimated_pos(
	    &pos); // TODO : Position of the drone

	// Initialize setpoint
	memset(&setpoint_BG_, 0, sizeof(setpoint_BG_));

	// Filtere uprange, since it sometimes gives a low spike that
	up_range_filtered_ = update_median_filter_f(&medFilt, up_range_);
	if (up_range_filtered_ < 0.05f) {
		up_range_filtered_ = up_range_;
	}
	if (flowdeck_isinit && multiranger_isinit) {
		correctly_initialized_ = true;
	}

#if EXPLORATION_METHOD == 3
	uint8_t rssi_beacon_threshold = 41;
	if (keep_flying == true &&
	    (!correctly_initialized || up_range < 0.2f ||
	     (!outbound && rssi_beacon_filtered < rssi_beacon_threshold))) {
		keep_flying = 0;
	}
#else
	if (keep_flying_ == true && (!correctly_initialized_ || up_range_ < 0.2f)) {
		keep_flying_ = 0;
	}
#endif

	state_ = 0;

	// Main flying code
	if (keep_flying_) {
		if (taken_off_) {
			/*
			 * If the flight is given a OK
			 *  and the crazyflie has taken off
			 *   then perform state machine
			 */
			vel_w_cmd_ = 0;
			hover(&setpoint_BG_, nominal_height_);

#if EXPLORATION_METHOD == 1 // WALL_FOLLOWING
			// wall following state machine
			state_ = exploration_controller_.wall_follower(
			    &vel_x_cmd_, &vel_y_cmd_, &vel_w_cmd_, front_range_,
			    right_range_, heading_rad_, 1);
#endif
#if EXPLORATION_METHOD == 2 // WALL_FOLLOWER_AND_AVOID
			if (id_inter_closest > my_id) {
				rssi_inter_filtered = 140;
			}

			state = exploration_controller_.wall_follower_and_avoid_controller(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, left_range,
			    right_range, heading_rad, rssi_inter_filtered);
#endif
#if EXPLORATION_METHOD == 3 // SwWARM GRADIENT BUG ALGORITHM

			bool priority = false;
			if (id_inter_closest > my_id) {
				priority = true;
			} else {
				priority = false;
			}
			// TODO make outbound depended on battery.
			state = exploration_controller_.sgba_controller(
			    &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, &state_wf,
			    front_range, left_range, right_range, back_range, heading_rad,
			    (float)pos.x, (float)pos.y, rssi_beacon_filtered,
			    rssi_inter_filtered, rssi_angle_inter_closest, priority,
			    outbound);

			memcpy(&p_reply.data[1], &rssi_angle, sizeof(float));

#endif

			// convert yaw rate commands to degrees
			float vel_w_cmd_convert = rad_to_deg(vel_w_cmd_);

			vel_command(&setpoint_BG_, vel_x_cmd_, vel_y_cmd_,
			            vel_w_cmd_convert, nominal_height_);
			on_the_ground_ = false;
		} else {
			/*
			 * If the flight is given a OK
			 *  but the crazyflie  has not taken off
			 *   then take off
			 */
			if (porting::us_timestamp() >=
			    takeoffdelaytime + 1000 * 1000 * my_id) {

				take_off(&setpoint_BG_, nominal_height_);
				if (height_ > nominal_height_) {
					taken_off_ = true;

#if EXPLORATION_METHOD == 1 // wall following
					exploration_controller_.wall_follower_init(0.4, 0.5, 1);
#endif
#if EXPLORATION_METHOD == 2 // wallfollowing with avoid
					if (my_id % 2 == 1)
						exploration_controller_
						    .init_wall_follower_and_avoid_controller(0.4, 0.5,
						                                             -1);
					else
						exploration_controller_
						    .init_wall_follower_and_avoid_controller(0.4, 0.5,
						                                             1);

#endif
#if EXPLORATION_METHOD == 3 // Swarm Gradient Bug Algorithm
					if (my_id == 4 || my_id == 8) {
						exploration_controller_.init_sgba_controller(0.4, 0.5,
						                                             -0.8);
					} else if (my_id == 2 || my_id == 6) {
						exploration_controller_.init_sgba_controller(0.4, 0.5,
						                                             0.8);
					} else if (my_id == 3 || my_id == 7) {
						exploration_controller_.init_sgba_controller(0.4, 0.5,
						                                             -2.4);
					} else if (my_id == 5 || my_id == 9) {
						exploration_controller_.init_sgba_controller(0.4, 0.5,
						                                             2.4);
					} else {
						exploration_controller_.init_sgba_controller(0.4, 0.5,
						                                             0.8);
					}

#endif
				}
				on_the_ground_ = false;
			} else {
				shut_off_engines(&setpoint_BG_);
				taken_off_ = false;
			}
		}
	} else {
		if (taken_off_) {
			/*
			 * If the flight is given a not OK
			 *  but the Crazyflie  has already taken off
			 *   then land
			 */
			land(&setpoint_BG_, 0.2f);
			if (height_ < 0.1f) {
				shut_off_engines(&setpoint_BG_);
				taken_off_ = false;
			}
			on_the_ground_ = false;

		} else {

			/*
			 * If the flight is given a not OK
			 *  and Crazyflie has landed
			 *   then keep engines off
			 */
			shut_off_engines(&setpoint_BG_);
			takeoffdelaytime = porting::us_timestamp();
			on_the_ground_ = true;
		}
	}

#if EXPLORATION_METHOD != 1
	if (porting::us_timestamp() >= radioSendBroadcastTime + 1000 * 500) {
		exploration::send_p2p_packet_broadcast(&p_reply);
		radioSendBroadcastTime = porting::us_timestamp();
	}

#endif
	porting::commander_set_setpoint(&setpoint_BG_, STATE_MACHINE_COMMANDER_PRI);
}

void exploration::StateMachine::p2p_callback_handler(P2PPacket *p) {
	id_inter_ext_ = p->data[0];

	if (id_inter_ext_ == 0x63) {
		// rssi_beacon =rssi_inter;
		keep_flying_ = p->data[1];
	} else if (id_inter_ext_ == 0x64) {
		rssi_beacon_ = p->rssi;

	} else {
		rssi_inter_ = p->rssi;
		memcpy(&rssi_angle_inter_ext_, &p->data[1], sizeof(float));

		rssi_array_other_drones_[id_inter_ext_] = rssi_inter_;
		time_array_other_drones_[id_inter_ext_] = porting::us_timestamp();
		rssi_angle_array_other_drones_[id_inter_ext_] = rssi_angle_inter_ext_;
	}
}
