#include "exploration/SGBA.hpp"
#include "math_supp.hpp"
#include "porting.hpp"
#include <cstdio>

static int transition(int new_state, float *state_start_time) {
	*state_start_time =
	    static_cast<float>(static_cast<double>(porting::timestamp_us()) / 1e6);

	return new_state;
}

// Static helper functions
static bool logicIsCloseTo(float real_value, float checked_value,
                           float margin) {
	return real_value > checked_value - margin &&
	       real_value < checked_value + margin;
}

// Command functions
static void commandTurn(float *vel_w, float max_rate) { *vel_w = max_rate; }

void exploration::SGBA::init(float new_ref_distance_from_wall,
                             float max_speed_ref, float begin_wanted_heading,
                             float origin_x, float origin_y) {
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
	wanted_angle = begin_wanted_heading;
	first_run = true;

	origin_x_ = origin_x;
	origin_y_ = origin_y;
}

void exploration::SGBA::controller(float *vel_x, float *vel_y, float *vel_w,
                                  float *rssi_angle, float front_range,
                                  float left_range, float right_range,
                                  float back_range, float current_heading,
                                  float current_pos_x, float current_pos_y,
                                  uint8_t rssi_inter, float rssi_angle_inter,
                                  bool priority, bool outbound) {

	// Initalize static variables
	static int state = 2;
	// static float previous_heading = 0;
	static int state_wf = 0;
	static float wanted_angle_dir = 0;
	static float pos_x_hit = 0;
	static float pos_y_hit = 0;
	static float pos_x_sample = 0;
	static float pos_y_sample = 0;
	// static float pos_x_move = 0;
	// static float pos_y_move = 0;
	static bool overwrite_and_reverse_direction = false;
	static float direction = 1;
	static bool cannot_go_to_goal = false;
	static uint8_t prev_rssi = 150;
	static int diff_rssi = 0;
	static bool rssi_sample_reset = false;
	static uint8_t correct_heading_array[8] = {0};

	static bool first_time_inbound = true;
	static float wanted_angle_hit = 0;

	// if it is reinitialized
	if (first_run) {

		wanted_angle_dir = wrap_to_pi(
		    current_heading -
		    wanted_angle); // to determine the direction when turning to goal

		overwrite_and_reverse_direction = false;
		state = 2;

		state_start_time = static_cast<float>(
		    static_cast<double>(porting::timestamp_us()) / 1e6);
		first_run = false;
	}

	if (first_time_inbound && !outbound) {
		wanted_angle_dir = wrap_to_pi(current_heading - wanted_angle);
		state = transition(2, &state_start_time);
		first_time_inbound = false;
	}

	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = rotate_to_goal
	// 3 = wall_following
	// 4 = move out of way

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) { // FORWARD
		if (front_range < ref_distance_from_wall + 0.2F) {

			// if looping is detected, reverse direction (only on outbound)
			if (overwrite_and_reverse_direction) {
				direction = -1.0F * direction;
				overwrite_and_reverse_direction = false;
			} else {
				if (left_range < right_range && left_range < 2.0F) {
					direction = -1.0F;
				} else if (left_range > right_range && right_range < 2.0F) {
					direction = 1.0F;

				} else if (left_range > 2.0F && right_range > 2.0F) {
					direction = 1.0F;
				} else {
				}
			}

			pos_x_hit = current_pos_x;
			pos_y_hit = current_pos_y;
			wanted_angle_hit = wanted_angle;

			wf_.init(0.4F, 0.5, 3);

			for (int it = 0; it < 8; it++) {
				correct_heading_array[it] = 0;
			}

			state = transition(3, &state_start_time); // wall_following
		}
	} else if (state == 2) { // ROTATE_TO_GOAL
		// check if heading is close to the preferred_angle
		bool goal_check =
		    logicIsCloseTo(wrap_to_pi(current_heading - wanted_angle), 0, 0.1F);
		if (front_range < ref_distance_from_wall + 0.2F) {
			cannot_go_to_goal = true;
			wf_.init(0.4F, 0.5, 3);

			state = transition(3, &state_start_time); // wall_following
		}
		if (goal_check) {
			state = transition(1, &state_start_time); // forward
		}
	} else if (state == 3) { // WALL_FOLLOWING
		// if another drone is close and there is no right of way, move out of
		// the way
		if (priority == false && rssi_inter < rssi_threshold) {
			if (outbound) {
				if ((rssi_angle_inter < 0 && wanted_angle < 0) ||
				    (rssi_angle_inter > 0 && wanted_angle > 0)) {
					wanted_angle = -1 * wanted_angle;
					wanted_angle_dir =
					    wrap_to_pi(current_heading - wanted_angle);
					// state= transition(2);
				}
			}
			if (rssi_inter < rssi_collision_threshold) {
				state = transition(4, &state_start_time);
			}
		}

		// If going forward with wall following and cannot_go_to_goal bool is
		// still on
		//    turn it off!
		if (state_wf == 5 && cannot_go_to_goal) {
			cannot_go_to_goal = false;
		}

		// Check if the goal is reachable from the current point of view of the
		// agent
		float bearing_to_goal = wrap_to_pi(wanted_angle - current_heading);
		bool goal_check_WF = false;
		if (direction == -1) {
			goal_check_WF = (bearing_to_goal < 0 && bearing_to_goal > -1.5F);
		} else {
			goal_check_WF = (bearing_to_goal > 0 && bearing_to_goal < 1.5F);
		}

		// Check if bug went into a looping while wall following,
		//    if so, then forse the reverse direction predical.
		float rel_x_loop = current_pos_x - pos_x_hit;
		float rel_y_loop = current_pos_y - pos_y_hit;
		float loop_angle = wrap_to_pi(std::atan2(rel_y_loop, rel_x_loop));

		if (std::abs(wrap_to_pi(wanted_angle_hit + pi<float> - loop_angle)) <
		    1) {
			overwrite_and_reverse_direction = true;
		}

		// if during wallfollowing, agent goes around wall, and heading is close
		// to rssi _angle
		//      got to rotate to goal
		if ((state_wf == 6 || state_wf == 8) && goal_check_WF &&
		    front_range > ref_distance_from_wall + 0.4F && !cannot_go_to_goal) {
			wanted_angle_dir = wrap_to_pi(
			    current_heading - wanted_angle); // to determine the direction
			                                     // when turning to goal
			state = transition(2, &state_start_time); // rotate_to_goal
		}

		// If going straight
		//    determine by the gradient of the crazyradio what the approx
		//    direction is.
		if (state_wf == 5) {
			if (!outbound) {
				// Reset sample gathering
				if (rssi_sample_reset) {
					rssi_sample_reset = false;
					pos_x_sample = current_pos_x;
					pos_y_sample = current_pos_y;
				}

				// if the crazyflie traveled for 1 meter, than measure if it
				// went into the right path
				float rel_x_sample = current_pos_x - pos_x_sample;
				float rel_y_sample = current_pos_y - pos_y_sample;
				auto distance = std::sqrt(rel_x_sample * rel_x_sample +
				                          rel_y_sample * rel_y_sample);
				if (distance > 1.0F) {
					porting_->debug_print("CALCULATING NEW ANGLE\n");
					wanted_angle = wrap_to_pi(std::atan2(
					    origin_y_ - current_pos_y, origin_x_ - current_pos_x));
				}
			}
		} else {
			rssi_sample_reset = true;
		}
	} else if (state == 4) { // MOVE_OUT_OF_WAY
		// once the drone has gone by, rotate to goal
		if (rssi_inter >= rssi_collision_threshold) {
			state = transition(2, &state_start_time); // rotate_to_goal
		}
	}

	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x = 0;
	float temp_vel_y = 0;
	float temp_vel_w = 0;

	if (state == 1) { // FORWARD
		// stop moving if there is another drone in the way
		// forward max speed
		if (left_range < ref_distance_from_wall) {
			temp_vel_y = -0.2F;
		}
		if (right_range < ref_distance_from_wall) {
			temp_vel_y = 0.2F;
		}
		temp_vel_x = 0.5;
		//}

	} else if (state == 2) { // ROTATE_TO_GOAL
		// rotate to goal, determined on the sign
		if (wanted_angle_dir < 0) {
			commandTurn(&temp_vel_w, 0.5);
		} else {
			commandTurn(&temp_vel_w, -0.5);
		}

	} else if (state == 3) { // WALL_FOLLOWING
		// Get the values from the wallfollowing
		if (direction == -1) {
			state_wf = wf_.controller(&temp_vel_x, &temp_vel_y, &temp_vel_w,
			                          front_range, left_range, current_heading,
			                          direction);
		} else {
			state_wf = wf_.controller(&temp_vel_x, &temp_vel_y, &temp_vel_w,
			                          front_range, right_range, current_heading,
			                          direction);
		}
	} else if (state == 4) { // MOVE_AWAY

		float save_distance = 0.7F;
		if (left_range < save_distance) {
			temp_vel_y = temp_vel_y - 0.5F;
		}
		if (right_range < save_distance) {
			temp_vel_y = temp_vel_y + 0.5F;
		}
		if (front_range < save_distance) {
			temp_vel_x = temp_vel_x - 0.5F;
		}
		if (back_range < save_distance) {
			temp_vel_x = temp_vel_x + 0.5F;
		}
	}

	*rssi_angle = wanted_angle;

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;
}
