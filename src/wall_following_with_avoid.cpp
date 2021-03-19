#include <cmath>

#include "exploration/wall_following_with_avoid.hpp"
#include "porting/porting.hpp"

int exploration::WallFollowingWithAvoid::transition(int new_state) {
	float t = porting::us_timestamp() / 1e6;
	state_start_time_ = t;
	return new_state;
}

// statemachine functions
void exploration::WallFollowingWithAvoid::
    init_wall_follower_and_avoid_controller(float new_ref_distance_from_wall,
                                            float max_speed_ref,
                                            float starting_local_direction) {
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed_ = max_speed_ref;
	local_direction_ = starting_local_direction;
	first_run_ = true;
}

int exploration::WallFollowingWithAvoid::wall_follower_and_avoid_controller(
    float *vel_x, float *vel_y, float *vel_w, float front_range,
    float left_range, float right_range, float current_heading,
    uint8_t rssi_other_drone) {

	// Initalize static variables
	static int state = 1;
	static int rssi_collision_threshold = 43;

	// if it is reinitialized
	if (first_run_) {
		state = 1;
		float t = porting::us_timestamp() / 1e6;
		state_start_time_ = t;
		first_run_ = false;
	}

	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = wall_following
	// 3 = move_out_of_way

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) { // FORWARD
		// if front range is close, start wallfollowing
		if (front_range < ref_distance_from_wall + 0.2f) {
			wallFollowing_.wall_follower_init(ref_distance_from_wall, 0.5, 3);
			state = transition(2); // wall_following
		}
	} else if (state == 2) { // WALL_FOLLOWING

		if (rssi_other_drone < rssi_collision_threshold) {
			state = transition(3);
		}
	} else if (state == 3) { // MOVE_OUT_OF_WAY
		if (rssi_other_drone > rssi_collision_threshold) {
			state = transition(1);
		}
	}
	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x = 0;
	float temp_vel_y = 0;
	float temp_vel_w = 0;

	if (state == 1) { // FORWARD
		// forward max speed
		temp_vel_x = 0.5;

	} else if (state == 2) { // WALL_FOLLOWING
		// Get the values from the wallfollowing
		if (local_direction_ == 1) {
			wallFollowing_.wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w,
			                             front_range, right_range,
			                             current_heading, local_direction_);
		} else if (local_direction_ == -1) {
			wallFollowing_.wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w,
			                             front_range, left_range,
			                             current_heading, local_direction_);
		}
	} else if (state == 3) { // MOVE_OUT_OF_WAY

		float save_distance = 0.7f;
		if (left_range < save_distance) {
			temp_vel_y = temp_vel_y - 0.5f;
		}
		if (right_range < save_distance) {
			temp_vel_y = temp_vel_y + 0.5f;
		}
		if (front_range < save_distance) {
			temp_vel_x = temp_vel_x - 0.5f;
		}
	}

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

	return state;
}
