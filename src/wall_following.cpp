#include <cmath>

#include "exploration/wall_following.hpp"
#include "math_supp.hpp"
#include "porting/porting.hpp"

void exploration::WallFollowing::wall_follower_init(
    float new_ref_distance_from_wall, float max_speed_ref, int init_state) {
	ref_distance_from_wall_ = new_ref_distance_from_wall;
	max_speed_ = max_speed_ref;
	first_run_ = true;
	state_ = init_state;
}

// Static helper functions
static bool logic_is_close_to(float real_value, float checked_value,
                              float margin) {
	if (real_value > checked_value - margin &&
	    real_value < checked_value + margin) {
		return true;
	} else {
		return false;
	}
}

void exploration::WallFollowing::command_turn(float *vel_x, float *vel_w,
                                              float ref_rate) {
	*vel_x = 0.0;
	*vel_w = direction_ * ref_rate;
}

void exploration::WallFollowing::command_align_corner(
    float *vel_y, float *vel_w, float ref_rate, float range,
    float wanted_distance_from_corner) {

	if (range > wanted_distance_from_corner + 0.3f) {
		*vel_w = direction_ * ref_rate;
		*vel_y = 0;

	} else {
		if (range > wanted_distance_from_corner) {
			*vel_y = direction_ * (-1 * max_speed_ / 3);
		} else {
			*vel_y = direction_ * (max_speed_ / 3);
		}
		*vel_w = 0;
	}
}

static void command_hover(float *vel_x, float *vel_y, float *vel_w) {
	*vel_x = 0.0;
	*vel_y = 0.0;
	*vel_w = 0.0;
}

void exploration::WallFollowing::command_forward_along_wall(float *vel_x,
                                                            float *vel_y,
                                                            float range) {
	*vel_x = max_speed_;
	bool check_distance_wall =
	    logic_is_close_to(ref_distance_from_wall_, range, 0.1);
	*vel_y = 0;
	if (!check_distance_wall) {
		if (range > ref_distance_from_wall_) {
			*vel_y = direction_ * (-1 * max_speed_ / 2);
		} else {
			*vel_y = direction_ * (max_speed_ / 2);
		}
	}
}

void exploration::WallFollowing::command_turn_around_corner_and_adjust(
    float *vel_x, float *vel_y, float *vel_w, float radius, float range) {
	*vel_x = max_speed_;
	*vel_w = direction_ * (-1 * (*vel_x) / radius);
	bool check_distance_to_wall =
	    logic_is_close_to(ref_distance_from_wall_, range, 0.1);
	if (!check_distance_to_wall) {
		if (range > ref_distance_from_wall_) {
			*vel_y = direction_ * (-1 * max_speed_ / 3);

		}

		else {
			*vel_y = direction_ * (max_speed_ / 3);
		}
	}
}

void exploration::WallFollowing::command_turn_and_adjust(float *vel_y,
                                                         float *vel_w,
                                                         float rate,
                                                         float range) {
	*vel_w = direction_ * rate;
	*vel_y = 0;
}

int exploration::WallFollowing::transition(int new_state) {
	float t = porting::us_timestamp() / 1e6;
	state_start_time_ = t;
	return new_state;
}

void exploration::WallFollowing::adjust_distance_wall(float distance_wall_new) {
	ref_distance_from_wall_ = distance_wall_new;
}

int exploration::WallFollowing::wall_follower(float *vel_x, float *vel_y,
                                              float *vel_w, float front_range,
                                              float side_range,
                                              float current_heading,
                                              int direction_turn) {

	direction_ = direction_turn;
	static float previous_heading = 0;
	static float angle = 0;
	static bool around_corner_go_back = false;
	float now = porting::us_timestamp() / 1e6;

	if (first_run_) {
		previous_heading = current_heading;
		//  around_corner_first_turn = false;
		around_corner_go_back = false;
		first_run_ = false;
	}

	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = hover
	// 3 = turn_to_find_wall
	// 4 = turn_to_allign_to_wall
	// 5 = forward along wall
	// 6 = rotate_around_wall
	// 7 = rotate_in_corner
	// 8 = find corner

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state_ == 1) { // FORWARD
		if (front_range < ref_distance_from_wall_ + 0.2f) {
			state_ = transition(3);
		}
	} else if (state_ == 2) { // HOVER

	} else if (state_ == 3) { // TURN_TO_FIND_WALL
		// check if wall is found
		bool side_range_check =
		    side_range < ref_distance_from_wall_ / (float)cos(0.78f) + 0.2f;
		bool front_range_check =
		    front_range < ref_distance_from_wall_ / (float)cos(0.78f) + 0.2f;
		if (side_range_check && front_range_check) {
			previous_heading = current_heading;
			angle = direction_ *
			        (1.57f - (float)atan(front_range / side_range) + 0.1f);
			state_ = transition(4); // go to turn_to_allign_to_wall
		}
		if (side_range < 1.0f && front_range > 2.0f) {
			//  around_corner_first_turn = true;
			around_corner_go_back = false;
			previous_heading = current_heading;
			state_ = transition(8); // go to rotate_around_wall
		}
	} else if (state_ == 4) { // TURN_TO_ALLIGN_TO_WALL
		bool allign_wall_check = logic_is_close_to(
		    wrap_to_pi(current_heading - previous_heading), angle, 0.1f);
		if (allign_wall_check) {
			// prev_side_range = side_range;
			state_ = transition(5);
		}
	} else if (state_ == 5) { // FORWARD_ALONG_WALL

		// If side range is out of reach,
		//    end of the wall is reached
		if (side_range > ref_distance_from_wall_ + 0.3f) {
			//  around_corner_first_turn = true;
			state_ = transition(8);
		}
		// If front range is small
		//    then corner is reached
		if (front_range < ref_distance_from_wall_ + 0.2f) {
			previous_heading = current_heading;
			state_ = transition(7);
		}

	} else if (state_ == 6) { // ROTATE_AROUND_WALL
		if (front_range < ref_distance_from_wall_ + 0.2f) {
			state_ = transition(3);
		}

	} else if (state_ == 7) { // ROTATE_IN_CORNER
		// Check if heading goes over 0.8 rad
		bool check_heading_corner = logic_is_close_to(
		    fabs(wrap_to_pi(current_heading - previous_heading)), 0.8f, 0.1f);
		if (check_heading_corner) {
			state_ = transition(3);
		}

	} else if (state_ == 8) { // FIND_CORNER
		if (side_range <= ref_distance_from_wall_) {
			state_ = transition(6);
		}

	}

	else {
	}

	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x = 0;
	float temp_vel_y = 0;
	float temp_vel_w = 0;

	if (state_ == 1) { // FORWARD
		temp_vel_x = max_speed_;
		temp_vel_y = 0.0;
		temp_vel_w = 0.0;

	} else if (state_ == 2) { // HOVER
		command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);

	} else if (state_ == 3) { // TURN_TO_FIND_WALL
		command_turn(&temp_vel_x, &temp_vel_w, max_rate_);
		temp_vel_y = 0.0;

	} else if (state_ == 4) { // TURN_TO_ALLIGN_TO_WALL

		if (now - state_start_time_ < 1.0f) {
			command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
		} else { // then turn again
			command_turn(&temp_vel_x, &temp_vel_w, max_rate_);
			temp_vel_y = 0;
		}

	} else if (state_ == 5) { // FORWARD_ALONG_WALL
		command_forward_along_wall(&temp_vel_x, &temp_vel_y, side_range);
		temp_vel_w = 0.0f;

		// commandForwardAlongWallHeadingSine(&temp_vel_x,
		// &temp_vel_y,&temp_vel_w, side_range);

	} else if (state_ == 6) { // ROTATE_AROUND_WALL
		// If first time around corner
		// first try to find the corner again

		// if side range is larger than prefered distance from wall
		if (side_range > ref_distance_from_wall_ + 0.5f) {

			// check if scanning has already occured
			if (wrap_to_pi(fabs(current_heading - previous_heading)) > 0.8f) {
				around_corner_go_back = true;
			}
			// turn and adjust distnace to corner from that point
			if (around_corner_go_back) {
				// go back if it already went into one direction
				command_turn_and_adjust(&temp_vel_y, &temp_vel_w,
				                        -1 * max_rate_, side_range);
				temp_vel_x = 0.0f;
			} else {
				command_turn_and_adjust(&temp_vel_y, &temp_vel_w, max_rate_,
				                        side_range);
				temp_vel_x = 0.0f;
			}
		} else {
			// continue to turn around corner
			previous_heading = current_heading;
			around_corner_go_back = false;
			command_turn_around_corner_and_adjust(
			    &temp_vel_x, &temp_vel_y, &temp_vel_w, ref_distance_from_wall_,
			    side_range);
		}

	} else if (state_ == 7) { // ROTATE_IN_CORNER
		command_turn(&temp_vel_x, &temp_vel_w, max_rate_);
		temp_vel_y = 0;

	} else if (state_ == 8) { // FIND_CORNER
		command_align_corner(&temp_vel_y, &temp_vel_w, -1 * max_rate_,
		                     side_range, ref_distance_from_wall_);
		temp_vel_x = 0;
	}

	else {
		// State does not exist so hover!!
		command_hover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
	}

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

	return state_;
}
