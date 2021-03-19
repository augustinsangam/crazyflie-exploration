#ifndef WALL_FOLLOWING_WITH_AVOID_HPP
#define WALL_FOLLOWING_WITH_AVOID_HPP

#include <cstdint>

#include "wall_following.hpp"

namespace exploration {

class WallFollowingWithAvoid {

public:
	void
	init_wall_follower_and_avoid_controller(float new_ref_distance_from_wall,
	                                        float max_speed_ref,
	                                        float starting_local_direction);
	int wall_follower_and_avoid_controller(float *vel_x, float *vel_y,
	                                       float *vel_w, float front_range,
	                                       float left_range, float right_range,
	                                       float current_heading,
	                                       uint8_t rssi_other_drone);

private:
	int transition(int new_state);

	float state_start_time_;
	bool first_run_ = true;
	float ref_distance_from_wall = 0.5;
	float max_speed_ = 0.5;
	float local_direction_ = 1;
	WallFollowing wallFollowing_;
};

} // namespace exploration
#endif /* WALL_FOLLOWING_WITH_AVOID_HPP */
