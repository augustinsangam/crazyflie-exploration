#ifndef WALL_FOLLOWING_HPP
#define WALL_FOLLOWING_HPP

#include <cstdint>

namespace exploration {

class WallFollowing {

public:
	int wall_follower(float *vel_x, float *vel_y, float *vel_w,
	                  float front_range, float side_range,
	                  float current_heading, int direction_turn);

	void wall_follower_init(float new_ref_distance_from_wall,
	                        float max_speed_ref, int init_state);

private:
	void command_turn(float *vel_x, float *vel_w, float ref_rate);
	void command_align_corner(float *vel_y, float *vel_w, float ref_rate,
	                          float range, float wanted_distance_from_corner);
	void command_forward_along_wall(float *vel_x, float *vel_y, float range);
	void command_turn_around_corner_and_adjust(float *vel_x, float *vel_y,
	                                           float *vel_w, float radius,
	                                           float range);
	void command_turn_and_adjust(float *vel_y, float *vel_w, float rate,
	                             float range);
	int transition(int new_state);
	void adjust_distance_wall(float distance_wall_new);

	float ref_distance_from_wall_ = 0;
	float max_speed_ = 0.5;
	float max_rate_ = 0.5;
	float direction_ = 1;
	float first_run_ = false;
	int state_ = 1;
	float state_start_time_;
};

} // namespace exploration
#endif /* WALL_FOLLOWING_HPP */
