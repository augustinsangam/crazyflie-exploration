#ifndef EXPLORATION_SGBA_HPP
#define EXPLORATION_SGBA_HPP

#include <cstdint>

#include "WallFollowing.hpp"
#include "porting.hpp"

namespace exploration {

class SGBA {
public:
	explicit SGBA(porting::DroneLayer *porting) : porting_{porting} {}

	void init(float new_ref_distance_from_wall, float max_speed_ref,
	          float begin_wanted_heading, float origin_x, float origin_y);
	void controller(float *vel_x, float *vel_y, float *vel_w, float *rssi_angle,
	               float front_range, float left_range, float right_range,
	               float back_range, float current_heading, float current_pos_x,
	               float current_pos_y, uint8_t rssi_inter,
	               float rssi_angle_inter, bool priority, bool outbound);

private:
	porting::DroneLayer *porting_;

	WallFollowing wf_;
	float state_start_time;
	bool first_run = true;
	float ref_distance_from_wall = 0;
	float max_speed = 0.5;
	uint8_t rssi_threshold =
	    58; // normal batteries 50/52/53/53 bigger batteries 55/57/59
	uint8_t rssi_collision_threshold =
	    50; // normal batteris 43/45/45/46 bigger batteries 48/50/52
	float wanted_angle = 0;

	float origin_x_, origin_y_;
};

} // namespace exploration

#endif /* EXPLORATION_SGBA_HPP */
