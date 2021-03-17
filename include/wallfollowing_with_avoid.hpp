#ifndef __WALLFOLLOWING_MULTIRANGER_WITH_AVOID_HPP_
#define __WALLFOLLOWING_MULTIRANGER_WITH_AVOID_HPP_

#include <cstdint>

void init_wall_follower_and_avoid_controller(float new_ref_distance_from_wall, float max_speed_ref,
    float starting_local_direction);
int wall_follower_and_avoid_controller(float *vel_x, float *vel_y, float *vel_w, float front_range, float left_range,
                                       float right_range,  float current_heading, uint8_t rssi_other_drone);

#endif /* __WALLFOLLOWING_MULTIRANGER_WITH_AVOID_HPP_ */
