#ifndef __WALLFOLLOWING_MULTIRANGER_ONBOARD_HPP__
#define __WALLFOLLOWING_MULTIRANGER_ONBOARD_HPP__
#include <stdint.h>
#include <stdbool.h>

int wall_follower(float *vel_x, float *vel_y, float *vel_w, float front_range, float side_range, float current_heading,
                  int direction_turn);

void adjustDistanceWall(float distance_wall_new);

void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref, int init_state);
#endif /* __WALLFOLLOWING_MULTIRANGER_ONBOARD_HPP__ */
