#ifndef __SGBA_HPP__
#define __SGBA_HPP__
#include <cstdint>

void init_sgba_controller(float new_ref_distance_from_wall, float max_speed_ref,
                          float begin_wanted_heading);
int sgba_controller(float *vel_x, float *vel_y, float *vel_w, float *rssi_angle,
                    int *state_wallfollowing, float front_range,
                    float left_range, float right_range, float back_range,
                    float current_heading, float current_pos_x,
                    float current_pos_y, uint8_t rssi_beacon,
                    uint8_t rssi_inter, float rssi_angle_inter, bool priority,
                    bool outbound);

#endif /* __SGBA_HPP__ */
