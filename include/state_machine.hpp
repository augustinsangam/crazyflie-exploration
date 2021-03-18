#ifndef __STATE_MACHINE_HPP__
#define __STATE_MACHINE_HPP__
#include "p2p.hpp"
#include "sgba.hpp"
#include "stabilizer_types.hpp"
#include "wallfollowing.hpp"
#include "wallfollowing_with_avoid.hpp"

namespace sgba {

class wall_following_controller {
public:
  void sgba_fsm_loop_iteration(void *param);
  void p2pCallbackHandler(P2PPacket *p);

private:
  bool keep_flying = false;

  float height;

  bool taken_off = false;
  float nominal_height = 0.3;

  uint8_t rssi_inter;
  uint8_t rssi_inter_filtered;
  uint8_t rssi_inter_closest;

  float rssi_angle_inter_ext;
  float rssi_angle_inter_closest;
  uint8_t rssi_beacon;
  uint8_t rssi_beacon_filtered;

  uint8_t id_inter_ext;
  sgba::setpoint_t setpoint_BG;
  float vel_x_cmd, vel_y_cmd, vel_w_cmd;
  float heading_rad;
  float right_range;
  float front_range;
  float left_range;
  float up_range;
  float back_range;
  float rssi_angle;
  int state;

#if EXPLORATION_METHOD == 3
  int state_wf;
#endif

  float up_range_filtered;
  int varid;
  bool on_the_ground = true;
  bool correctly_initialized;
  uint8_t rssi_array_other_drones[9] = {150, 150, 150, 150, 150,
                                        150, 150, 150, 150};
  uint64_t time_array_other_drones[9] = {0};
  float rssi_angle_array_other_drones[9] = {500.0f};
  uint8_t id_inter_closest = 100;

#if EXPLORATION_METHOD == 1
  WallFollowing exploration_controller_;
#elif EXPLORATION_METHOD == 2
  WallFollowingWithAvoid exploration_controller_;
#elif EXPLORATION_METHOD == 3
  Sgba exploration_controller_;
#endif
};

} // namespace sgba
#endif /* __STATE_MACHINE_HPP__ */
