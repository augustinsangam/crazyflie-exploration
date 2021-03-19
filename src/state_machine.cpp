#include <cerrno>
#include <cmath>
#include <cstring>

#include "median_filter.hpp"
#include "porting.hpp"
#include "state_machine.hpp"

#define STATE_MACHINE_COMMANDER_PRI 3

#define MANUAL_STARTUP_TIMEOUT M2T(3000)

static void take_off(exploration::setpoint_t *sp, float velocity) {
  sp->mode.x = exploration::modeVelocity;
  sp->mode.y = exploration::modeVelocity;
  sp->mode.z = exploration::modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = velocity;
  sp->mode.yaw = exploration::modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void land(exploration::setpoint_t *sp, float velocity) {
  sp->mode.x = exploration::modeVelocity;
  sp->mode.y = exploration::modeVelocity;
  sp->mode.z = exploration::modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = -velocity;
  sp->mode.yaw = exploration::modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void hover(exploration::setpoint_t *sp, float height) {
  sp->mode.x = exploration::modeVelocity;
  sp->mode.y = exploration::modeVelocity;
  sp->mode.z = exploration::modeAbs;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->position.z = height;
  sp->mode.yaw = exploration::modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void vel_command(exploration::setpoint_t *sp, float vel_x, float vel_y,
                        float yaw_rate, float height) {
  sp->mode.x = exploration::modeVelocity;
  sp->mode.y = exploration::modeVelocity;
  sp->mode.z = exploration::modeAbs;
  sp->velocity.x = vel_x;
  sp->velocity.y = vel_y;
  sp->position.z = height;
  sp->mode.yaw = exploration::modeVelocity;
  sp->attitudeRate.yaw = yaw_rate;
  sp->velocity_body = true;
}

static void shut_off_engines(exploration::setpoint_t *sp) {
  sp->mode.x = exploration::modeDisable;
  sp->mode.y = exploration::modeDisable;
  sp->mode.z = exploration::modeDisable;
  sp->mode.yaw = exploration::modeDisable;
}

static int32_t find_minimum(uint8_t a[], int32_t n) {
  int32_t c, min, index;

  min = a[0];
  index = 0;

  for (c = 1; c < n; c++) {
    if (a[c] < min) {
      index = c;
      min = a[c];
    }
  }

  return index;
}

void exploration::StateMachine::iteration_loop() {
  static struct MedianFilterFloat medFilt;
  init_median_filter_f(&medFilt, 5);
  static struct MedianFilterFloat medFilt_2;
  init_median_filter_f(&medFilt_2, 5);
  static struct MedianFilterFloat medFilt_3;
  init_median_filter_f(&medFilt_3, 13);
  // p2p_register_cb(p2pCallbackHandler);
  uint64_t address = porting::config_block_get_radio_address();
  uint8_t my_id = (uint8_t)((address)&0x00000000ff);
  static exploration::P2PPacket p_reply;
  p_reply.port = 0x00;
  p_reply.data[0] = my_id;
  memcpy(&p_reply.data[1], &rssi_angle, sizeof(float));
  p_reply.size = 5;

#if EXPLORATION_METHOD != 1
  static uint64_t radioSendBroadcastTime = 0;
#endif

  static uint64_t takeoffdelaytime = 0;

#if EXPLORATION_METHOD == 3
  static bool outbound = true;
#endif

  porting::system_wait_start();
  porting::ticks_delay(porting::ms_to_ticks(3000));

  while (1) {
    // some delay before the whole thing starts
    porting::ticks_delay(TICKS_PER_FSM_LOOP);
    // For every 1 second, reset the RSSI value to high if it hasn't been
    // received for a while
    for (uint8_t it = 0; it < 9; it++)
      if (porting::us_timestamp() >=
          time_array_other_drones[it] + 1000 * 1000) {
        time_array_other_drones[it] = porting::us_timestamp() + 1000 * 1000 + 1;
        rssi_array_other_drones[it] = 150;
        rssi_angle_array_other_drones[it] = 500.0f;
      }

    // get RSSI, id and angle of closests crazyflie.
    id_inter_closest = (uint8_t)find_minimum(rssi_array_other_drones, 9);
    rssi_inter_closest = rssi_array_other_drones[id_inter_closest];
    rssi_angle_inter_closest = rssi_angle_array_other_drones[id_inter_closest];

    rssi_inter_filtered =
        (uint8_t)update_median_filter_f(&medFilt_2, (float)rssi_inter_closest);

    // checking init of multiranger and flowdeck
    uint8_t multiranger_isinit = porting::get_deck_bc_multiranger();
    uint8_t flowdeck_isinit = porting::get_deck_bc_flow2();

    // get current height and heading
    height = porting::get_kalman_state_z();
    float heading_deg = porting::get_stabilizer_yaw();
    heading_rad = heading_deg * (float)M_PI / 180.0f;

    // t RSSI of beacon
    rssi_beacon = porting::get_radio_rssi();
    rssi_beacon_filtered =
        (uint8_t)update_median_filter_f(&medFilt_3, (float)rssi_beacon);

    // Select which laser range sensor readings to use
    if (multiranger_isinit) {
      front_range = porting::get_front_range() / 1000.0f;
      right_range = porting::get_right_range() / 1000.0f;
      left_range = porting::get_left_range() / 1000.0f;
      back_range = porting::get_back_range() / 1000.0f;
      up_range = porting::get_up_range() / 1000.0f;
    }

    // Get position estimate of kalman filter
    exploration::point_t pos;
    porting::estimator_kalman_get_estimated_pos(&pos); // TODO : Position of the drone

    // Initialize setpoint
    memset(&setpoint_BG, 0, sizeof(setpoint_BG));

    // Filtere uprange, since it sometimes gives a low spike that
    up_range_filtered = update_median_filter_f(&medFilt, up_range);
    if (up_range_filtered < 0.05f) {
      up_range_filtered = up_range;
    }
    if (flowdeck_isinit && multiranger_isinit) {
      correctly_initialized = true;
    }

#if EXPLORATION_METHOD == 3
    uint8_t rssi_beacon_threshold = 41;
    if (keep_flying == true &&
        (!correctly_initialized || up_range < 0.2f ||
         (!outbound && rssi_beacon_filtered < rssi_beacon_threshold))) {
      keep_flying = 0;
    }
#else
    if (keep_flying == true && (!correctly_initialized || up_range < 0.2f)) {
      keep_flying = 0;
    }
#endif

    state = 0;

    // Main flying code
    if (keep_flying) {
      if (taken_off) {
        /*
         * If the flight is given a OK
         *  and the crazyflie has taken off
         *   then perform state machine
         */
        vel_w_cmd = 0;
        hover(&setpoint_BG, nominal_height);

#if EXPLORATION_METHOD == 1 // WALL_FOLLOWING
        // wall following state machine
        state = exploration_controller_.wall_follower(
            &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, right_range,
            heading_rad, 1);
#endif
#if EXPLORATION_METHOD == 2 // WALL_FOLLOWER_AND_AVOID
        if (id_inter_closest > my_id) {
          rssi_inter_filtered = 140;
        }

        state = exploration_controller_.wall_follower_and_avoid_controller(
            &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, left_range,
            right_range, heading_rad, rssi_inter_filtered);
#endif
#if EXPLORATION_METHOD == 3 // SwWARM GRADIENT BUG ALGORITHM

        bool priority = false;
        if (id_inter_closest > my_id) {
          priority = true;
        } else {
          priority = false;
        }
        // TODO make outbound depended on battery.
        state = exploration_controller_.sgba_controller(
            &vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, &state_wf,
            front_range, left_range, right_range, back_range, heading_rad,
            (float)pos.x, (float)pos.y, rssi_beacon_filtered,
            rssi_inter_filtered, rssi_angle_inter_closest, priority, outbound);

        memcpy(&p_reply.data[1], &rssi_angle, sizeof(float));

#endif

        // convert yaw rate commands to degrees
        float vel_w_cmd_convert = vel_w_cmd * 180.0f / (float)M_PI;

        vel_command(&setpoint_BG, vel_x_cmd, vel_y_cmd, vel_w_cmd_convert,
                    nominal_height);
        on_the_ground = false;
      } else {
        /*
         * If the flight is given a OK
         *  but the crazyflie  has not taken off
         *   then take off
         */
        if (porting::us_timestamp() >= takeoffdelaytime + 1000 * 1000 * my_id) {

          take_off(&setpoint_BG, nominal_height);
          if (height > nominal_height) {
            taken_off = true;

#if EXPLORATION_METHOD == 1 // wall following
            exploration_controller_.wall_follower_init(0.4, 0.5, 1);
#endif
#if EXPLORATION_METHOD == 2 // wallfollowing with avoid
            if (my_id % 2 == 1)
              exploration_controller_.init_wall_follower_and_avoid_controller(
                  0.4, 0.5, -1);
            else
              exploration_controller_.init_wall_follower_and_avoid_controller(
                  0.4, 0.5, 1);

#endif
#if EXPLORATION_METHOD == 3 // Swarm Gradient Bug Algorithm
            if (my_id == 4 || my_id == 8) {
              exploration_controller_.init_sgba_controller(0.4, 0.5, -0.8);
            } else if (my_id == 2 || my_id == 6) {
              exploration_controller_.init_sgba_controller(0.4, 0.5, 0.8);
            } else if (my_id == 3 || my_id == 7) {
              exploration_controller_.init_sgba_controller(0.4, 0.5, -2.4);
            } else if (my_id == 5 || my_id == 9) {
              exploration_controller_.init_sgba_controller(0.4, 0.5, 2.4);
            } else {
              exploration_controller_.init_sgba_controller(0.4, 0.5, 0.8);
            }

#endif
          }
          on_the_ground = false;
        } else {
          shut_off_engines(&setpoint_BG);
          taken_off = false;
        }
      }
    } else {
      if (taken_off) {
        /*
         * If the flight is given a not OK
         *  but the Crazyflie  has already taken off
         *   then land
         */
        land(&setpoint_BG, 0.2f);
        if (height < 0.1f) {
          shut_off_engines(&setpoint_BG);
          taken_off = false;
        }
        on_the_ground = false;

      } else {

        /*
         * If the flight is given a not OK
         *  and Crazyflie has landed
         *   then keep engines off
         */
        shut_off_engines(&setpoint_BG);
        takeoffdelaytime = porting::us_timestamp();
        on_the_ground = true;
      }
    }

#if EXPLORATION_METHOD != 1
    if (porting::us_timestamp() >= radioSendBroadcastTime + 1000 * 500) {
      exploration::radiolinkSendP2PPacketBroadcast(&p_reply);
      radioSendBroadcastTime = porting::us_timestamp();
    }

#endif
    porting::commander_set_setpoint(&setpoint_BG, STATE_MACHINE_COMMANDER_PRI);
  }
}

void exploration::StateMachine::p2pCallbackHandler(P2PPacket *p) {
  id_inter_ext = p->data[0];

  if (id_inter_ext == 0x63) {
    // rssi_beacon =rssi_inter;
    keep_flying = p->data[1];
  } else if (id_inter_ext == 0x64) {
    rssi_beacon = p->rssi;

  } else {
    rssi_inter = p->rssi;
    memcpy(&rssi_angle_inter_ext, &p->data[1], sizeof(float));

    rssi_array_other_drones[id_inter_ext] = rssi_inter;
    time_array_other_drones[id_inter_ext] = porting::us_timestamp();
    rssi_angle_array_other_drones[id_inter_ext] = rssi_angle_inter_ext;
  }
}
