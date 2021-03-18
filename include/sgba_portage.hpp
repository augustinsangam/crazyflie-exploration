#ifndef __SGBA_PORTAGE__HPP__
#define __SGBA_PORTAGE__HPP__

#include "p2p.hpp"
#include "stabilizer_types.hpp"
#include <cstdint>

namespace sgba {

/**
 * Get microsecond-resolution timestamp.
 */
uint64_t us_timestamp(void);
uint64_t config_block_get_radio_address(void);
void system_wait_start(void);
void ticks_delay(uint32_t nTicksToDelay);
uint32_t ms_to_ticks(uint16_t ms);
void commander_set_setpoint(setpoint_t *setpoint, int priority);
void estimator_kalman_get_estimated_pos(point_t *pos);
bool radiolinkSendP2PPacketBroadcast(P2PPacket *p2pp);

uint8_t get_deck_bc_multiranger();
uint8_t get_deck_bc_flow2();
float get_kalman_state_z();
float get_stabilizer_yaw();
uint8_t get_radio_rssi();

float get_front_range(); // Between 0 and 1
float get_right_range(); // Between 0 and 1
float get_left_range();  // Between 0 and 1
float get_back_range();  // Between 0 and 1
float get_up_range();    // Between 0 and 1

#ifndef TICKS_PER_FSM_LOOP
#warning Macro "TICKS_PER_FSM_LOOP" is required. Set to default value 10
#define TICKS_PER_FSM_LOOP 10
#endif

/*
Switch to multiple methods, that increases in complexity
* 1 = wall_following: Go forward and follow walls with the multiranger
* 2 = wall following with avoid: This also follows walls but will move away if
another Crazyflie with an lower ID is coming close
* 3 = SGBA: The SGBA method
that incorporates the above methods.

NOTE: the switching between outbound and
inbound has not been implemented yet
*/
#ifndef EXPLORATION_METHOD
#warning Macro "EXPLORATION_METHOD" is required. Set to default value 1
#define EXPLORATION_METHOD 3
#endif

} // namespace sgba

#endif /* __SGBA_PORTAGE__HPP__ */
