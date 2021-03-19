#ifndef PORTING__HPP
#define PORTING__HPP

#include <cstdint>

#include "exploration/p2p.hpp"
#include "exploration/stabilizer_types.hpp"

#ifndef TICKS_PER_FSM_LOOP
#	error "Macro `TICKS_PER_FSM_LOOP` is required"
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
#	error "Macro `EXPLORATION_METHOD` is required"
#endif

namespace porting {

/**
 * Get microsecond-resolution timestamp.
 */
uint64_t us_timestamp(void);
uint64_t config_block_get_radio_address(void);
void system_wait_start(void);
void ticks_delay(uint32_t nTicksToDelay);
uint32_t ms_to_ticks(uint16_t ms);
void commander_set_setpoint(exploration::setpoint_t *setpoint, int priority);
void estimator_kalman_get_estimated_pos(exploration::point_t *pos);
bool send_p2p_packet_broadcast(exploration::P2PPacket *p2pp);

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

} // namespace porting

#endif /* PORTING__HPP */
