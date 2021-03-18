#ifndef __SGBA_PORTAGE__
#define __SGBA_PORTAGE__

#include <cstdint>
#include "stabilizer_types.hpp"
#include "p2p.hpp"

namespace sgba {

/**
 * Get microsecond-resolution timestamp.
 */
uint64_t us_timestamp(void);
void p2p_register_cb(P2PCallback cb);
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
float get_left_range(); // Between 0 and 1
float get_back_range(); // Between 0 and 1
float get_up_range(); // Between 0 and 1


#ifndef TICKS_PER_FSM_LOOP 
#warning Macro "TICKS_PER_FSM_LOOP" is required. Set to default value 10
#define TICKS_PER_FSM_LOOP 10
#endif


} // namespace sgba

#endif /* __SGBA_PORTAGE__ */