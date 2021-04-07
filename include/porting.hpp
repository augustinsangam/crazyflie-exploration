#ifndef PORTING_HPP
#define PORTING_HPP

#include <cmath>
#include <cstdint>

#include "exploration/types.hpp"

namespace porting {

class DroneLayer {
public:
	explicit DroneLayer(void *ctx) : ctx_(ctx) {}

	void kalman_estimated_pos(exploration::point_t *pos);

	void radiolink_broadcast_packet(exploration::P2PPacket *packet);

	void system_wait_start();

	void delay_ms(uint32_t t_ms);

	void commander_set_point(exploration::setpoint_t *sp, int prio);

	uint64_t config_block_radio_address();

	uint8_t deck_bc_multiranger();
	uint8_t deck_bc_flow2();

	uint8_t radio_rssi();

	float_t kalman_state_z();

	float_t stabilizer_yaw();

	float_t get_battery_level(); // Between 0 and 1

	float_t range_front(); // Between 0 and 1
	float_t range_left();  // Between 0 and 1
	float_t range_back();  // Between 0 and 1
	float_t range_right(); // Between 0 and 1
	float_t range_up();    // Between 0 and 1

	void debug_print(const char * fmt, ...);

private:
	void *ctx_ = nullptr;
};

uint64_t timestamp_us();

} // namespace porting

#endif /* PORTING_HPP */
