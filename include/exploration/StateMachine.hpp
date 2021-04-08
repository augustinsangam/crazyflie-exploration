#ifndef EXPLORATION_STATEMACHINE_HPP
#define EXPLORATION_STATEMACHINE_HPP

#include "DroneState.hpp"
#include "Explorer.hpp"
#include "WallFollowing.hpp"
#include "median_filter.hpp"
#include "porting.hpp"
#include "types.hpp"
#include <array>
#include <cstdint>

#if EXPLORATION_METHOD == 1
#	include "exploration/WallFollowing.hpp"
#elif EXPLORATION_METHOD == 2
#	include "exploration/WallFollowingWithAvoid.hpp"
#elif EXPLORATION_METHOD == 3
#	include "exploration/SGBA.hpp"
#endif

namespace exploration {

class StateMachine {
public:
	explicit StateMachine(porting::DroneLayer *porting)
	    : state_{DroneState::onTheGround}, porting_{porting}, explorer_(porting) {}
	[[nodiscard]] inline DroneState get_state() const { return state_; }

	void init();

	void step();

	void take_off_robot();

	void land_robot();

	void start_mission();

	void end_mission();

	void return_to_base();

	void p2p_callback_handler(P2PPacket *p);

private:
	DroneState state_;
	porting::DroneLayer *porting_;
	Explorer explorer_;

	void set_state(DroneState state);

	void step_on_the_ground(setpoint_t *sp);
	void step_taking_off(setpoint_t *sp);
	void step_landing(setpoint_t *sp);
	void step_crashed(setpoint_t *sp);
	void step_exploring(setpoint_t *sp);
	void step_standby(setpoint_t *sp);
	void step_returning_to_base(setpoint_t *sp);

	static constexpr float nominal_height = 0.3F;

	uint8_t my_id;
	struct MedianFilterFloat medFilt, medFilt_2, medFilt_3;
	float rssi_angle;
	float front_range, right_range, left_range, back_range, up_range;
	uint8_t rssi_beacon;
	uint64_t takeoffdelaytime = 0;
	std::array<std::uint64_t, 9> time_array_other_drones{0};
	std::array<std::uint8_t, 9> rssi_array_other_drones{150, 150, 150, 150, 150,
	                                                    150, 150, 150, 150};
	std::array<float, 9> rssi_angle_array_other_drones{500.0F};
	bool correctly_initialized = false;
	bool keep_flying = false, taken_off = false;
	int exploration_state = 0;

#if EXPLORATION_METHOD != 1
	uint64_t radioSendBroadcastTime = 0;
	P2PPacket p_reply;
#endif

#if EXPLORATION_METHOD == 1
	bool outbound = true;
#endif

#if EXPLORATION_METHOD == 1
	WallFollowing exploration_controller_;
#elif EXPLORATION_METHOD == 2
	WallFollowingWithAvoid exploration_controller_;
#elif EXPLORATION_METHOD == 3
	SGBA exploration_controller_;
	int state_wf_;
	static constexpr bool outbound_{true};
#endif
};

} // namespace exploration

#endif /* EXPLORATION_STATEMACHINE_HPP */
