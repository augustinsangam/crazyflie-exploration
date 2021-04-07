#ifndef STATE_HPP
#define STATE_HPP

#include <string>

namespace exploration {

enum class DroneState {
	onTheGround,
	takingOff,
	landing,
	crashed,
	exploring,
	standBy,
	returningToBase
};

static std::string getDroneStateName(DroneState d) {
	switch (d) {
	case DroneState::onTheGround:
		return "onTheGround";
	case DroneState::takingOff:
		return "takingOff";
	case DroneState::landing:
		return "landing";
	case DroneState::crashed:
		return "crashed";
	case DroneState::exploring:
		return "exploring";
	case DroneState::standBy:
		return "standBy";
	case DroneState::returningToBase:
		return "returningToBase";
	}
}

} // namespace exploration

#endif /* STATE_HPP */
