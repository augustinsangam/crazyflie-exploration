#ifndef STATE_HPP
#define STATE_HPP

#include <cstdint>
#include <string>
#include <array>

namespace exploration {

enum class DroneState {
	onTheGround, // 0
	takingOff, // 1
	landing, // 2
	crashed, // 3
	exploring, // 4
	standBy, // 5
	returningToBase // 6
};

static constexpr const std::string_view & drone_state_to_name(DroneState state) {
	constexpr std::array<std::string_view, 7> types{"onTheGround", "takingOff", "landing", "crashed", "exploring", "standBy", "returningToBase"};
	return types.at(static_cast<std::size_t>(state));
}

} // namespace exploration

#endif /* STATE_HPP */
