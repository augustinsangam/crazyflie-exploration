#ifndef SETPOINT_HPP
#define SETPOINT_HPP

#include "types.hpp"

namespace exploration {

class SetPoint {
public:
	static void take_off(setpoint_t *sp, float velocity) {
		sp->mode.x = modeVelocity;
		sp->mode.y = modeVelocity;
		sp->mode.z = modeVelocity;
		sp->velocity.x = 0.0;
		sp->velocity.y = 0.0;
		sp->velocity.z = velocity;
		sp->mode.yaw = modeVelocity;
		sp->attitudeRate.yaw = 0.0;
	}

	static void land(setpoint_t *sp, float velocity) {
		sp->mode.x = modeVelocity;
		sp->mode.y = modeVelocity;
		sp->mode.z = modeVelocity;
		sp->velocity.x = 0.0;
		sp->velocity.y = 0.0;
		sp->velocity.z = -velocity;
		sp->mode.yaw = modeVelocity;
		sp->attitudeRate.yaw = 0.0;
	}

	static void hover(setpoint_t *sp, float height) {
		sp->mode.x = modeVelocity;
		sp->mode.y = modeVelocity;
		sp->mode.z = modeAbs;
		sp->velocity.x = 0.0;
		sp->velocity.y = 0.0;
		sp->position.z = height;
		sp->mode.yaw = modeVelocity;
		sp->attitudeRate.yaw = 0.0;
	}

	static void vel_command(setpoint_t *sp, float vel_x, float vel_y,
	                         float yaw_rate, float height) {
		sp->mode.x = modeVelocity;
		sp->mode.y = modeVelocity;
		sp->mode.z = modeAbs;
		sp->velocity.x = vel_x;
		sp->velocity.y = vel_y;
		sp->position.z = height;
		sp->mode.yaw = modeVelocity;
		sp->attitudeRate.yaw = yaw_rate;
		sp->velocity_body = true;
	}

	static void shut_off_engines(setpoint_t *sp) {
		sp->mode.x = modeDisable;
		sp->mode.y = modeDisable;
		sp->mode.z = modeDisable;
		sp->mode.yaw = modeDisable;
	}
};

} // namespace exploration

#endif /* SETPOINT_HPP */
