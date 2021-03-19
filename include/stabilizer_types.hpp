#ifndef STABILIZER_TYPES_HPP
#define STABILIZER_TYPES_HPP

#include <cstdint>

#include "imu_types.hpp"

namespace exploration {
/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

/** Attitude in euler angle form */
typedef struct attitude_s {
	uint32_t timestamp; // Timestamp when the data was computed

	float roll;
	float pitch;
	float yaw;
} attitude_t;

/* vector */
#define vec3d_size 3
typedef float vec3d[vec3d_size];
typedef float mat3d[vec3d_size][vec3d_size];

/* x,y,z vector */
struct vec3_s {
	uint32_t timestamp; // Timestamp when the data was computed

	float x;
	float y;
	float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
	uint32_t timestamp;

	union {
		struct {
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct {
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;

typedef enum mode_e { modeDisable = 0, modeAbs, modeVelocity } stab_mode_t;

typedef struct setpoint_s {
	uint32_t timestamp;

	attitude_t attitude;     // deg
	attitude_t attitudeRate; // deg/s
	quaternion_t attitudeQuaternion;
	float thrust;
	point_t position;    // m
	velocity_t velocity; // m/s
	acc_t acceleration;  // m/s^2
	bool velocity_body;  // true if velocity is given in body frame; false if
	                     // velocity is given in world frame

	struct {
		stab_mode_t x;
		stab_mode_t y;
		stab_mode_t z;
		stab_mode_t roll;
		stab_mode_t pitch;
		stab_mode_t yaw;
		stab_mode_t quat;
	} mode;
} setpoint_t;

// Frequencies to bo used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an
// arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_500_HZ
#define POSITION_RATE RATE_100_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK)                                         \
	((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

} // namespace exploration
#endif /* STABILIZER_TYPES_HPP */
