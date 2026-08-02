#include "flight_logic.h"
#include "PID.h"
#include "mixer.h"
#include "state.h"
#include "main.h" // Essential for ESC_SetThrottle and get_timer_channel

// External PID instances from main.c
// Active flight stack PIDs:
extern PIDController pid_pos_z, pid_vel_z;
extern PIDController pid_roll_angle, pid_pitch_angle, pid_yaw_angle;
extern PIDController pid_roll_rate, pid_pitch_rate, pid_yaw_rate;
extern float altitude_base_thrust_raw;

float wrap_deg(float a);

float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

uint8_t FlightLogic_Update(vehicleState_t* state, targetState_t* target, droneState_t* drone) {
	uint8_t sat = 0;
	(void)drone;

	float dt = state->dt_sec; // from MCU_dT seconds, clamped
	dt = clampf(dt, 0.001f, 0.01f);

	pid_roll_angle.cycle_time_seconds = dt;
	pid_pitch_angle.cycle_time_seconds = dt;
	pid_yaw_angle.cycle_time_seconds = dt;

	pid_roll_rate.cycle_time_seconds = dt;
	pid_pitch_rate.cycle_time_seconds = dt;
	pid_yaw_rate.cycle_time_seconds = dt;

	pid_pos_z.cycle_time_seconds = dt;
	pid_vel_z.cycle_time_seconds = dt;

	// 1. ALTITUDE CONTROL (Cascaded)
	float corr_vel_z = PID_Calculate(&pid_pos_z, state->z, target->z); // Setpoint last
	float target_vz  = corr_vel_z + target->ff_vz;
	float thrust_adj = PID_Calculate(&pid_vel_z, state->vz, target_vz);
	// 2. BASE THRUST SELECTION
	float base_thrust = altitude_base_thrust_raw + thrust_adj;
	base_thrust = clampf(base_thrust, 15.0f, 85.0f);

	// 2. ATTITUDE OUTER LOOP (Angle -> Rate)
	// We wrap the Yaw error to ensure we take the shortest path
	target->rate_roll  = PID_Calculate(&pid_roll_angle,  state->roll,  target->roll);
	target->rate_pitch = PID_Calculate(&pid_pitch_angle, state->pitch, target->pitch);

	if (target->yaw_hold_enabled) {
		float yaw_err = wrap_deg(target->yaw - state->yaw);
		target->rate_yaw = PID_Calculate(&pid_yaw_angle, 0.0f, yaw_err);
	} else {
		// Rate-command mode: leave target->rate_yaw as set by the caller.
		PID_ResetWithMeasurement(&pid_yaw_angle, 0.0f);
	}
	// 3. ATTITUDE INNER LOOP (Rate -> Torque)
	// We compare the Desired Rate to the RAW Gyro data (state->roll_rate / pitch_rate / yaw_rate)
	// Note: Use the 'rate' variables from your state struct
	float roll_torque  = PID_Calculate(&pid_roll_rate,  state->roll_rate,  target->rate_roll);
	float pitch_torque = PID_Calculate(&pid_pitch_rate, state->pitch_rate, target->rate_pitch);
	float yaw_torque   = PID_Calculate(&pid_yaw_rate,   state->yaw_rate,   target->rate_yaw);

	// 4. MOTOR MIXING (frame selected inside mixer config)
	float motor_pcts[4];
	sat = Mixer_Apply(base_thrust, roll_torque, pitch_torque, yaw_torque, motor_pcts);

	// 5. HARDWARE ACTUATION (channel order stays M1..M4)
	// Plus: M1 Front, M2 Right, M3 Rear, M4 Left
	// X:    M1 FrontRight, M2 RearRight, M3 RearLeft, M4 FrontLeft
	ESC_SetThrottle(TIM_CHANNEL_1, motor_pcts[0]);
	ESC_SetThrottle(TIM_CHANNEL_2, motor_pcts[1]);
	ESC_SetThrottle(TIM_CHANNEL_3, motor_pcts[2]);
	ESC_SetThrottle(TIM_CHANNEL_4, motor_pcts[3]);
	return sat;
}

float wrap_deg(float a) {
	while (a > 180.0f) a -= 360.0f;
	while (a < -180.0f) a += 360.0f;
	return a;
}
