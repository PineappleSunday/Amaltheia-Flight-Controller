#include "flight_logic.h"
#include "PID.h"
#include "mixer.h"
#include "state.h"
#include "main.h" // Essential for ESC_SetThrottle and get_timer_channel

// External PID instances from main.c
extern PIDController pid_roll, pid_pitch, pid_yaw;
// Assuming you add velocity/position PIDs to your main.c globals
extern PIDController pid_pos_z, pid_vel_z;
extern PIDController pid_roll_angle, pid_pitch_angle, pid_yaw_angle;
extern PIDController pid_roll_rate, pid_pitch_rate, pid_yaw_rate;

float wrap_deg(float a);

void FlightLogic_Update(vehicleState_t* state, targetState_t* target) {
	// 1. ALTITUDE CONTROL (Cascaded)
	float corr_vel_z = PID_Calculate(&pid_pos_z, target->z, state->z); // Setpoint first
	float target_vz  = corr_vel_z + target->ff_vz;
	float thrust_adj = PID_Calculate(&pid_vel_z, target_vz, state->vz);
	float base_thrust = 30.0f + thrust_adj;

	// 2. ATTITUDE OUTER LOOP (Angle -> Rate)
	// We wrap the Yaw error to ensure we take the shortest path
	float yaw_err = wrap_deg(target->yaw - state->yaw);

	// The output of these PIDs is the "Desired Rotation Rate" (deg/s)
	target->rate_roll  = PID_Calculate(&pid_roll_angle,  target->roll,  state->roll);
	target->rate_pitch = PID_Calculate(&pid_pitch_angle, target->pitch, state->pitch);
	target->rate_yaw   = PID_Calculate(&pid_yaw_angle,   yaw_err,      0); // Error already wrapped

	// 3. ATTITUDE INNER LOOP (Rate -> Torque)
	// We compare the Desired Rate to the RAW Gyro data (state->roll_rate / pitch_rate / yaw_rate)
    // Note: Use the 'rate' variables from your state struct
	float roll_torque  = PID_Calculate(&pid_roll_rate,  target->rate_roll,  state->roll_rate);
	float pitch_torque = PID_Calculate(&pid_pitch_rate, target->rate_pitch, state->pitch_rate);
	float yaw_torque   = PID_Calculate(&pid_yaw_rate,   target->rate_yaw,   state->yaw_rate);

	// 4. MOTOR MIXING (Plus Configuration)
	float motor_pcts[4];
	Mixer_Apply(base_thrust, roll_torque, pitch_torque, yaw_torque, motor_pcts);

	// 5. HARDWARE ACTUATION
	ESC_SetThrottle(TIM_CHANNEL_1, motor_pcts[0]); // Front
	ESC_SetThrottle(TIM_CHANNEL_2, motor_pcts[1]); // Right
	ESC_SetThrottle(TIM_CHANNEL_3, motor_pcts[2]); // Rear
	ESC_SetThrottle(TIM_CHANNEL_4, motor_pcts[3]); // Left
}

float wrap_deg(float a) {
	while (a > 180.0f) a -= 360.0f;
	while (a < -180.0f) a += 360.0f;
	return a;
}
