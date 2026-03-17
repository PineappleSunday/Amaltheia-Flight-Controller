/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (USB REMOVED - UART DEBUG ONLY)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lsm303.h"
#include "i3gd20.h"
#include "PID.h"
#include <math.h>
#include <stdio.h>   // For printf
#include <stdlib.h>  // For atof, atoi
#include <string.h>  // For string manipulation
#include "state.h"       // Defines VehicleState, TargetState
#include "AHRS.h"        // Defines ahrsSensor_t and AHRS_Update
#include "navigation.h"  // Defines MissionManager and Navigation_GetTarget
#include "flight_logic.h"// Defines FlightLogic_Update
#include <float.h>
#include "biquadButter.h"
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SPI_FRAME_LEN 80
#define TELEMETRY_HEADER_NORMAL 0xDEADBEEFUL
#define TELEMETRY_HEADER_PID_TUNE 0x54554E45UL
typedef struct __attribute__((packed)) {
	uint32_t header; // 0xDEADBEEF
	float timestamp;
	float roll;
	float pitch;
	float yaw;
	float altitude;
	float voltage;
	uint8_t armed; //0x00:UNARMED  0xFF:ARMED
	uint8_t drone_mode;    // Primary mode (e.g., Manual, Auto)
	uint8_t flight_mode;   // Current flight state (e.g., Stabilize, AltHold)
	// Motor Thrust %
	uint8_t motor1_T;
	uint8_t motor2_T;
	uint8_t motor3_T;
	uint8_t motor4_T;
	uint8_t sensor_status;
	float setpoint;       // What we want (e.g., Target Rate deg/s)
	float measurement;    // What we have (e.g., Gyro Rate deg/s)
	float error;          // setpoint - measurement
	float p_term;         // Proportional Output Pitch Rate
	float i_term;         // Integral Output Pitch Rate
	float d_term;         // Derivative Output Pitch Rate
	float output_sum;     // Final PID output (to mixer)
	int16_t cmd_roll_deg_x100;
	int16_t cmd_pitch_deg_x100;
	int16_t cmd_yaw_deg_x100;

	uint8_t sat_flags;   // bits for motor hi/lo clamp or PID clamp

	int16_t i_state;     // active integrator (or roll only)

	int16_t gyro_p;      // deg/s*100
	int16_t gyro_q;      // deg/s*100
	int16_t gyro_r;      // deg/s*100
	uint8_t magic_footer;
} Telemetry_Packet_t;
_Static_assert(sizeof(Telemetry_Packet_t) == 80, "Telemetry Struct size mismatch!");

typedef struct __attribute__((packed)) {
	uint32_t header;
	float timestamp;
	uint16_t sequence;
	uint8_t packet_type;
	uint8_t loop_id;
	uint8_t axis_id;
	uint8_t flags;
	uint8_t sat_flags;
	uint8_t reserved0;
	float setpoint;
	float measurement;
	float error;
	float p_term;
	float i_term;
	float d_term;
	float output_sum;
	float kp;
	float ki;
	float kd;
	float reference_cmd;
	float plant_state;
	int16_t motor1_T_x100;
	int16_t motor2_T_x100;
	int16_t motor3_T_x100;
	int16_t motor4_T_x100;
	int16_t gyro_p_x100;
	int16_t gyro_q_x100;
	int16_t gyro_r_x100;
	uint8_t magic_footer;
	uint8_t reserved1;
} PIDTune_Packet_t;
_Static_assert(sizeof(PIDTune_Packet_t) == 80, "PID Tune Struct size mismatch!");

static bool normal_telem = true;
static uint8_t dbg_axis = 1;   // 0=roll, 1=pitch, 2=yaw

typedef enum {
	TELEM_MODE_NORMAL = 0,
	TELEM_MODE_PID_TUNE = 1
} telemetryMode_t;

typedef enum {
	PID_TUNE_LOOP_ROLL_RATE = 1,
	PID_TUNE_LOOP_PITCH_RATE = 2,
	PID_TUNE_LOOP_YAW_RATE = 3
} pidTuneLoop_t;


typedef enum {
	LOG_TYPE_NONE = 0,
	LOG_TYPE_ROLL_RATE,   // Inner Loop Roll
	LOG_TYPE_PITCH_RATE,  // Inner Loop Pitch
	LOG_TYPE_YAW_RATE,    // Inner Loop Yaw
	LOG_TYPE_ALT_VEL,     // Vertical Velocity Loop
	LOG_TYPE_POS_X,       // Outer Loop Position X
	LOG_TYPE_RAW_SENSORS  // Vibration Analysis
} LogType_t;



//_Static_assert(sizeof(Engineer_Packet_t)  == SPI_FRAME_LEN, "Engineer frame != SPI_FRAME_LEN");
static volatile uint8_t spi5_need_rearm = 0;

typedef enum {
	STATE_INIT,
	STATE_TUNE,
	STATE_DATA_DUMP,
	STATE_MODE_SEL,
	STATE_MODE_SEL_COMPLETE
} StartControlState_t;

typedef enum {
	INIT,
	SPOOLUP,
	TAKEOFF,
	TRANSISTION
} takeoff_t;
float take_off_thrust = 50.0f;

uint32_t takeoff_count = 1;

typedef enum {
	AXIS_PITCH,
	AXIS_ROLL,
	AXIS_DONE
} TuneAxis_t;


typedef enum {
	TUNE_IDLE,
	TUNE_INJECT_PULSE,
	TUNE_MEASURE_WAIT,
	TUNE_CALCULATE,
	TUNE_COOLDOWN
} TuneState_t;

#define TUNE_LOG_SIZE 1200 // 1.2s at 1kHz
static uint16_t dump_idx = 0;
static bool is_dumping = false;
float gyro_log[TUNE_LOG_SIZE];
float target_log[TUNE_LOG_SIZE];
uint16_t log_idx = 0;

volatile bool is_logging = false;
// Now declare the actual variables
StartControlState_t StartControlState = STATE_INIT;
TuneState_t state_tune = TUNE_IDLE;
takeoff_t takeoff_state = INIT;

void Navigation_Init(MissionManager* mgr, Waypoint* waypoints, uint16_t count, const vehicleState_t* current_state) {
	mgr->waypoints = waypoints;
	mgr->total_waypoints = count;
	mgr->current_index = 0;
	mgr->hover_start_time = 0.0f;
	mgr->prev_dist = FLT_MAX;
	mgr->wp_start_time = 0.0f;
	mgr->landing_start_t = 0.0f;
	mgr->is_complete = 0;

	// Start the first leg from where the drone is currently located
	mgr->prev_wp_pos[0] = current_state->x;
	mgr->prev_wp_pos[1] = current_state->y;
	mgr->prev_wp_pos[2] = current_state->z;
	mgr->prev_wp_pos[3] = current_state->yaw;
}

// -- FILTERING --
BiquadFilter_t filter_gyro_roll;
BiquadFilter_t filter_gyro_pitch;
BiquadFilter_t filter_gyro_yaw;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ESC Throttle COntrol
#define ESC_MIN_PULSE 1000  // 1000us (0%)
#define ESC_MAX_PULSE 2000  // 2000us (100%)
#define ESC_ARM_PULSE 1000  // Arming signal

#define LIDAR_BUF_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi5_rx;
DMA_HandleTypeDef hdma_spi5_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
volatile uint8_t is_system_armed = 0; // 0: Locked, 1: Armed
volatile uint32_t last_heartbeat_tick = 0; // Tracks last valid command
volatile uint8_t is_estop_active = 0; // 0: Normal, 1: Emergency STOP
volatile uint8_t is_land_cmd_active = 0; // 0: Normal, 1: Emergency Landing
volatile uint8_t tune_request = 0;

typedef enum { READ_ACCEL, READ_MAG } sensor_state_t;


// Define a test mission:

Waypoint mission_waypoints[] = {
		// { {x, y, z, yaw}, toa, hover_duration, tolerance, action }
		{{0.0f, 0.0f, 0.8f, 0.0f},  0.0f, 0.0f, 0.10f, WP_ACTION_MOVE},  // Step 1: Takeoff to 0.8m
		{{0.0f, 0.0f, 0.8f, 0.0f},  0.0f, 8.0f, 0.15f, WP_ACTION_HOVER}, // Step 2: Hover in place
		{{0.0f, 0.0f, 0.0f, 0.0f},  0.0f, 0.0f, 0.08f, WP_ACTION_LAND}   // Step 3: Land
};
uint16_t total_wp_count = 3;
//TIM3 > APB2 > Motor PWM Control
// Global instances for the Flight Stack
vehicleState_t  g_state;      // The current estimated state (Kinematics)
targetState_t g_target;	;     // The desired state (Setpoints)
MissionManager g_mission;    // The mission and waypoint sequencer
droneState_t g_drone_status; // Global metadata for modes, battery, and status



volatile float target_throttle = 0.0f; // The throttle we want


Telemetry_Packet_t telem_data;
PIDTune_Packet_t pid_tune_data;

uint8_t spi_rx_buffer[SPI_FRAME_LEN];
uint8_t spi_tx_buf[SPI_FRAME_LEN];

static volatile uint8_t spi5_frame_done = 0;
static volatile uint32_t spi5_frame_counter = 0;
static uint32_t spi5_last_arm_tick = 0;
static telemetryMode_t g_telem_mode = TELEM_MODE_NORMAL;
static pidTuneLoop_t g_pid_tune_loop = PID_TUNE_LOOP_PITCH_RATE;
static uint16_t g_pid_tune_sequence = 0;

uint8_t command_ready = 0;
float commandZ = 0;
//volatile uint8_t spi5_busy = 0;

LSM303 imu;
LSM303_Raw raw;
I3GD20 i3gd20;
I3GD20_Raw gyro_raw;

uint8_t tf_state = 0;
uint8_t tf_buf[9];
uint8_t tf_idx = 0;
uint16_t tf_checksum = 0;

static ahrsSensor_t raw_data;


typedef struct {
	uint32_t start_time;
	uint32_t duration;
	float pulse_throttle;
	float baseline_throttle;
	bool active;
	uint32_t channel;
} MotorPulse_t;
MotorPulse_t diagnostic_pulse = {0};
#define DIAGNOSTIC_MAX_THROTTLE 15.0f

// Logic Flags
volatile uint8_t tuning_triggered = 0;
volatile uint8_t in_diagnostic_mode = 0;
uint32_t last = 0;

// PID controllers
PIDController pid_roll;
PIDController pid_pitch;
PIDController pid_yaw;
PIDController pid_pos_z; // The outer loop (Altitude)
PIDController pid_vel_z; // The inner loop (Vertical Velocity)
// Angle PIDs (Outer Loop)
PIDController pid_roll_angle;
PIDController pid_pitch_angle;
PIDController pid_yaw_angle;

// Rate PIDs (Inner Loop)
PIDController pid_roll_rate;
PIDController pid_pitch_rate;
PIDController pid_yaw_rate;

// LIDAR Vars
uint8_t lidar_dma_buffer[LIDAR_BUF_SIZE];
uint8_t lidar_read_idx = 0;
volatile float range_dist_cm = 0.0f;
static float lidar_z_hist[5] = {0};
static uint8_t lidar_hist_idx = 0;
static uint8_t lidar_hist_count = 0;
static float lidar_z_filt = 0.0f;
static float lidar_z_prev = 0.0f;
static float lidar_vz_filt = 0.0f;
static uint8_t lidar_est_initialized = 0;

// Telem Vars



// I2C LSM303 Globals

// --- State flags ---
volatile uint8_t accel_ready = 0;
volatile uint8_t mag_ready   = 0;
volatile uint8_t i2c_busy    = 0;

// State Machine
typedef enum {
	I2C_IDLE,
	I2C_ACCEL_TX,
	I2C_ACCEL_RX,
	I2C_MAG_TX,
	I2C_MAG_RX
} i2c_state_t;

volatile i2c_state_t i2c_state = I2C_IDLE;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Vehicle_State_Init(droneState_t* state);
static void SPI5_ArmNextFrame(void);
void start_control(void);
bool run_autotune_step(float *rate_setpoint, float current_rate);
static void ResetActiveFlightPIDsFromState(const vehicleState_t* state);
static void AcquireSensorsAndUpdateState(float dt_sec);
static void BuildPIDTunePacket(float dt_sec, uint8_t sat_flags);
static void StageActiveTelemetryFrame(void);

//SPI1 GYRO
//I2C1 ACCEL/MAG
//UART2 ST-LINK
//SPI5 ESP8826

/* Custom ESC Control Functions */
void ESC_ArmAll(void);
void ESC_Disarm(void);
void ESC_SetThrottle(uint32_t channel, float percentage);
uint32_t get_timer_channel(int motor_num);
void Process_TELEM_Command(uint8_t* Buf, uint32_t Len);
bool telemCMDPulse = false; //
uint32_t telemCMDTimeStart;
uint32_t unknownTelemCMD_counter;
uint32_t CMDpulseTime = 1000;

static void I2C1_Scan(void);
void Process_Lidar_DMA(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Define the ITM port for SWV
int _write(int file, char *ptr, int len) {
	for (int i = 0; i < len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}

static float median5f(const float v[5]) {
	float t[5] = {v[0], v[1], v[2], v[3], v[4]};
	for (int i = 0; i < 4; i++) {
		for (int j = i + 1; j < 5; j++) {
			if (t[j] < t[i]) {
				float tmp = t[i];
				t[i] = t[j];
				t[j] = tmp;
			}
		}
	}
	return t[2];
}

static void ResetActiveFlightPIDsFromState(const vehicleState_t* state) {
	const float roll = (state != NULL) ? state->roll : 0.0f;
	const float pitch = (state != NULL) ? state->pitch : 0.0f;
	const float z = (state != NULL) ? state->z : 0.0f;
	const float vz = (state != NULL) ? state->vz : 0.0f;
	const float roll_rate = (state != NULL) ? state->roll_rate : 0.0f;
	const float pitch_rate = (state != NULL) ? state->pitch_rate : 0.0f;
	const float yaw_rate = (state != NULL) ? state->yaw_rate : 0.0f;

	PID_ResetWithMeasurement(&pid_roll_angle, roll);
	PID_ResetWithMeasurement(&pid_pitch_angle, pitch);
	// Yaw-angle loop uses wrapped yaw error with actual=0.0f in FlightLogic_Update.
	PID_ResetWithMeasurement(&pid_yaw_angle, 0.0f);

	PID_ResetWithMeasurement(&pid_pos_z, z);
	PID_ResetWithMeasurement(&pid_vel_z, vz);

	PID_ResetWithMeasurement(&pid_roll_rate, roll_rate);
	PID_ResetWithMeasurement(&pid_pitch_rate, pitch_rate);
	PID_ResetWithMeasurement(&pid_yaw_rate, yaw_rate);
}

static void AcquireSensorsAndUpdateState(float dt_sec) {
	LSM303_Process_DMA(&imu);

	// ACCELEROMETER Parsing
	if (imu.accel_ready) {
		// Atomic snapshot of the 6-byte buffer
		uint8_t accel_snap[6];
		__disable_irq();
		memcpy(accel_snap, imu.accel_raw, 6);
		imu.accel_ready = false; // Clear flag in struct
		__enable_irq();

		// Parse from snapshot (Little Endian: L, H)
		raw_data.ax = (int16_t)((accel_snap[1] << 8) | accel_snap[0]) * imu.accel_g_per_lsb;
		raw_data.ay = (int16_t)((accel_snap[3] << 8) | accel_snap[2]) * imu.accel_g_per_lsb;
		raw_data.az = (int16_t)((accel_snap[5] << 8) | accel_snap[4]) * imu.accel_g_per_lsb;
	}

	// MAGNETOMETER Parsing
	if (imu.mag_ready) {
		uint8_t mag_snap[6];
		__disable_irq();
		memcpy(mag_snap, imu.mag_raw, 6);
		imu.mag_ready = false; // Clear flag in struct
		__enable_irq();

		if (imu.variant == LSM303_DLHC) {
			// DLHC: Big Endian and X-Z-Y order
			raw_data.mx = (int16_t)((mag_snap[0] << 8) | mag_snap[1]) * imu.mag_gauss_per_lsb;
			raw_data.mz = (int16_t)((mag_snap[2] << 8) | mag_snap[3]) * imu.mag_gauss_per_lsb;
			raw_data.my = (int16_t)((mag_snap[4] << 8) | mag_snap[5]) * imu.mag_gauss_per_lsb;
		} else {
			// AGR: Little Endian and X-Y-Z order
			raw_data.mx = (int16_t)((mag_snap[1] << 8) | mag_snap[0]) * imu.mag_gauss_per_lsb;
			raw_data.my = (int16_t)((mag_snap[3] << 8) | mag_snap[2]) * imu.mag_gauss_per_lsb;
			raw_data.mz = (int16_t)((mag_snap[5] << 8) | mag_snap[4]) * imu.mag_gauss_per_lsb;
		}
	}

	// --- GYROSCOPE ---
	if (i3gd20.initialized && I3GD20_ReadGyro(&i3gd20, &gyro_raw)) {
		float gx_raw = gyro_raw.gx * i3gd20.dps_per_lsb;
		float gy_raw = gyro_raw.gy * i3gd20.dps_per_lsb;
		float gz_raw = gyro_raw.gz * i3gd20.dps_per_lsb;

		// BIQUAD FILTERING
		raw_data.gx = Biquad_Process(&filter_gyro_roll,  gx_raw);
		raw_data.gy = Biquad_Process(&filter_gyro_pitch, gy_raw);
		raw_data.gz = Biquad_Process(&filter_gyro_yaw,   gz_raw);
	}

	// Lidar-based altitude/vertical-velocity estimator.
	// - Innovation gate rejects one-sample spikes unless accel indicates real motion.
	// - Median-of-5 removes impulse outliers.
	// - LPF + derivative provides a stable vz estimate for the vertical controller.
	float z_raw = range_dist_cm * 0.01f; // meters
	float z_candidate = z_raw;
	if (lidar_est_initialized) {
		float dz = z_candidate - lidar_z_prev;
		float max_step = 0.40f * dt_sec + 0.02f; // m/sample gate
		float a_norm = sqrtf(raw_data.ax * raw_data.ax + raw_data.ay * raw_data.ay + raw_data.az * raw_data.az);
		bool accel_event = fabsf(a_norm - 1.0f) > 0.25f;
		if (fabsf(dz) > max_step && !accel_event) {
			z_candidate = lidar_z_prev;
		}
	}

	lidar_z_hist[lidar_hist_idx] = z_candidate;
	lidar_hist_idx = (lidar_hist_idx + 1) % 5;
	if (lidar_hist_count < 5) lidar_hist_count++;

	float z_med = (lidar_hist_count < 5) ? z_candidate : median5f(lidar_z_hist);

	if (!lidar_est_initialized) {
		lidar_z_filt = z_med;
		lidar_z_prev = z_med;
		lidar_vz_filt = 0.0f;
		lidar_est_initialized = 1;
	} else {
		float alpha_z = dt_sec / (0.08f + dt_sec);
		lidar_z_filt += alpha_z * (z_med - lidar_z_filt);

		float vz_raw = (lidar_z_filt - lidar_z_prev) / dt_sec;
		lidar_z_prev = lidar_z_filt;

		float alpha_v = dt_sec / (0.12f + dt_sec);
		lidar_vz_filt += alpha_v * (vz_raw - lidar_vz_filt);
	}

	g_state.z = range_dist_cm / 100;
	g_state.vz = lidar_vz_filt;

	// STATE ESTIMATION (AHRS & Kalman)
	AHRS_Update(&raw_data, &g_state, dt_sec);
}

static void BuildPIDTunePacket(float dt_sec, uint8_t sat_flags) {
	PIDController* pid_active = &pid_pitch_rate;
	float setpoint = g_target.rate_pitch;
	float measurement = g_state.pitch_rate;
	float reference_cmd = g_target.pitch;
	float plant_state = g_state.pitch;
	uint8_t axis_id = 1;

	switch (g_pid_tune_loop) {
	case PID_TUNE_LOOP_ROLL_RATE:
		pid_active = &pid_roll_rate;
		setpoint = g_target.rate_roll;
		measurement = g_state.roll_rate;
		reference_cmd = g_target.roll;
		plant_state = g_state.roll;
		axis_id = 0;
		break;

	case PID_TUNE_LOOP_YAW_RATE:
		pid_active = &pid_yaw_rate;
		setpoint = g_target.rate_yaw;
		measurement = g_state.yaw_rate;
		reference_cmd = g_target.yaw;
		plant_state = g_state.yaw;
		axis_id = 2;
		break;

	case PID_TUNE_LOOP_PITCH_RATE:
	default:
		break;
	}

	uint8_t flags = 0;
	if (is_system_armed) {
		flags |= (1U << 0);
	}
	if (g_drone_status.drone_mode != MODE_MANUAL_LEVEL &&
			g_drone_status.drone_mode != MODE_MISSION &&
			g_drone_status.drone_mode != MODE_THRUST_STAND) {
		flags |= (1U << 1);
	}
	if (g_state.isTuning) {
		flags |= (1U << 2);
	}

	pid_tune_data.header = TELEMETRY_HEADER_PID_TUNE;
	pid_tune_data.timestamp = dt_sec;
	pid_tune_data.sequence = g_pid_tune_sequence++;
	pid_tune_data.packet_type = 1;
	pid_tune_data.loop_id = (uint8_t)g_pid_tune_loop;
	pid_tune_data.axis_id = axis_id;
	pid_tune_data.flags = flags;
	pid_tune_data.sat_flags = sat_flags;
	pid_tune_data.reserved0 = 0;
	pid_tune_data.setpoint = setpoint;
	pid_tune_data.measurement = measurement;
	pid_tune_data.error = pid_active->previous_error;
	pid_tune_data.p_term = pid_active->p_out;
	pid_tune_data.i_term = pid_active->i_out;
	pid_tune_data.d_term = pid_active->d_out;
	pid_tune_data.output_sum = pid_active->output;
	pid_tune_data.kp = pid_active->kp;
	pid_tune_data.ki = pid_active->ki;
	pid_tune_data.kd = pid_active->kd;
	pid_tune_data.reference_cmd = reference_cmd;
	pid_tune_data.plant_state = plant_state;
	pid_tune_data.motor1_T_x100 = (int16_t)(telem_data.motor1_T * 100);
	pid_tune_data.motor2_T_x100 = (int16_t)(telem_data.motor2_T * 100);
	pid_tune_data.motor3_T_x100 = (int16_t)(telem_data.motor3_T * 100);
	pid_tune_data.motor4_T_x100 = (int16_t)(telem_data.motor4_T * 100);
	pid_tune_data.gyro_p_x100 = (int16_t)(g_state.roll_rate * 100.0f);
	pid_tune_data.gyro_q_x100 = (int16_t)(g_state.pitch_rate * 100.0f);
	pid_tune_data.gyro_r_x100 = (int16_t)(g_state.yaw_rate * 100.0f);
	pid_tune_data.magic_footer = 0xAB;
	pid_tune_data.reserved1 = 0;
}

static void StageActiveTelemetryFrame(void) {
	if (g_telem_mode == TELEM_MODE_PID_TUNE &&
			StartControlState == STATE_MODE_SEL_COMPLETE) {
		BuildPIDTunePacket(g_state.dt_sec, telem_data.sat_flags);
		memcpy(spi_tx_buf, &pid_tune_data, SPI_FRAME_LEN);
		return;
	}

	telem_data.header = TELEMETRY_HEADER_NORMAL;
	telem_data.magic_footer = 0xAB;
	memcpy(spi_tx_buf, &telem_data, SPI_FRAME_LEN);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI5_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */



	// --- 1. Initialize Telemetry Structure ---
	telem_data.header = 0xDEADBEEF;
	telem_data.magic_footer = 0xAB;
	telem_data.timestamp = 0.0f;
	telem_data.voltage = 12.0f;
	telem_data.sensor_status = 0;
	telem_data.armed = 0x00;
	telem_data.flight_mode = 0x05;




	// Fill with test pattern to verify DMA is reading memory
	// memset(&telem_data.roll, 0xAA, 12);


	// --- 3. Start Lidar DMA ---
	HAL_UART_Receive_DMA(&huart1, lidar_dma_buffer, LIDAR_BUF_SIZE);
	HAL_Delay(100);



	I2C1_Scan();
	Vehicle_State_Init(&g_drone_status);
	last = HAL_GetTick();
	g_drone_status.flight_mode = 10; // 0 = START UP
	HAL_Delay(10);
	while(StartControlState != STATE_MODE_SEL_COMPLETE) { start_control(); }

	// NOTE TO COLIN
	// MODE SELECT
	// ONCE MODE SELECT
	// HAND OFF TO NAVIGATION STATE MACHINE
	// Get Circular logic back to START CONTROL once Mode is finished or after emergency
	// Will eventually want to setup a continue waypoint



	float main_last = 0.00f;



	// Loading Mission
	// Link the waypoints to the manager and provide the current state for the start position
	if (g_drone_status.drone_mode == MODE_MISSION){
		Navigation_Init(&g_mission, mission_waypoints, total_wp_count, &g_state);
		g_drone_status.drone_mode = 2;
		// Safety: Set the mission start time to the current clock
		g_mission.wp_start_time = (float)HAL_GetTick() / 1000.0f;
	}




	ResetActiveFlightPIDsFromState(&g_state);
	g_target.yaw = g_state.yaw;
	g_target.yaw_hold_enabled = true;
	g_target.rate_yaw = 0.0f;

	SPI5_ArmNextFrame();

	g_state.offGround = false;
	float flight_leg_height = 0.2f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		uint8_t do_rearm = 0;

		if (spi5_need_rearm) {
			spi5_need_rearm = 0;
			do_rearm = 1;
		}

		if (spi5_frame_done) {
			spi5_frame_done = 0;

			uint8_t cmd_work_buf[SPI_FRAME_LEN];
			__disable_irq();
			memcpy(cmd_work_buf, spi_rx_buffer, SPI_FRAME_LEN);
			memset(spi_rx_buffer, 0, SPI_FRAME_LEN);
			__enable_irq();

			if (cmd_work_buf[0] == '$') {
				Process_TELEM_Command(cmd_work_buf, SPI_FRAME_LEN);
			}
			do_rearm = 1;
		}

		if (do_rearm) {
			SPI5_ArmNextFrame();
		}
		// 1. Process Lidar
		Process_Lidar_DMA();

		uint32_t now = HAL_GetTick();

		if (telemCMDPulse){
			// Has 1000ms passed?
			g_drone_status.flight_mode = 91;
			if ((now - telemCMDTimeStart) > CMDpulseTime) {
				telemCMDPulse = false;
				ESC_SetThrottle(TIM_CHANNEL_1, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_2, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_3, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_4, 0.0f);
			}
		}
		// 2. Run Control Loop at 500Hz
		if (now - main_last >= 2) {

				float dt_sec = (now - main_last) / 1000.0f;
				if (dt_sec <= 0.0f || dt_sec > 0.5f) dt_sec = 0.002f;
				main_last = now;
				g_state.dt_sec = dt_sec;

				uint8_t takeoff_override_this_tick = 0;
				AcquireSensorsAndUpdateState(dt_sec);
				static uint32_t state_timer = 0;

			if ((!g_state.offGround) && (is_system_armed)){
				switch(takeoff_state) {
				case INIT:
					g_drone_status.flight_mode = 81;
					PID_Reset(&pid_pos_z); // Clear old errors
					PID_Reset(&pid_vel_z);
					g_target.yaw = g_state.yaw;
					g_target.yaw_hold_enabled = false;
					g_target.rate_yaw = 0.0f;
					// Startup guard: keep motors at idle and freeze nav/control this tick
					ESC_SetThrottle(TIM_CHANNEL_1, 10.0f);
					ESC_SetThrottle(TIM_CHANNEL_2, 10.0f);
					ESC_SetThrottle(TIM_CHANNEL_3, 10.0f);
					ESC_SetThrottle(TIM_CHANNEL_4, 10.0f);
					takeoff_override_this_tick = 1;
					state_timer = now;
					takeoff_state = SPOOLUP;
					break;

				case SPOOLUP:
					// Give motors 500ms to reach idle speed
					g_drone_status.flight_mode = 82;
					g_target.yaw = g_state.yaw;
					g_target.yaw_hold_enabled = false;
					g_target.rate_yaw = 0.0f;
					ESC_SetThrottle(TIM_CHANNEL_1, 15.0f);
					ESC_SetThrottle(TIM_CHANNEL_2, 15.0f);
					ESC_SetThrottle(TIM_CHANNEL_3, 15.0f);
					ESC_SetThrottle(TIM_CHANNEL_4, 15.0f);
					takeoff_override_this_tick = 1;
					if (now - state_timer > 500) {
						state_timer = now;
						takeoff_state = TAKEOFF;
					}
					break;

				case TAKEOFF:
					g_drone_status.flight_mode = 83;
					g_target.roll  = 0.0f;
					g_target.pitch = 0.0f;
					g_target.yaw = g_state.yaw;
					g_target.yaw_hold_enabled = false;
					g_target.rate_yaw = 0.0f;
					g_target.z     = flight_leg_height + 0.2f;
					g_target.ff_vz = 0.5f;

					// Allow control update in TAKEOFF to climb toward z target

					if (g_state.z > flight_leg_height) {
						takeoff_state = TRANSISTION;
						takeoff_count = 0;
					} else {
						takeoff_count++;
					}
					break;

				case TRANSISTION:
					g_drone_status.flight_mode = 84;
					g_target.yaw = g_state.yaw;
					g_target.yaw_hold_enabled = true;
					g_target.rate_yaw = 0.0f;
					ResetActiveFlightPIDsFromState(&g_state);
					g_state.offGround = true; // Hand over to main flight controller
					g_drone_status.flight_mode = 0x08; //Stabilize
					break;
				}
			}
			/*printf("IMU,%ld,AX=%f,AY=%f,AZ=%f,MX=%f,MY=%f,MZ=%f,GX=%f,GY=%f,GZ=%f\r\n",
			       HAL_GetTick(),
			       raw_data.ax, raw_data.ay, raw_data.az,
			       raw_data.mx, raw_data.my, raw_data.mz,
			       raw_data.gx, raw_data.gy, raw_data.gz);
			 */
			// 3. NAVIGATION (Mission Manager)
			// 2. MISSION LOGIC
			uint8_t invalid_mode_requested = 0;
			if (!takeoff_override_this_tick &&
					(g_drone_status.drone_mode != MODE_MANUAL_LEVEL) &&
					(g_drone_status.drone_mode != MODE_MISSION) &&
					(g_drone_status.drone_mode != MODE_THRUST_STAND)) {
				invalid_mode_requested = 1;
			}

			if (is_estop_active){
				// Force a landing setpoint: Stay at current X/Y, but descend Z
				g_target.x = g_state.x;
				g_target.y = g_state.y;
				g_drone_status.flight_mode = 4; // 4 = EMERGENCY LANDING
				// Use your tiered descent rates from navigation.c
				float descent_rate = (g_state.z > 5.0f) ? 0.4f : 0.15f;
				g_target.z = g_state.z - (descent_rate * dt_sec);
				if (g_target.z < 0.4f) g_target.z = 0.0f;

				g_target.ff_vz = -descent_rate;

				// Auto-disarm immediately
				ResetActiveFlightPIDsFromState(&g_state);
				g_drone_status.flight_mode = 0; // 0 = DISARMED/IDLE/ONGROUND
				g_state.offGround = false;
				// Force immediate hardware override to 0%
				for(int i = 1; i <= 4; i++) {
					ESC_SetThrottle(get_timer_channel(i), 0.0f);
				}
				is_system_armed = 0;
				is_estop_active = 0; // Reset for next boot

			} else if (is_land_cmd_active) {
				// FUTURE USE NOT IN CURRENT IMPLEMENTATION
				// Force a landing setpoint: Stay at current X/Y, but descend Z
				g_target.x = g_state.x;
				g_target.y = g_state.y;
				g_drone_status.flight_mode = 4; // 4 = EMERGENCY LANDING
				// Use your tiered descent rates from navigation.c
				float descent_rate = (g_state.z > 5.0f) ? 0.4f : 0.15f;
				g_target.z = g_state.z - (descent_rate * dt_sec);
				if (g_target.z < 0.4f) g_target.z = 0.0f;

				g_target.ff_vz = -descent_rate;

				// Auto-disarm once on the ground
				if (g_state.z <= flight_leg_height){//flight_leg_height) {
					ResetActiveFlightPIDsFromState(&g_state);
					g_drone_status.flight_mode = 0; // 0 = DISARMED/IDLE/ONGROUND
					g_state.offGround = false;
					// Force immediate hardware override to 0%
					for(int i = 1; i <= 4; i++) {
						ESC_SetThrottle(get_timer_channel(i), 0.0f);
					}
					is_system_armed = 0;
					is_estop_active = 0; // Reset for next boot
				}

			} else if (!takeoff_override_this_tick && invalid_mode_requested) {
				// Safety fallback for unsupported/reserved modes: inhibit control outputs
				g_target.rate_roll = 0.0f;
				g_target.rate_pitch = 0.0f;
				g_target.rate_yaw = 0.0f;
				g_target.ff_vz = 0.0f;

				if (is_system_armed) {
					for(int i = 1; i <= 4; i++) {
						ESC_SetThrottle(get_timer_channel(i), 0.0f);
					}
				}
			} else if (!takeoff_override_this_tick &&
					(g_drone_status.drone_mode == MODE_MANUAL_LEVEL) &&
					(takeoff_state != TAKEOFF)){
				g_drone_status.flight_mode = 8; //Stabilize
				g_drone_status.drone_mode = 1; // Signal Manual Mode
				g_target.yaw_hold_enabled = true;
				if (isnan(g_state.roll) || isnan(g_state.pitch) || isnan(g_state.yaw)) {
					// Force reset the state so the math can recover
					g_state.roll = 0.0f;
					g_state.pitch = 0.0f;
					g_state.yaw = 0.0f;
				}
				if (isnan(g_target.rate_roll)) {
					g_target.rate_roll = 0.0f;
					g_target.rate_pitch = 0.0f;
					g_target.rate_yaw = 0.0f;
				}
				g_target.z = commandZ + flight_leg_height;

			} else if (!takeoff_override_this_tick &&
					(g_drone_status.drone_mode == MODE_MISSION)) {
				// Normal Navigation logic
				g_drone_status.drone_mode = 2;
				g_drone_status.flight_mode = 0x08; //Stabilize
				Navigation_GetTarget(&g_mission, (float)now / 1000.0f, &g_state, &g_target);
				g_target.yaw_hold_enabled = true;

			} else if (g_drone_status.drone_mode == MODE_THRUST_STAND) {
				g_drone_status.drone_mode = 3;
				continue;

			}

			// 4. FLIGHT CONTROL (Executive Logic)
			// This is where we run PIDs and then the Mixer
			// 3. FLIGHT CONTROL & TELEMETRY
			if (isnan(g_target.rate_pitch) || isnan(g_target.rate_roll)) {
				g_target.rate_pitch = 0.0f;
				g_target.rate_roll = 0.0f;
				g_target.rate_yaw = 0.0f;
			}
			if (invalid_mode_requested) {
				telem_data.sat_flags = 0;
			}
			else if (is_system_armed && !takeoff_override_this_tick) {
				g_drone_status.flight_mode = 8;
				telem_data.sat_flags = FlightLogic_Update(&g_state, &g_target, &g_drone_status);
			}
			// Check if the system is disarmed AND if any motor has a non-zero throttle
			else if (telem_data.motor1_T > 0 || telem_data.motor2_T > 0 ||
					telem_data.motor3_T > 0 || telem_data.motor4_T > 0) {
				telem_data.armed = 0x00;
				// Force motors to 0% only once when transitioning to/remaining in a disarmed state
				for(int i = 1; i <= 4; i++) {
					ESC_SetThrottle(get_timer_channel(i), 0.0f);
				}
			}

			// 5. UPDATE TELEMETRY (Your Exact Atomic Block)
			// Mapping g_state back to your required telemetry variables

			if (normal_telem){
				telem_data.header 				= 0xDEADBEEF;       // UINT32
				telem_data.timestamp 			= dt_sec;        // float 1
				telem_data.roll 				= g_state.roll;       // float 2 (Estimated Roll)
				telem_data.pitch 				= g_state.pitch;     // float 3 (Estimated Pitch)
				telem_data.yaw 					= g_state.yaw;         // float 4 (Estimated Yaw)
				telem_data.altitude 			= range_dist_cm;  // float 5 (Raw Lidar in cm)
				telem_data.voltage 				= commandZ;           // float 6
				telem_data.armed 				= is_system_armed ? 0xFF : 0x00;
				// Map the modes to the telemetry packet
				telem_data.drone_mode  			= g_drone_status.drone_mode;
				telem_data.flight_mode 			= g_drone_status.flight_mode;
				telem_data.setpoint    			= g_target.rate_pitch;
				telem_data.measurement 			= g_state.pitch_rate;
				telem_data.error       			= pid_pitch_rate.previous_error;
				telem_data.p_term      			= pid_pitch_rate.p_out;
				telem_data.i_term      			= pid_pitch_rate.i_out;
				telem_data.d_term      			= pid_pitch_rate.d_out;
				telem_data.output_sum  			= pid_pitch_rate.output;
				telem_data.cmd_roll_deg_x100 	= (int16_t)(g_target.roll  * 100.0f);
				telem_data.cmd_pitch_deg_x100 	= (int16_t)(g_target.pitch * 100.0f);
				telem_data.cmd_yaw_deg_x100   	= (int16_t)(g_target.yaw   * 100.0f);

				telem_data.i_state 				= (int16_t)(pid_pitch_rate.i_out * 100.0f);

				telem_data.gyro_p 				= (int16_t)(g_state.roll_rate  * 100.0f);
				telem_data.gyro_q 				= (int16_t)(g_state.pitch_rate * 100.0f);
				telem_data.gyro_r 				= (int16_t)(g_state.yaw_rate   * 100.0f);
				telem_data.magic_footer 		= 0xAB;       // UINT8
			} else {
				// DEBUG TELEMETRY (multiplex roll/pitch/yaw rate loops)
				static uint8_t dbg_axis = 0;         // 0=roll, 1=pitch, 2=yaw
				dbg_axis = (dbg_axis + 1) % 3;

				telem_data.header              = 0xDEADBEEF;   // UINT32
				telem_data.timestamp           = dt_sec;

				// Keep these exactly as-is
				telem_data.roll                = g_state.roll;
				telem_data.pitch               = g_state.pitch;
				telem_data.yaw                 = g_state.yaw;
				telem_data.altitude            = range_dist_cm;
				telem_data.voltage             = commandZ;

				telem_data.armed               = is_system_armed ? 0xFF : 0x00;

				telem_data.flight_mode         = g_drone_status.flight_mode;

				telem_data.cmd_roll_deg_x100   = (int16_t)(g_target.roll  * 100.0f);
				telem_data.cmd_pitch_deg_x100  = (int16_t)(g_target.pitch * 100.0f);
				telem_data.cmd_yaw_deg_x100    = (int16_t)(g_target.yaw   * 100.0f);

				// Tag which axis this packet is (use sensor_status low 2 bits)
				telem_data.sensor_status       = (telem_data.sensor_status & 0xFC) | (dbg_axis & 0x03);

				// MUX the PID debug fields by axis
				if (dbg_axis == 0) { // Roll rate loop
					telem_data.setpoint     = g_target.rate_roll;
					telem_data.measurement  = g_state.roll_rate;
					telem_data.error        = pid_roll_rate.previous_error;
					telem_data.p_term       = pid_roll_rate.p_out;
					telem_data.i_term       = pid_roll_rate.i_out;
					telem_data.d_term       = pid_roll_rate.d_out;
					telem_data.output_sum   = pid_roll_rate.output;
					telem_data.i_state      = (int16_t)(pid_roll_rate.i_out * 100.0f);
				} else if (dbg_axis == 1) { // Pitch rate loop
					telem_data.setpoint     = g_target.rate_pitch;
					telem_data.measurement  = g_state.pitch_rate;
					telem_data.error        = pid_pitch_rate.previous_error;
					telem_data.p_term       = pid_pitch_rate.p_out;
					telem_data.i_term       = pid_pitch_rate.i_out;
					telem_data.d_term       = pid_pitch_rate.d_out;
					telem_data.output_sum   = pid_pitch_rate.output;
					telem_data.i_state      = (int16_t)(pid_pitch_rate.i_out * 100.0f);
				} else { // Yaw rate loop
					telem_data.setpoint     = g_target.rate_yaw;
					telem_data.measurement  = g_state.yaw_rate;
					telem_data.error        = pid_yaw_rate.previous_error;
					telem_data.p_term       = pid_yaw_rate.p_out;
					telem_data.i_term       = pid_yaw_rate.i_out;
					telem_data.d_term       = pid_yaw_rate.d_out;
					telem_data.output_sum   = pid_yaw_rate.output;
					telem_data.i_state      = (int16_t)(pid_yaw_rate.i_out * 100.0f);
				}

				telem_data.gyro_p              = (int16_t)(g_state.roll_rate  * 100.0f);
				telem_data.gyro_q              = (int16_t)(g_state.pitch_rate * 100.0f);
				telem_data.gyro_r              = (int16_t)(g_state.yaw_rate   * 100.0f);

				telem_data.magic_footer        = 0xAB;
			}

		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_SLAVE;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void start_control(void) {

	switch (StartControlState) {
	case STATE_INIT:
		g_drone_status.flight_mode = 5;
		// --- 1. PRE-FLIGHT CONTROLLER SETUP ---\

		PID_Init(&pid_pos_z, 0.3f, 0.0f, 0.0f, 0.002f, 5.0f);   // Position P gain

		PID_Init(&pid_vel_z, 0.2f, 0.01f, 0.0f, 0.002f, 10.0f); // Velocity PID with I-limit

		PID_Init(&pid_roll_angle,  0.4f, 0.000f, 0.000f, 0.002f, 10.0f);
		PID_Init(&pid_pitch_angle, 0.4f, 0.000f, 0.000f, 0.002f, 10.0f);
		PID_Init(&pid_yaw_angle,   0.3f, 0.000f, 0.000f, 0.002f, 10.0f);

		PID_Init(&pid_roll_rate, 0.1f, 0.0f, 0.000f, 0.002f, 0.35f);
		PID_Init(&pid_pitch_rate, 0.1f, 0.0f, 0.000f, 0.002f, 0.35f);
		PID_Init(&pid_yaw_rate, 0.12f, 0.0f, 0.000f, 0.002f, 0.12f);

		
		

		float d_alpha = PID_Calculate_Alpha(20.0f, 0.002f);
		pid_roll_rate.d_low_pass_alpha = d_alpha;
		pid_pitch_rate.d_low_pass_alpha = d_alpha;

		AHRS_Init();

		// Setup filters for Roll, Pitch, and Yaw Gyros
		Biquad_Set_Lowpass(&filter_gyro_pitch, 80.0f, 500.0f);
		Biquad_Set_Lowpass(&filter_gyro_roll,  80.0f, 500.0f);
		Biquad_Set_Lowpass(&filter_gyro_yaw,   80.0f, 500.0f);

		// Turn off the ESC Beep...un-safe, but oh well
		ESC_ArmAll();
		ESC_Disarm();


		// Accel/Mag (I2C1) - Configure Registers
		if (LSM303_Init(&imu, &hi2c1, LSM303_ACCEL_SCALE_2G)) {
			telem_data.sensor_status |= 0x02; // Bit 1: LSM Hardware Found
		}
		if (I3GD20_Init(&i3gd20, &hspi1)) {
			I3GD20_CalibrateZeroRate(&i3gd20, 500); // 1000 samples
			telem_data.sensor_status |= 0x01; // Bit 0: Gyro Ready
		}
		// SPI Begin
		HAL_Delay(200);          // optional “let ESP settle” gate (helps your battery case)
		SPI5_ArmNextFrame();
		// --- 2. HARDWARE INITIALIZATION LOOP ---
		uint8_t sensorInit = 0;
		while (sensorInit == 0)
		{
			g_drone_status.flight_mode = 5;
			// Clear status bits that require "Fresh" verification this frame
			// We keep Bit 0 (Gyro Init) and Bit 1 (LSM Init) if they passed once,
			// but we MUST verify the DATA is fresh.

			// 1. Trigger fresh DMA samples
			LSM303_Process_DMA(&imu);
			Process_Lidar_DMA();

			// 2. Small delay to allow DMA to complete
			HAL_Delay(50);

			if (spi5_frame_done)
			{
				spi5_frame_done = 0;

				uint8_t cmd_work_buf[SPI_FRAME_LEN];
				__disable_irq();
				memcpy(cmd_work_buf, spi_rx_buffer, SPI_FRAME_LEN);
				memset(spi_rx_buffer, 0, SPI_FRAME_LEN);
				__enable_irq();
				/*
				// --- DEBUG PRINT START ---
				// Print to Stimulus Port 0
				printf("CMD: %02X %02X\r\n", spi_rx_buffer[0], spi_rx_buffer[1]);
				// --- DEBUG PRINT END ---
				 */
				// Now it’s safe to re-arm (DMA can reuse spi_rx_buffer)
				SPI5_ArmNextFrame();

				// Only treat as command if it starts with '$'
				if (cmd_work_buf[0] == '$') {
					Process_TELEM_Command(cmd_work_buf, SPI_FRAME_LEN);
				}
			}
			if (spi5_need_rearm) {
				spi5_need_rearm = 0;
				SPI5_ArmNextFrame();
			}

			// 3. Update data only if the hardware has provided a fresh packet
			if (imu.accel_ready) {
				uint8_t accel_snap[6];
				__disable_irq();
				memcpy(accel_snap, imu.accel_raw, 6);
				imu.accel_ready = false;
				__enable_irq();

				raw_data.ax = (int16_t)((accel_snap[1] << 8) | accel_snap[0]) * imu.accel_g_per_lsb;
				raw_data.ay = (int16_t)((accel_snap[3] << 8) | accel_snap[2]) * imu.accel_g_per_lsb;
				raw_data.az = (int16_t)((accel_snap[5] << 8) | accel_snap[4]) * imu.accel_g_per_lsb;

				// This bit now means: "I have a fresh, valid gravity sample"
				telem_data.sensor_status |= 0x02;
			}
			if (imu.mag_ready) {
				uint8_t mag_snap[6];
				__disable_irq();
				memcpy(mag_snap, imu.mag_raw, 6);
				imu.mag_ready = false;
				__enable_irq();

				// 1. Parse based on variant (DLHC vs AGR)
				if (imu.variant == LSM303_DLHC) {
					raw_data.mx = (int16_t)((mag_snap[0] << 8) | mag_snap[1]) * imu.mag_gauss_per_lsb;
					raw_data.mz = (int16_t)((mag_snap[2] << 8) | mag_snap[3]) * imu.mag_gauss_per_lsb;
					raw_data.my = (int16_t)((mag_snap[4] << 8) | mag_snap[5]) * imu.mag_gauss_per_lsb;
				} else {
					raw_data.mx = (int16_t)((mag_snap[1] << 8) | mag_snap[0]) * imu.mag_gauss_per_lsb;
					raw_data.my = (int16_t)((mag_snap[3] << 8) | mag_snap[2]) * imu.mag_gauss_per_lsb;
					raw_data.mz = (int16_t)((mag_snap[5] << 8) | mag_snap[4]) * imu.mag_gauss_per_lsb;
				}

				// This bit confirms we are receiving data packets
				telem_data.sensor_status |= 0x10; // Let's use Bit 4 (0x10) for Mag Health
			}
			// 4. Verification Gate
			if (telem_data.sensor_status == 0x1B) { // 0x1B = Mag + Lidar + Accel + Gyro

				// Gravity Vector Check
				float accel_mag = sqrtf(raw_data.ax*raw_data.ax + raw_data.ay*raw_data.ay + raw_data.az*raw_data.az);

				// Magnetic Field Magnitude Check
				float mag_field_strength = sqrtf(raw_data.mx*raw_data.mx + raw_data.my*raw_data.my + raw_data.mz*raw_data.mz);

				bool gravity_ok = (accel_mag > 0.85f && accel_mag < 1.15f);
				bool mag_ok = (mag_field_strength > 0.2f && mag_field_strength < 0.9f);

				if (gravity_ok && mag_ok) {
					sensorInit = 1; // Success! Exit loop
					//printf("ALL SYSTEMS GO: G=%.2fg, Mag=%.2f Gauss\r\n", accel_mag, mag_field_strength);
				} else {
					if (!gravity_ok)
						if (!mag_ok)
							HAL_Delay(200);
				}
			}
			HAL_Delay(50);
			telem_data.header = 0xDEADBEEF;       // UINT32        // float 1
			telem_data.altitude = range_dist_cm;  // float 5 (Raw Lidar in cm)
			telem_data.voltage = 15.0f;           // float 6
			telem_data.armed = is_system_armed ? 0xFF : 0x00;
			// Map the modes to the telemetry packet
			telem_data.drone_mode  = g_drone_status.drone_mode;
			telem_data.flight_mode = g_drone_status.flight_mode;

			telem_data.magic_footer = 0xAB;       // UINT8
			telem_data.header = 0xDEADBEEF;       // Redundant header as per your code
		}

		StartControlState = STATE_MODE_SEL;
		break;

	case STATE_TUNE:
	{
		// =========================
		// SAFETY POLICY
		// =========================
		// If you want "bench tune" (no motors), keep this.
		// If you want "in-air tune", require is_system_armed==1 and props on, etc.
		const bool allow_motor_output = (is_system_armed != 0);

		uint32_t now_ms = HAL_GetTick();

		// Persistent supervisor state
		static uint8_t entered = 0;
		static TuneAxis_t axis = AXIS_PITCH;
		static uint16_t telem_decimator = 0;   // 1kHz -> 100Hz telemetry
		static uint32_t last_1khz = 0;



		// -------------------------
		// One-time entry init
		// -------------------------
		if (!entered) {
			entered = 1;
			axis = AXIS_PITCH;
			telem_decimator = 0;
			g_state.isTuning = true;
			telem_data.flight_mode = 0x07; // TUNING
			// Reset logs
			log_idx = 0;
			dump_idx = 0;
			is_logging = false;
			is_dumping = false;

			// Reset tune FSM + kick it
			state_tune = TUNE_IDLE;
			tuning_triggered = 1;

			// Zero setpoints
			g_target.rate_pitch = 0.0f;
			g_target.rate_roll  = 0.0f;
			g_target.rate_yaw   = 0.0f;

			// Optional: force attitude hold targets to 0
			// g_target.roll = 0; g_target.pitch = 0; g_target.yaw = 0;

			// Timing
			last_1khz = now_ms;

		}

		// -------------------------
		// 1 kHz loop gate
		// -------------------------
		if ((now_ms - last_1khz) >= 1) {
			float dt = (now_ms - last_1khz) * 0.001f;
			dt = clampf(dt, 0.001f, 0.01f);
			last_1khz = now_ms;

			g_state.dt_sec = dt;

			telem_decimator++;

			// =========================
			// 1) SENSE / UPDATE STATE
			// =========================
			if (i3gd20.initialized && I3GD20_ReadGyro(&i3gd20, &gyro_raw)) {
				float gx_raw = gyro_raw.gx * i3gd20.dps_per_lsb;
				float gy_raw = gyro_raw.gy * i3gd20.dps_per_lsb;
				float gz_raw = gyro_raw.gz * i3gd20.dps_per_lsb;

				// 2. BIQUAD FILTERING (THE NEW STEP)
				raw_data.gx = Biquad_Process(&filter_gyro_roll,  gx_raw);
				raw_data.gy = Biquad_Process(&filter_gyro_pitch, gy_raw);
				raw_data.gz = Biquad_Process(&filter_gyro_yaw,   gz_raw);
			}

			// accel helps AHRS stability during pokes
			LSM303_Process_DMA(&imu);
			if (imu.accel_ready) {
				uint8_t snap[6];
				__disable_irq();
				memcpy(snap, imu.accel_raw, 6);
				imu.accel_ready = false;
				__enable_irq();

				raw_data.ax = (int16_t)((snap[1] << 8) | snap[0]) * imu.accel_g_per_lsb;
				raw_data.ay = (int16_t)((snap[3] << 8) | snap[2]) * imu.accel_g_per_lsb;
				raw_data.az = (int16_t)((snap[5] << 8) | snap[4]) * imu.accel_g_per_lsb;
			}

			AHRS_Update(&raw_data, &g_state, dt);

			// =========================
			// 2) SELECT ACTIVE AXIS
			// =========================
			float *sp = NULL;
			float rate_meas = 0.0f;

			if (axis == AXIS_PITCH) {
				sp = &g_target.rate_pitch;
				rate_meas = g_state.pitch_rate;
				g_target.rate_roll = 0.0f;
				g_target.rate_yaw  = 0.0f;
			} else if (axis == AXIS_ROLL) {
				sp = &g_target.rate_roll;
				rate_meas = g_state.roll_rate;
				g_target.rate_pitch = 0.0f;
				g_target.rate_yaw   = 0.0f;
			} else {
				// finished all axes
				g_state.isTuning = false;
				StartControlState = STATE_DATA_DUMP;
				entered = 0; // so tune can be run again later
				break;
			}

			// =========================
			// 3) RUN AUTOTUNE STEP (updates *sp)
			// =========================
			bool axis_done = run_autotune_step(sp, rate_meas);
			static TuneState_t prev_tune_state = TUNE_IDLE;

			// Detect the exact moment we transition INTO a pulse
			if (state_tune == TUNE_INJECT_PULSE && prev_tune_state != TUNE_INJECT_PULSE) {
				is_logging = true;
				log_idx = 0; // Fresh start for this magnitude
			}
			prev_tune_state = state_tune;
			// =========================
			// 4) CONTROL OUTPUT POLICY
			// =========================
			if (allow_motor_output) {
				telem_data.sat_flags = FlightLogic_Update(&g_state, &g_target, &g_drone_status);
			} else {
				// hard force motors to 0 while tuning (optional)
				for (int i = 1; i <= 4; i++) {
					ESC_SetThrottle(get_timer_channel(i), 0.0f);
				}
			}

			// =========================
			// 5) LOGGING (1kHz, 600 samples)
			// =========================
			if (is_logging && log_idx < TUNE_LOG_SIZE) {
				gyro_log[log_idx]   = rate_meas;  // measured response
				target_log[log_idx] = *sp;        // commanded poke
				log_idx++;
			}

			// =========================
			// 6) AXIS COMPLETE -> ADVANCE
			// =========================
			if (axis_done) {
				is_logging = false;

				// Pad log if we didn’t fill the entire window
				for (; log_idx < TUNE_LOG_SIZE; log_idx++) {
					gyro_log[log_idx]   = gyro_log[(log_idx > 0) ? (log_idx - 1) : 0];
					target_log[log_idx] = 0.0f;
				}

				// Next axis
				if (axis == AXIS_PITCH) {
					axis = AXIS_ROLL;
					state_tune = TUNE_IDLE;
					tuning_triggered = 1;
					log_idx = 0;
				} else {
					axis = AXIS_DONE;
					g_state.isTuning = false;
					StartControlState = STATE_DATA_DUMP;
					entered = 0;
					for (int i = 1; i <= 4; i++) {
						ESC_SetThrottle(get_timer_channel(i), 0.0f);
					}
				}
			}

			// =========================
			// 7) TELEMETRY @ 100Hz
			// =========================
			if (telem_decimator >= 10) {
				telem_decimator = 0;

				// Put useful live signals in packet for your plotter
				telem_data.setpoint    = *sp;
				telem_data.measurement = rate_meas;

				PIDController* pid_active = &pid_pitch_rate;
				if (axis == AXIS_ROLL) pid_active = &pid_roll_rate;

				telem_data.error      = pid_active->previous_error;
				telem_data.p_term     = pid_active->p_out;
				telem_data.i_term     = pid_active->i_out;
				telem_data.d_term     = pid_active->d_out;
				telem_data.output_sum = pid_active->output;
				// send one frame
				SPI5_ArmNextFrame();
			}
		}
	}
	break;


	case STATE_DATA_DUMP:
	{
		telem_data.flight_mode = 254;
		uint32_t now = HAL_GetTick();
		if (now - last >= 10) {
			last = now;

			telem_data.timestamp   = (float)dump_idx;
			telem_data.pitch       = gyro_log[dump_idx];
			telem_data.voltage     = target_log[dump_idx];
			telem_data.flight_mode = 0xFE;

			SPI5_ArmNextFrame();

			dump_idx++;

			if (dump_idx >= TUNE_LOG_SIZE) {
				dump_idx = 0;
				StartControlState = STATE_MODE_SEL;
			}
		}
		break;
	}

	case STATE_MODE_SEL: {
		static uint8_t entered = 0;
		static uint32_t mode_sel_last_ms = 0;
		if (!entered) {
			g_drone_status.drone_mode = 0;
			entered = 1;
			mode_sel_last_ms = HAL_GetTick();
			SPI5_ArmNextFrame();
		}

		telem_data.flight_mode = 0x06; // Signal HUD we are in Mode Select
		uint8_t do_rearm = 0;

		// 1. Check for hardware errors
		if (spi5_need_rearm) {
			spi5_need_rearm = 0;
			do_rearm = 1;
		}
		uint32_t now = HAL_GetTick();

		if (telemCMDPulse){
			// Has 1000ms passed?
			g_drone_status.flight_mode = 91;
			if ((now - telemCMDTimeStart) > CMDpulseTime) {
				telemCMDPulse = false;
				ESC_SetThrottle(TIM_CHANNEL_1, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_2, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_3, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_4, 0.0f);
			}
		}

		// 2. Process incoming SPI frames
		if (spi5_frame_done) {
			spi5_frame_done = 0;

			uint8_t cmd_work_buf[SPI_FRAME_LEN];
			__disable_irq();
			memcpy(cmd_work_buf, spi_rx_buffer, SPI_FRAME_LEN);
			memset(spi_rx_buffer, 0, SPI_FRAME_LEN);
			__enable_irq();

			// Check for commands EVERY frame to ensure responsiveness
			if (cmd_work_buf[0] == '$') {
				Process_TELEM_Command(cmd_work_buf, SPI_FRAME_LEN);
			}

			do_rearm = 1;
		}

		if (do_rearm) {
			SPI5_ArmNextFrame();
		}




		telem_data.timestamp  = g_state.dt_sec;
		telem_data.roll       = g_state.roll;
		telem_data.pitch      = g_state.pitch;
		telem_data.yaw        = g_state.yaw;
		telem_data.altitude   = range_dist_cm;
		telem_data.armed      = is_system_armed ? 0xFF : 0x00;
		telem_data.drone_mode = g_drone_status.drone_mode;

		// --- RESTORED STATE MACHINE CRITERIA ---

		// Criterion A: Autotune Request
		if (tune_request) {
			tune_request = 0;
			entered = 0;
			StartControlState = STATE_TUNE;
			break;
		}

		// Criterion B: Primary Mode Selection
		// Fixed: Using your global instance g_drone_status
		if (g_drone_status.drone_mode != 0) {
			StartControlState = STATE_MODE_SEL_COMPLETE;
			mode_sel_last_ms = 0;
			entered = 0;
		}
		break;
	}
	case STATE_MODE_SEL_COMPLETE:
		// do nothing
		break;
	}

}
static void SPI5_ArmNextFrame(void)
{

	// --------------------------------
	__HAL_SPI_CLEAR_OVRFLAG(&hspi5);
	StageActiveTelemetryFrame();

	// 3. Clear RX buffer
	spi_rx_buffer[0] = 0;

	// 5. Arm the DMA
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi5,
			spi_tx_buf,
			spi_rx_buffer,
			SPI_FRAME_LEN);

	// 6. VISUAL DEBUG
	if (status != HAL_OK) {
		// If this lights up, the OVR clear didn't work or DMA is broken
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
	}

	spi5_last_arm_tick = HAL_GetTick();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		LSM303_XferCpltCallback(&imu, false); // false = This was a Transmit (TX)
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		LSM303_XferCpltCallback(&imu, true);  // true = This was a Receive (RX)
	}
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI5)
	{
		spi5_frame_done = 1;
		spi5_frame_counter++;


	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI5) {
		// Abort the current failed transfer to clear hardware busy flags
		HAL_SPI_Abort(hspi);
		hspi->State = HAL_SPI_STATE_READY;
		spi5_need_rearm = 1;
	}
}

void Process_TELEM_Command(uint8_t* Buf, uint32_t Len) {
	// 1. ESP8266 marks all valid commands with '$' at index 0
	if (Buf[0] != '$') return;

	// 2. Create local copy and IMMEDIATELY clear the hardware buffer
	// This prevents race conditions and "echoed" commands.
	char local_buf[SPI_FRAME_LEN + 1];
	uint32_t n = (Len > SPI_FRAME_LEN) ? SPI_FRAME_LEN : Len;
	memcpy(local_buf, Buf, n);
	local_buf[n] = '\0';
	memset(Buf, 0, Len);

	last_heartbeat_tick = HAL_GetTick();

	// The actual command string starts at index 1 (after the '$')
	char* cmd = &local_buf[1];

	// 3. Token-Based Switch Switchboard
	switch (cmd[0]) {

	case 'x': case 'X': // --- EMERGENCY STOP ---
		ESC_Disarm();
		is_estop_active = 1;
		is_system_armed = 0;
		g_mission.landing_start_t = (float)HAL_GetTick() / 1000.0f;
		if (g_mission.current_index < g_mission.total_waypoints) {
			g_mission.waypoints[g_mission.current_index].action = WP_ACTION_LAND;
		}
		break;

	case 'a': // --- ARM or ALL ---
		if (cmd[1] == 'r' && cmd[2] == 'm') { // "arm"
			ESC_ArmAll();
		}
		else if (cmd[1] == 'l' && cmd[2] == 'l') { // "all p25.0"
			char* t_ptr = strchr(cmd, 'p');
			if (t_ptr != NULL) {
				char* val_start = t_ptr + 1;
				while (*val_start == ' ') val_start++;

				float throttle = strtof(val_start, NULL);
				float motor_scales[4] = {1.0f, 1.0f, 1.0f, 1.0f};
				Mixer_GetMotorScales(motor_scales);
				ESC_SetThrottle(TIM_CHANNEL_1, throttle * motor_scales[0]);
				ESC_SetThrottle(TIM_CHANNEL_2, throttle * motor_scales[1]);
				ESC_SetThrottle(TIM_CHANNEL_3, throttle * motor_scales[2]);
				ESC_SetThrottle(TIM_CHANNEL_4, throttle * motor_scales[3]);
			}
		}
		break;

	case 'm': // --- MOTOR or MODE ---
		// Check if it's "mode"
		if (cmd[1] == 'o' && cmd[2] == 'd') { // "mode 1|2|3"
			char* val_ptr = strchr(cmd, ' ');
			if (val_ptr != NULL) {
				int requested_mode = atoi(val_ptr + 1);
				if (requested_mode == MODE_MANUAL_LEVEL ||
						requested_mode == MODE_MISSION ||
						requested_mode == MODE_THRUST_STAND) {
					g_drone_status.drone_mode = (uint8_t)requested_mode;
				} else {
					// Unsupported/reserved mode IDs (including obsolete mode 4) are ignored
					unknownTelemCMD_counter += 1;
				}
			} else {
				unknownTelemCMD_counter += 1;
			}
		}
		// Check if it's "motor X tY" (e.g. "m1 p10" or "motor 1 p10")
		else {
			// Find the first digit in the string to identify motor number
			int motor_num = 0;
			for(int i=0; i<10; i++) {
				if(isdigit((unsigned char)cmd[i])) {
					motor_num = cmd[i] - '0';
					break;
				}
			}
			if (strchr(cmd, 'p')){
				char* t_ptr = strchr(cmd, 'p');
				if (t_ptr != NULL && motor_num >= 1 && motor_num <= 4) {
					// Move pointer past 'p', then skip any spaces
					char* val_start = t_ptr + 1;
					while (*val_start == ' ') val_start++;

					float throttle = strtof(val_start, NULL);
					ESC_SetThrottle(get_timer_channel(motor_num), throttle);
				}
			}
			// Pulse motor for 100ms
			else if (strchr(cmd, 'r')) {
				char* t_ptr = strchr(cmd, 'r');
				if (t_ptr != NULL && motor_num >= 1 && motor_num <= 4) {
					// Move pointer past 't', then skip any spaces
					char* val_start = t_ptr + 1;
					while (*val_start == ' ') val_start++;

					float throttle = strtof(val_start, NULL);
					ESC_SetThrottle(get_timer_channel(motor_num), throttle);
					telemCMDPulse = true;
					telemCMDTimeStart = HAL_GetTick();
					g_drone_status.flight_mode = 91;
				}
			}
			else {
				unknownTelemCMD_counter += 1;
			}
		}
		break;

	case 'z': case 'Z':
		g_drone_status.drone_mode = 1;
		commandZ = strtof(cmd + 1, NULL);
		g_target.z = commandZ;
		break;

	case 't': // --- TUNE ---
		if (cmd[1] == 'u') {
			tune_request = 1;
		} else if (cmd[1] == 'p') {
			char* val_ptr = strchr(cmd, ' ');
			if (val_ptr != NULL) {
				int requested_loop = atoi(val_ptr + 1);
				if (requested_loop == PID_TUNE_LOOP_ROLL_RATE ||
						requested_loop == PID_TUNE_LOOP_PITCH_RATE ||
						requested_loop == PID_TUNE_LOOP_YAW_RATE) {
					g_pid_tune_loop = (pidTuneLoop_t)requested_loop;
					g_telem_mode = TELEM_MODE_PID_TUNE;
				} else {
					unknownTelemCMD_counter += 1;
				}
			} else {
				unknownTelemCMD_counter += 1;
			}
		} else if (cmd[1] == 'n') {
			g_telem_mode = TELEM_MODE_NORMAL;
		} else {
			unknownTelemCMD_counter += 1;
		}
		break;
	}
}
/**
 * @brief Helper to map Motor ID 1-4 to TIM_CHANNEL_x
 */
uint32_t get_timer_channel(int motor_num) {
	switch(motor_num) {
	case 1:  return TIM_CHANNEL_1;
	case 2:  return TIM_CHANNEL_2;
	case 3:  return TIM_CHANNEL_3;
	case 4:  return TIM_CHANNEL_4;
	default: return TIM_CHANNEL_1;
	}
}

void ESC_SetThrottle(uint32_t channel, float percentage) {
	// 1. Safety Clamping
	if (percentage < 0.0f) percentage = 0.0f;
	if (percentage > 100.0f) percentage = 100.0f;

	// 2. Linear Mapping: 0-100% -> 1000-2000us
	// Formula: Pulse = 1000 + (Percent * 10)
	uint32_t pulse = (uint32_t)(ESC_MIN_PULSE + (percentage * (ESC_MAX_PULSE - ESC_MIN_PULSE) / 100.0f));

	// 3. Hardware Register Update
	__HAL_TIM_SET_COMPARE(&htim3, channel, pulse);

	// 4. Update Telemetry Data (Fixed Syntax)
	if (channel == TIM_CHANNEL_1) {
		telem_data.motor1_T = percentage;
	} else if (channel == TIM_CHANNEL_2) {
		telem_data.motor2_T = percentage;
	} else if (channel == TIM_CHANNEL_3) {
		telem_data.motor3_T = percentage;
	} else if (channel == TIM_CHANNEL_4) {
		telem_data.motor4_T = percentage;
	}
}

void ESC_SetThrottle_Safe(uint32_t channel, float percentage) {
	if (in_diagnostic_mode && percentage > DIAGNOSTIC_MAX_THROTTLE) {
		percentage = DIAGNOSTIC_MAX_THROTTLE;
	}
	ESC_SetThrottle(channel, percentage);
}

void Trigger_Motor_Pulse(uint32_t channel, float pulse_val, uint32_t duration_ms) {
	diagnostic_pulse.channel = channel;
	diagnostic_pulse.pulse_throttle = pulse_val;
	diagnostic_pulse.duration = duration_ms;
	diagnostic_pulse.start_time = HAL_GetTick();
	diagnostic_pulse.active = true;

	// Immediate physical update
	ESC_SetThrottle(channel, pulse_val);
}

void Update_Pulse_Manager(void) {
	if (!diagnostic_pulse.active) return;

	if (HAL_GetTick() - diagnostic_pulse.start_time >= diagnostic_pulse.duration) {
		// Time is up! Return to safety baseline
		ESC_SetThrottle(diagnostic_pulse.channel, 0.0f);
		diagnostic_pulse.active = false;
	}
}

bool run_autotune_step(float *rate_setpoint, float current_rate)
{
	static uint32_t state_timer = 0;
	static float max_rate_observed = 0.0f;
	static const float magnitudes[] = {10.0f, -10.0f, 20.0f, -20.0f, 30.0f, -30.0f};
	static int mag_idx = 0;

	switch (state_tune)
	{
	case TUNE_IDLE:
		*rate_setpoint = 0.0f;
		if (tuning_triggered) {
			tuning_triggered = 0;
			state_tune = TUNE_INJECT_PULSE;
			state_timer = HAL_GetTick();
			max_rate_observed = 0.0f;
		}
		return false;

	case TUNE_INJECT_PULSE:
		*rate_setpoint = magnitudes[mag_idx];
		if (fabsf(current_rate) > fabsf(max_rate_observed)) max_rate_observed = current_rate;

		if (HAL_GetTick() - state_timer >= 80) { // 80ms is perfect
			state_tune = TUNE_MEASURE_WAIT;
			state_timer = HAL_GetTick();
			*rate_setpoint = 0.0f;
		}
		return false;

	case TUNE_MEASURE_WAIT:
		*rate_setpoint = 0.0f;
		// Raise threshold to 5.0 to ignore ground vibration "jitter"
		if (fabsf(current_rate) < 5.0f || (HAL_GetTick() - state_timer >= 300)) {
			state_tune = TUNE_CALCULATE;
		}
		return false;

	case TUNE_CALCULATE:
		mag_idx++;
		state_tune = TUNE_COOLDOWN;
		state_timer = HAL_GetTick();
		return false;

	case TUNE_COOLDOWN:
		*rate_setpoint = 0.0f;
		// 300ms is plenty for a 5" or smaller drone to stop ringing
		if (HAL_GetTick() - state_timer >= 300) {
			if (mag_idx >= 6) {
				mag_idx = 0;
				state_tune = TUNE_IDLE;
				return true; // Axis Done
			} else {
				state_tune = TUNE_IDLE;
				tuning_triggered = 1; // Trigger next magnitude immediately
				return false;
			}
		}
		return false;
	}
	return false;
}


void ESC_ArmAll(void) {
	// Set all channels to 1000us (0%)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_ARM_PULSE); // PC6
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_ARM_PULSE); // PB5
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_ARM_PULSE); // PC8
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_ARM_PULSE); // PC9
	// Zero active flight-control PID memory before enabling outputs
	ResetActiveFlightPIDsFromState(&g_state);
	// Enable PWM Generation
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	// Wait for ESC Init Beeps (standard BLHeli startup is ~2-3 seconds)
	HAL_Delay(3000);
	is_system_armed = 1;
	//IDLE Motors
	target_throttle = 8;
	ESC_SetThrottle(TIM_CHANNEL_1, target_throttle);
	ESC_SetThrottle(TIM_CHANNEL_2, target_throttle);
	ESC_SetThrottle(TIM_CHANNEL_3, target_throttle);
	ESC_SetThrottle(TIM_CHANNEL_4, target_throttle);
	telem_data.armed = 0xFF;
	telem_data.motor1_T = target_throttle;
	telem_data.motor2_T = target_throttle;
	telem_data.motor3_T = target_throttle;
	telem_data.motor4_T = target_throttle;
	HAL_Delay(3000);

}
void ESC_Disarm(void) {
	// Turn off the ESC Beep
	g_drone_status.flight_mode = 0; // 0 = DISARMED/IDLE/ONGROUND
	g_drone_status.armed = 0;
	// Force immediate hardware override to 0%
	for(int i = 1; i <= 4; i++) {
		ESC_SetThrottle(get_timer_channel(i), 0.0f);
	}
	is_system_armed = 0;
	HAL_Delay(100);
}
void Process_Lidar_DMA(void) {
	uint8_t write_idx = (LIDAR_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) % LIDAR_BUF_SIZE;	telem_data.sensor_status |= 0x08;
	while (lidar_read_idx != write_idx) {
		uint8_t b = lidar_dma_buffer[lidar_read_idx];
		switch(tf_state) {
		case 0: if (b == 0x59) { tf_state = 1; tf_checksum = 0x59; } break;
		case 1: if (b == 0x59) { tf_state = 2; tf_idx = 0; tf_checksum += 0x59; } else tf_state = 0; break;
		case 2:
			tf_buf[tf_idx++] = b;
			if (tf_idx < 7) tf_checksum += b;
			if (tf_idx >= 7) {
				if ((uint8_t)(tf_checksum & 0xFF) == tf_buf[6]) {
					uint16_t dist_raw = tf_buf[0] + (tf_buf[1] << 8);
					range_dist_cm = (float)dist_raw;
				}
				tf_state = 0;
			}
			break;
		}
		lidar_read_idx++;
		if (lidar_read_idx >= LIDAR_BUF_SIZE) lidar_read_idx = 0;
	}
}

static void I2C1_Scan(void) {
	for (uint8_t addr = 1; addr < 0x7F; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 5) == HAL_OK) {
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
