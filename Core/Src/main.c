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
#include "kalman.h"
#include <math.h>
#include <stdio.h>   // For printf
#include <stdlib.h>  // For atof, atoi
#include <string.h>  // For string manipulation
#include "state.h"       // Defines VehicleState, TargetState
#include "AHRS.h"        // Defines ahrsSensor_t and AHRS_Update
#include "navigation.h"  // Defines MissionManager and Navigation_GetTarget
#include "flight_logic.h"// Defines FlightLogic_Update
#include <float.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
	uint8_t magic_footer;
} Telemetry_Packet_t;

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
I2C_HandleTypeDef hi2c1;
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
volatile uint8_t is_estop_active = 0; // 0: Normal, 1: Emergency Landing

typedef enum { READ_ACCEL, READ_MAG } sensor_state_t;


// Define a test mission:

Waypoint mission_waypoints[] = {
		// { {x, y, z, yaw}, toa, hover_duration, tolerance, action }
		{{0.0f, 0.0f, 1.0f, 0.0f},  0.0f, 0.0f, 0.10f, WP_ACTION_MOVE}, // Step 1: Target 1m height
		{{0.0f, 0.0f, 1.0f, 0.0f},  0.0f, 5.0f, 0.15f, WP_ACTION_HOVER},// Step 2: Target (1,1,1) then hover
		{{0.0f, 0.0f, 2.0f, 0.0f},  0.0f, 0.0f, 0.10f, WP_ACTION_MOVE},
		{{0.0f, 0.0f, 2.0f, 0.0f},  0.0f, 5.0f, 0.15f, WP_ACTION_HOVER},
		{{0.0f, 0.0f, 0.0f, 0.0f},  0.0f, 0.0f, 0.05f, WP_ACTION_LAND}  // Step 3: Land
};
uint16_t total_wp_count = 5;
//TIM3 > APB2 > Motor PWM Control
// Global instances for the Flight Stack
vehicleState_t  g_state;      // The current estimated state (Kinematics)
targetState_t   g_target;     // The desired state (Setpoints)
MissionManager g_mission;    // The mission and waypoint sequencer
droneState_t g_drone_status; // Global metadata for modes, battery, and status



volatile float target_throttle = 0.0f; // The throttle we want


Telemetry_Packet_t telem_data;
static volatile uint8_t spi5_frame_done = 0;
static volatile uint32_t spi5_frame_counter = 0;
static uint32_t spi5_last_arm_tick = 0;
//uint8_t spi_rx_buffer[sizeof(Telemetry_Packet_t)];
uint8_t command_ready = 0;
//volatile uint8_t spi5_busy = 0;

LSM303 imu;
LSM303_Raw raw;
I3GD20 i3gd20;
I3GD20_Raw gyro_raw;

uint8_t tf_state = 0;
uint8_t tf_buf[9];
uint8_t tf_idx = 0;
uint16_t tf_checksum = 0;

Kalman_t kf_roll;
Kalman_t kf_pitch;
Kalman_t kf_yaw;

static ahrsSensor_t raw_data;

// PID controllers
PIDController pid_roll;
PIDController pid_pitch;
PIDController pid_yaw;
PIDController pid_pos_z; // The outer loop (Altitude)
PIDController pid_vel_z; // The inner loop (Vertical Velocity)
// LIDAR Vars
uint8_t lidar_dma_buffer[LIDAR_BUF_SIZE];
uint8_t lidar_read_idx = 0;
volatile float range_dist_cm = 0.0f;

// Telem Vars
Telemetry_Packet_t telem_data;
uint8_t spi_rx_buffer[sizeof(Telemetry_Packet_t)]; // For incoming commands from ESP8266

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
/* USER CODE BEGIN PFP */
void Vehicle_State_Init(droneState_t* state);
static void SPI5_ArmNextFrame(void);

//SPI1 GYRO
//I2C1 ACCEL/MAG
//UART2 ST-LINK
//SPI5 ESP8826

/* Custom ESC Control Functions */
void ESC_ArmAll(void);
void ESC_SetThrottle(uint32_t channel, float percentage);
uint32_t get_timer_channel(int motor_num);
void Process_TELEM_Command(uint8_t* Buf, uint32_t Len);

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
	/* USER CODE BEGIN 2 */
	printf("\r\n=== FLIGHT CONTROLLER BOOT ===\r\n");


	// --- 1. Initialize Telemetry Structure ---
	telem_data.header = 0xDEADBEEF;
	telem_data.magic_footer = 0xAB;
	telem_data.timestamp = 0.0f;
	telem_data.voltage = 12.0f;
	telem_data.sensor_status = 0;
	telem_data.armed = 0x00;


	// Fill with test pattern to verify DMA is reading memory
	// memset(&telem_data.roll, 0xAA, 12);


	// --- 3. Start Lidar DMA ---
	HAL_UART_Receive_DMA(&huart1, lidar_dma_buffer, LIDAR_BUF_SIZE);
	HAL_Delay(100);
	printf("Lidar DMA Started\r\n");


	I2C1_Scan();

	// --- 1. PRE-FLIGHT CONTROLLER SETUP ---
	// This only happens once
	PID_Init(&pid_roll,  1.0f, 0.0f, 0.0f, 0.01f, 100.0f);
	PID_Init(&pid_pitch, 1.0f, 0.0f, 0.0f, 0.01f, 100.0f);
	PID_Init(&pid_yaw,   1.0f, 0.0f, 0.0f, 0.01f, 100.0f);
	PID_Init(&pid_pos_z, 1.5f, 0.0f, 0.0f, 0.01f, 0.0f);   // Position P gain
	PID_Init(&pid_vel_z, 2.0f, 0.5f, 0.1f, 0.01f, 50.0f); // Velocity PID with I-limit

	Kalman_Init(&kf_roll,  0.003f, 0.03f);
	Kalman_Init(&kf_pitch, 0.003f, 0.03f);
	Kalman_Init(&kf_yaw,   0.005f, 0.1f);


	// Accel/Mag (I2C1) - Configure Registers
	if (LSM303_Init(&imu, &hi2c1, LSM303_ACCEL_SCALE_2G)) {
		telem_data.sensor_status |= 0x02; // Bit 1: LSM Hardware Found
	}
	if (I3GD20_Init(&i3gd20, &hspi1)) {
		printf("Calibrating Gyro... DO NOT MOVE\r\n");
		I3GD20_CalibrateZeroRate(&i3gd20, 1000); // 1000 samples
		telem_data.sensor_status |= 0x01; // Bit 0: Gyro Ready
	}
	// SPI Begin
	HAL_Delay(200);          // optional “let ESP settle” gate (helps your battery case)
	SPI5_ArmNextFrame();
	printf("SPI5 DMA armed (per-frame mode)\r\n");
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

			uint8_t cmd_work_buf[sizeof(Telemetry_Packet_t)];
			__disable_irq();
			memcpy(cmd_work_buf, spi_rx_buffer, sizeof(cmd_work_buf));
			__enable_irq();

			// Now it’s safe to re-arm (DMA can reuse spi_rx_buffer)
			SPI5_ArmNextFrame();

			// Only treat as command if it starts with '$'
			if (cmd_work_buf[0] == '$') {
				Process_TELEM_Command(cmd_work_buf, sizeof(cmd_work_buf));
			}
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
			bool mag_ok = (mag_field_strength > 0.2f && mag_field_strength < 0.8f);

			if (gravity_ok && mag_ok) {
				sensorInit = 1; // Success! Exit loop
				//printf("ALL SYSTEMS GO: G=%.2fg, Mag=%.2f Gauss\r\n", accel_mag, mag_field_strength);
			} else {
				if (!gravity_ok) printf("REJECT: Gravity %.2fg out of range.\r\n", accel_mag);
				if (!mag_ok) printf("REJECT: Magnetic Field %.2fG out of range. Check for interference.\r\n", mag_field_strength);
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
	printf("Initialization Complete. Entering Control Loop.\r\n");
	float last = 0.00f;



	// Loading Mission
	printf("Loading Hover Mission: %d Waypoints\r\n", total_wp_count);
	// Link the waypoints to the manager and provide the current state for the start position
	Navigation_Init(&g_mission, mission_waypoints, total_wp_count, &g_state);

	// Safety: Set the mission start time to the current clock
	g_mission.wp_start_time = (float)HAL_GetTick() / 1000.0f;
	Vehicle_State_Init(&g_drone_status);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (spi5_frame_done)
		{
			spi5_frame_done = 0;

			uint8_t cmd_work_buf[sizeof(Telemetry_Packet_t)];
			__disable_irq();
			memcpy(cmd_work_buf, spi_rx_buffer, sizeof(cmd_work_buf));
			__enable_irq();

			// Now it’s safe to re-arm (DMA can reuse spi_rx_buffer)
			SPI5_ArmNextFrame();

			// Only treat as command if it starts with '$'
			if (cmd_work_buf[0] == '$') {
				Process_TELEM_Command(cmd_work_buf, sizeof(cmd_work_buf));
			}
		}
		// 1. Process Lidar
		Process_Lidar_DMA();

		uint32_t now = HAL_GetTick();
		// 2. Run Control Loop at 100Hz (10ms)
		if (now - last >= 10) {
			float dt_sec = (now - last) / 1000.0f;
			last = now;

			LSM303_Process_DMA(&imu);
			// 1. DATA ACQUISITION
			// Read raw sensor bits into your ahrsSensor_t struct


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
			// --- 3. GYROSCOPE: 1:1 Capture ---
			if (i3gd20.initialized && I3GD20_ReadGyro(&i3gd20, &gyro_raw)) {
				raw_data.gx = gyro_raw.gx * i3gd20.dps_per_lsb;
				raw_data.gy = gyro_raw.gy * i3gd20.dps_per_lsb;
				raw_data.gz = gyro_raw.gz * i3gd20.dps_per_lsb;
			}

			// 2. STATE ESTIMATION (AHRS & Kalman)
			// This internally runs the Kalman Predict/Update and populates g_state
			AHRS_Update(&raw_data, &g_state, dt_sec);

			// Update altitude in the state struct from Lidar (converted to meters)
			g_state.z = range_dist_cm / 100.0f;

			// 3. NAVIGATION (Mission Manager)
			// 2. MISSION LOGIC
			if (is_estop_active) {
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
				if (g_state.z <= 0.3f) {
					PID_Reset(&pid_roll);
					PID_Reset(&pid_pitch);
					PID_Reset(&pid_yaw);
					PID_Reset(&pid_pos_z);
					PID_Reset(&pid_vel_z);
					g_drone_status.flight_mode = 0; // 0 = DISARMED/IDLE/ONGROUND
					// Force immediate hardware override to 0%
					for(int i = 1; i <= 4; i++) {
						ESC_SetThrottle(get_timer_channel(i), 0.0f);
					}
					is_system_armed = 0;
					is_estop_active = 0; // Reset for next boot
				}
			} else {
				// Normal Navigation logic
				Navigation_GetTarget(&g_mission, (float)now / 1000.0f, &g_state, &g_target);
			}

			// 4. FLIGHT CONTROL (Executive Logic)
			// This is where we run PIDs and then the Mixer
			// 3. FLIGHT CONTROL & TELEMETRY
			if (is_system_armed) {
				FlightLogic_Update(&g_state, &g_target);
			}
			// Check if the system is disarmed AND if any motor has a non-zero throttle
			else if (telem_data.motor1_T > 0 || telem_data.motor2_T > 0 ||
					telem_data.motor3_T > 0 || telem_data.motor4_T > 0) {
				telem_data.armed = 0x00;
				// Force motors to 0% only once when transitioning to/remaining in a disarmed state
				for(int i = 1; i <= 4; i++) {
					ESC_SetThrottle(get_timer_channel(i), 0.0f);
				}
				printf("Safety: Motors Zeroed and Latched.\r\n");
			}



			// 5. UPDATE TELEMETRY (Your Exact Atomic Block)
			// Mapping g_state back to your required telemetry variables
			telem_data.header = 0xDEADBEEF;       // UINT32
			telem_data.timestamp = dt_sec;        // float 1
			telem_data.roll = g_state.roll;       // float 2 (Estimated Roll)
			telem_data.pitch = g_state.pitch;     // float 3 (Estimated Pitch)
			telem_data.yaw = g_state.yaw;         // float 4 (Estimated Yaw)
			telem_data.altitude = range_dist_cm;  // float 5 (Raw Lidar in cm)
			telem_data.voltage = 15.0f;           // float 6
			telem_data.armed = is_system_armed ? 0xFF : 0x00;
			// Map the modes to the telemetry packet
			telem_data.drone_mode  = g_drone_status.drone_mode;
			telem_data.flight_mode = g_drone_status.flight_mode;

			telem_data.magic_footer = 0xAB;       // UINT8
			telem_data.header = 0xDEADBEEF;       // Redundant header as per your code
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

static void SPI5_ArmNextFrame(void)
{
	// Ensure telemetry header/footer are always valid before TX
	telem_data.header = 0xDEADBEEF;
	telem_data.magic_footer = 0xAB;

	// Clear RX buffer so stale junk doesn’t look like a command
	spi_rx_buffer[0] = 0;

	// Start one DMA transaction for exactly one frame
	if (HAL_SPI_TransmitReceive_DMA(&hspi5,
			(uint8_t*)&telem_data,
			spi_rx_buffer,
			sizeof(Telemetry_Packet_t)) != HAL_OK)
	{
		// If it fails, you can light an LED or set an error flag
		// printf("SPI5 DMA arm failed: %lu\r\n", hspi5.ErrorCode);
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

void Process_TELEM_Command(uint8_t* Buf, uint32_t Len) {
	// 1. Create a local string copy so we don't mess with the DMA memory
	char local_buf[sizeof(Telemetry_Packet_t) + 1];
	memcpy(local_buf, Buf, (Len > sizeof(Telemetry_Packet_t)) ? sizeof(Telemetry_Packet_t) : Len);
	local_buf[sizeof(Telemetry_Packet_t)] = '\0'; // Force null termination
	// If the buffer is floating high (0xFF) or empty, ignore it
	if (local_buf[0] == 0xFF || local_buf[0] == 0x00) return;
	// 2. Check for empty buffer - if byte 0 is 0, nothing new arrived
	if (local_buf[0] == 0) return;

	last_heartbeat_tick = HAL_GetTick();

	// 3. Process commands from the LOCAL copy
	if (strstr(local_buf, "arm") != NULL) {
		printf("Executing ESC Arming Sequence...\r\n");
		ESC_ArmAll();
		// Instead of memset, we tell the Huzzah we're done by clearing the source
		memset(Buf, 0, Len);
		memset(local_buf, 0, Len);
	}
	else if (strstr(local_buf, "x") != NULL || strstr(local_buf, "X") != NULL) {
		for(uint32_t ch = 1; ch <= 4; ch++) {
			ESC_SetThrottle(get_timer_channel(ch), 0.0f);
		}
		printf("!!! E-STOP RECEIVED: Initiating Emergency Landing !!!\r\n");
		is_estop_active = 1;

		// Force Mission Manager into Landing mode immediately
		g_mission.landing_start_t = (float)HAL_GetTick() / 1000.0f;

		// Optional: Update a specific waypoint action if you want to use the nav logic
		if (g_mission.current_index < g_mission.total_waypoints) {
			g_mission.waypoints[g_mission.current_index].action = WP_ACTION_LAND;
		}
		memset(local_buf, 0, Len);
		memset(Buf, 0, Len);
	}
	else if (strstr(local_buf, "m") != NULL || strstr(local_buf, "M") != NULL) {
		int motor_num = atoi(&local_buf[1]);
		char* t_ptr = strchr(local_buf, 't');
		if (t_ptr != NULL && motor_num >= 1 && motor_num <= 4) {
			float throttle = atof(t_ptr + 1);
			ESC_SetThrottle(get_timer_channel(motor_num), throttle);
			printf("Motor %d -> %.1f%%\r\n", motor_num, throttle);
		}
		memset(Buf, 0, Len);
		memset(local_buf, 0, Len);
	} else if (strstr(local_buf, "all") != NULL || strstr(local_buf, "ALL") != NULL) {

		char* t_ptr = strchr(local_buf, 't');
		if (t_ptr != NULL) {
			float throttle = atof(t_ptr + 1);
			ESC_SetThrottle(TIM_CHANNEL_1, throttle);
			ESC_SetThrottle(TIM_CHANNEL_2, throttle);
			ESC_SetThrottle(TIM_CHANNEL_3, throttle);
			ESC_SetThrottle(TIM_CHANNEL_4, throttle);
			printf("All Motors -> %.1f%%\r\n", throttle);
		}
		memset(Buf, 0, Len);
		memset(local_buf, 0, Len);
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

void ESC_ArmAll(void) {
	// Set all channels to 1000us (0%)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_ARM_PULSE); // PC6
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_ARM_PULSE); // PB5
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_ARM_PULSE); // PC8
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_ARM_PULSE); // PC9

	// Enable PWM Generation
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	// Wait for ESC Init Beeps (standard BLHeli startup is ~2-3 seconds)
	HAL_Delay(3000);
	is_system_armed = 1;
	target_throttle = 5;
	ESC_SetThrottle(TIM_CHANNEL_1, target_throttle);
	ESC_SetThrottle(TIM_CHANNEL_2, target_throttle);
	ESC_SetThrottle(TIM_CHANNEL_3, target_throttle);
	ESC_SetThrottle(TIM_CHANNEL_4, target_throttle);
	telem_data.armed = 0xFF;
	telem_data.motor1_T = target_throttle;
	telem_data.motor2_T = target_throttle;
	telem_data.motor3_T = target_throttle;
	telem_data.motor4_T = target_throttle;

}

void Process_Lidar_DMA(void) {
	uint8_t write_idx = (LIDAR_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) % LIDAR_BUF_SIZE;
	telem_data.sensor_status |= 0x08;
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
	printf("I2C1 scan:\r\n");
	for (uint8_t addr = 1; addr < 0x7F; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 5) == HAL_OK) {
			printf("  - 0x%02X FOUND\r\n", addr);
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
