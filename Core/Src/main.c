/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lsm303.h"
#include "i3gd20.h"
#include "PID.h"
#include <math.h>
#include <stdio.h>   		// For printf
#include <stdlib.h>  		// For atof, atoi
#include <string.h>  		// For string manipulation
#include "state.h"       	// Defines VehicleState, TargetState
#include "AHRS.h"        	// Defines ahrsSensor_t and AHRS_Update
#include "navigation.h"  	// Defines MissionManager and Navigation_GetTarget
#include "flight_logic.h"	// Defines FlightLogic_Update
#include <float.h>
#include "biquadButter.h"
#include <ctype.h>
#include "usbd_cdc_if.h"
#include "mixer.h"
#include "bme280.h"
#include "gt_u7.h"
#include "ibit.h"
#include "calibration.h"
#include "sensor_frames.h"
#include "telemetry.h"
#include "altitudeEst.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SPI_FRAME_LEN 				TELEMETRY_SPI_FRAME_LEN

//_Static_assert(sizeof(Engineer_Packet_t)  == SPI_FRAME_LEN, "Engineer frame != SPI_FRAME_LEN");
static volatile uint8_t spi5_need_rearm = 0;

typedef enum {
	STATE_INIT,
	STATE_MODE_SEL,
	STATE_MODE_SEL_COMPLETE
} StartControlState_t;

typedef enum {
	INIT,
	SPOOLUP,
	TAKEOFF,
	TRANSISTION
} takeoff_t;

#define ALTITUDE_HOVER_THRUST_RAW          63.0f
#define TAKEOFF_BASE_THRUST_START_RAW      70.0f
#define TAKEOFF_BASE_THRUST_END_RAW        85.0f
#define TAKEOFF_BASE_THRUST_RAMP_MS        2000u
#define PBIT_PERIOD_MS                     250u
#define PBIT_REQUIRED_MASK                 (IBIT_SENSOR_GPS | IBIT_SENSOR_ACCEL_MAG | IBIT_SENSOR_BME280 | IBIT_SENSOR_GYRO | IBIT_SENSOR_LIDAR | IBIT_SENSOR_TELEMETRY)
#define PBIT_REQUIRED_SENSOR_STATUS_MASK   (SENSOR_STATUS_GYRO_READY | SENSOR_STATUS_LSM_READY | SENSOR_STATUS_MAG_OK | SENSOR_STATUS_BME280_READY)

#if (HSE_VALUE == 8000000U)
#define AMALTHEIA_PLLM                     4U
#elif (HSE_VALUE == 12000000U)
#define AMALTHEIA_PLLM                     6U
#else
#error "Unsupported HSE_VALUE. Configure AMALTHEIA_PLLM for the board oscillator."
#endif

float altitude_base_thrust_raw = ALTITUDE_HOVER_THRUST_RAW;

uint32_t takeoff_count = 1;

// Now declare the actual variables
StartControlState_t StartControlState = STATE_INIT;
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
#define ESC_PWM_CHANNEL               TIM_CHANNEL_1 //PC6
#define ESC_PWM_MIN_US                1000U
#define ESC_PWM_MAX_US                2000U
#define ESC_PWM_ARM_US                ESC_PWM_MIN_US
#define ESC_PWM_TIMER_PSC             95U
#define ESC_PWM_TIMER_ARR             1999U
#define LIDAR_BUF_SIZE 128
#define GPS_BUF_SIZE 512

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
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile uint8_t is_system_armed = 0; // 0: Locked, 1: Armed
volatile uint32_t last_heartbeat_tick = 0; // Tracks last valid command
volatile uint8_t is_estop_active = 0; // 0: Normal, 1: Emergency STOP
volatile uint8_t is_land_cmd_active = 0; // 0: Normal, 1: Emergency Landing

typedef enum { READ_ACCEL, READ_MAG } sensor_state_t;


// Define a test mission:

Waypoint mission_waypoints[] = {
		// { {x, y, z, yaw}, toa, hover_duration, tolerance, action }
		{{0.0f, 0.0f, 0.6f, 0.0f},  0.0f, 0.0f, 0.10f, WP_ACTION_MOVE},  // Step 1: Takeoff to 0.6m
		{{0.0f, 0.0f, 0.6f, 0.0f},  0.0f, 8.0f, 0.15f, WP_ACTION_HOVER}, // Step 2: Hover in place
		{{0.0f, 0.0f, 0.0f, 0.0f},  0.0f, 0.0f, 0.08f, WP_ACTION_LAND}   // Step 3: Land
};
uint16_t total_wp_count = 3;
//TIM3 > APB1 > Motor PWM Control
// Global instances for the Flight Stack
vehicleState_t  g_state;      // The current estimated state (Kinematics)
targetState_t g_target;	;     // The desired state (Setpoints)
MissionManager g_mission;    // The mission and waypoint sequencer
droneState_t g_drone_status; // Global metadata for modes, battery, and status

volatile float target_throttle = 0.0f; // The throttle we want

uint8_t spi_rx_buffer[SPI_FRAME_LEN];
uint8_t spi_tx_buf[SPI_FRAME_LEN];

// Double-buffering for SPI5 RX to avoid disabling interrupts
static uint8_t spi_rx_buffer_a[SPI_FRAME_LEN];
static uint8_t spi_rx_buffer_b[SPI_FRAME_LEN];
// The buffer DMA is currently writing to. Toggled in the ISR.
static uint8_t* volatile spi_dma_current_rx_buf = spi_rx_buffer_a;
// The buffer that is filled and ready for the main loop to process.
static uint8_t* volatile spi_main_process_buf = NULL;
// Flag to signal to the main loop that a buffer is ready.
static volatile bool spi_main_buf_ready = false;
static uint8_t spi_cmd_irq_buf[SPI_FRAME_LEN];
static volatile bool spi_cmd_irq_pending = false;
static volatile uint32_t spi5_frame_counter = 0;
static volatile uint32_t spi5_dropped_frame_counter = 0;
static volatile uint32_t spi5_cmd_rx_counter = 0;
static volatile uint32_t spi5_noncmd_rx_counter = 0;
static volatile uint8_t spi5_last_rx0 = 0;
static volatile uint8_t spi5_last_rx1 = 0;
static volatile uint32_t spi5_last_cmd_tick = 0;
static uint32_t spi5_last_arm_tick = 0;
static volatile uint16_t telem_cmd_ack_counter = 0;
static volatile uint8_t telem_cmd_ack_status = 0; // 0 none, 1 accepted, 2 rejected
static volatile uint8_t telem_cmd_ack_code = 0;
static volatile bool huzzah_handshake_ok = false;
static volatile uint32_t huzzah_handshake_tick = 0;
static float soft_land_x = 0.0f;
static float soft_land_y = 0.0f;
static float soft_land_yaw = 0.0f;
uint8_t command_ready = 0;
float commandZ = 0;

LSM303 imu;
LSM303_Raw raw;
I3GD20 i3gd20;
I3GD20_Raw gyro_raw;
BME280 bme280;
static BME280_Data bme280_data;
static AltitudeEstimator altitude_est;
static bool bme280_ready = false;
static uint8_t bme280_addr = BME280_I2C_ADDR_PRIM;
static uint32_t bme280_last_read_ms = 0;
static uint32_t bme280_last_init_attempt_ms = 0;
static uint8_t bme280_chip_id = 0;
static uint8_t i2c3_first_ack_addr = 0;
static uint8_t i2c3_ack_count = 0;
static float bme280_status = 0.0f; // 1 ready, 0 not attempted, negative values diagnose init/read failure.

GTU7 gps;
static GTU7_Fix gps_fix;
static uint8_t gps_dma_buffer[GPS_BUF_SIZE];
static bool gps_ready = false;
static uint32_t gps_last_init_attempt_ms = 0;
static uint32_t pbit_last_update_ms = 0;
static uint8_t pbit_mask = 0u;
static bool pbit_ok = false;
static bool pbit_report_active = false;

uint8_t tf_state = 0;
uint8_t tf_buf[9];
uint8_t tf_idx = 0;
uint16_t tf_checksum = 0;

static ahrsSensor_t raw_data;


// Logic Flags
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
void start_control(void);

static bool BME280_TryInit(void);
static bool BME280_TryInitAtAddress(uint8_t addr7);
static bool EnterMode(uint8_t mode);
static bool GPS_TryStart(uint32_t now_ms);
static bool IsGPSCommsReady(void);
static bool IsGPSLockReady(void);
static void UpdatePBIT(uint32_t now_ms, bool force);

static void SPI5_ArmNextFrame(void);
static void ResetActiveFlightPIDsFromState(const vehicleState_t* state);
static void AcquireSensorData(void);
static void UpdateState(float dt_sec);
static void StageActiveTelemetryFrame(void);
static void UpdateTelemetryGPSStatus(void);
static void UpdateAltitudeEstimator(uint32_t now_ms);
static void Process_VCP_Command_Line(const uint8_t* line);
static void Process_VCP_Command_Queue(void);
static void Process_SPI5_Frame(void);
static void BeginSoftLanding(void);
static void CompleteSoftLanding(void);
static void EnterPermanentFaultBlink(void);
static void SetIBITOkLed(bool on);
static void SetModeSelectLed(bool on);
static void UpdateGPSLockLed(bool gps_locked);
static void LSM303_DisableSharedIRQs(void);
static void LSM303_EnableSharedIRQs(void);
static void SPI5_DisableSharedIRQs(void);
static void SPI5_EnableSharedIRQs(void);
//SPI1 GYRO
//I2C1 ACCEL/MAG
//USB CDC (Virtual COM)
//SPI5 ESP8826

static TelemetryCompactDiagnostics BuildTelemetryDiagnostics(void);
static TelemetryVCPContext BuildTelemetryVCPContext(void);


/* Custom ESC Control Functions */
void ESC_ArmAll(void);
void ESC_EnableIdleSignal(void);
void ESC_Disarm(void);
void ESC_SetThrottle(uint32_t channel, float percentage);
uint32_t get_timer_channel(int motor_num);
void Process_TELEM_Command(uint8_t* Buf, uint32_t Len);
bool telemCMDPulse = false; //
uint32_t telemCMDTimeStart;
uint32_t unknownTelemCMD_counter;
uint32_t CMDpulseTime = 1000;

static void I2C1_Scan(void);
static void I2C3_Scan(void);
void Process_Lidar_DMA(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static bool BME280_TryInitAtAddress(uint8_t addr7) {
	HAL_StatusTypeDef probe_status;

	bme280_addr = addr7;
	probe_status = BME280_Probe(&hi2c3, addr7, &bme280_chip_id);
	if (probe_status != HAL_OK) {
		bme280_status = -1.0f; // No ACK or chip-id register read failed.
		return false;
	}

	if (bme280_chip_id != BME280_CHIP_ID) {
		bme280_status = -2.0f; // A device answered, but it is not a BME280.
		return false;
	}

	if (!BME280_Init(&bme280, &hi2c3, addr7)) {
		bme280_status = -3.0f; // Probe passed, but reset/calibration/config failed.
		return false;
	}

	bme280_addr = addr7;
	bme280_ready = true;
	bme280_status = 1.0f;
	telem_data.sensor_status |= SENSOR_STATUS_BME280_READY;
	bme280_last_read_ms = HAL_GetTick();
	if (BME280_Read(&bme280, &bme280_data) != HAL_OK) {
		bme280_ready = false;
		telem_data.sensor_status &= (uint8_t)~SENSOR_STATUS_BME280_READY;
		bme280_status = -4.0f; // First data read failed after init.
		return false;
	}
	UpdateAltitudeEstimator(HAL_GetTick());

	return true;
}

static bool BME280_TryInit(void) {
	bme280_last_init_attempt_ms = HAL_GetTick();
	bme280_ready = false;
	telem_data.sensor_status &= (uint8_t)~SENSOR_STATUS_BME280_READY;

	if (BME280_TryInitAtAddress(BME280_I2C_ADDR_PRIM)) {
		return true;
	}

	if (BME280_TryInitAtAddress(BME280_I2C_ADDR_SEC)) {
		return true;
	}

	I2C3_Scan();
	if (i2c3_ack_count == 0u) {
		bme280_status = -5.0f; // No devices responded anywhere on I2C3.
		bme280_addr = 0u;
		bme280_chip_id = 0u;
	} else {
		bme280_status = -6.0f; // I2C3 has devices, but no BME280 at 0x76/0x77.
		bme280_addr = i2c3_first_ack_addr;
		bme280_chip_id = i2c3_ack_count;
	}

	return false;
}

static void UpdateTelemetryGPSStatus(void) {
	const bool gps_locked = (gps_ready && gps_fix.valid);

	Telemetry_UpdateGPSStatus(gps_ready, gps_fix.valid);
	UpdateGPSLockLed(gps_locked);
}

static void UpdateAltitudeEstimator(uint32_t now_ms) {
	AltitudeEst_Update(
			&altitude_est,
			(gps_ready && gps_fix.valid),
			gps_fix.altitude_m,
			bme280_ready,
			bme280_data.altitude_m,
			now_ms);
}

static bool GPS_TryStart(uint32_t now_ms) {
	gps_last_init_attempt_ms = now_ms;

	gps_ready = GTU7_Init(&gps, &huart2);
	if (gps_ready) {
		gps_ready = GTU7_AttachDMARxBuffer(&gps, gps_dma_buffer, GPS_BUF_SIZE);
	}
	if (gps_ready) {
		gps_ready = (GTU7_StartRxDMA(&gps) == HAL_OK);
	}

	return gps_ready;
}

static bool IsGPSCommsReady(void) {
	return gps_ready && (gps.sentences_ok > 0u);
}

static bool IsGPSLockReady(void) {
	return gps_ready && gps_fix.valid;
}

static void UpdatePBIT(uint32_t now_ms, bool force) {
	if (!force && ((now_ms - pbit_last_update_ms) < PBIT_PERIOD_MS)) {
		return;
	}
	pbit_last_update_ms = now_ms;

	const IBIT_StatusInputs pbit_status = {
			.gps_ready = IsGPSCommsReady(),
			.bme280_ready = bme280_ready,
			.lidar_ready = ((telem_data.sensor_status & SENSOR_STATUS_LIDAR_OK) != 0u),
			.telemetry_ready = huzzah_handshake_ok
	};

	pbit_mask = IBIT_Evaluate(&imu, &i3gd20, &pbit_status);
	pbit_ok = IBIT_HasRequired(pbit_mask, PBIT_REQUIRED_MASK) &&
			((telem_data.sensor_status & PBIT_REQUIRED_SENSOR_STATUS_MASK) == PBIT_REQUIRED_SENSOR_STATUS_MASK);

	SetIBITOkLed(pbit_ok);
}

static TelemetryCompactDiagnostics BuildTelemetryDiagnostics(void) {
	TelemetryCompactDiagnostics diag = {
		.spi5_frame_counter = spi5_frame_counter,
		.spi5_cmd_rx_counter = spi5_cmd_rx_counter,
		.spi5_dropped_frame_counter = spi5_dropped_frame_counter,
		.spi5_noncmd_rx_counter = spi5_noncmd_rx_counter,
		.spi5_last_rx0 = spi5_last_rx0,
		.spi5_last_rx1 = spi5_last_rx1,
		.telem_cmd_ack_status = telem_cmd_ack_status,
		.telem_cmd_ack_code = telem_cmd_ack_code,
		.telem_cmd_ack_counter = telem_cmd_ack_counter
	};
	return diag;
}

static TelemetryVCPContext BuildTelemetryVCPContext(void) {
	TelemetryVCPContext ctx = {
		.state = &g_state,
		.target = &g_target,
		.drone_status = &g_drone_status,
		.mission = &g_mission,
		.raw_data = &raw_data,
		.bme280_data = &bme280_data,
		.gps_fix = &gps_fix,
		.pid_roll_rate = &pid_roll_rate,
		.pid_pitch_rate = &pid_pitch_rate,
		.pid_yaw_rate = &pid_yaw_rate,
		.pid_vel_z = &pid_vel_z,
		.imu = &imu,
		.gyro = &i3gd20,
		.bme280_ready = bme280_ready,
		.gps_ready = gps_ready,
		.altitude_bme_bias_valid = AltitudeEst_BiasValid(&altitude_est),
		.pbit_report_active = pbit_report_active,
		.pbit_ok = pbit_ok,
		.bme280_status = bme280_status,
		.altitude_bme_bias_m = AltitudeEst_GetBmeBiasM(&altitude_est),
		.altitude_bme_corrected_m = AltitudeEst_GetCorrectedBmeAltitudeM(&altitude_est),
		.pbit_mask = pbit_mask,
		.pbit_required_mask = PBIT_REQUIRED_MASK,
		.bme280_chip_id = bme280_chip_id,
		.bme280_addr = bme280_addr
	};
	return ctx;
}

static bool SPIBufferContains(const uint8_t* buf, const char* pattern) {
	size_t pattern_len = strlen(pattern);
	if (pattern_len == 0u || pattern_len > SPI_FRAME_LEN) {
		return false;
	}
	for (size_t i = 0; i <= (SPI_FRAME_LEN - pattern_len); i++) {
		if (memcmp(&buf[i], pattern, pattern_len) == 0) {
			return true;
		}
	}
	return false;
}

static void LSM303_DisableSharedIRQs(void) {
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
}

static void LSM303_EnableSharedIRQs(void) {
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
}

static void SPI5_DisableSharedIRQs(void) {
	HAL_NVIC_DisableIRQ(SPI5_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream4_IRQn);
}

static void SPI5_EnableSharedIRQs(void) {
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	HAL_NVIC_EnableIRQ(SPI5_IRQn);
}

// Helper function to find the offset of a valid SPI command frame in a buffer
static int FindSPICommandOffset(const uint8_t* buf) {
	for (int i = 0; i < SPI_FRAME_LEN - 1; i++) {
		if (buf[i] == '$' && buf[i + 1] >= 0x20u && buf[i + 1] <= 0x7Eu) {
			return i;
		}
	}
	return -1;
}

// Helper function to copy a SPI command frame from source to destination with an offset
static void CopySPICommandFrame(uint8_t* dst, const uint8_t* src, int offset) {
	memset(dst, 0, SPI_FRAME_LEN);
	if (offset < 0 || offset >= SPI_FRAME_LEN) {
		return;
	}
	memcpy(dst, &src[offset], SPI_FRAME_LEN - (uint32_t)offset);
}

// Helper function to compute the median of 5 float values
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
// Resets the active flight PID controllers using the provided vehicle state. If the state pointer is NULL,
// it defaults to zero values for all parameters.
// This function ensures that the PID controllers are initialized with the current state of the vehicle.
static void ResetActiveFlightPIDsFromState(const vehicleState_t* state)
{
    /*
     * vehicleState_t attitude/rates are in the AHRS / FC frame:
     *   +X -> M3
     *   +Y -> M4
     *
     * Active flight-control PIDs operate in the aircraft frame:
     *   +X -> M1 (front)
     *   +Y -> M2 (right)
     *
     * Therefore the aircraft frame is a 180 deg rotation about Z:
     *   roll_aircraft       = -roll_ahrs
     *   pitch_aircraft      = -pitch_ahrs
     *   roll_rate_aircraft  = -roll_rate_ahrs
     *   pitch_rate_aircraft = -pitch_rate_ahrs
     *
     * Yaw rate is unchanged by a 180 deg rotation about Z.
     */

    const float roll =
        (state != NULL) ? -state->roll : 0.0f;

    const float pitch =
        (state != NULL) ? -state->pitch : 0.0f;

    const float roll_rate =
        (state != NULL) ? -state->roll_rate : 0.0f;

    const float pitch_rate =
        (state != NULL) ? -state->pitch_rate : 0.0f;

    const float yaw_rate =
        (state != NULL) ? state->yaw_rate : 0.0f;

    const float z =
        (state != NULL) ? state->z : 0.0f;

    const float vz =
        (state != NULL) ? state->vz : 0.0f;


    PID_ResetWithMeasurement(&pid_roll_angle, roll);
    PID_ResetWithMeasurement(&pid_pitch_angle, pitch);

    /*
     * Yaw-angle loop operates on a precomputed wrapped yaw error
     * with actual = 0 in FlightLogic_Update().
     */
    PID_ResetWithMeasurement(&pid_yaw_angle, 0.0f);

    PID_ResetWithMeasurement(&pid_pos_z, z);
    PID_ResetWithMeasurement(&pid_vel_z, vz);

    PID_ResetWithMeasurement(&pid_roll_rate, roll_rate);
    PID_ResetWithMeasurement(&pid_pitch_rate, pitch_rate);
    PID_ResetWithMeasurement(&pid_yaw_rate, yaw_rate);
}

static bool EnterMode(uint8_t mode) {
	switch (mode) {

	case MODE_MANUAL_LEVEL:
		// Mode 1: Manual Level Mode
		pbit_report_active = false;
		takeoff_state = INIT;
		g_state.offGround = false;
		ResetActiveFlightPIDsFromState(&g_state);
		g_drone_status.drone_mode = MODE_MANUAL_LEVEL;
		return true;

	case MODE_MISSION:
		// Mode 2: Mission Mode
		if (!IsGPSLockReady()) {
			unknownTelemCMD_counter += 1;
			return false;
		}
		pbit_report_active = false;
		Navigation_Init(&g_mission, mission_waypoints, total_wp_count, &g_state);
		takeoff_state = INIT;
		g_state.offGround = false;
		ResetActiveFlightPIDsFromState(&g_state);
		g_drone_status.drone_mode = MODE_MISSION;
		return true;

	case MODE_THRUST_STAND:
	    // Mode 3: Thrust Stand Mode
		pbit_report_active = false;
		takeoff_state = INIT;
		g_state.offGround = false;
		ResetActiveFlightPIDsFromState(&g_state);
		g_drone_status.drone_mode = mode;
		return true;

	case MODE_ENGINEER:
		// Mode 4: Engineer Mode, stabilized control with raw engineering VCP telemetry.
		pbit_report_active = true;
		takeoff_state = INIT;
		g_state.offGround = false;
		g_target.roll = 0.0f;
		g_target.pitch = 0.0f;
		g_target.yaw = g_state.yaw;
		g_target.yaw_hold_enabled = true;
		g_target.rate_roll = 0.0f;
		g_target.rate_pitch = 0.0f;
		g_target.rate_yaw = 0.0f;
		g_target.ff_vz = 0.0f;
		ResetActiveFlightPIDsFromState(&g_state);
		g_drone_status.drone_mode = MODE_ENGINEER;
		g_drone_status.flight_mode = 7;
		return true;

	default:
		unknownTelemCMD_counter += 1;
		return false;
	}
}

static void AcquireSensorData(void) {
	const uint32_t now_ms = HAL_GetTick();

	LSM303_Process_DMA(&imu); // Process any new accel/mag data from DMA
	if (!gps_ready && ((now_ms - gps_last_init_attempt_ms) >= 1000u)) {
		(void)GPS_TryStart(now_ms);
	}
	if (gps_ready) {
		GTU7_ProcessDMARing(&gps);
		if (GTU7_HasFreshFix(&gps)) {
			(void)GTU7_GetLatest(&gps, &gps_fix);
			GTU7_ClearFreshFlag(&gps);
			UpdateAltitudeEstimator(now_ms);
		}
	}
	UpdateTelemetryGPSStatus();

	// ACCELEROMETER Parsing
	if (imu.accel_ready) {
		// Atomic snapshot of the 6-byte buffer
		uint8_t accel_snap[6];
		LSM303_DisableSharedIRQs();
		memcpy(accel_snap, imu.accel_raw, 6);
		imu.accel_ready = false; // Clear flag in struct
		LSM303_EnableSharedIRQs();

		Telemetry_RecordLSM303AccelRaw(accel_snap);

		Vec3f accel_pcb;
		LSM303_DecodeAccelG(&imu, accel_snap, &accel_pcb.x, &accel_pcb.y, &accel_pcb.z);

		Vec3f accel_body = SensorFrames_PcbToBody(accel_pcb);
		raw_data.ax = accel_body.x;
		raw_data.ay = accel_body.y;
		raw_data.az = accel_body.z;
		telem_data.sensor_status |= SENSOR_STATUS_LSM_READY;
	}

	// MAGNETOMETER Parsing
	if (imu.mag_ready) {
		uint8_t mag_snap[6];
		LSM303_DisableSharedIRQs();
		memcpy(mag_snap, imu.mag_raw, 6);
		imu.mag_ready = false; // Clear flag in struct
		LSM303_EnableSharedIRQs();

		Telemetry_RecordLSM303MagRaw(mag_snap, imu.variant);
		Vec3f mag_pcb;
		if (imu.variant == LSM303_DLHC) {
			// DLHC: Big Endian and X-Z-Y order
			mag_pcb.x = (int16_t)((mag_snap[0] << 8) | mag_snap[1]) * imu.mag_gauss_per_lsb;
			mag_pcb.z = (int16_t)((mag_snap[2] << 8) | mag_snap[3]) * imu.mag_gauss_per_lsb;
			mag_pcb.y = (int16_t)((mag_snap[4] << 8) | mag_snap[5]) * imu.mag_gauss_per_lsb;
		} else {
			// AGR: Little Endian and X-Y-Z order
			mag_pcb.x = (int16_t)((mag_snap[1] << 8) | mag_snap[0]) * imu.mag_gauss_per_lsb;
			mag_pcb.y = (int16_t)((mag_snap[3] << 8) | mag_snap[2]) * imu.mag_gauss_per_lsb;
			mag_pcb.z = (int16_t)((mag_snap[5] << 8) | mag_snap[4]) * imu.mag_gauss_per_lsb;
		}

		Vec3f mag_body = SensorFrames_PcbToBody(mag_pcb);
		raw_data.mx = mag_body.x;
		raw_data.my = mag_body.y;
		raw_data.mz = mag_body.z;
		telem_data.sensor_status |= SENSOR_STATUS_MAG_OK;
	}

	// --- GYROSCOPE ---
	if (i3gd20.initialized) {
		telem_data.sensor_status |= SENSOR_STATUS_GYRO_READY;
	}
	if (i3gd20.initialized && I3GD20_ReadGyro(&i3gd20, &gyro_raw)) {
		Telemetry_RecordGyroRaw(i3gd20.raw);

		float gx_raw = gyro_raw.gx * i3gd20.dps_per_lsb;
		float gy_raw = gyro_raw.gy * i3gd20.dps_per_lsb;
		float gz_raw = gyro_raw.gz * i3gd20.dps_per_lsb;

		if (g_gyro_calibration.valid) {
			gx_raw -= g_gyro_calibration.gx_bias_dps;
			gy_raw -= g_gyro_calibration.gy_bias_dps;
			gz_raw -= g_gyro_calibration.gz_bias_dps;
		}

		// BIQUAD FILTERING
		float gx_filtered = Biquad_Process(&filter_gyro_roll,  gx_raw);
		float gy_filtered = Biquad_Process(&filter_gyro_pitch, gy_raw);
		float gz_filtered = Biquad_Process(&filter_gyro_yaw,   gz_raw);

		Vec3f gyro_pcb = {
			.x =  gx_filtered,
			.y =  gy_filtered,
			.z =  gz_filtered
		};
		Vec3f gyro_body = SensorFrames_PcbToBody(gyro_pcb);
		raw_data.gx = gyro_body.x;
		raw_data.gy = gyro_body.y;
		raw_data.gz = gyro_body.z;
	}

	if (bme280_ready && ((now_ms - bme280_last_read_ms) >= 50u)) {
		bme280_last_read_ms = now_ms;
		if (BME280_Read(&bme280, &bme280_data) != HAL_OK) {
			bme280_ready = false;
			telem_data.sensor_status &= (uint8_t)~SENSOR_STATUS_BME280_READY;
			bme280_status = -4.0f;
		} else {
			telem_data.sensor_status |= SENSOR_STATUS_BME280_READY;
			UpdateAltitudeEstimator(bme280_last_read_ms);
		}
	} else if (!bme280_ready && ((now_ms - bme280_last_init_attempt_ms) >= 1000u)) {
		(void)BME280_TryInit();
	}
}

static void UpdateState(float dt_sec) {
	g_state.dt_sec = dt_sec;

	// Lidar-based altitude System
    // - Pure 5-sample block average (0.01s at 500Hz) with simple velocity low-pass.
    
    static uint8_t sample_count = 0;
    static float z_accumulator = 0.0f;
    static float block_avg_z = 0.0f;
    static float last_block_avg_z = 0.0f;
    
    float z_candidate = range_dist_cm * 0.01f; // meters
    
    if (!lidar_est_initialized) {
        // Prime the initial value on the very first pass
        block_avg_z = z_candidate;
        // Assuming lidar_est_initialized is set to 1 elsewhere after this init phase
    }
    
    // Accumulate the raw samples
    z_accumulator += z_candidate;
    sample_count++;
    
    // Process block average every 5 samples (10ms)
    if (sample_count >= 5) {
        last_block_avg_z = block_avg_z;
        
        // Calculate the new filtered Z altitude
        block_avg_z = z_accumulator / 5.0f; 
        
        // Simple derivative for Z-velocity (dv / dt). dt is exactly 5 loops.
        float raw_vz = (block_avg_z - last_block_avg_z) / (5.0f * dt_sec);
        
        // Basic low-pass filter for the velocity to prevent PID D-term harshness
        lidar_vz_filt = (lidar_vz_filt * 0.8f) + (raw_vz * 0.2f);
        
        // Reset block accumulators for the next 10ms window
        z_accumulator = 0.0f;
        sample_count = 0;
    }
    
    // Output the current estimates to the flight controller state
    g_state.z = block_avg_z;    
    g_state.vz = lidar_vz_filt;

	// STATE ESTIMATION (AHRS & Kalman)
	AHRS_Update(&raw_data, &g_state, dt_sec);
}

static void StageActiveTelemetryFrame(void) {
	const TelemetryCompactDiagnostics diag = BuildTelemetryDiagnostics();
	Telemetry_StageActiveFrame(
			spi_tx_buf,
			huzzah_handshake_ok,
			(StartControlState == STATE_MODE_SEL_COMPLETE),
			&diag,
			gps_ready,
			&gps_fix,
			HAL_GetTick());
}

static void Process_VCP_Command_Line(const uint8_t* line) {
	uint8_t cmd_buf[SPI_FRAME_LEN] = {0};
	uint16_t src_i = 0u;
	uint16_t dst_i = 0u;

	if (line == NULL) return;

	while (line[src_i] == ' ' || line[src_i] == '\t') {
		src_i++;
	}

	cmd_buf[dst_i++] = '$';
	if (line[src_i] == '$') {
		src_i++;
	}

	while ((line[src_i] != '\0') && (dst_i < (SPI_FRAME_LEN - 1u))) {
		cmd_buf[dst_i++] = line[src_i++];
	}

	Process_TELEM_Command(cmd_buf, SPI_FRAME_LEN);
}

static void Process_VCP_Command_Queue(void) {
	uint8_t line[SPI_FRAME_LEN];

	(void)VCP_Command_HadOverflow();

	while (VCP_Command_ReadLine(line, sizeof(line))) {
		Process_VCP_Command_Line(line);
	}
}

static void Process_SPI5_Frame(void) {
	uint8_t cmd_buf[SPI_FRAME_LEN];
	uint8_t* ready_buf;

	SPI5_DisableSharedIRQs();
	if (spi_cmd_irq_pending) {
		memcpy(cmd_buf, spi_cmd_irq_buf, SPI_FRAME_LEN);
		spi_cmd_irq_pending = false;
		SPI5_EnableSharedIRQs();

		spi5_last_rx0 = cmd_buf[0];
		spi5_last_rx1 = cmd_buf[1];
		spi5_cmd_rx_counter++;
		spi5_last_cmd_tick = HAL_GetTick();
		Process_TELEM_Command(cmd_buf, SPI_FRAME_LEN);
		return;
	}

	if (!spi_main_buf_ready || spi_main_process_buf == NULL) {
		SPI5_EnableSharedIRQs();
		return;
	}

	ready_buf = spi_main_process_buf;
	spi_main_process_buf = NULL;
	spi_main_buf_ready = false;
	SPI5_EnableSharedIRQs();

	memcpy(cmd_buf, (const void*)ready_buf, SPI_FRAME_LEN);
	spi5_last_rx0 = cmd_buf[0];
	spi5_last_rx1 = cmd_buf[1];
	if (SPIBufferContains(cmd_buf, TELEMETRY_HANDSHAKE_ESP)) {
		huzzah_handshake_ok = true;
		huzzah_handshake_tick = HAL_GetTick();
	}
	int cmd_offset = FindSPICommandOffset(cmd_buf);
	if (cmd_offset >= 0) {
		if (cmd_offset > 0) {
			uint8_t normalized_cmd[SPI_FRAME_LEN];
			CopySPICommandFrame(normalized_cmd, cmd_buf, cmd_offset);
			memcpy(cmd_buf, normalized_cmd, SPI_FRAME_LEN);
		}
		spi5_cmd_rx_counter++;
		spi5_last_cmd_tick = HAL_GetTick();
		Process_TELEM_Command(cmd_buf, SPI_FRAME_LEN);
	} else {
		spi5_noncmd_rx_counter++;
	}
}

static void BeginSoftLanding(void) {
	is_land_cmd_active = 1;
	is_estop_active = 0;
	takeoff_state = INIT;
	g_drone_status.drone_mode = MODE_MISSION;
	if (is_system_armed) {
		g_state.offGround = true;
	}
	g_mission.landing_start_t = (float)HAL_GetTick() / 1000.0f;
	soft_land_x = g_state.x;
	soft_land_y = g_state.y;
	soft_land_yaw = g_state.yaw;
	g_target.x = soft_land_x;
	g_target.y = soft_land_y;
	g_target.yaw = soft_land_yaw;
	g_target.yaw_hold_enabled = true;
	g_target.rate_yaw = 0.0f;
	ResetActiveFlightPIDsFromState(&g_state);
}

static void CompleteSoftLanding(void) {
	ESC_Disarm();
	is_land_cmd_active = 0;
	is_estop_active = 0;
	g_state.offGround = false;
	takeoff_state = INIT;
	g_mission.is_complete = 0;
	g_mission.landing_start_t = 0.0f;
	g_drone_status.drone_mode = 0;
	g_drone_status.flight_mode = 0;
	StartControlState = STATE_MODE_SEL;
}

static void SetIBITOkLed(bool on) {
	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FC_GREEN_LED_GPIO_Port, FC_GREEN_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SetModeSelectLed(bool on) {
	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FC_BLUE_LED_GPIO_Port, FC_BLUE_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SetSPI5ErrorLed(bool on) {
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FC_RED_LED_GPIO_Port, FC_RED_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void UpdateGPSLockLed(bool gps_locked) {
	static bool was_locked = false;
	static bool led_on = false;
	static uint32_t last_toggle_ms = 0u;
	const uint32_t now_ms = HAL_GetTick();

	if (!gps_locked) {
		was_locked = false;
		led_on = false;
		last_toggle_ms = now_ms;
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		return;
	}

	if (!was_locked) {
		was_locked = true;
		led_on = true;
		last_toggle_ms = now_ms;
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		return;
	}

	if ((now_ms - last_toggle_ms) >= 500u) {
		led_on = !led_on;
		last_toggle_ms = now_ms;
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

static void EnterPermanentFaultBlink(void) {
	const uint16_t all_led_pins = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin;
	const uint16_t all_fc_led_pins = FC_GREEN_LED_Pin | FC_RED_LED_Pin | FC_BLUE_LED_Pin;

	HAL_GPIO_WritePin(GPIOD, all_led_pins, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, all_fc_led_pins, GPIO_PIN_RESET);
	while (1) {
		HAL_GPIO_TogglePin(GPIOD, all_led_pins);
		HAL_GPIO_TogglePin(GPIOE, all_fc_led_pins);
		for (volatile uint32_t i = 0; i < 800000u; i++) {
			__NOP();
		}
	}
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
  MX_I2C1_Init(); // Initialize I2C1 for LSM303 (Accelerometer/Magnetometer)
  MX_USART2_UART_Init(); // Initialize USART2 for GPS (GTU7)
  MX_SPI1_Init(); // Initialize SPI1 for Gyroscope (I3GD20)
  MX_USART1_UART_Init(); // Initialize USART1 for Lidar (PWM/Serial)
  MX_SPI5_Init(); // Initialize SPI5 for ESP8266 (WiFi/Telemetry)
  MX_TIM3_Init(); // Initialize TIM3 for ESC PWM Control
  MX_I2C3_Init(); // Initialize I2C3 for BME280 (Barometer/Temperature/Humidity)
  MX_ADC1_Init(); // Initialize ADC1 for Battery Voltage Sensing
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */



	// --- 1. Initialize Telemetry Structure ---
	telem_data.header = 0xDEADBEEF;
	telem_data.magic_footer = 0xAB;
	telem_data.timestamp = 0.0f;
	telem_data.voltage = 12.0f;
	telem_data.sensor_status = 0;
	telem_data.armed = 0x00;
	telem_data.flight_mode = 0x05;
	AltitudeEst_Init(&altitude_est, HAL_GetTick());




	// Fill with test pattern to verify DMA is reading memory
	// memset(&telem_data.roll, 0xAA, 12);


	// --- 3. Start Lidar DMA ---
	HAL_UART_Receive_DMA(&huart1, lidar_dma_buffer, LIDAR_BUF_SIZE);
	// --- 4. Start GPS DMA ---
	(void)GPS_TryStart(HAL_GetTick());
	HAL_Delay(100);



	I2C1_Scan();
	I2C3_Scan();
	Vehicle_State_Init(&g_drone_status);
	last = HAL_GetTick();
	g_drone_status.flight_mode = 5; // Use a proper enum for FLIGHT_MODE_STARTUP
	HAL_Delay(10);
	// Run the pre-flight sequence until a flight mode is selected.
	start_control();

	// HAND OFF TO NAVIGATION STATE MACHINE
	// Get Circular logic back to START CONTROL once Mode is finished or after emergency
	// Will eventually want to setup a continue waypoint



	float main_last = 0.00f;


	g_drone_status.flight_mode = 0; // Set to a known state like FLIGHT_MODE_IDLE
	// Loading Mission
	// Link the waypoints to the manager and provide the current state for the start position
	if (g_drone_status.drone_mode == MODE_MISSION){
		EnterMode(MODE_MISSION);
	}

	ResetActiveFlightPIDsFromState(&g_state);
	g_target.yaw = g_state.yaw;
	g_target.yaw_hold_enabled = true;
	g_target.rate_yaw = 0.0f;

	SPI5_ArmNextFrame();

	g_state.offGround = false;
	float flight_takeoff_height_threshold = 0.5f; // Units meters. 
	float flight_landing_height_threshold = 0.2f; // Units meters.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// Process any completed SPI frames from the telemetry radio.
		// This is now non-blocking and does not disable interrupts.
		Process_SPI5_Frame();

		// Process any commands from the USB Virtual COM Port.
		Process_VCP_Command_Queue();

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
			AcquireSensorData();
			UpdateState(dt_sec);
			UpdatePBIT(now, false);
			static uint32_t state_timer = 0;
			altitude_base_thrust_raw = ALTITUDE_HOVER_THRUST_RAW;

			if (is_system_armed &&
					!g_state.offGround &&
					g_drone_status.drone_mode == MODE_MISSION &&
					!IsGPSLockReady()) {
				ESC_Disarm();
				takeoff_state = INIT;
				g_drone_status.drone_mode = 0;
				StartControlState = STATE_MODE_SEL;
				start_control();
				continue;
			}

			// --- TAKEOFF STATE MACHINE ---
			// Only valid when the system is armed and drone is on ground
			// On-Ground is procedural, not a sensor state.
			if ((!g_state.offGround) && (is_system_armed)){
				switch(takeoff_state) {
				case INIT:
					g_drone_status.flight_mode = 81;
					// Reset altitude state to ensure we start from a known ground-level.
					// This prevents stale/noisy sensor data from causing a premature transition.
					lidar_est_initialized = 0;
					g_state.z = 0.0f;
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
					// Give motors 2000ms to reach idle speed
					g_drone_status.flight_mode = 82;
					g_target.yaw = g_state.yaw;
					g_target.yaw_hold_enabled = false;
					g_target.rate_yaw = 0.0f;
					ESC_SetThrottle(TIM_CHANNEL_1, 15.0f);
					ESC_SetThrottle(TIM_CHANNEL_2, 15.0f);
					ESC_SetThrottle(TIM_CHANNEL_3, 15.0f);
					ESC_SetThrottle(TIM_CHANNEL_4, 15.0f);
					takeoff_override_this_tick = 1;
					if (now - state_timer > 2000) {
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
					g_target.z     = flight_takeoff_height_threshold + 0.2f;
					g_target.ff_vz = 0.5f; 
					uint32_t ramp_ms = now - state_timer;
					if (ramp_ms > TAKEOFF_BASE_THRUST_RAMP_MS) {
						ramp_ms = TAKEOFF_BASE_THRUST_RAMP_MS;
					}
					float ramp_fraction = (float)ramp_ms / (float)TAKEOFF_BASE_THRUST_RAMP_MS;
					altitude_base_thrust_raw = TAKEOFF_BASE_THRUST_START_RAW +
							((TAKEOFF_BASE_THRUST_END_RAW - TAKEOFF_BASE_THRUST_START_RAW) * ramp_fraction);

					// Allow control update in TAKEOFF to climb toward z target
					takeoff_override_this_tick = 0; // Allow normal control to run

					if (g_state.z > flight_takeoff_height_threshold) {
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
					g_target.ff_vz = 0.0f;
					ResetActiveFlightPIDsFromState(&g_state);
					takeoff_override_this_tick = 0; // Allow normal control to run
					g_state.offGround = true; // Hand over to main flight controller
					g_drone_status.flight_mode = 0x08; //Stabilize
					break;
				}
			}
			// 2. MISSION LOGIC
			uint8_t invalid_mode_requested = 0;
			if (!takeoff_override_this_tick &&
					(g_drone_status.drone_mode != MODE_MANUAL_LEVEL) &&
					(g_drone_status.drone_mode != MODE_MISSION) &&
					(g_drone_status.drone_mode != MODE_THRUST_STAND) &&
					(g_drone_status.drone_mode != MODE_ENGINEER)) {
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
				// Soft landing: hold current X/Y and descend under normal control.
				g_target.x = soft_land_x;
				g_target.y = soft_land_y;
				g_target.yaw = soft_land_yaw;
				g_target.yaw_hold_enabled = true;
				g_target.rate_yaw = 0.0f;
				g_drone_status.flight_mode = 3; // LANDING
				float descent_rate = (g_state.z > 5.0f) ? 0.4f : 0.15f;
				g_target.z = g_state.z - (descent_rate * dt_sec);
				if (g_target.z < 0.4f) g_target.z = 0.0f;

				g_target.ff_vz = -descent_rate;

				// Auto-disarm once on the ground, then return to Mode Select.
				if (!is_system_armed || g_state.z <= flight_landing_height_threshold) {
					CompleteSoftLanding();
					start_control();
					continue;
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
					(g_drone_status.drone_mode == MODE_ENGINEER) &&
					(takeoff_state != TAKEOFF)) {
				g_drone_status.flight_mode = 7; // Engineer stabilize
				g_target.roll = 0.0f;
				g_target.pitch = 0.0f;
				g_target.yaw_hold_enabled = true;
				if (isnan(g_target.rate_roll)) {
					g_target.rate_roll = 0.0f;
					g_target.rate_pitch = 0.0f;
					g_target.rate_yaw = 0.0f;
				}
				g_target.z = commandZ + flight_takeoff_height_threshold;
				g_target.ff_vz = 0.0f;
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
				g_target.z = commandZ + flight_takeoff_height_threshold;

			} else if (!takeoff_override_this_tick &&
					(g_drone_status.drone_mode == MODE_MISSION)) {
				g_drone_status.drone_mode = 2;
				if (g_state.offGround) {
					// Mission navigation owns targets only after executive takeoff hands off.
					g_drone_status.flight_mode = 0x08; //Stabilize
					Navigation_GetTarget(&g_mission, (float)now / 1000.0f, &g_state, &g_target);
					g_target.yaw_hold_enabled = true;
					// Check for mission completion (e.g., after landing)
					if (g_mission.is_complete) {
						ESC_Disarm(); // This function already sets motors to 0 and is_system_armed to 0
						g_state.offGround = false;
						takeoff_state = INIT; // Reset takeoff state machine
						g_drone_status.drone_mode = 0; // Go back to idle/mode-select
						g_mission.is_complete = 0; // Reset mission flag
						StartControlState = STATE_MODE_SEL; // Re-enter mode selection loop
						start_control(); // Restart the mode selection process	
						continue; // Skip the rest of this control loop iteration
					}
				}
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
				if (!is_land_cmd_active && !(!g_state.offGround && takeoff_state == TAKEOFF)) {
					g_drone_status.flight_mode =
							(g_drone_status.drone_mode == MODE_ENGINEER) ? 7 : 8;
				}
				telem_data.sat_flags = FlightLogic_Update(&g_state, &g_target, &g_drone_status);
				if (is_land_cmd_active) {
					g_drone_status.flight_mode = 3;
				}
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

			telem_data.header 				= 0xDEADBEEF;       // UINT32
			telem_data.timestamp 			= (float)HAL_GetTick() * 0.001f;        // float 1
			telem_data.roll 				= g_state.roll;       // float 2 (Estimated Roll)
			telem_data.pitch 				= g_state.pitch;     // float 3 (Estimated Pitch)
			telem_data.yaw 					= g_state.yaw;         // float 4 (Estimated Yaw)
			telem_data.altitude 			= range_dist_cm;  // float 5 (Raw Lidar in cm)
			telem_data.voltage 				= commandZ;           // float 6
			telem_data.armed 				= is_system_armed ? 0xFF : 0x00;
			// Map the modes to the telemetry packet
			telem_data.drone_mode  			= g_drone_status.drone_mode;
			telem_data.flight_mode 			= g_drone_status.flight_mode;
			UpdateTelemetryGPSStatus();
			Telemetry_UpdateCompactGPSFields(gps_ready, &gps_fix);
			const TelemetryCompactDiagnostics diag = BuildTelemetryDiagnostics();
			Telemetry_UpdateCompactDiagnostics(&diag, now);
			telem_data.magic_footer 		= 0xAB;       // UINT8

			const TelemetryVCPContext vcp_ctx = BuildTelemetryVCPContext();
			Telemetry_VCP_TrySend(now, &vcp_ctx);
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
  RCC_OscInitStruct.PLL.PLLM = AMALTHEIA_PLLM;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  huart2.Init.BaudRate = GT_U7_DEFAULT_BAUD;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FC_GREEN_LED_Pin|FC_RED_LED_Pin|FC_BLUE_LED_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : FC_GREEN_LED_Pin FC_RED_LED_Pin FC_BLUE_LED_Pin */
  GPIO_InitStruct.Pin = FC_GREEN_LED_Pin|FC_RED_LED_Pin|FC_BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

	// This function now runs its own loop until a flight mode is selected.
	while (StartControlState != STATE_MODE_SEL_COMPLETE)
	{
	switch (StartControlState) {
	case STATE_INIT:
		// INIT ESC
		ESC_EnableIdleSignal();
		g_drone_status.drone_mode = 51; // Init Mode
		SetIBITOkLed(false);
		SetModeSelectLed(false);
		UpdateGPSLockLed(false);
		pbit_report_active = true;
		pbit_ok = false;
		pbit_mask = 0u;
		pbit_last_update_ms = 0u;
		telem_data.sensor_status &= (uint8_t)~(
				SENSOR_STATUS_GYRO_READY |
				SENSOR_STATUS_LSM_READY |
				SENSOR_STATUS_LIDAR_OK |
				SENSOR_STATUS_MAG_OK |
				SENSOR_STATUS_BME280_READY |
				SENSOR_STATUS_GPS_FIX);
		// --- 1. PRE-FLIGHT CONTROLLER SETUP ---\

		PID_Init(&pid_pos_z, 0.3f, 0.0f, 0.0f, 0.002f, 5.0f);   // Position P gain

		PID_Init(&pid_vel_z, 0.2f, 0.01f, 0.0f, 0.002f, 10.0f); // Velocity PID with I-limit

		PID_Init(&pid_roll_angle,  0.4f, 0.000f, 0.000f, 0.002f, 10.0f);
		PID_Init(&pid_pitch_angle, 0.4f, 0.000f, 0.000f, 0.002f, 10.0f);
		PID_Init(&pid_yaw_angle,   0.3f, 0.000f, 0.000f, 0.002f, 10.0f);

		PID_Init(&pid_roll_rate, 0.1f, 0.0f, 0.000f, 0.002f, 0.35f);
		PID_Init(&pid_pitch_rate, 0.1f, 0.0f, 0.000f, 0.002f, 0.35f);
		PID_Init(&pid_yaw_rate, 0.12f, 0.0f, 0.000f, 0.002f, 0.12f);

		// -- Frame Configuration -- 
		Mixer_SetFrameType(MIXER_FRAME_PLUS);
		

		float d_alpha = PID_Calculate_Alpha(20.0f, 0.002f);
		pid_roll_rate.d_low_pass_alpha = d_alpha;
		pid_pitch_rate.d_low_pass_alpha = d_alpha;

		AHRS_Init();

		// Setup filters for Roll, Pitch, and Yaw Gyros
		Biquad_Set_Lowpass(&filter_gyro_pitch, 80.0f, 500.0f);
		Biquad_Set_Lowpass(&filter_gyro_roll,  80.0f, 500.0f);
		Biquad_Set_Lowpass(&filter_gyro_yaw,   80.0f, 500.0f);

		// Keep ESCs quiet with a valid low signal without arming the aircraft.

		HAL_Delay(1000); // Attempting to give time to let frame settle before starting the init loop

		// Accel/Mag (I2C1) - Configure Registers
		(void)LSM303_Init(&imu, &hi2c1, LSM303_ACCEL_SCALE_2G);

		(void)BME280_TryInit();
		// SPI Begin
		HAL_Delay(200);          // optional “let ESP settle” gate (helps your battery case)
		SPI5_ArmNextFrame();
		// --- 2. HARDWARE INITIALIZATION LOOP ---
		uint8_t sensorInit = 0;
		const bool bypass_sensor_init_gate = true; // DEVELOPMENT ONLY: bypass sensor comms/fault gate.
		uint32_t init_loop_start_ms = HAL_GetTick();
		uint32_t last_sensor_check_ms = 0;
		uint32_t last_gyro_init_attempt_ms = 0;
		uint32_t init_last_state_update_ms = init_loop_start_ms;
		while (sensorInit == 0)
		{
			uint32_t now_ms = HAL_GetTick();
			g_drone_status.flight_mode = 52; // Sensor Init Mode
			// Clear status bits that require "Fresh" verification this frame

			// SAFETY: Add a timeout to the sensor init loop.
			// If sensors are not ready after 10 seconds, enter a permanent fault state.
			if (!bypass_sensor_init_gate && ((now_ms - init_loop_start_ms) > 10000)) {
				g_drone_status.flight_mode = 255; // FAULT_STATE
				EnterPermanentFaultBlink();
			}

			if (bypass_sensor_init_gate) {
				SetIBITOkLed(true);
				sensorInit = 1;
				break;
			}

			if (!i3gd20.initialized &&
					((last_gyro_init_attempt_ms == 0u) ||
							((now_ms - last_gyro_init_attempt_ms) >= 1000u))) {
				last_gyro_init_attempt_ms = now_ms;
				g_drone_status.flight_mode = 53; // Gyro zero-rate calibration
				if (I3GD20_Init(&i3gd20, &hspi1)) {
					I3GD20_CalibrateZeroRate(&i3gd20, 1000); // 1000 samples
					telem_data.sensor_status |= SENSOR_STATUS_GYRO_READY;
				}
			}
			if (gps_ready) {
				g_drone_status.flight_mode = 54; // GPS service mode
			}
			Process_Lidar_DMA();
			uint32_t init_state_elapsed_ms = now_ms - init_last_state_update_ms;
			if (init_state_elapsed_ms >= 2u) {
				float init_dt_sec = (float)init_state_elapsed_ms * 0.001f;
				if (init_dt_sec <= 0.0f || init_dt_sec > 0.050f) {
					init_dt_sec = 0.002f;
				}
				init_last_state_update_ms = now_ms;
				AcquireSensorData();
				UpdateState(init_dt_sec);
			}

			// 2. Non-blocking check at ~20Hz to allow Lidar processing to run frequently
			if ((now_ms - last_sensor_check_ms) < 50) {
				// Continue spinning the loop to process Lidar/VCP commands
				continue;
			}
			last_sensor_check_ms = now_ms;

			// Process SPI and VCP commands during the sensor init loop
			Process_SPI5_Frame();
			Process_VCP_Command_Queue();

			g_drone_status.flight_mode = 5;
			// Verification gate reads raw_data populated by AcquireSensorData().
			// First stage: comms/alive PBIT. Second stage: fresh data validity checks.
			UpdatePBIT(now_ms, true);
			if (pbit_ok) {

				// Gravity Vector Check
				float accel_mag = sqrtf(raw_data.ax*raw_data.ax + raw_data.ay*raw_data.ay + raw_data.az*raw_data.az);

				// Magnetic Field Magnitude Check
				float mag_field_strength = sqrtf(raw_data.mx*raw_data.mx + raw_data.my*raw_data.my + raw_data.mz*raw_data.mz);

				bool gravity_ok = (accel_mag > 0.85f && accel_mag < 1.15f);
				bool mag_ok = (mag_field_strength > 0.2f && mag_field_strength < 0.9f);

				if (gravity_ok && mag_ok) {
					SetIBITOkLed(true);
					sensorInit = 1; // Success! Exit loop
					//printf("ALL SYSTEMS GO: G=%.2fg, Mag=%.2f Gauss\r\n", accel_mag, mag_field_strength);
				} else {
					if (!gravity_ok) 
						if (!mag_ok)
							HAL_Delay(200);
				}
			}
			telem_data.header = 0xDEADBEEF;       // UINT32        // float 1
			telem_data.altitude = range_dist_cm;  // float 5 (Raw Lidar in cm)
			telem_data.voltage = 15.0f;           // float 6
			telem_data.armed = is_system_armed ? 0xFF : 0x00;
			// Map the modes to the telemetry packet
			telem_data.drone_mode  = g_drone_status.drone_mode;
			telem_data.flight_mode = pbit_ok ? FLIGHT_MODE_PBIT_OK : FLIGHT_MODE_PBIT_FAIL;

			telem_data.magic_footer = 0xAB;       // UINT8
			telem_data.header = 0xDEADBEEF;       // Redundant header as per your code
			const TelemetryVCPContext vcp_ctx = BuildTelemetryVCPContext();
			Telemetry_VCP_TrySend(HAL_GetTick(), &vcp_ctx);
		}

		StartControlState = STATE_MODE_SEL;
		break;


	case STATE_MODE_SEL: {
		g_drone_status.flight_mode = 6; // Mode Select
		static uint8_t entered = 0;
		static uint32_t mode_sel_last_ms = 0;
		if (!entered) {
			g_drone_status.drone_mode = 0;
			SetModeSelectLed(true);
			entered = 1;
			mode_sel_last_ms = HAL_GetTick();
			SPI5_ArmNextFrame();
		}
		pbit_report_active = true;

		telem_data.flight_mode = 0x06; // Signal HUD we are in Mode Select
		uint32_t now = HAL_GetTick();
		Process_SPI5_Frame();
		Process_VCP_Command_Queue();

		Process_Lidar_DMA();
		uint32_t mode_sel_elapsed_ms = now - mode_sel_last_ms;
		if (mode_sel_elapsed_ms >= 2u) {
			float mode_sel_dt_sec = (float)mode_sel_elapsed_ms * 0.001f;
			if (mode_sel_dt_sec <= 0.0f || mode_sel_dt_sec > 0.050f) {
				mode_sel_dt_sec = 0.002f;
			}
			mode_sel_last_ms = now;
			AcquireSensorData();
			UpdateState(mode_sel_dt_sec);
		}
		UpdatePBIT(now, false);

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
		
		telem_data.timestamp  = g_state.dt_sec;
		telem_data.roll       = g_state.roll;
		telem_data.pitch      = g_state.pitch;
		telem_data.yaw        = g_state.yaw;
		telem_data.altitude   = range_dist_cm;
		telem_data.armed      = is_system_armed ? 0xFF : 0x00;
		telem_data.drone_mode = g_drone_status.drone_mode;
		telem_data.flight_mode = pbit_ok ? FLIGHT_MODE_PBIT_OK : FLIGHT_MODE_PBIT_FAIL;
		const TelemetryVCPContext vcp_ctx = BuildTelemetryVCPContext();
		Telemetry_VCP_TrySend(now, &vcp_ctx);

		// Primary Mode Selection
		// Fixed: Using your global instance g_drone_status
		if (g_drone_status.drone_mode != 0) {
			SetModeSelectLed(false);
			pbit_report_active = (g_drone_status.drone_mode == MODE_ENGINEER);
			StartControlState = STATE_MODE_SEL_COMPLETE;
			mode_sel_last_ms = 0;
			entered = 0;
		}
		break;
	}
	case STATE_MODE_SEL_COMPLETE:
		SetModeSelectLed(false);
		// do nothing
		break;
	}
	}

}
static void SPI5_ArmNextFrame(void)
{
	__HAL_SPI_CLEAR_OVRFLAG(&hspi5);
	StageActiveTelemetryFrame(); // Prepares spi_tx_buf

	// Arm the DMA with the buffer the ISR has designated for the next transfer.
	// Note: We cast away volatile, which is safe here as we are just passing the address to the HAL function.
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi5,
			spi_tx_buf,
			(uint8_t*)spi_dma_current_rx_buf,
			SPI_FRAME_LEN);

	if (status != HAL_OK) {
		// DMA arming failed. This is a critical error.
		// A more robust system might try to re-init the SPI peripheral.
		SetSPI5ErrorLed(true);
	} else {
		SetSPI5ErrorLed(false);
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
		spi5_frame_counter++;
		uint8_t* completed_rx_buf = spi_dma_current_rx_buf;
		if (SPIBufferContains(completed_rx_buf, TELEMETRY_HANDSHAKE_ESP)) {
			huzzah_handshake_ok = true;
			huzzah_handshake_tick = HAL_GetTick();
		}
		int cmd_offset = FindSPICommandOffset(completed_rx_buf);
		bool completed_is_cmd = (cmd_offset >= 0);

		if (completed_is_cmd) {
			if (spi_cmd_irq_pending) {
				spi5_dropped_frame_counter++;
			}
			CopySPICommandFrame(spi_cmd_irq_buf, completed_rx_buf, cmd_offset);
			spi_cmd_irq_pending = true;
		}

		// Hand the completed DMA buffer to the main loop. If the main loop is
		// still processing the previous frame, keep reusing this DMA buffer and
		// drop/overwrite the new frame instead of touching the held buffer.
		if (spi_main_buf_ready || completed_is_cmd) {
			// Main loop hasn't processed the last frame.
			if (spi_main_buf_ready && !completed_is_cmd) {
				spi5_dropped_frame_counter++;
			}
		} else {
			spi_main_process_buf = completed_rx_buf;
			spi_main_buf_ready = true;

			if (completed_rx_buf == spi_rx_buffer_a) {
				spi_dma_current_rx_buf = spi_rx_buffer_b;
			} else {
				spi_dma_current_rx_buf = spi_rx_buffer_a;
			}
		}

		// Immediately arm the next DMA transfer into the new buffer.
		SPI5_ArmNextFrame();
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI5) {
		// An SPI error occurred (e.g., Overrun). Abort to reset HAL state.
		HAL_SPI_Abort(hspi);
		hspi->State = HAL_SPI_STATE_READY;
		// Immediately try to re-arm the next DMA transfer to recover.
		SPI5_ArmNextFrame();
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
	while (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n') {
		cmd++;
	}
	for (char* p = cmd; *p != '\0'; p++) {
		*p = (char)tolower((unsigned char)*p);
	}
	bool command_accepted = false;

	// 3. Token-Based Switch Switchboard
	switch (cmd[0]) {

	case 'x': // --- EMERGENCY STOP ---
		ESC_Disarm();
		is_estop_active = 1;
		is_system_armed = 0;
		g_mission.landing_start_t = (float)HAL_GetTick() / 1000.0f;
		if (g_mission.current_index < g_mission.total_waypoints) {
			g_mission.waypoints[g_mission.current_index].action = WP_ACTION_LAND;
		}
		command_accepted = true;
		break;

	case 'a': // --- ARM or ALL ---
		if (cmd[1] == 'r' && cmd[2] == 'm') { // "arm"
			if (g_drone_status.drone_mode == MODE_MISSION && !IsGPSLockReady()) {
				unknownTelemCMD_counter += 1;
				// Need to add indication to the user that GPS lock is required for arming in Mission mode.
			} else {
				ESC_ArmAll();
				command_accepted = true;
			}
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
				command_accepted = true;
			}
		}
		break;

	case 'b': { // --- BIT SNAPSHOT ---
		if (strncmp(cmd, "bit", 3) == 0) {
			const TelemetryVCPContext vcp_ctx = BuildTelemetryVCPContext();
			Telemetry_VCP_SendBIT(HAL_GetTick(), &vcp_ctx);
			command_accepted = true;
		} else {
			unknownTelemCMD_counter += 1;
		}
		break;
	}

	case 'i': { // --- IBIT SNAPSHOT ---
		if (strncmp(cmd, "ibit", 4) == 0) {
			const TelemetryVCPContext vcp_ctx = BuildTelemetryVCPContext();
			Telemetry_VCP_SendBIT(HAL_GetTick(), &vcp_ctx);
			command_accepted = true;
		} else {
			unknownTelemCMD_counter += 1;
		}
		break;
	}

	case 'l': // --- SOFT LAND ---
		if (strncmp(cmd, "land", 4) == 0) {
			BeginSoftLanding();
			command_accepted = true;
		} else {
			unknownTelemCMD_counter += 1;
		}
		break;

	case 'm': // --- MOTOR or MODE ---
		// Check if it's "mode". Accept "mode 1", "mode1", "mode=1", etc.
		if (strncmp(cmd, "mode", 4) == 0) { // "mode 1|2|3|4"
			char* val_ptr = cmd + 4;
			while (*val_ptr == ' ' || *val_ptr == '\t' || *val_ptr == '=' || *val_ptr == ':') {
				val_ptr++;
			}
			if (isdigit((unsigned char)*val_ptr)) {
				int requested_mode = atoi(val_ptr);
				if (requested_mode == MODE_MANUAL_LEVEL ||
						requested_mode == MODE_MISSION ||
						requested_mode == MODE_THRUST_STAND ||
						requested_mode == MODE_ENGINEER) {
					command_accepted = EnterMode((uint8_t)requested_mode);
				} else {
					// Unsupported/reserved mode IDs are ignored
					unknownTelemCMD_counter += 1;
				}
			} else {
				unknownTelemCMD_counter += 1;
			}
		}
		// Check if it's "motor X pY" (e.g. "m1 p10" or "motor 1 p10")
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
					command_accepted = true;
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
					command_accepted = true;
				}
			}
			else {
				unknownTelemCMD_counter += 1;
			}
		}
		break;

	case 'r': // --- RETURN TO HOME / SOFT LAND ALIAS ---
		if (strncmp(cmd, "rth", 3) == 0) {
			BeginSoftLanding();
			command_accepted = true;
		} else {
			unknownTelemCMD_counter += 1;
		}
		break;

	case 'z': case 'Z':
		g_drone_status.drone_mode = 1;
		commandZ = strtof(cmd + 1, NULL);
		g_target.z = commandZ;
		command_accepted = true;
		break;

	}

	telem_cmd_ack_counter++;
	telem_cmd_ack_status = command_accepted ? 1u : 2u;
	telem_cmd_ack_code = (uint8_t)(cmd[0] ? cmd[0] : '?');
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
void ESC_EnableIdleSignal(void) {
	// Send continuous minimum PWM so ESCs see a valid low-throttle signal.
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_ARM_PULSE); // PC6
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_ARM_PULSE); // PB5
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_ARM_PULSE); // PC8
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_ARM_PULSE); // PC9

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	target_throttle = 0.0f;
	telem_data.armed = 0x00;
	telem_data.motor1_T = 0;
	telem_data.motor2_T = 0;
	telem_data.motor3_T = 0;
	telem_data.motor4_T = 0;
	g_drone_status.armed = 0;
	g_drone_status.motor1_T = 0;
	g_drone_status.motor2_T = 0;
	g_drone_status.motor3_T = 0;
	g_drone_status.motor4_T = 0;
	is_system_armed = 0;
}
void ESC_Disarm(void) {
	// Return ESC outputs and flight state to the unarmed low-throttle condition.
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
	uint8_t write_idx = (LIDAR_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) % LIDAR_BUF_SIZE;
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
					telem_data.sensor_status |= SENSOR_STATUS_LIDAR_OK;
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

static void I2C3_Scan(void) {
	i2c3_first_ack_addr = 0u;
	i2c3_ack_count = 0u;

	for (uint8_t addr = 1; addr < 0x7F; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c3, addr << 1, 1, 5) == HAL_OK) {
			if (i2c3_first_ack_addr == 0u) {
				i2c3_first_ack_addr = addr;
			}
			if (i2c3_ack_count < 255u) {
				i2c3_ack_count++;
			}
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
	EnterPermanentFaultBlink();
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
