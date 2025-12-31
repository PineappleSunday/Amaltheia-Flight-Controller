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
	// Motor Thrust %
	uint8_t motor1_T;
	uint8_t motor2_T;
	uint8_t motor3_T;
	uint8_t motor4_T;
	uint8_t sensor_status;
	uint8_t magic_footer;
} Telemetry_Packet_t;
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
volatile uint8_t command_ready = 0; // For COMMAND FROM TELEM

I2C_HandleTypeDef hi2c1;

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
//TIM3 > APB2 > Motor PWM Control

uint8_t rx_byte;              // Holds the incoming character
uint8_t rx_buffer[10];        // Buffer to build the command string
uint8_t rx_index = 0;         // Current position in buffer
volatile float target_throttle = 0.0f; // The throttle we want
// TELEM Data Length

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

// PID controllers
PIDController pid_roll;
PIDController pid_pitch;
PIDController pid_yaw;

// LIDAR Vars
uint8_t lidar_dma_buffer[LIDAR_BUF_SIZE];
uint8_t lidar_read_idx = 0;
volatile float range_dist_cm = 0.0f;

// Telem Vars
Telemetry_Packet_t telem_data;
uint8_t spi_rx_buffer[32]; // For incoming commands from ESP8266
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

	HAL_SPI_TransmitReceive_DMA(&hspi5, (uint8_t*)&telem_data, spi_rx_buffer, 34);
	// --- 1. Initialize Telemetry Structure ---
	telem_data.header = 0xDEADBEEF;
	telem_data.magic_footer = 0xAB;
	telem_data.timestamp = 0.0f;
	telem_data.voltage = 12.0f;
	telem_data.sensor_status = 0;

	// Fill with test pattern to verify DMA is reading memory
	// memset(&telem_data.roll, 0xAA, 12);

	// --- 2. Start SPI5 Circular DMA ---
	// Slave Mode: Waits for ESP8266 to pull CS Low
	if (HAL_SPI_TransmitReceive_DMA(&hspi5, (uint8_t*)&telem_data, spi_rx_buffer, sizeof(Telemetry_Packet_t)) != HAL_OK) {
		printf("SPI5 DMA Start Failed!\r\n");
		HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
		HAL_Delay(50);
	} else {
		printf("SPI5 DMA Started (Circular Mode)\r\n");
	}

	// --- 3. Start Lidar DMA ---
	HAL_UART_Receive_DMA(&huart1, lidar_dma_buffer, LIDAR_BUF_SIZE);
	printf("Lidar DMA Started\r\n");

	I2C1_Scan();

	// --- SENSOR SETUP ---
	uint8_t current_status = 0;

	// Gyro (SPI1)
	memset(&gyro_raw, 0, sizeof(gyro_raw));
	if (!I3GD20_Init(&i3gd20, &hspi1)) {
		printf("I3GD20 init FAILED\r\n");
	} else {
		printf("I3GD20 init OK\r\n");
		I3GD20_CalibrateZeroRate(&i3gd20, 1000);
		current_status |= 0x01;
	}

	// Accel/Mag (I2C1)
	memset(&imu, 0, sizeof(imu));
	if (!LSM303_Init(&imu, &hi2c1, LSM303_ACCEL_SCALE_2G)) {
		printf("LSM303 init FAILED\r\n");
	} else {
		printf("LSM303 init OK\r\n");
		current_status |= 0x06;
	}

	// --- CONTROL INIT ---
	PID_Init(&pid_roll,  1.0f, 0.0f, 0.0f, 0.01f, 100.0f);
	PID_Init(&pid_pitch, 1.0f, 0.0f, 0.0f, 0.01f, 100.0f);
	PID_Init(&pid_yaw,   1.0f, 0.0f, 0.0f, 0.01f, 100.0f);

	Kalman_Init(&kf_roll,  0.003f, 0.03f);
	Kalman_Init(&kf_pitch, 0.003f, 0.03f);
	Kalman_Init(&kf_yaw,   0.005f, 0.1f);

	memset(&raw, 0, sizeof(raw));
	uint32_t last = HAL_GetTick();

	printf("Initialization Complete. Entering Loop.\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		// --- 1. SPI5 COMMAND PROCESSING ---
		// The DMA is constantly filling spi_rx_buffer in the background.
		// We pass the buffer to our parser to look for 'arm', 'm1 t25', etc.
		//Process_TELEM_Command(spi_rx_buffer, sizeof(Telemetry_Packet_t));
		// --- SPI5 WATCHDOG ------------------------------------------------
		// If SPI state gets stuck or errors out (due to ESP boot noise)
		// --- 2. SPI5 WATCHDOG ------------------------------------------------
		if (hspi5.State == HAL_SPI_STATE_READY || hspi5.ErrorCode != HAL_SPI_ERROR_NONE)
		{
			HAL_SPI_Abort(&hspi5);
			hspi5.ErrorCode = HAL_SPI_ERROR_NONE;
			HAL_SPI_TransmitReceive_DMA(&hspi5, (uint8_t*)&telem_data, spi_rx_buffer, sizeof(Telemetry_Packet_t));
		}
		// ------------------------------------------------------------------
		if (command_ready) {
			Process_TELEM_Command(spi_rx_buffer, 34);
			command_ready = 0; // Clear the flag
			// Re-arm DMA here if in Normal mode
			HAL_SPI_TransmitReceive_DMA(&hspi5, (uint8_t*)&telem_data, spi_rx_buffer, 34);
		}
		// 1. Process Lidar
		Process_Lidar_DMA();

		uint32_t now = HAL_GetTick();
		// Process ESC Commands
		//ESC_SetThrottle(TIM_CHANNEL_1, target_throttle);
		//ESC_SetThrottle(TIM_CHANNEL_2, target_throttle);
		//ESC_SetThrottle(TIM_CHANNEL_3, target_throttle);
		//ESC_SetThrottle(TIM_CHANNEL_4, target_throttle);
		// 2. Run Control Loop at 100Hz (10ms)
		if (now - last >= 10) {

			float dt_sec = (now - last) / 1000.0f;
			last = now;

			// Heartbeat LED (Slow Blink to show alive)
			static uint32_t led_timer = 0;
			if (now - led_timer > 500) {
				HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
				led_timer = now;
			}
			uint32_t now = HAL_GetTick();
			// --- HEARTBEAT FAILSAFE ---
			if (now - last_heartbeat_tick > 5000) { // 500ms timeout
				target_throttle = 0.0f;
				ESC_SetThrottle(TIM_CHANNEL_1, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_2, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_3, 0.0f);
				ESC_SetThrottle(TIM_CHANNEL_4, 0.0f);
				// Optional: Toggle an Error LED
			}

			// --- Static Variables ---
			static float ax_g = 0, ay_g = 0, az_g = 0;
			static float mx_G = 0, my_G = 0, mz_G = 0;
			static float gx = 0, gy = 0, gz = 0;

			// --- Reads ---
			if (LSM303_ReadAccel(&imu, &raw)) {
				ax_g = raw.ax * imu.accel_g_per_lsb;
				ay_g = raw.ay * imu.accel_g_per_lsb;
				az_g = raw.az * imu.accel_g_per_lsb;
			}

			if (LSM303_ReadMag(&imu, &raw)) {
				mx_G = raw.mx * imu.mag_gauss_per_lsb;
				my_G = raw.my * imu.mag_gauss_per_lsb;
				mz_G = raw.mz * imu.mag_gauss_per_lsb;
			}

			if (i3gd20.initialized && I3GD20_ReadGyro(&i3gd20, &gyro_raw)) {
				float gx_phys = gyro_raw.gx * i3gd20.dps_per_lsb;
				float gy_phys = gyro_raw.gy * i3gd20.dps_per_lsb;
				float gz_phys = gyro_raw.gz * i3gd20.dps_per_lsb;

				gx = gy_phys;
				gy = gx_phys;
				gz = -gz_phys;

				Kalman_Predict(&kf_roll, gx, dt_sec);
				Kalman_Predict(&kf_pitch, gy, dt_sec);
				Kalman_Predict(&kf_yaw, gz, dt_sec);
			}

			// --- Fusion ---
			float ax = ax_g;
			float ay = ay_g;
			float az = az_g;
			float mx = mx_G - 0.24f;
			float my = -(my_G - 0.24f);
			float mz = -(mz_G + 0.08f);

			float accel_roll = atan2f(ay, az) * 57.29578f;
			float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;

			float est_roll  = Kalman_Update(&kf_roll,  accel_roll);
			float est_pitch = Kalman_Update(&kf_pitch, accel_pitch);

			float phi = est_roll * 0.0174533f;
			float theta = est_pitch * 0.0174533f;
			float By = my * cosf(phi) - mz * sinf(phi);
			float Bx = mx * cosf(theta) + (my * sinf(phi) + mz * cosf(phi)) * sinf(theta);
			float mag_yaw = atan2f(-By, Bx) * 57.29578f;

			float est_yaw = Kalman_Update(&kf_yaw, mag_yaw);

			// --- Update Telemetry (Atomic Block) ---
			telem_data.header = 0xDEADBEEF; //UINT32
			telem_data.timestamp = dt_sec; //float 1
			telem_data.roll = est_roll; //float 2
			telem_data.pitch = est_pitch; //float 3
			telem_data.yaw = est_yaw; //float 4
			telem_data.altitude = range_dist_cm; //float 5
			telem_data.voltage = 15.0f; //float 6
			telem_data.sensor_status = current_status; //UINT8
			telem_data.magic_footer = 0xAB; //UINT8
			telem_data.header = 0xDEADBEEF;
			telem_data.timestamp = dt_sec;
			telem_data.roll = est_roll;
			telem_data.pitch = est_pitch;
			telem_data.yaw = est_yaw;
			telem_data.altitude = range_dist_cm;
			telem_data.voltage = 15.0f; // Telemetry constant
			telem_data.sensor_status = current_status;
			telem_data.magic_footer = 0xAB;

			// --- Print to UART (Debug) ---
			// This will now appear on your Serial Terminal
			printf("DATA,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.4f\r\n",
					gx, gy, gz,
					ax, ay, az,
					mx, my, mz,
					range_dist_cm,
					est_roll, est_pitch, est_yaw,
					dt_sec);
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
	hi2c1.Init.ClockSpeed = 400000;
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
	htim3.Init.Prescaler = 95;
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

	/* DMA interrupt init */
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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI5) {
	        command_ready = 1;
	        // Do NOT call Process_TELEM_Command here
	    }
}

void Process_TELEM_Command(uint8_t* Buf, uint32_t Len) {
	// 1. Create a local string copy so we don't mess with the DMA memory
	char local_buf[35];
	memcpy(local_buf, Buf, (Len > 34) ? 34 : Len);
	local_buf[34] = '\0'; // Force null termination
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
		printf("!!! EMERGENCY STOP !!!\r\n");
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
	return;
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
