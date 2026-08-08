#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "AHRS.h"
#include "PID.h"
#include "bme280.h"
#include "gt_u7.h"
#include "i3gd20.h"
#include "lsm303.h"
#include "navigation.h"
#include "state.h"

#define TELEMETRY_SPI_FRAME_LEN      80U
#define TELEMETRY_HEADER_NORMAL      0xDEADBEEFUL
#define TELEMETRY_HANDSHAKE_FC       "Arbiter"
#define TELEMETRY_HANDSHAKE_ESP      "peace"
#define SENSOR_STATUS_GYRO_READY     0x01u
#define SENSOR_STATUS_LSM_READY      0x02u
#define SENSOR_STATUS_LIDAR_OK       0x08u
#define SENSOR_STATUS_MAG_OK         0x10u
#define SENSOR_STATUS_BME280_READY   0x40u
#define SENSOR_STATUS_GPS_FIX        0x20u

typedef struct __attribute__((packed)) {
	uint32_t header;
	float timestamp;
	float roll;
	float pitch;
	float yaw;
	float altitude;
	float voltage;
	uint8_t armed;
	uint8_t drone_mode;
	uint8_t flight_mode;
	uint8_t motor1_T;
	uint8_t motor2_T;
	uint8_t motor3_T;
	uint8_t motor4_T;
	uint8_t sensor_status;
	float setpoint;
	float measurement;
	float error;
	float p_term;
	float i_term;
	float d_term;
	float output_sum;
	int16_t cmd_roll_deg_x100;
	int16_t cmd_pitch_deg_x100;
	int16_t cmd_yaw_deg_x100;
	uint8_t sat_flags;
	int16_t i_state;
	int16_t gyro_p;
	int16_t gyro_q;
	int16_t gyro_r;
	uint8_t magic_footer;
} Telemetry_Packet_t;

typedef struct {
	uint32_t spi5_frame_counter;
	uint32_t spi5_cmd_rx_counter;
	uint32_t spi5_dropped_frame_counter;
	uint32_t spi5_noncmd_rx_counter;
	uint8_t spi5_last_rx0;
	uint8_t spi5_last_rx1;
	uint8_t telem_cmd_ack_status;
	uint8_t telem_cmd_ack_code;
	uint16_t telem_cmd_ack_counter;
} TelemetryCompactDiagnostics;

typedef struct {
	const vehicleState_t* state;
	const targetState_t* target;
	const droneState_t* drone_status;
	const MissionManager* mission;
	const ahrsSensor_t* raw_data;
	const BME280_Data* bme280_data;
	const GTU7_Fix* gps_fix;
	const PIDController* pid_roll_rate;
	const PIDController* pid_pitch_rate;
	const PIDController* pid_yaw_rate;
	const PIDController* pid_vel_z;
	const LSM303* imu;
	const I3GD20* gyro;
	bool bme280_ready;
	bool gps_ready;
	bool altitude_bme_bias_valid;
	bool pbit_report_active;
	bool pbit_ok;
	float bme280_status;
	float altitude_bme_bias_m;
	float altitude_bme_corrected_m;
	uint8_t pbit_mask;
	uint8_t pbit_required_mask;
	uint8_t bme280_chip_id;
	uint8_t bme280_addr;
} TelemetryVCPContext;

extern Telemetry_Packet_t telem_data;

void Telemetry_UpdateGPSStatus(bool gps_ready, bool gps_fix_valid);
void Telemetry_UpdateCompactGPSFields(bool gps_ready, const GTU7_Fix* gps_fix);
void Telemetry_UpdateCompactDiagnostics(const TelemetryCompactDiagnostics* diag, uint32_t now_ms);
void Telemetry_StageActiveFrame(
	uint8_t* spi_tx_buf,
	bool handshake_ok,
	bool mode_select_complete,
	const TelemetryCompactDiagnostics* diag,
	bool gps_ready,
	const GTU7_Fix* gps_fix,
	uint32_t now_ms);

void Telemetry_RecordLSM303AccelRaw(const uint8_t raw[6]);
void Telemetry_RecordLSM303MagRaw(const uint8_t raw[6], LSM303_Variant variant);
void Telemetry_RecordGyroRaw(const uint8_t raw[6]);

void Telemetry_VCP_TrySend(uint32_t now_ms, const TelemetryVCPContext* ctx);
void Telemetry_VCP_SendBIT(uint32_t now_ms, const TelemetryVCPContext* ctx);

#endif // TELEMETRY_H
