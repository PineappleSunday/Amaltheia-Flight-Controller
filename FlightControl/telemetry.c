#include "telemetry.h"

#include "usbd_cdc_if.h"

#include <math.h>
#include <string.h>

#define VCP_DUMP_ENABLE      1U
#define VCP_DUMP_PERIOD_MS   10U
#define VCP_DUMP_HEADER      0x31504356UL  // "VCP1"
#define VCP_ENGINEER_HEADER  0x31454356UL  // "VCE1"
#define VCP_BIT_HEADER       0x31424356UL  // "VCB1"
#define VCP_DUMP_FOOTER      0xABU

typedef struct __attribute__((packed)) {
	uint32_t header;
	uint32_t tick_ms;
	uint16_t sequence;
	uint8_t armed;
	uint8_t drone_mode;
	uint8_t flight_mode;
	uint8_t sat_flags;
	float dt_sec;

	float x, y, z;
	float vx, vy, vz;
	float roll, pitch, yaw;
	float roll_rate, pitch_rate, yaw_rate;

	float accel_x, accel_y, accel_z;
	float mag_x, mag_y, mag_z;
	float gyro_x, gyro_y, gyro_z;

	float target_x, target_y, target_z;
	float target_roll, target_pitch, target_yaw;
	float target_rate_roll, target_rate_pitch, target_rate_yaw;
	float target_ff_vz;
	float mission_wp_index, mission_wp_total, mission_wp_action, mission_wp_remaining_dist;
	float bme280_valid, bme280_temp_c, bme280_pressure_pa, bme280_humidity_rh, bme280_altitude_m;
	float gps_valid, gps_sats, gps_lat_deg, gps_lon_deg, gps_alt_m, gps_speed_mps, gps_course_deg, gps_hdop;

	float pid_roll_p, pid_roll_i, pid_roll_d, pid_roll_out;
	float pid_pitch_p, pid_pitch_i, pid_pitch_d, pid_pitch_out;
	float pid_yaw_p, pid_yaw_i, pid_yaw_d, pid_yaw_out;
	float pid_velz_p, pid_velz_i, pid_velz_d, pid_velz_out;

	float motor1_pct, motor2_pct, motor3_pct, motor4_pct;
	uint8_t magic_footer;
} VCPDumpPacket_t;

typedef struct __attribute__((packed)) {
	uint32_t header;
	uint32_t tick_ms;
	uint16_t sequence;
	uint8_t armed;
	uint8_t drone_mode;
	uint8_t flight_mode;
	uint8_t sat_flags;
	float dt_sec;

	uint8_t accel_raw_bytes[6];
	uint8_t mag_raw_bytes[6];
	uint8_t gyro_raw_bytes[6];

	int16_t accel_raw_counts[3];
	int16_t mag_raw_counts[3];
	int16_t gyro_raw_counts[3];

	float accel_x, accel_y, accel_z;
	float mag_x, mag_y, mag_z;
	float gyro_x, gyro_y, gyro_z;

	uint8_t magic_footer;
} VCPEngineerPacket_t;

typedef struct __attribute__((packed)) {
	uint32_t header;
	uint32_t tick_ms;
	uint16_t sequence;
	uint32_t timestamp_us;
	uint8_t lsm303_accel_whoami;
	uint8_t lsm303_mag_whoami_agr;
	uint8_t lsm303_mag_id_a;
	uint8_t lsm303_mag_id_b;
	uint8_t lsm303_mag_id_c;
	uint8_t lsm303_variant;
	uint8_t lsm303_init_ok;
	uint8_t gyro_whoami;
	uint8_t gyro_init_ok;
	uint8_t lsm303_accel_ctrl1;
	uint8_t lsm303_accel_ctrl4;
	uint8_t lsm303_mag_cfg_a;
	uint8_t lsm303_mag_cfg_b;
	uint8_t lsm303_mag_cfg_c;
	uint8_t lsm303_temp_cfg;
	uint8_t gyro_ctrl_reg1;
	uint8_t gyro_ctrl_reg4;
	uint8_t magic_footer;
} VCPBitPacket_t;

_Static_assert(sizeof(Telemetry_Packet_t) == 80, "Telemetry Struct size mismatch!");
_Static_assert(sizeof(VCPDumpPacket_t) == 291, "VCP dump packet size mismatch!");
_Static_assert(sizeof(VCPEngineerPacket_t) == 91, "VCP engineer packet size mismatch!");
_Static_assert(sizeof(VCPBitPacket_t) == 32, "VCP BIT packet size mismatch!");

Telemetry_Packet_t telem_data;

static VCPDumpPacket_t vcp_dump_packet;
static VCPEngineerPacket_t vcp_engineer_packet;
static VCPBitPacket_t vcp_bit_packet;
static uint16_t vcp_dump_sequence = 0;
static uint32_t vcp_last_tx_ms = 0;

static uint8_t sensor_raw_accel_bytes[6];
static uint8_t sensor_raw_mag_bytes[6];
static uint8_t sensor_raw_gyro_bytes[6];
static int16_t sensor_raw_accel_counts[3];
static int16_t sensor_raw_mag_counts[3];
static int16_t sensor_raw_gyro_counts[3];

static int16_t little_endian_i16(uint8_t lo, uint8_t hi)
{
	return (int16_t)(((uint16_t)hi << 8) | lo);
}

static int16_t big_endian_i16(uint8_t hi, uint8_t lo)
{
	return (int16_t)(((uint16_t)hi << 8) | lo);
}

static int16_t lsm303_accel_raw_count(const uint8_t raw[6], uint8_t axis)
{
	return (int16_t)(little_endian_i16(raw[(uint8_t)(axis * 2u)], raw[(uint8_t)(axis * 2u + 1u)]) >> 4);
}

static uint32_t telemetry_micros(uint32_t now_ms)
{
	return now_ms * 1000u;
}

void Telemetry_UpdateGPSStatus(bool gps_ready, bool gps_fix_valid)
{
	if (gps_ready && gps_fix_valid) {
		telem_data.sensor_status |= SENSOR_STATUS_GPS_FIX;
	} else {
		telem_data.sensor_status &= (uint8_t)~SENSOR_STATUS_GPS_FIX;
	}
}

void Telemetry_UpdateCompactGPSFields(bool gps_ready, const GTU7_Fix* gps_fix)
{
	const bool gps_valid = gps_ready && (gps_fix != 0) && gps_fix->valid;
	telem_data.setpoint = gps_valid ? 1.0f : 0.0f;
	telem_data.measurement = (gps_fix != 0) ? (float)gps_fix->satellites : 0.0f;
	telem_data.error = (gps_fix != 0) ? (float)gps_fix->latitude_deg : 0.0f;
	telem_data.p_term = (gps_fix != 0) ? (float)gps_fix->longitude_deg : 0.0f;
	telem_data.i_term = (gps_fix != 0) ? gps_fix->altitude_m : 0.0f;
	telem_data.d_term = (gps_fix != 0) ? gps_fix->speed_mps : 0.0f;
	telem_data.output_sum = (gps_fix != 0) ? gps_fix->hdop : 0.0f;
}

void Telemetry_UpdateCompactDiagnostics(const TelemetryCompactDiagnostics* diag, uint32_t now_ms)
{
	telem_data.timestamp = (float)now_ms * 0.001f;
	if (diag == 0) {
		return;
	}

	telem_data.cmd_roll_deg_x100 = (int16_t)(diag->spi5_frame_counter & 0x7FFFu);
	telem_data.cmd_pitch_deg_x100 = (int16_t)(diag->spi5_cmd_rx_counter & 0x7FFFu);
	telem_data.cmd_yaw_deg_x100 = (int16_t)(diag->spi5_dropped_frame_counter & 0x7FFFu);
	telem_data.i_state = (int16_t)(diag->spi5_noncmd_rx_counter & 0x7FFFu);
	telem_data.gyro_p = (int16_t)(((uint16_t)diag->spi5_last_rx0 << 8) | diag->spi5_last_rx1);
	telem_data.gyro_q = (int16_t)(((uint16_t)diag->telem_cmd_ack_status << 8) | diag->telem_cmd_ack_code);
	telem_data.gyro_r = (int16_t)(diag->telem_cmd_ack_counter & 0x7FFFu);
}

void Telemetry_StageActiveFrame(
	uint8_t* spi_tx_buf,
	bool handshake_ok,
	bool mode_select_complete,
	const TelemetryCompactDiagnostics* diag,
	bool gps_ready,
	const GTU7_Fix* gps_fix,
	uint32_t now_ms)
{
	if (spi_tx_buf == 0) {
		return;
	}

	if (!handshake_ok && !mode_select_complete) {
		memset(spi_tx_buf, 0, TELEMETRY_SPI_FRAME_LEN);
		memcpy(spi_tx_buf, TELEMETRY_HANDSHAKE_FC, strlen(TELEMETRY_HANDSHAKE_FC));
		return;
	}

	telem_data.header = TELEMETRY_HEADER_NORMAL;
	telem_data.magic_footer = VCP_DUMP_FOOTER;
	Telemetry_UpdateCompactGPSFields(gps_ready, gps_fix);
	Telemetry_UpdateCompactDiagnostics(diag, now_ms);
	memcpy(spi_tx_buf, &telem_data, TELEMETRY_SPI_FRAME_LEN);
}

void Telemetry_RecordLSM303AccelRaw(const uint8_t raw[6])
{
	if (raw == 0) {
		return;
	}

	memcpy(sensor_raw_accel_bytes, raw, sizeof(sensor_raw_accel_bytes));
	sensor_raw_accel_counts[0] = lsm303_accel_raw_count(raw, 0u);
	sensor_raw_accel_counts[1] = lsm303_accel_raw_count(raw, 1u);
	sensor_raw_accel_counts[2] = lsm303_accel_raw_count(raw, 2u);
}

void Telemetry_RecordLSM303MagRaw(const uint8_t raw[6], LSM303_Variant variant)
{
	if (raw == 0) {
		return;
	}

	memcpy(sensor_raw_mag_bytes, raw, sizeof(sensor_raw_mag_bytes));
	if (variant == LSM303_DLHC) {
		sensor_raw_mag_counts[0] = big_endian_i16(raw[0], raw[1]);
		sensor_raw_mag_counts[2] = big_endian_i16(raw[2], raw[3]);
		sensor_raw_mag_counts[1] = big_endian_i16(raw[4], raw[5]);
	} else {
		sensor_raw_mag_counts[0] = little_endian_i16(raw[0], raw[1]);
		sensor_raw_mag_counts[1] = little_endian_i16(raw[2], raw[3]);
		sensor_raw_mag_counts[2] = little_endian_i16(raw[4], raw[5]);
	}
}

void Telemetry_RecordGyroRaw(const uint8_t raw[6])
{
	if (raw == 0) {
		return;
	}

	memcpy(sensor_raw_gyro_bytes, raw, sizeof(sensor_raw_gyro_bytes));
	sensor_raw_gyro_counts[0] = little_endian_i16(raw[0], raw[1]);
	sensor_raw_gyro_counts[1] = little_endian_i16(raw[2], raw[3]);
	sensor_raw_gyro_counts[2] = little_endian_i16(raw[4], raw[5]);
}

void Telemetry_VCP_SendBIT(uint32_t now_ms, const TelemetryVCPContext* ctx)
{
	const LSM303* imu = (ctx != 0) ? ctx->imu : 0;
	const I3GD20* gyro = (ctx != 0) ? ctx->gyro : 0;

	vcp_bit_packet.header = VCP_BIT_HEADER;
	vcp_bit_packet.tick_ms = now_ms;
	vcp_bit_packet.sequence = vcp_dump_sequence++;
	vcp_bit_packet.timestamp_us = telemetry_micros(now_ms);
	vcp_bit_packet.lsm303_accel_whoami = (imu != 0) ? imu->accel_who_am_i : 0u;
	vcp_bit_packet.lsm303_mag_whoami_agr = (imu != 0) ? imu->mag_who_am_i_agr : 0u;
	vcp_bit_packet.lsm303_mag_id_a = (imu != 0) ? imu->mag_id_a : 0u;
	vcp_bit_packet.lsm303_mag_id_b = (imu != 0) ? imu->mag_id_b : 0u;
	vcp_bit_packet.lsm303_mag_id_c = (imu != 0) ? imu->mag_id_c : 0u;
	vcp_bit_packet.lsm303_variant = (imu != 0) ? (uint8_t)imu->variant : (uint8_t)LSM303_UNKNOWN;
	vcp_bit_packet.lsm303_init_ok = ((telem_data.sensor_status & SENSOR_STATUS_LSM_READY) != 0u) ? 1u : 0u;
	vcp_bit_packet.gyro_whoami = (gyro != 0) ? gyro->who_am_i : 0u;
	vcp_bit_packet.gyro_init_ok = ((gyro != 0) && gyro->initialized) ? 1u : 0u;
	vcp_bit_packet.lsm303_accel_ctrl1 = (imu != 0) ? imu->accel_ctrl1 : 0u;
	vcp_bit_packet.lsm303_accel_ctrl4 = (imu != 0) ? imu->accel_ctrl4 : 0u;
	vcp_bit_packet.lsm303_mag_cfg_a = (imu != 0) ? imu->mag_cfg_a : 0u;
	vcp_bit_packet.lsm303_mag_cfg_b = (imu != 0) ? imu->mag_cfg_b : 0u;
	vcp_bit_packet.lsm303_mag_cfg_c = (imu != 0) ? imu->mag_cfg_c : 0u;
	vcp_bit_packet.lsm303_temp_cfg = (imu != 0) ? imu->temp_cfg : 0u;
	vcp_bit_packet.gyro_ctrl_reg1 = (gyro != 0) ? gyro->ctrl_reg1 : 0u;
	vcp_bit_packet.gyro_ctrl_reg4 = (gyro != 0) ? gyro->ctrl_reg4 : 0u;
	vcp_bit_packet.magic_footer = VCP_DUMP_FOOTER;

	(void)CDC_Transmit_FS((uint8_t*)&vcp_bit_packet, (uint16_t)sizeof(vcp_bit_packet));
}

static void vcp_engineer_fill_and_send(uint32_t now_ms, const TelemetryVCPContext* ctx)
{
	const vehicleState_t* state = (ctx != 0) ? ctx->state : 0;
	const droneState_t* drone = (ctx != 0) ? ctx->drone_status : 0;
	const ahrsSensor_t* raw_data = (ctx != 0) ? ctx->raw_data : 0;

	vcp_engineer_packet.header = VCP_ENGINEER_HEADER;
	vcp_engineer_packet.tick_ms = now_ms;
	vcp_engineer_packet.sequence = vcp_dump_sequence++;
	vcp_engineer_packet.armed = telem_data.armed;
	vcp_engineer_packet.drone_mode = (drone != 0) ? drone->drone_mode : 0u;
	vcp_engineer_packet.flight_mode = (drone != 0) ? drone->flight_mode : 0u;
	vcp_engineer_packet.sat_flags = (ctx != 0) ? ctx->pbit_mask : telem_data.sat_flags;
	vcp_engineer_packet.dt_sec = (state != 0) ? state->dt_sec : 0.0f;

	memcpy(vcp_engineer_packet.accel_raw_bytes, sensor_raw_accel_bytes, sizeof(vcp_engineer_packet.accel_raw_bytes));
	memcpy(vcp_engineer_packet.mag_raw_bytes, sensor_raw_mag_bytes, sizeof(vcp_engineer_packet.mag_raw_bytes));
	memcpy(vcp_engineer_packet.gyro_raw_bytes, sensor_raw_gyro_bytes, sizeof(vcp_engineer_packet.gyro_raw_bytes));
	memcpy(vcp_engineer_packet.accel_raw_counts, sensor_raw_accel_counts, sizeof(vcp_engineer_packet.accel_raw_counts));
	memcpy(vcp_engineer_packet.mag_raw_counts, sensor_raw_mag_counts, sizeof(vcp_engineer_packet.mag_raw_counts));
	memcpy(vcp_engineer_packet.gyro_raw_counts, sensor_raw_gyro_counts, sizeof(vcp_engineer_packet.gyro_raw_counts));

	vcp_engineer_packet.accel_x = (raw_data != 0) ? raw_data->ax : 0.0f;
	vcp_engineer_packet.accel_y = (raw_data != 0) ? raw_data->ay : 0.0f;
	vcp_engineer_packet.accel_z = (raw_data != 0) ? raw_data->az : 0.0f;
	vcp_engineer_packet.mag_x = (raw_data != 0) ? raw_data->mx : 0.0f;
	vcp_engineer_packet.mag_y = (raw_data != 0) ? raw_data->my : 0.0f;
	vcp_engineer_packet.mag_z = (raw_data != 0) ? raw_data->mz : 0.0f;
	vcp_engineer_packet.gyro_x = (raw_data != 0) ? raw_data->gx : 0.0f;
	vcp_engineer_packet.gyro_y = (raw_data != 0) ? raw_data->gy : 0.0f;
	vcp_engineer_packet.gyro_z = (raw_data != 0) ? raw_data->gz : 0.0f;
	vcp_engineer_packet.magic_footer = VCP_DUMP_FOOTER;

	(void)CDC_Transmit_FS((uint8_t*)&vcp_engineer_packet, (uint16_t)sizeof(vcp_engineer_packet));
}

void Telemetry_VCP_TrySend(uint32_t now_ms, const TelemetryVCPContext* ctx)
{
#if VCP_DUMP_ENABLE
	if ((now_ms - vcp_last_tx_ms) < VCP_DUMP_PERIOD_MS) {
		return;
	}
	vcp_last_tx_ms = now_ms;

	const vehicleState_t* state = (ctx != 0) ? ctx->state : 0;
	const targetState_t* target = (ctx != 0) ? ctx->target : 0;
	const droneState_t* drone = (ctx != 0) ? ctx->drone_status : 0;
	const MissionManager* mission = (ctx != 0) ? ctx->mission : 0;
	const ahrsSensor_t* raw_data = (ctx != 0) ? ctx->raw_data : 0;
	const BME280_Data* bme = (ctx != 0) ? ctx->bme280_data : 0;
	const GTU7_Fix* gps = (ctx != 0) ? ctx->gps_fix : 0;

	if ((drone != 0) && drone->drone_mode == MODE_ENGINEER) {
		vcp_engineer_fill_and_send(now_ms, ctx);
		return;
	}

	vcp_dump_packet.header = VCP_DUMP_HEADER;
	vcp_dump_packet.tick_ms = now_ms;
	vcp_dump_packet.sequence = vcp_dump_sequence++;
	vcp_dump_packet.armed = telem_data.armed;
	vcp_dump_packet.drone_mode = (drone != 0) ? drone->drone_mode : 0u;
	vcp_dump_packet.flight_mode = ((ctx != 0) && ctx->pbit_report_active)
			? (ctx->pbit_ok ? FLIGHT_MODE_PBIT_OK : FLIGHT_MODE_PBIT_FAIL)
			: ((drone != 0) ? drone->flight_mode : 0u);
	vcp_dump_packet.sat_flags = telem_data.sat_flags;
	vcp_dump_packet.dt_sec = (state != 0) ? state->dt_sec : 0.0f;

	vcp_dump_packet.x = (state != 0) ? state->x : 0.0f;
	vcp_dump_packet.y = (state != 0) ? state->y : 0.0f;
	vcp_dump_packet.z = (state != 0) ? state->z : 0.0f;
	vcp_dump_packet.vx = (state != 0) ? state->vx : 0.0f;
	vcp_dump_packet.vy = (state != 0) ? state->vy : 0.0f;
	vcp_dump_packet.vz = (state != 0) ? state->vz : 0.0f;
	vcp_dump_packet.roll = (state != 0) ? state->roll : 0.0f;
	vcp_dump_packet.pitch = (state != 0) ? state->pitch : 0.0f;
	vcp_dump_packet.yaw = (state != 0) ? state->yaw : 0.0f;
	vcp_dump_packet.roll_rate = (state != 0) ? state->roll_rate : 0.0f;
	vcp_dump_packet.pitch_rate = (state != 0) ? state->pitch_rate : 0.0f;
	vcp_dump_packet.yaw_rate = (state != 0) ? state->yaw_rate : 0.0f;

	vcp_dump_packet.accel_x = (raw_data != 0) ? raw_data->ax : 0.0f;
	vcp_dump_packet.accel_y = (raw_data != 0) ? raw_data->ay : 0.0f;
	vcp_dump_packet.accel_z = (raw_data != 0) ? raw_data->az : 0.0f;
	vcp_dump_packet.mag_x = (raw_data != 0) ? raw_data->mx : 0.0f;
	vcp_dump_packet.mag_y = (raw_data != 0) ? raw_data->my : 0.0f;
	vcp_dump_packet.mag_z = (raw_data != 0) ? raw_data->mz : 0.0f;
	vcp_dump_packet.gyro_x = (raw_data != 0) ? raw_data->gx : 0.0f;
	vcp_dump_packet.gyro_y = (raw_data != 0) ? raw_data->gy : 0.0f;
	vcp_dump_packet.gyro_z = (raw_data != 0) ? raw_data->gz : 0.0f;

	vcp_dump_packet.target_x = (target != 0) ? target->x : 0.0f;
	vcp_dump_packet.target_y = (target != 0) ? target->y : 0.0f;
	vcp_dump_packet.target_z = (target != 0) ? target->z : 0.0f;
	vcp_dump_packet.target_roll = (target != 0) ? target->roll : 0.0f;
	vcp_dump_packet.target_pitch = (target != 0) ? target->pitch : 0.0f;
	vcp_dump_packet.target_yaw = (target != 0) ? target->yaw : 0.0f;
	vcp_dump_packet.target_rate_roll = (target != 0) ? target->rate_roll : 0.0f;
	vcp_dump_packet.target_rate_pitch = (target != 0) ? target->rate_pitch : 0.0f;
	vcp_dump_packet.target_rate_yaw = (target != 0) ? target->rate_yaw : 0.0f;
	vcp_dump_packet.target_ff_vz = (target != 0) ? target->ff_vz : 0.0f;

	if ((mission != 0) && mission->waypoints != 0 && mission->current_index < mission->total_waypoints && state != 0) {
		const Waypoint* wp = &mission->waypoints[mission->current_index];
		float dx = wp->position[0] - state->x;
		float dy = wp->position[1] - state->y;
		float dz = wp->position[2] - state->z;

		vcp_dump_packet.mission_wp_index = (float)mission->current_index;
		vcp_dump_packet.mission_wp_total = (float)mission->total_waypoints;
		vcp_dump_packet.mission_wp_action = (float)wp->action;
		vcp_dump_packet.mission_wp_remaining_dist = sqrtf(dx*dx + dy*dy + dz*dz);
	} else {
		vcp_dump_packet.mission_wp_index = (mission != 0) ? (float)mission->current_index : 0.0f;
		vcp_dump_packet.mission_wp_total = (mission != 0) ? (float)mission->total_waypoints : 0.0f;
		vcp_dump_packet.mission_wp_action = -1.0f;
		vcp_dump_packet.mission_wp_remaining_dist = 0.0f;
	}

	vcp_dump_packet.bme280_valid = (ctx != 0) ? ctx->bme280_status : 0.0f;
	vcp_dump_packet.bme280_temp_c = ((ctx != 0) && ctx->bme280_ready && bme != 0) ? bme->temperature_c : (float)((ctx != 0) ? ctx->bme280_chip_id : 0u);
	vcp_dump_packet.bme280_pressure_pa = ((ctx != 0) && ctx->bme280_ready && bme != 0) ? bme->pressure_pa : (float)((ctx != 0) ? ctx->bme280_addr : 0u);
	vcp_dump_packet.bme280_humidity_rh = (bme != 0) ? bme->humidity_rh : 0.0f;
	vcp_dump_packet.bme280_altitude_m = ((ctx != 0) && ctx->altitude_bme_bias_valid)
			? ctx->altitude_bme_corrected_m
			: ((bme != 0) ? bme->altitude_m : 0.0f);
	vcp_dump_packet.gps_valid = ((ctx != 0) && ctx->gps_ready && gps != 0 && gps->valid) ? 1.0f : 0.0f;
	vcp_dump_packet.gps_sats = (gps != 0) ? (float)gps->satellites : 0.0f;
	vcp_dump_packet.gps_lat_deg = (gps != 0) ? (float)gps->latitude_deg : 0.0f;
	vcp_dump_packet.gps_lon_deg = (gps != 0) ? (float)gps->longitude_deg : 0.0f;
	vcp_dump_packet.gps_alt_m = (gps != 0) ? gps->altitude_m : 0.0f;
	vcp_dump_packet.gps_speed_mps = (gps != 0) ? gps->speed_mps : 0.0f;
	vcp_dump_packet.gps_course_deg = (gps != 0) ? gps->course_deg : 0.0f;
	vcp_dump_packet.gps_hdop = (gps != 0) ? gps->hdop : 0.0f;

	const PIDController* pr = (ctx != 0) ? ctx->pid_roll_rate : 0;
	const PIDController* pp = (ctx != 0) ? ctx->pid_pitch_rate : 0;
	const PIDController* py = (ctx != 0) ? ctx->pid_yaw_rate : 0;
	const PIDController* pz = (ctx != 0) ? ctx->pid_vel_z : 0;

	vcp_dump_packet.pid_roll_p = (pr != 0) ? pr->p_out : 0.0f;
	vcp_dump_packet.pid_roll_i = (pr != 0) ? pr->i_out : 0.0f;
	vcp_dump_packet.pid_roll_d = (pr != 0) ? pr->d_out : 0.0f;
	vcp_dump_packet.pid_roll_out = (pr != 0) ? pr->output : 0.0f;
	vcp_dump_packet.pid_pitch_p = (pp != 0) ? pp->p_out : 0.0f;
	vcp_dump_packet.pid_pitch_i = (pp != 0) ? pp->i_out : 0.0f;
	vcp_dump_packet.pid_pitch_d = (pp != 0) ? pp->d_out : 0.0f;
	vcp_dump_packet.pid_pitch_out = (pp != 0) ? pp->output : 0.0f;
	vcp_dump_packet.pid_yaw_p = (py != 0) ? py->p_out : 0.0f;
	vcp_dump_packet.pid_yaw_i = (py != 0) ? py->i_out : 0.0f;
	vcp_dump_packet.pid_yaw_d = (py != 0) ? py->d_out : 0.0f;
	vcp_dump_packet.pid_yaw_out = (py != 0) ? py->output : 0.0f;
	vcp_dump_packet.pid_velz_p = (pz != 0) ? pz->p_out : 0.0f;
	vcp_dump_packet.pid_velz_i = (pz != 0) ? pz->i_out : 0.0f;
	vcp_dump_packet.pid_velz_d = (pz != 0) ? pz->d_out : 0.0f;
	vcp_dump_packet.pid_velz_out = (pz != 0) ? pz->output : 0.0f;

	vcp_dump_packet.motor1_pct = telem_data.motor1_T;
	vcp_dump_packet.motor2_pct = telem_data.motor2_T;
	vcp_dump_packet.motor3_pct = telem_data.motor3_T;
	vcp_dump_packet.motor4_pct = telem_data.motor4_T;
	vcp_dump_packet.magic_footer = VCP_DUMP_FOOTER;

	(void)CDC_Transmit_FS((uint8_t*)&vcp_dump_packet, (uint16_t)sizeof(vcp_dump_packet));
#endif
}
