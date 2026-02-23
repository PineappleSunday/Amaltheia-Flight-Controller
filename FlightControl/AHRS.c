#include "AHRS.h"
#include <stdbool.h>
#include <math.h>

AHRS_Offsets_t g_offsets = {0.0f, 0.0f, 0.0f, false};
static bool accel_trust = true;
static bool mag_trust = true;
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // [w, x, y, z]
static float beta = AHRS_MADGWICK_BETA;

static float wrap_deg(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0) a += 360.0f;
    return a - 180.0f;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float vec3_norm(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static void quat_normalize(float* qv)
{
    float n = sqrtf(qv[0] * qv[0] + qv[1] * qv[1] + qv[2] * qv[2] + qv[3] * qv[3]);
    if (n < 1e-9f) {
        qv[0] = 1.0f;
        qv[1] = 0.0f;
        qv[2] = 0.0f;
        qv[3] = 0.0f;
        return;
    }
    qv[0] /= n;
    qv[1] /= n;
    qv[2] /= n;
    qv[3] /= n;
}
static float wrap_rad(float a)
{
    a = fmodf(a + (float)M_PI, 2.0f*(float)M_PI);
    if (a < 0) a += 2.0f*(float)M_PI;
    return a - (float)M_PI;
}


static void quat_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}


static void quat_to_euler_deg(const float* qv, float* roll_deg, float* pitch_deg, float* yaw_deg)
{
    const float q0 = qv[0], q1 = qv[1], q2 = qv[2], q3 = qv[3];

    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = clampf(sinp, -1.0f, 1.0f);
    float pitch = asinf(sinp);

    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    const float r2d = 57.2957795f;
    *roll_deg = roll * r2d;
    *pitch_deg = pitch * r2d;
    *yaw_deg = yaw * r2d;
}

void AHRS_Init(void)
{
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    accel_trust = true;
    mag_trust = true;

}

void AHRS_SetBeta(float b)
{
    if (b > 0.0f && b < 1.0f) {
        beta = b;
    }
}

/**
 * @brief Performs sensor fusion to update the global VehicleState.
 */
void AHRS_Update(ahrsSensor_t* raw, vehicleState_t* state, float dt)
{
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.010f) dt = 0.010f;

    // Gyro mapping aligned to body frame with accel/mag already aligned:
    // [gx_body, gy_body, gz_body] = [-Gy_sensor, Gx_sensor, Gz_sensor]
    float gx_dps = -raw->gy - g_offsets.roll_bias;  // body X
    float gy_dps =  raw->gx - g_offsets.pitch_bias; // body Y
    float gz_dps =  raw->gz - g_offsets.yaw_bias;   // body Z
    // Remap gyro to Accel/Mag Frame, also Body Frame
    float ax = raw->ax;
    float ay = raw->ay;
    float az = raw->az;

    float mx = raw->mx;
    float my = raw->my;
    float mz = raw->mz;

    static bool mag_trust = true;
    static float mag_ref = 0.0f;   // nominal |m| in your units (learned)
    static bool mag_ref_set = false;



    float a_mag = vec3_norm(ax, ay, az);
    float amag_err = fabsf(a_mag - 1.0f);
    if (accel_trust) {
        if (amag_err > 0.25f) accel_trust = false;
    } else {
        if (amag_err < 0.15f) accel_trust = true;
    }

    if (a_mag > 1e-6f) {
        ax /= a_mag;
        ay /= a_mag;
        az /= a_mag;
    } else {
        ax = 0.0f;
        ay = 0.0f;
        az = 0.0f;
    }

    float m_mag_raw = vec3_norm(mx, my, mz);
    // magnitude-based trust (raw)
    if (m_mag_raw < 1e-6f) {
        mag_trust = false;
    } else {
        if (!mag_ref_set && accel_trust) {
            mag_ref = m_mag_raw;
            mag_ref_set = true;
            mag_trust = true;
        } else if (mag_ref_set && accel_trust) {
            mag_ref = 0.999f*mag_ref + 0.001f*m_mag_raw;
        }

        if (mag_ref_set) {
            float mm_err = fabsf(m_mag_raw - mag_ref) / (mag_ref + 1e-6f);
            if (mag_trust) { if (mm_err > 0.25f) mag_trust = false; }
            else           { if (mm_err < 0.15f) mag_trust = true;  }
        } else {
            mag_trust = false;
        }
    }


    // normalize for heading use

    float mx_u=0, my_u=0, mz_u=0;
    if (m_mag_raw > 1e-6f) {
        mx_u = mx / m_mag_raw;
        my_u = my/ m_mag_raw;
        mz_u = mz / m_mag_raw;
    }


    const float d2r = 0.0174532925f;
    float gx = gx_dps * d2r;
    float gy = gy_dps * d2r;
    float gz = gz_dps * d2r;

    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    // Quaternion derivative from gyro
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    if (accel_trust) {
        float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
        float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
        float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

        // Canonical Madgwick IMU gradient: s = J^T * f
        float s0 = (-2.0f * q2) * f1 + ( 2.0f * q1) * f2;
        float s1 = ( 2.0f * q3) * f1 + ( 2.0f * q0) * f2 + (-4.0f * q1) * f3;
        float s2 = (-2.0f * q0) * f1 + ( 2.0f * q3) * f2 + (-4.0f * q2) * f3;
        float s3 = ( 2.0f * q1) * f1 + ( 2.0f * q2) * f2;

        float s_norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (s_norm > 1e-9f) {
            s0 /= s_norm;
            s1 /= s_norm;
            s2 /= s_norm;
            s3 /= s_norm;
            qDot0 -= beta * s0;
            qDot1 -= beta * s1;
            qDot2 -= beta * s2;
            qDot3 -= beta * s3;
        }
    }

    q[0] += qDot0 * dt;
    q[1] += qDot1 * dt;
    q[2] += qDot2 * dt;
    q[3] += qDot3 * dt;
    quat_normalize(q);

    if (mag_trust) {

        // 1) roll/pitch from quaternion (rad)
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        float sinr_cosp = 2.0f*(q0*q1 + q2*q3);
        float cosr_cosp = 1.0f - 2.0f*(q1*q1 + q2*q2);
        float roll = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f*(q0*q2 - q3*q1);
        sinp = clampf(sinp, -1.0f, 1.0f);
        float pitch = asinf(sinp);

        // 2) tilt-comp mag (body -> horizontal)
        float cr = cosf(roll),  sr = sinf(roll);
        float cp = cosf(pitch), sp = sinf(pitch);

        float mxh = mx_u*cp + mz_u*sp;
        float myh = mx_u*sr*sp + my_u*cr - mz_u*sr*cp;

        // 3) yaw from mag (rad)  (may need sign flip depending on your frame)
        float yaw_mag = atan2f(-myh, mxh);

        // 4) yaw estimate from quaternion (rad)
        float siny_cosp = 2.0f*(q0*q3 + q1*q2);
        float cosy_cosp = 1.0f - 2.0f*(q2*q2 + q3*q3);
        float yaw_est = atan2f(siny_cosp, cosy_cosp);

        // 5) innovation (rad!)
        float yaw_err = wrap_rad(yaw_mag - yaw_est);

        // gate
        if (fabsf(yaw_err) > 1.0f) {
            mag_trust = false;
        } else {
            const float K0 = 2.5f;
            float yaw_rate = fabsf(gz);                // rad/s
            float K = K0 / (1.0f + 2.0f*yaw_rate);

            float delta = K * yaw_err * dt;
            delta = clampf(delta, -0.05f, 0.05f);

            float half = 0.5f * delta;
            float qcorr[4] = { cosf(half), 0.0f, 0.0f, sinf(half) };

            float qnew[4];
            quat_mul(qcorr, q, qnew);
            q[0]=qnew[0]; q[1]=qnew[1]; q[2]=qnew[2]; q[3]=qnew[3];
            quat_normalize(q);
        }
    }

    float yaw_deg = 0.0f;
    float dummy_roll = 0.0f, dummy_pitch = 0.0f;
    quat_to_euler_deg(q, &dummy_roll, &dummy_pitch, &yaw_deg);

    // Build roll/pitch from predicted gravity in body frame using the same
    // convention as legacy accel equations:
    // accel_roll  = atan2(-ay, az)
    // accel_pitch = atan2(ax, sqrt(ay^2 + az^2))
    float g_bx = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float g_by = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    float g_bz = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    const float r2d = 57.2957795f;
    float roll_deg = atan2f(-g_by, g_bz) * r2d;
    float pitch_deg = atan2f(g_bx, sqrtf(g_by * g_by + g_bz * g_bz)) * r2d;

    state->roll  = roll_deg;
    state->pitch = pitch_deg;
    state->yaw   = wrap_deg(yaw_deg);

    state->q0 = q[0];
    state->q1 = q[1];
    state->q2 = q[2];
    state->q3 = q[3];

    state->gyro_x = gx;
    state->gyro_y = gy;
    state->gyro_z = gz;

    state->roll_rate  = gx_dps;
    state->pitch_rate = gy_dps;
    state->yaw_rate   = gz_dps;
}

