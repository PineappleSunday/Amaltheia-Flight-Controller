#include "AHRS.h"
#include <stdbool.h>
#include <math.h>

/*
 * FC / AHRS estimator frame established by calibration:
 *
 *   AHRS +X = FC +X = Accel X = Gyro X = Mag X = toward M3
 *   AHRS +Y = FC +Y = Accel Y = Gyro Y = Mag Y = toward M4
 *   AHRS +Z = FC +Z = Accel Z = Gyro Z = Mag Z
 *
 * The operational aircraft frame is rotated 180 deg about +Z:
 *
 *   Aircraft +X = toward M1 (nose)
 *   Aircraft +Y = toward M2 (right)
 *
 * AHRS remains in the calibrated FC frame. Conversion to the aircraft
 * control frame occurs downstream before flight-control PID calculations.
 */

static bool accel_trust = true;
static bool mag_trust = true;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /* [w, x, y, z] */
static float beta = AHRS_MADGWICK_BETA;

/* Magnetometer field-strength reference. */
static float mag_ref = 0.0f;
static bool mag_ref_set = false;

static float wrap_deg(float a)
{
    a = fmodf(a + 180.0f, 360.0f);

    if (a < 0.0f) {
        a += 360.0f;
    }

    return a - 180.0f;
}

static float wrap_rad(float a)
{
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);

    if (a < 0.0f) {
        a += 2.0f * (float)M_PI;
    }

    return a - (float)M_PI;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }

    if (x > hi) {
        return hi;
    }

    return x;
}

static float vec3_norm(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static void quat_normalize(float *qv)
{
    const float n = sqrtf(
        qv[0] * qv[0] +
        qv[1] * qv[1] +
        qv[2] * qv[2] +
        qv[3] * qv[3]
    );

    if (n < 1e-9f) {
        qv[0] = 1.0f;
        qv[1] = 0.0f;
        qv[2] = 0.0f;
        qv[3] = 0.0f;
        return;
    }

    const float inv_n = 1.0f / n;

    qv[0] *= inv_n;
    qv[1] *= inv_n;
    qv[2] *= inv_n;
    qv[3] *= inv_n;
}

// Quaternion multiplication: out = a * b
static void quat_mul(
    const float a[4],
    const float b[4],
    float out[4]
)
{
    out[0] =
        a[0] * b[0] -
        a[1] * b[1] -
        a[2] * b[2] -
        a[3] * b[3];

    out[1] =
        a[0] * b[1] +
        a[1] * b[0] +
        a[2] * b[3] -
        a[3] * b[2];

    out[2] =
        a[0] * b[2] -
        a[1] * b[3] +
        a[2] * b[0] +
        a[3] * b[1];

    out[3] =
        a[0] * b[3] +
        a[1] * b[2] -
        a[2] * b[1] +
        a[3] * b[0];
}

static void quat_to_euler_deg(
    const float *qv,
    float *roll_deg,
    float *pitch_deg,
    float *yaw_deg
)
{
    const float q0 = qv[0];
    const float q1 = qv[1];
    const float q2 = qv[2];
    const float q3 = qv[3];

    /* Roll: rotation about body X. */
    const float sinr_cosp =
        2.0f * (q0 * q1 + q2 * q3);

    const float cosr_cosp =
        1.0f - 2.0f * (q1 * q1 + q2 * q2);

    const float roll =
        atan2f(sinr_cosp, cosr_cosp);

    /* Pitch: rotation about body Y. */
    float sinp =
        2.0f * (q0 * q2 - q3 * q1);

    sinp = clampf(sinp, -1.0f, 1.0f);

    const float pitch = asinf(sinp);

    /* Yaw: rotation about body Z. */
    const float siny_cosp =
        2.0f * (q0 * q3 + q1 * q2);

    const float cosy_cosp =
        1.0f - 2.0f * (q2 * q2 + q3 * q3);

    const float yaw =
        atan2f(siny_cosp, cosy_cosp);

    const float r2d = 57.2957795f;

    *roll_deg  = roll  * r2d;
    *pitch_deg = pitch * r2d;
    *yaw_deg   = yaw   * r2d;
}

void AHRS_Init(void)
{
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;

    accel_trust = true;
    mag_trust = true;

    mag_ref = 0.0f;
    mag_ref_set = false;
}

void AHRS_SetBeta(float b)
{
    if (b > 0.0f && b < 1.0f) {
        beta = b;
    }
}

/**
 * @brief Update attitude estimate.
 *
 * Input requirements:
 *
 *   raw->ax, ay, az : calibrated acceleration in body frame, units of g
 *   raw->gx, gy, gz : calibrated angular rate in body frame, deg/s
 *   raw->mx, my, mz : calibrated magnetic field in body frame
 *
 * Level stationary vehicle should read approximately:
 *
 *   accel = [0, 0, +1] g
 */
void AHRS_Update(
    ahrsSensor_t *raw,
    vehicleState_t *state,
    float dt
)
{
    if (raw == NULL || state == NULL) {
        return;
    }

    /*
     * Do not silently truncate a large elapsed time.
     * Reject obviously invalid scheduling intervals instead.
     *
     * Normal flight operation is expected to be much faster than 50 ms.
     */
    if (dt <= 0.0f || dt > 0.050f) {
        return;
    }

    /*
     * -------------------------------------------------------------------------
     * SENSOR INPUT
     * -------------------------------------------------------------------------
     *
     * Calibration testing showed:
     *
     *   Accel X = Gyro X = Mag X = body X
     *   Accel Y = Gyro Y = Mag Y = body Y
     *   Accel Z = Gyro Z = Mag Z = body Z
     *
     * Therefore there is no remapping here.
     */

    const float gx_dps = raw->gx;
    const float gy_dps = raw->gy;
    const float gz_dps = raw->gz;

    float ax = raw->ax;
    float ay = raw->ay;
    float az = raw->az;

    const float mx = raw->mx;
    const float my = raw->my;
    const float mz = raw->mz;

    /*
     * -------------------------------------------------------------------------
     * ACCELEROMETER TRUST / NORMALIZATION
     * -------------------------------------------------------------------------
     */

    const float a_mag = vec3_norm(ax, ay, az);

    float accel_weight = 0.0f;

    if (a_mag > 1e-6f) {
        const float amag_err =
            fabsf(a_mag - 1.0f);

        /*
         * Full accel correction:
         *     |a| error <= 0.10 g
         *
         * Correction fades to zero:
         *     0.10 g -> 0.25 g
         *
         * No accel correction:
         *     |a| error >= 0.25 g
         */
        accel_weight =
            1.0f -
            clampf(
                (amag_err - 0.10f) / 0.15f,
                0.0f,
                1.0f
            );

        /*
         * Separate hysteretic flag retained for diagnostics and
         * initialization logic.
         */
        if (accel_trust) {
            if (amag_err > 0.25f) {
                accel_trust = false;
            }
        } else {
            if (amag_err < 0.15f) {
                accel_trust = true;
            }
        }

        const float inv_a_mag = 1.0f / a_mag;

        ax *= inv_a_mag;
        ay *= inv_a_mag;
        az *= inv_a_mag;
    } else {
        ax = 0.0f;
        ay = 0.0f;
        az = 0.0f;

        accel_weight = 0.0f;
        accel_trust = false;
    }

    /*
     * -------------------------------------------------------------------------
     * MAGNETOMETER TRUST
     * -------------------------------------------------------------------------
     *
     * IMPORTANT:
     * hard-iron / soft-iron calibration should be applied BEFORE this function.
     */

    const float m_mag_raw =
        vec3_norm(mx, my, mz);

    float mx_u = 0.0f;
    float my_u = 0.0f;
    float mz_u = 0.0f;

    if (m_mag_raw > 1e-6f) {

        /*
         * Establish field-strength reference once.
         *
         * Do not continuously adapt the reference during flight; otherwise
         * a sustained magnetic disturbance can slowly become "normal".
         */
        if (!mag_ref_set && accel_trust) {
            mag_ref = m_mag_raw;
            mag_ref_set = true;
            mag_trust = true;
        }

        if (mag_ref_set) {
            const float mm_err =
                fabsf(m_mag_raw - mag_ref) /
                (mag_ref + 1e-6f);

            /*
             * Hysteresis:
             * >25% field-strength deviation -> reject
             * <15% deviation -> allow recovery
             */
            if (mag_trust) {
                if (mm_err > 0.25f) {
                    mag_trust = false;
                }
            } else {
                if (mm_err < 0.15f) {
                    mag_trust = true;
                }
            }
        } else {
            mag_trust = false;
        }

        const float inv_m =
            1.0f / m_mag_raw;

        mx_u = mx * inv_m;
        my_u = my * inv_m;
        mz_u = mz * inv_m;

    } else {
        mag_trust = false;
    }

    /*
     * -------------------------------------------------------------------------
     * GYROSCOPE PROPAGATION
     * -------------------------------------------------------------------------
     */

    const float d2r = 0.017453292519943295f;

    const float gx = gx_dps * d2r;
    const float gy = gy_dps * d2r;
    const float gz = gz_dps * d2r;

    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    float qDot0 =
        0.5f * (-q1 * gx - q2 * gy - q3 * gz);

    float qDot1 =
        0.5f * ( q0 * gx + q2 * gz - q3 * gy);

    float qDot2 =
        0.5f * ( q0 * gy - q1 * gz + q3 * gx);

    float qDot3 =
        0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /*
     * -------------------------------------------------------------------------
     * MADGWICK ACCELEROMETER CORRECTION
     * -------------------------------------------------------------------------
     */

    if (accel_weight > 0.001f) {
        /*
        * Accelerometer vector used by the Madgwick gravity objective.
        *
        * IMPORTANT:
        * This is NOT a sensor-axis or body-frame remapping.
        *
        * Calibration and rotation testing established that the physical
        * sensor axes are already aligned:
        *
        *     accel X <-> gyro X <-> body X
        *     accel Y <-> gyro Y <-> body Y
        *     accel Z <-> gyro Z <-> body Z
        *
        * The quaternion propagation also uses those body axes directly.
        *
        * However, testing showed that the horizontal accelerometer
        * components have the opposite sign from the gravity-vector
        * convention assumed by the Madgwick objective below.
        *
        * Therefore only the vector presented to the gravity correction
        * is converted:
        *
        *     Madgwick X = -Accel X
        *     Madgwick Y = -Accel Y
        *     Madgwick Z = +Accel Z
        *
        * Z remains positive because the calibrated level orientation
        * measures approximately [0, 0, +1 g], which matches the
        * identity-quaternion gravity prediction [0, 0, +1].
        *
        * This convention was validated experimentally by comparing
        * AHRS roll/pitch against gyro body rates and accel-derived
        * attitude with the original nonzero beta.
        */
        const float ax_mad = -ax;
        const float ay_mad = -ay;
        const float az_mad =  az;
        /*
         * Gravity-vector residual.
         *
         * At q = [1, 0, 0, 0]:
         *
         *     predicted gravity = [0, 0, +1]
         */
        const float f1 =
            2.0f * (q1 * q3 - q0 * q2) - ax_mad;

        const float f2 =
            2.0f * (q0 * q1 + q2 * q3) - ay_mad;

        const float f3 =
            2.0f * (0.5f - q1 * q1 - q2 * q2) - az_mad;

        const float s0 =
            (-2.0f * q2) * f1 +
            ( 2.0f * q1) * f2;

        const float s1 =
            ( 2.0f * q3) * f1 +
            ( 2.0f * q0) * f2 +
            (-4.0f * q1) * f3;

        const float s2 =
            (-2.0f * q0) * f1 +
            ( 2.0f * q3) * f2 +
            (-4.0f * q2) * f3;

        const float s3 =
            ( 2.0f * q1) * f1 +
            ( 2.0f * q2) * f2;

        const float s_norm =
            sqrtf(
                s0 * s0 +
                s1 * s1 +
                s2 * s2 +
                s3 * s3
            );

        if (s_norm > 1e-9f) {

            const float beta_eff =
                beta * accel_weight;

            const float inv_s =
                1.0f / s_norm;

            qDot0 -= beta_eff * s0 * inv_s;
            qDot1 -= beta_eff * s1 * inv_s;
            qDot2 -= beta_eff * s2 * inv_s;
            qDot3 -= beta_eff * s3 * inv_s;
        }
    }

    /*
     * Integrate quaternion.
     */
    q[0] += qDot0 * dt;
    q[1] += qDot1 * dt;
    q[2] += qDot2 * dt;
    q[3] += qDot3 * dt;

    quat_normalize(q);

    /*
     * -------------------------------------------------------------------------
     * MAGNETOMETER YAW CORRECTION
     * -------------------------------------------------------------------------
     */

    if (mag_trust) {

        q0 = q[0];
        q1 = q[1];
        q2 = q[2];
        q3 = q[3];

        /*
         * Current roll / pitch from quaternion.
         */
        const float sinr_cosp =
            2.0f * (q0 * q1 + q2 * q3);

        const float cosr_cosp =
            1.0f - 2.0f * (q1 * q1 + q2 * q2);

        const float roll =
            atan2f(sinr_cosp, cosr_cosp);

        float sinp =
            2.0f * (q0 * q2 - q3 * q1);

        sinp =
            clampf(sinp, -1.0f, 1.0f);

        const float pitch =
            asinf(sinp);

        /*
         * Tilt compensation.
         */
        const float cr = cosf(roll);
        const float sr = sinf(roll);
        const float cp = cosf(pitch);
        const float sp = sinf(pitch);

        const float mxh =
            mx_u * cp +
            mz_u * sp;

        const float myh =
            mx_u * sr * sp +
            my_u * cr -
            mz_u * sr * cp;

        /*
         * Magnetic heading.
         *
         * NOTE:
         * The "-myh" sign must still be confirmed by the final positive-Z
         * heading test on the assembled quad.
         */
        const float yaw_mag =
            atan2f(-myh, mxh);

        /*
         * Quaternion yaw estimate.
         */
        const float siny_cosp =
            2.0f * (q0 * q3 + q1 * q2);

        const float cosy_cosp =
            1.0f - 2.0f * (q2 * q2 + q3 * q3);

        const float yaw_est =
            atan2f(siny_cosp, cosy_cosp);

        const float yaw_err =
            wrap_rad(yaw_mag - yaw_est);

        /*
         * Gross innovation gate.
         */
        if (fabsf(yaw_err) > 1.0f) {

            mag_trust = false;

        } else {

            /*
             * Reduced heading correction gain to avoid yaw "rubber banding".
             */
            const float K0 = 0.5f;

            const float yaw_rate =
                fabsf(gz);

            const float K =
                K0 /
                (1.0f + 2.0f * yaw_rate);

            float delta =
                K * yaw_err * dt;

            delta =
                clampf(
                    delta,
                    -0.05f,
                    0.05f
                );

            const float half =
                0.5f * delta;

            const float qcorr[4] = {
                cosf(half),
                0.0f,
                0.0f,
                sinf(half)
            };

            float qnew[4];

            quat_mul(
                qcorr,
                q,
                qnew
            );

            q[0] = qnew[0];
            q[1] = qnew[1];
            q[2] = qnew[2];
            q[3] = qnew[3];

            quat_normalize(q);
        }
    }

    /*
     * -------------------------------------------------------------------------
     * OUTPUT STATE
     * -------------------------------------------------------------------------
     */

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    quat_to_euler_deg(
        q,
        &roll_deg,
        &pitch_deg,
        &yaw_deg
    );

    state->roll  = roll_deg;
    state->pitch = pitch_deg;
    state->yaw   = wrap_deg(yaw_deg);

    state->q0 = q[0];
    state->q1 = q[1];
    state->q2 = q[2];
    state->q3 = q[3];

    /*
     * Store angular velocity in radians/sec.
     */
    state->gyro_x = gx;
    state->gyro_y = gy;
    state->gyro_z = gz;

    /*
     * Controller-facing body rates remain degrees/sec.
     */
    state->roll_rate  = gx_dps;
    state->pitch_rate = gy_dps;
    state->yaw_rate   = gz_dps;
}