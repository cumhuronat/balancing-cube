#ifndef quat_utils_h
#define quat_utils_h

#include <math.h>

// Pure quaternion helpers shared by the firmware and the host-side tests
// (software/tests/test_quat_utils.cpp). No Arduino dependencies. Hamilton
// convention throughout, matching the rest of the codebase: the estimate q
// maps body to world and the body-frame up direction is the third row of the
// rotation matrix R(q).

// One accelerometer correction step with gain k = lds * dt. Inputs ax, ay, az
// must be normalized. Updates q in place; the caller renormalizes q afterwards
// (AttitudeEstimator::estimate() always has). Returns false when the geometry
// is numerically singular and the step was skipped.
// The correction attracts the estimate to the measured gravity direction only
// while the up-vector error is below 90 degrees (beyond that this Gibbs
// construction pushes toward the antipode). Boot initialization via
// quat_from_accel() and the fusion validity window keep the estimator deep
// inside this basin.
static inline bool quat_accel_correct(float k, float ax, float ay, float az,
        float& q0, float& q1, float& q2, float& q3) {
    // Calculate rotation quaternion measurement
    float qm0 =  ax * q2 - ay * q1 - az * q0;
    float qm1 = -ax * q3 - ay * q0 + az * q1;
    float qm2 =  ax * q0 - ay * q3 + az * q2;
    float qm3 = -ax * q1 - ay * q2 - az * q3;

    // Calculate rotation quaternion error
    float qe0 = q0 * qm0 + q1 * qm1 + q2 * qm2 + q3 * qm3;
    float qe1 = q0 * qm1 - q1 * qm0 - q2 * qm3 + q3 * qm2;
    float qe2 = q0 * qm2 + q1 * qm3 - q2 * qm0 - q3 * qm1;
    float qe3 = q0 * qm3 - q1 * qm2 + q2 * qm1 - q3 * qm0;

    // Skip when numerically singular (up-vector error at exactly 90 degrees)
    if (fabsf(qe0) < 1e-6f) {
        return false;
    }

    // Calculate rotation Gibbs-vector error
    float se1 = qe1 / qe0;
    float se2 = qe2 / qe0;
    float se3 = qe3 / qe0;

    // Bound the Gibbs vector: it diverges as the up-vector error approaches
    // 90 degrees. The bound leaves errors below 30 degrees untouched.
    float se_mag = sqrtf(se1 * se1 + se2 * se2 + se3 * se3);
    const float se_max = 0.5774f; // tan(30 deg)
    if (se_mag > se_max) {
        float f = se_max / se_mag;
        se1 *= f;
        se2 *= f;
        se3 *= f;
    }

    // Calculate rotation quaternion error time derivative
    float qe0_dot = -q1 * se1 - q2 * se2 - q3 * se3;
    float qe1_dot =  q0 * se1 - q3 * se2 + q2 * se3;
    float qe2_dot =  q3 * se1 + q0 * se2 - q1 * se3;
    float qe3_dot = -q2 * se1 + q1 * se2 + q0 * se3;

    // Correct rotation quaternion
    q0 += k * qe0_dot;
    q1 += k * qe1_dot;
    q2 += k * qe2_dot;
    q3 += k * qe3_dot;
    return true;
}

// Attitude from a raw (not necessarily normalized) rest accelerometer reading:
// the shortest-arc rotation whose body-frame up matches the measurement. Yaw
// is arbitrary (it is unobservable and later pinned by quat_pin_yaw). Falls
// back to a pure roll when the reading points nearly straight down.
static inline void quat_from_accel(float ax, float ay, float az,
        float& q0, float& q1, float& q2, float& q3) {
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > 0.0f) {
        ax /= an;
        ay /= an;
        az /= an;
    }
    if (az < -0.9999f) {
        // Nearly upside down: shortest arc is ill-conditioned, use a 180
        // degree roll about the body x axis
        q0 = 0.0f;
        q1 = 1.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        return;
    }
    q0 = 1.0f + az;
    q1 = ay;
    q2 = -ax;
    q3 = 0.0f;
    float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= n;
    q1 /= n;
    q2 /= n;
    q3 /= n;
}

// Pin the (unobservable) world-yaw of the estimate q onto the yaw orbit of the
// reference qu: left-multiplies q by the conjugate of the optimal world-z
// twist, so that the error conj(q) (x) qu becomes tilt-only with non-negative
// scalar part. Verified to be the minimizer of the error angle over all world
// yaws, to leave the body-frame up direction untouched, and to be immune to
// the quaternion double cover. Returns false (q untouched) when the
// configuration is degenerate (tilt error near 180 degrees).
static inline bool quat_pin_yaw(float qu0, float qu1, float qu2, float qu3,
        float& q0, float& q1, float& q2, float& q3) {
    float c = q0 * qu0 + q1 * qu1 + q2 * qu2 + q3 * qu3;
    float s = -q0 * qu3 - q1 * qu2 + q2 * qu1 + q3 * qu0;
    float n = sqrtf(c * c + s * s);
    if (n < 0.5f) {
        return false;
    }
    float w = c / n;
    float z = -s / n;
    float p0 = w * q0 - z * q3;
    float p1 = w * q1 - z * q2;
    float p2 = w * q2 + z * q1;
    float p3 = w * q3 + z * q0;
    float pn = sqrtf(p0 * p0 + p1 * p1 + p2 * p2 + p3 * p3);
    q0 = p0 / pn;
    q1 = p1 / pn;
    q2 = p2 / pn;
    q3 = p3 / pn;
    return true;
}


// Compose a small body-frame rotation (rotation vector rx, ry, rz in rad)
// onto the right side of a reference quaternion: out = q (x) exp(r/2).
// Used to apply the learned auto-trim to the trajectory reference; a
// body-frame (right-side) composition stays correct through the yaw spin
// trajectory. Output is renormalized.
static inline void quat_compose_body(float q0, float q1, float q2, float q3,
        float rx, float ry, float rz, float& o0, float& o1, float& o2, float& o3) {
    float ang = sqrtf(rx * rx + ry * ry + rz * rz);
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    if (ang > 1e-9f) {
        float h = ang * 0.5f;
        float s = sinf(h) / ang;
        w = cosf(h);
        x = rx * s;
        y = ry * s;
        z = rz * s;
    }
    o0 = q0 * w - q1 * x - q2 * y - q3 * z;
    o1 = q0 * x + q1 * w + q2 * z - q3 * y;
    o2 = q0 * y - q1 * z + q2 * w + q3 * x;
    o3 = q0 * z + q1 * y - q2 * x + q3 * w;
    float n = sqrtf(o0 * o0 + o1 * o1 + o2 * o2 + o3 * o3);
    o0 /= n;
    o1 /= n;
    o2 /= n;
    o3 /= n;
}

// Body-frame "up" direction implied by the attitude quaternion: the third
// row of R(q). Shared by the landing phase machine and the host tests.
static inline void quat_body_up(float q0, float q1, float q2, float q3,
        float& ux, float& uy, float& uz) {
    ux = 2.0f * (q1 * q3 - q0 * q2);
    uy = 2.0f * (q2 * q3 + q0 * q1);
    uz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

// Angle (rad) between two 3-vectors (need not be normalized); clamped acos
static inline float vec3_angle(float ax, float ay, float az,
        float bx, float by, float bz) {
    float na = sqrtf(ax * ax + ay * ay + az * az);
    float nb = sqrtf(bx * bx + by * by + bz * bz);
    if (na < 1e-9f || nb < 1e-9f) {
        return 0.0f;
    }
    float c = (ax * bx + ay * by + az * bz) / (na * nb);
    if (c > 1.0f) { c = 1.0f; } else if (c < -1.0f) { c = -1.0f; }
    return acosf(c);
}

// Clamp the norm of a 3-vector in place (used to bound the auto-trim angle)
static inline void vec3_clamp_norm(float& x, float& y, float& z, float max_norm) {
    float n = sqrtf(x * x + y * y + z * z);
    if (n > max_norm) {
        float f = max_norm / n;
        x *= f;
        y *= f;
        z *= f;
    }
}

#endif
