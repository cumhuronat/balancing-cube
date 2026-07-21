// Host-side tests for src/definitions/quat_utils.h (pure quaternion math).
// Build & run (from software/tests/):
//   mkdir -p .build && g++ -std=c++17 -O2 -o .build/test_quat_utils test_quat_utils.cpp && ./.build/test_quat_utils

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "../cube-controller/src/definitions/quat_utils.h"

static int checks = 0;

static void expect(bool cond, const char* what) {
    checks++;
    if (!cond) {
        printf("FAIL: %s\n", what);
        exit(1);
    }
}

struct Quat { float w, x, y, z; };

// Hamilton product a (x) b
static Quat qmul(Quat a, Quat b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

static Quat qconj(Quat a) { return {a.w, -a.x, -a.y, -a.z}; }

static Quat qnorm(Quat a) {
    float n = sqrtf(a.w*a.w + a.x*a.x + a.y*a.y + a.z*a.z);
    return {a.w/n, a.x/n, a.y/n, a.z/n};
}

// Body-frame "up" direction = third row of R(q)
static void body_up(Quat q, float out[3]) {
    out[0] = 2.0f * (q.x*q.z - q.w*q.y);
    out[1] = 2.0f * (q.y*q.z + q.w*q.x);
    out[2] = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}

// Deterministic PRNG so failures reproduce
static unsigned int rng_state = 12345u;
static float frand() { // uniform [-1, 1]
    rng_state = rng_state * 1664525u + 1013904223u;
    return ((rng_state >> 8) & 0xFFFF) / 32767.5f - 1.0f;
}

static Quat qrand() {
    Quat q = {frand(), frand(), frand(), frand()};
    if (fabsf(q.w) + fabsf(q.x) + fabsf(q.y) + fabsf(q.z) < 1e-3f) q.w = 1.0f;
    return qnorm(q);
}

// Rotation of `angle` about a random axis, as a quaternion
static Quat qrand_angle(float angle) {
    float ax = frand(), ay = frand(), az = frand();
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-3f) { ax = 1.0f; n = 1.0f; }
    float s = sinf(angle / 2.0f) / n;
    return {cosf(angle / 2.0f), ax*s, ay*s, az*s};
}

// Error angle between estimate q and reference r: 2*acos(|scalar(conj(q) (x) r)|)
static float err_angle(Quat q, Quat r) {
    Quat e = qmul(qconj(q), r);
    float c = fabsf(e.w);
    if (c > 1.0f) c = 1.0f;
    return 2.0f * acosf(c);
}

// --- quat_from_accel tests ---

static void test_from_accel_matches_gravity() {
    for (int trial = 0; trial < 50; trial++) {
        // Random unit direction, away from the degenerate straight-down case
        float ax = frand(), ay = frand(), az = frand();
        float n = sqrtf(ax*ax + ay*ay + az*az);
        if (n < 1e-3f) { az = 1.0f; n = 1.0f; }
        ax /= n; ay /= n; az /= n;
        if (az < -0.98f) az = -az;
        // Feed unnormalized (scaled like m/s^2) to prove input scale is irrelevant
        float q0, q1, q2, q3;
        quat_from_accel(9.81f * ax, 9.81f * ay, 9.81f * az, q0, q1, q2, q3);
        float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        expect(fabsf(qn - 1.0f) < 1e-5f, "from_accel returns unit quaternion");
        float up[3]; body_up({q0, q1, q2, q3}, up);
        float dot = up[0]*ax + up[1]*ay + up[2]*az;
        expect(dot > 0.99999f, "from_accel body-up matches measured gravity");
        // Fixed point of the firmware's own correction step: proves the sign
        // conventions agree with correct()'s implied attitude
        float p0 = q0, p1 = q1, p2 = q2, p3 = q3;
        quat_accel_correct(1.0f, ax, ay, az, p0, p1, p2, p3);
        float d = fabsf(p0 - q0) + fabsf(p1 - q1) + fabsf(p2 - q2) + fabsf(p3 - q3);
        expect(d < 1e-4f, "from_accel output is a fixed point of quat_accel_correct");
    }
}

static void test_from_accel_degenerate_upside_down() {
    float q0, q1, q2, q3;
    quat_from_accel(0.0f, 0.0f, -9.81f, q0, q1, q2, q3);
    float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    expect(fabsf(qn - 1.0f) < 1e-5f, "degenerate case returns unit quaternion");
    float up[3]; body_up({q0, q1, q2, q3}, up);
    expect(up[2] < -0.9999f, "degenerate case points body-up straight down");
}

// --- quat_accel_correct tests ---

static void test_correct_flat_fixed_point() {
    float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
    bool applied = quat_accel_correct(0.04f, 0.0f, 0.0f, 1.0f, q0, q1, q2, q3);
    expect(applied, "flat: correction step ran");
    expect(fabsf(q0 - 1) < 1e-6f && fabsf(q1) < 1e-6f && fabsf(q2) < 1e-6f && fabsf(q3) < 1e-6f,
        "flat pose is a fixed point of the correction");
}

static void test_correct_converges_within_basin() {
    for (int trial = 0; trial < 50; trial++) {
        Quat qt = qrand();                       // true attitude
        float a[3]; body_up(qt, a);              // rest accel (normalized)
        // Start inside the 90-degree basin: perturb truth by < 60 degrees
        Quat q = qnorm(qmul(qt, qrand_angle(1.0f * frand())));
        float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        for (int i = 0; i < 20000; i++) {
            quat_accel_correct(0.04f, a[0], a[1], a[2], q0, q1, q2, q3);
            Quat qq = qnorm({q0, q1, q2, q3});
            q0 = qq.w; q1 = qq.x; q2 = qq.y; q3 = qq.z;
        }
        float up[3]; body_up({q0, q1, q2, q3}, up);
        float dot = up[0]*a[0] + up[1]*a[1] + up[2]*a[2];
        expect(dot > 0.9999f, "correction converges body-up onto measured gravity (in-basin)");
    }
}

static void test_correct_antipode_basin_documented() {
    // Pre-existing property: beyond 90 degrees of up-error the correction
    // converges to the antipode. Documented so nobody "fixes" boot init away.
    float th = 150.0f * (float)M_PI / 180.0f;
    float a[3] = {sinf(th), 0.0f, cosf(th)};
    float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
    for (int i = 0; i < 20000; i++) {
        quat_accel_correct(0.04f, a[0], a[1], a[2], q0, q1, q2, q3);
        Quat qq = qnorm({q0, q1, q2, q3});
        q0 = qq.w; q1 = qq.x; q2 = qq.y; q3 = qq.z;
    }
    float up[3]; body_up({q0, q1, q2, q3}, up);
    float dot = up[0]*a[0] + up[1]*a[1] + up[2]*a[2];
    expect(dot < -0.99f, "beyond 90 deg up-error the correction reaches the antipode (documented)");
}

static void test_correct_singular_guard() {
    // Exactly 90 degrees of up-error: qe0 == 0, step must be skipped untouched
    float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
    bool applied = quat_accel_correct(0.04f, 1.0f, 0.0f, 0.0f, q0, q1, q2, q3);
    expect(!applied, "singular geometry returns false");
    expect(q0 == 1 && q1 == 0 && q2 == 0 && q3 == 0, "singular geometry leaves q untouched");
}

// --- quat_pin_yaw tests ---

// World-z rotation by angle psi
static Quat qz(float psi) {
    return {cosf(psi / 2.0f), 0.0f, 0.0f, sinf(psi / 2.0f)};
}

// Smallest full error angle vs qu achievable by world-yawing q, by brute scan
static float min_err_over_yaw(Quat q, Quat qu) {
    float best = 1e9f;
    for (int i = 0; i < 720; i++) {
        float psi = i * (float)M_PI / 360.0f;
        float e = err_angle(qmul(qz(psi), q), qu);
        if (e < best) best = e;
    }
    return best;
}

static void run_pin_yaw_cases(Quat qu, const char* name) {
    for (int trial = 0; trial < 50; trial++) {
        Quat q = qrand();
        float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        bool applied = quat_pin_yaw(qu.w, qu.x, qu.y, qu.z, q0, q1, q2, q3);
        Quat p = {q0, q1, q2, q3};
        float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (!applied) {
            // Guard case: only legitimate when the whole yaw orbit is nearly
            // antipodal (tilt error near 180 deg)
            expect(min_err_over_yaw(q, qu) > 2.0f, "pin_yaw only declines near-antipodal configs");
            continue;
        }
        expect(fabsf(qn - 1.0f) < 1e-5f, "pin_yaw preserves unit norm");
        // Pinning is a pure world-yaw change: body-up must be untouched
        float u0[3], u1[3];
        body_up(q, u0);
        body_up(p, u1);
        float ddot = u0[0]*u1[0] + u0[1]*u1[1] + u0[2]*u1[2];
        expect(ddot > 0.99999f, "pin_yaw leaves the body-up direction unchanged");
        // The error must be minimal over all yaws, with non-negative scalar
        Quat e = qmul(qconj(p), qu);
        expect(e.w > -1e-6f, "pinned error has non-negative scalar part");
        float ea = err_angle(p, qu);
        expect(ea <= min_err_over_yaw(q, qu) + 2e-3f, "pinned error is the yaw-minimal error");
    }
    printf("  pin_yaw cases passed for %s reference\n", name);
}

static void test_pin_yaw() {
    // Corner reference from parameters.h (phi_e = 0)
    float half = acosf(sqrtf(3.0f) / 3.0f) / 2.0f;
    Quat qu_corner = {cosf(half), sqrtf(2.0f) / 2.0f * sinf(half), -sqrtf(2.0f) / 2.0f * sinf(half), 0.0f};
    // Edge reference from parameters.h (phi_e = -3 deg)
    float phi_e = -3.0f * (float)M_PI / 180.0f;
    Quat qu_edge = {cosf(phi_e / 2.0f - (float)M_PI / 8.0f), cosf(phi_e / 2.0f + 3.0f * (float)M_PI / 8.0f), 0.0f, 0.0f};
    run_pin_yaw_cases(qu_corner, "corner");
    run_pin_yaw_cases(qu_edge, "edge");
}

static void test_pin_yaw_double_cover() {
    float half = acosf(sqrtf(3.0f) / 3.0f) / 2.0f;
    Quat qu = {cosf(half), sqrtf(2.0f) / 2.0f * sinf(half), -sqrtf(2.0f) / 2.0f * sinf(half), 0.0f};
    // A pose near the reference, then negated (one net full physical turn)
    Quat q = qnorm(qmul(qz(0.7f), qmul(qu, qrand_angle(0.05f))));
    Quat qneg = {-q.w, -q.x, -q.y, -q.z};
    float a0 = q.w, a1 = q.x, a2 = q.y, a3 = q.z;
    float b0 = qneg.w, b1 = qneg.x, b2 = qneg.y, b3 = qneg.z;
    expect(quat_pin_yaw(qu.w, qu.x, qu.y, qu.z, a0, a1, a2, a3), "pin_yaw runs on q");
    expect(quat_pin_yaw(qu.w, qu.x, qu.y, qu.z, b0, b1, b2, b3), "pin_yaw runs on -q");
    Quat ea = qmul(qconj({a0, a1, a2, a3}), qu);
    Quat eb = qmul(qconj({b0, b1, b2, b3}), qu);
    expect(fabsf(ea.w - eb.w) < 1e-5f, "pinning is immune to the quaternion double cover");
    expect(ea.w > 0.99f, "near-reference pose pins to near-zero error");
}

static void test_pin_yaw_guard() {
    float half = acosf(sqrtf(3.0f) / 3.0f) / 2.0f;
    Quat qu = {cosf(half), sqrtf(2.0f) / 2.0f * sinf(half), -sqrtf(2.0f) / 2.0f * sinf(half), 0.0f};
    // 180 degrees about a WORLD horizontal axis (left-multiplied): the whole
    // yaw orbit is then exactly antipodal (n = 0). Note a 180-degree BODY-axis
    // rotation (right-multiplied) is NOT degenerate: world-yaw can partially
    // undo it (n = 1/sqrt(3)).
    Quat q = qnorm(qmul({0.0f, 1.0f, 0.0f, 0.0f}, qu));
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    bool applied = quat_pin_yaw(qu.w, qu.x, qu.y, qu.z, q0, q1, q2, q3);
    expect(!applied, "degenerate configuration returns false");
    expect(q0 == q.w && q1 == q.x && q2 == q.y && q3 == q.z, "degenerate configuration leaves q untouched");
}


// --- quat_compose_body / vec3_clamp_norm tests (auto-trim math) ---

static void test_compose_body_matches_qmul() {
    for (int trial = 0; trial < 50; trial++) {
        Quat q = qrand();
        float rx = 0.1f * frand(), ry = 0.1f * frand(), rz = 0.1f * frand();
        float ang = sqrtf(rx*rx + ry*ry + rz*rz);
        Quat r = {1.0f, 0.0f, 0.0f, 0.0f};
        if (ang > 1e-9f) {
            float s = sinf(ang / 2.0f) / ang;
            r = {cosf(ang / 2.0f), rx*s, ry*s, rz*s};
        }
        Quat want = qmul(q, r);
        float o0, o1, o2, o3;
        quat_compose_body(q.w, q.x, q.y, q.z, rx, ry, rz, o0, o1, o2, o3);
        float d = fabsf(o0-want.w) + fabsf(o1-want.x) + fabsf(o2-want.y) + fabsf(o3-want.z);
        expect(d < 1e-5f, "compose_body equals right-multiplied Hamilton product");
        float n = sqrtf(o0*o0 + o1*o1 + o2*o2 + o3*o3);
        expect(fabsf(n - 1.0f) < 1e-5f, "compose_body preserves unit norm");
    }
}

static void test_compose_body_zero_is_identity() {
    Quat q = qrand();
    float o0, o1, o2, o3;
    quat_compose_body(q.w, q.x, q.y, q.z, 0.0f, 0.0f, 0.0f, o0, o1, o2, o3);
    expect(o0 == q.w && o1 == q.x && o2 == q.y && o3 == q.z, "zero trim leaves reference untouched");
}

static void test_vec3_clamp_norm() {
    float x = 3.0f, y = 4.0f, z = 0.0f;      // norm 5
    vec3_clamp_norm(x, y, z, 1.0f);
    float n = sqrtf(x*x + y*y + z*z);
    expect(fabsf(n - 1.0f) < 1e-6f, "over-limit vector clamps to max norm");
    expect(fabsf(x/y - 3.0f/4.0f) < 1e-5f, "clamp preserves direction");
    float a = 0.01f, b = -0.02f, c = 0.005f;
    float a0 = a, b0 = b, c0 = c;
    vec3_clamp_norm(a, b, c, 1.0f);
    expect(a == a0 && b == b0 && c == c0, "under-limit vector is untouched");
}


// --- quat_pin_yaw_partial tests (armed yaw leak) ---

static void test_pin_yaw_partial() {
    float half = acosf(sqrtf(3.0f) / 3.0f) / 2.0f;
    Quat qu = {cosf(half), sqrtf(2.0f) / 2.0f * sinf(half), -sqrtf(2.0f) / 2.0f * sinf(half), 0.0f};
    for (int trial = 0; trial < 25; trial++) {
        Quat q = qnorm(qmul(qz(0.3f + 0.4f * frand()), qmul(qu, qrand_angle(0.1f))));
        // frac = 1 must equal the full pin
        float a0=q.w,a1=q.x,a2=q.y,a3=q.z, b0=q.w,b1=q.x,b2=q.y,b3=q.z;
        quat_pin_yaw(qu.w,qu.x,qu.y,qu.z, a0,a1,a2,a3);
        quat_pin_yaw_partial(qu.w,qu.x,qu.y,qu.z, b0,b1,b2,b3, 1.0f);
        expect(fabsf(a0-b0)+fabsf(a1-b1)+fabsf(a2-b2)+fabsf(a3-b3) < 1e-4f,
            "partial pin with frac=1 equals the full pin");
        // frac = 0 is identity
        float c0=q.w,c1=q.x,c2=q.y,c3=q.z;
        quat_pin_yaw_partial(qu.w,qu.x,qu.y,qu.z, c0,c1,c2,c3, 0.0f);
        expect(fabsf(c0-q.w)+fabsf(c1-q.x)+fabsf(c2-q.y)+fabsf(c3-q.z) < 1e-5f,
            "partial pin with frac=0 is the identity");
        // small frac reduces the yaw error and preserves body-up
        float d0=q.w,d1=q.x,d2=q.y,d3=q.z;
        quat_pin_yaw_partial(qu.w,qu.x,qu.y,qu.z, d0,d1,d2,d3, 0.1f);
        float u0[3], u1[3];
        body_up(q, u0);
        body_up({d0,d1,d2,d3}, u1);
        expect(u0[0]*u1[0]+u0[1]*u1[1]+u0[2]*u1[2] > 0.99999f,
            "partial pin leaves body-up unchanged");
        float e_before = err_angle(q, qu);
        float e_after = err_angle({d0,d1,d2,d3}, qu);
        expect(e_after < e_before + 1e-6f, "partial pin never increases the error");
    }
}

int main() {
    test_from_accel_matches_gravity();
    test_from_accel_degenerate_upside_down();
    test_correct_flat_fixed_point();
    test_correct_converges_within_basin();
    test_correct_antipode_basin_documented();
    test_correct_singular_guard();
    test_pin_yaw();
    test_pin_yaw_partial();
    test_pin_yaw_double_cover();
    test_pin_yaw_guard();
    test_compose_body_matches_qmul();
    test_compose_body_zero_is_identity();
    test_vec3_clamp_norm();
    printf("OK (%d checks)\n", checks);
    return 0;
}
