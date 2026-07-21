// Host-side tests for the soft-landing phase machine (controller_landing).
// Build & run (from software/tests/):
//   mkdir -p .build && g++ -std=c++17 -O2 -o .build/test_landing test_landing.cpp && ./.build/test_landing

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "../cube-controller/src/classes/controller_landing.cpp"

static int checks = 0;
static void expect(bool cond, const char* what) {
    checks++;
    if (!cond) { printf("FAIL: %s\n", what); exit(1); }
}

// Attitude with a prescribed body-up direction (yaw arbitrary)
static void q_from_up(float ux, float uy, float uz, float& q0, float& q1, float& q2, float& q3) {
    quat_from_accel(ux, uy, uz, q0, q1, q2, q3);
}

// Unit vector partway (angle a, rad) from unit vector u toward unit vector v
static void slerp_dir(float ux, float uy, float uz, float vx, float vy, float vz, float a,
        float& ox, float& oy, float& oz) {
    float d = ux*vx + uy*vy + uz*vz;
    float wx = vx - d*ux, wy = vy - d*uy, wz = vz - d*uz;
    float wn = sqrtf(wx*wx + wy*wy + wz*wz);
    wx /= wn; wy /= wn; wz /= wn;
    ox = ux*cosf(a) + wx*sinf(a);
    oy = uy*cosf(a) + wy*sinf(a);
    oz = uz*cosf(a) + wz*sinf(a);
}

static const float C = 0.5773503f;   // corner up component
static const float E = 0.7071068f;   // edge up component
static const float DX = 0.0f, DY = 0.7071068f, DZ = -0.7071068f; // descent axis

int main() {
    float q0, q1, q2, q3;

    // --- L0: lean grows along the descent axis; balance controller active ---
    LandingController lc;
    lc.start();
    expect(lc.phase == 0 && lc.use_balance_controller, "start enters L0 with balance control");
    q_from_up(C, C, C, q0, q1, q2, q3);
    for (int i = 0; i < 125; i++) {          // 0.5 s at the corner
        lc.update(q0, q1, q2, q3, 0, 0, 0, 0.2f, 0, 0, 0);
    }
    expect(lc.phase == 0, "still leaning while up stays at the corner");
    expect(lc.lean_y > 0.015f && lc.lean_z < -0.015f && fabsf(lc.lean_x) < 1e-6f,
        "lean vector grows along (0,+1,-1)");   // 0.5 s at 3 deg/s -> 0.0185 rad per component
    float lean_mag = sqrtf(lc.lean_y*lc.lean_y + lc.lean_z*lc.lean_z);
    expect(lean_mag <= lean_cap + 1e-6f, "lean respects the cap");

    // Up moves 7 deg toward the edge: descent begins
    float ux, uy, uz;
    slerp_dir(C, C, C, 0, E, E, 7.0f * (float)M_PI / 180.0f, ux, uy, uz);
    q_from_up(ux, uy, uz, q0, q1, q2, q3);
    lc.update(q0, q1, q2, q3, 0.3f, 0, 0, 0.4f, 0, 0, 0);
    expect(lc.phase == 1 && !lc.use_balance_controller, "6 deg departure enters L1");

    // --- L1 governor: torque signs, end to end ---
    // Falling exactly at the target rate with idle wheels: near-zero torque
    lc.update(q0, q1, q2, q3, omega_d1*DX, omega_d1*DY, omega_d1*DZ, 0.5f, 0, 0, 0);
    expect(fabsf(lc.tau_1) < 1e-3f && fabsf(lc.tau_2) < 1e-3f && fabsf(lc.tau_3) < 1e-3f,
        "on-target descent rate commands ~zero torque");

    // Falling too fast: the body reaction torque (-tau) must OPPOSE the descent axis
    lc.update(q0, q1, q2, q3, 3.0f*DX, 3.0f*DY, 3.0f*DZ, 0.5f, 0, 0, 0);
    float react_dot = (-lc.tau_1)*DX + (-lc.tau_2)*DY + (-lc.tau_3)*DZ;
    expect(react_dot < -1e-4f, "too-fast descent is braked (reaction opposes descent axis)");

    // Not yet falling: the governor pushes INTO the descent (rate tracking from below)
    lc.update(q0, q1, q2, q3, 0, 0, 0, 0.5f, 0, 0, 0);
    react_dot = (-lc.tau_1)*DX + (-lc.tau_2)*DY + (-lc.tau_3)*DZ;
    expect(react_dot > 1e-4f, "too-slow descent is driven forward");

    // Torque clamp holds under a large rate error
    lc.update(q0, q1, q2, q3, 6.0f*DX, 6.0f*DY, 6.0f*DZ, 0.5f, 0, 0, 0);
    expect(fabsf(lc.tau_1) <= tau_land_max + 1e-6f && fabsf(lc.tau_2) <= tau_land_max + 1e-6f &&
        fabsf(lc.tau_3) <= tau_land_max + 1e-6f, "governor torques respect the clamp");

    // --- L1 -> CATCH at the edge waypoint ---
    q_from_up(0, E, E, q0, q1, q2, q3);
    lc.update(q0, q1, q2, q3, 3.0f*DX, 3.0f*DY, 3.0f*DZ, 20.0f, 0, 0, 0);
    expect(lc.phase == 2, "edge waypoint enters the catch");

    // Catch brakes toward rest while still moving
    lc.update(q0, q1, q2, q3, 2.0f*DX, 2.0f*DY, 2.0f*DZ, 1.0f, 0, 0, 0);
    float rdot = (-lc.tau_1)*DX + (-lc.tau_2)*DY + (-lc.tau_3)*DZ;
    expect(rdot < -1e-4f, "catch brakes the residual rotation");

    // Near rest at the edge: hold begins
    lc.update(q0, q1, q2, q3, 0.2f*DX, 0.2f*DY, 0.2f*DZ, 0.5f, 0, 0, 0);
    expect(lc.phase == 3, "settled catch enters the hold");

    // --- HOLD: single-wheel edge balance ---
    // Tipped toward the face (beta_face > 0): wheel-1 torque must be negative
    // (body reaction +x pushes back toward the edge)
    float hx, hy, hz;
    slerp_dir(0, E, E, 0, 0, 1, 4.0f * (float)M_PI / 180.0f, hx, hy, hz);
    q_from_up(hx, hy, hz, q0, q1, q2, q3);
    lc.update(q0, q1, q2, q3, 0, 0, 0, 0.5f, 0, 0, 0);
    expect(lc.phase == 3, "hold persists inside the capture region");
    expect(lc.tau_1 < -1e-4f, "hold pushes back toward the edge when tipped to the face");

    // Gyroscopic compensation: wheel-3 momentum + body y-rate couples into
    // wheel 1; the difference with wheels stopped must equal -I_w_xx*w3*wy
    q_from_up(0, E, E, q0, q1, q2, q3);
    lc.update(q0, q1, q2, q3, 0, 0.2f, 0, 0.5f, 0, 0, 0);
    float t1_still = lc.tau_1;
    lc.update(q0, q1, q2, q3, 0, 0.2f, 0, 0.5f, 0, 0, 400.0f);
    float dgyro = lc.tau_1 - t1_still;
    expect(fabsf(dgyro - (-I_w_xx * 400.0f * 0.2f)) < 1e-4f,
        "hold compensates wheel gyroscopic coupling");

    // Backward fall (about the edge toward the -y face, beta_face < 0)
    // aborts straight to spin-down
    slerp_dir(0, E, E, 0, 1, 0, 5.0f * (float)M_PI / 180.0f, hx, hy, hz);
    q_from_up(hx, hy, hz, q0, q1, q2, q3);
    lc.update(q0, q1, q2, q3, 0.5f, 0, 0, 0.5f, 0, 0, 0);
    expect(lc.aborted && !lc.done, "backward fall from the hold aborts");

    // Fresh hold: completing the pause bows to the face descent
    lc.start();
    lc.phase = 3;
    q_from_up(0, E, E, q0, q1, q2, q3);
    for (int i = 0; i <= hold_cycles + 1; i++) {
        lc.update(q0, q1, q2, q3, 0, 0, 0, 0.5f, 0, 0, 0);
    }
    expect(lc.phase == 4 && !lc.aborted, "completed hold enters the face descent");

    // --- Face descent about -x; touchdown ---
    lc.update(q0, q1, q2, q3, -omega_d2, 0, 0, 1.0f, 0, 0, 0);
    expect(fabsf(lc.tau_1) < 1e-3f, "on-target face-descent rate commands ~zero wheel-1 torque");
    lc.update(q0, q1, q2, q3, -3.0f, 0, 0, 1.0f, 0, 0, 0);
    expect(-lc.tau_1 * (-1.0f) < -1e-4f, "too-fast face descent is braked about -x");

    q_from_up(0, 0, 1, q0, q1, q2, q3);
    lc.update(q0, q1, q2, q3, -1.0f, 0, 0, 2.0f, 0, 0, 0);
    expect(lc.phase == 5 && lc.done && !lc.aborted, "face waypoint is touchdown");
    expect(lc.tau_1 == 0.0f && lc.tau_2 == 0.0f && lc.tau_3 == 0.0f, "touchdown zeroes torques");
    expect(lc.peak_omega >= 3.0f && lc.peak_dev >= 2.0f, "landing stats recorded");

    // Catch timeout aborts (never settles, never progresses)
    lc.start();
    lc.phase = 2;
    q_from_up(0, E, E, q0, q1, q2, q3);
    for (int i = 0; i <= catch_max + 1; i++) {
        lc.update(q0, q1, q2, q3, 2.0f*DX, 2.0f*DY, 2.0f*DZ, 1.0f, 0, 0, 0);
    }
    expect(lc.aborted, "catch that cannot settle aborts");

    // --- Aborts ---
    lc.start();
    q_from_up(C, C, C, q0, q1, q2, q3);
    lc.update(q0, q1, q2, q3, 9.0f, 0, 0, 0.2f, 0, 0, 0);
    expect(lc.aborted && !lc.done, "body-rate overspeed aborts");

    lc.start();
    lc.update(q0, q1, q2, q3, 0, 0, 0, 0.2f, 650.0f, 0, 0);
    expect(lc.aborted, "wheel overspeed aborts");

    lc.start();                              // L0 timeout proceeds to L1, then L1 times out -> abort
    for (int i = 0; i < land_t0_max + 2; i++) {
        lc.update(q0, q1, q2, q3, 0, 0, 0, 0.2f, 0, 0, 0);
    }
    expect(lc.phase == 1 && !lc.aborted, "L0 timeout hands over to the governor");
    for (int i = 0; i < land_t12_max + 2; i++) {
        lc.update(q0, q1, q2, q3, 0, 0, 0, 0.2f, 0, 0, 0);
    }
    expect(lc.aborted, "L1 timeout aborts");

    printf("OK (%d checks)\n", checks);
    return 0;
}
