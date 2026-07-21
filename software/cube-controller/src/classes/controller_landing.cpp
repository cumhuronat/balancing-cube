#include "controller_landing.h"

// Descent geometry (body frame, verified in the design review). The waypoint
// up-vectors trace corner -> x-parallel edge -> -z face; the body-frame
// lean/rate axis that moves body-up TOWARD the edge is (0, 1, -1)/sqrt(2)
// (the negative of the up-vector rotation axis), and the edge->face descent
// rate is about -x. Signs are load-bearing; see the design doc.
static const float up_corner_x = 0.5773503, up_corner_y = 0.5773503, up_corner_z = 0.5773503;
static const float up_edge_x = 0.0, up_edge_y = 0.7071068, up_edge_z = 0.7071068;
static const float up_face_x = 0.0, up_face_y = 0.0, up_face_z = 1.0;
static const float descent_x = 0.0, descent_y = 0.7071068, descent_z = -0.7071068;

// Constructor
LandingController::LandingController() {
    lean_x = lean_y = lean_z = 0.0;
    use_balance_controller = false;
    tau_1 = tau_2 = tau_3 = 0.0;
    phase = 0;
    done = false;
    aborted = false;
    peak_omega = 0.0;
    peak_dev = 0.0;
    cycles = 0;
}

// Enter phase L0
void LandingController::start() {
    lean_x = lean_y = lean_z = 0.0;
    use_balance_controller = true;
    tau_1 = tau_2 = tau_3 = 0.0;
    phase = 0;
    done = false;
    aborted = false;
    peak_omega = 0.0;
    peak_dev = 0.0;
    cycles = 0;
}

// One control cycle
void LandingController::update(float q0, float q1, float q2, float q3,
        float omega_x, float omega_y, float omega_z, float a_dev,
        float omega_w1, float omega_w2, float omega_w3) {
    if(done || aborted) {
        tau_1 = tau_2 = tau_3 = 0.0;
        return;
    }

    cycles++;

    // Landing report statistics
    float omega_mag = sqrtf(omega_x * omega_x + omega_y * omega_y + omega_z * omega_z);
    if(omega_mag > peak_omega) {
        peak_omega = omega_mag;
    }
    if(a_dev > peak_dev) {
        peak_dev = a_dev;
    }

    // Aborts (any phase): body rate above free-fall touchdown, wheel
    // overspeed, or a descent-phase timeout. The caller falls back to plain
    // spin-down. (An L0 timeout instead proceeds to L1 below: if the lean
    // alone has not tipped the cube, the governor drives the descent.)
    bool timeout = phase != 0 && cycles > (unsigned int) land_t12_max;
    if(omega_mag > omega_land_abort ||
        fabsf(omega_w1) > omega_w_abort || fabsf(omega_w2) > omega_w_abort ||
        fabsf(omega_w3) > omega_w_abort || timeout) {
        aborted = true;
        use_balance_controller = false;
        tau_1 = tau_2 = tau_3 = 0.0;
        return;
    }

    // Estimated body-up direction and angles to the waypoints
    float ux, uy, uz;
    quat_body_up(q0, q1, q2, q3, ux, uy, uz);
    float ang_corner = vec3_angle(ux, uy, uz, up_corner_x, up_corner_y, up_corner_z);
    float ang_edge = vec3_angle(ux, uy, uz, up_edge_x, up_edge_y, up_edge_z);
    float ang_face = vec3_angle(ux, uy, uz, up_face_x, up_face_y, up_face_z);

    if(phase == 0) {
        // L0: grow the body-frame lean along the descent axis; the caller
        // balances against the leaned reference
        float lean = lean_rate * dt * cycles;
        if(lean > lean_cap) {
            lean = lean_cap;
        }
        lean_x = lean * descent_x;
        lean_y = lean * descent_y;
        lean_z = lean * descent_z;

        // Tip has begun once the up-vector has moved off the corner waypoint
        // (or the lean has been at its cap long enough: hand over to the
        // governor, which will drive the descent)
        if(ang_corner >= lean_engage || cycles > (unsigned int) land_t0_max) {
            phase = 1;
            use_balance_controller = false;
            cycles = 0;
        }
        return;
    }

    if(phase == 1) {
        // L1: rate-governed descent about the corner->edge axis
        governor(omega_d1 * descent_x, omega_d1 * descent_y, omega_d1 * descent_z,
            omega_x, omega_y, omega_z, omega_w1, omega_w2, omega_w3);

        // Edge reached: waypoint proximity, or a gated contact spike
        if(ang_edge <= ang_edge_hit || (a_dev > land_contact && ang_edge <= ang_spike_gate)) {
            phase = 2;
            cycles = 0;
        }
        return;
    }

    if(phase == 2) {
        // L2: single-axis descent about -x (wheels 2/3 damp their axes)
        governor(-omega_d2, 0.0, 0.0, omega_x, omega_y, omega_z, omega_w1, omega_w2, omega_w3);

        // Face reached: touchdown
        if(ang_face <= ang_face_hit || (a_dev > land_contact && ang_face <= ang_spike_gate)) {
            phase = 3;
            done = true;
            tau_1 = tau_2 = tau_3 = 0.0;
        }
        return;
    }
}

// Rate governor: same torque mapping and gyroscopic terms as the balance
// controller's feedback linearization (the leading minus on the inertia
// terms is load-bearing: a positive mapping is positive feedback)
void LandingController::governor(float ref_x, float ref_y, float ref_z,
        float omega_x, float omega_y, float omega_z,
        float omega_w1, float omega_w2, float omega_w3) {
    float u_1 = kg_land * (ref_x - omega_x);
    float u_2 = kg_land * (ref_y - omega_y);
    float u_3 = kg_land * (ref_z - omega_z);

    tau_1 = -I_w_xx * (omega_w3 * omega_y - omega_w2 * omega_z) - I_c_xx_bar * u_1 - I_c_xy_bar * (u_2 + u_3);
    tau_2 = -I_w_xx * (omega_w1 * omega_z - omega_w3 * omega_x) - I_c_xx_bar * u_2 - I_c_xy_bar * (u_1 + u_3);
    tau_3 = -I_w_xx * (omega_w2 * omega_x - omega_w1 * omega_y) - I_c_xx_bar * u_3 - I_c_xy_bar * (u_1 + u_2);

    // Clamp to the landing torque budget
    if(tau_1 > tau_land_max) { tau_1 = tau_land_max; } else if(tau_1 < -tau_land_max) { tau_1 = -tau_land_max; }
    if(tau_2 > tau_land_max) { tau_2 = tau_land_max; } else if(tau_2 < -tau_land_max) { tau_2 = -tau_land_max; }
    if(tau_3 > tau_land_max) { tau_3 = tau_land_max; } else if(tau_3 < -tau_land_max) { tau_3 = -tau_land_max; }
}
