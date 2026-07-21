#ifndef controller_landing_h
#define controller_landing_h

#include "../definitions/parameters.h"
#include "../definitions/quat_utils.h"

// Soft-landing controller: lowers the cube from corner balance onto the -z
// face in a controlled descent.
// Pure logic (no hardware access) so the phase machine is host-testable.
class LandingController {
    public:
        // Constructor
        LandingController();
        // Enter phase L0 (called at the tap gesture while balancing)
        void start();
        // One control cycle while landing. Inputs: attitude estimate, body
        // rates (rad/s), accel-magnitude deviation from g (m/s^2), wheel
        // speeds (rad/s). Outputs via the public members below.
        void update(float q0, float q1, float q2, float q3,
            float omega_x, float omega_y, float omega_z, float a_dev,
            float omega_w1, float omega_w2, float omega_w3);
        // L0 lean rotation vector (rad, body frame) to compose onto the reference
        float lean_x, lean_y, lean_z;
        // True during L0: the .ino runs the balance controller with the leaned
        // reference; false afterwards: the governor torques below apply
        bool use_balance_controller;
        // Governor torques (Nm) for phases L1/L2
        float tau_1, tau_2, tau_3;
        // Phase (0 lean, 1 corner descent, 2 edge catch, 3 edge hold,
        // 4 face descent, 5 touched down)
        int phase;
        // Terminal flags: exactly one becomes true
        bool done, aborted;
        // Landing report statistics
        float peak_omega, peak_dev;

    private:
        // Rate governor: desired body rates -> wheel torques (same torque
        // mapping and gyroscopic terms as the balance controller)
        void governor(float ref_x, float ref_y, float ref_z,
            float omega_x, float omega_y, float omega_z,
            float omega_w1, float omega_w2, float omega_w3);
        // Map linearized inputs to clamped wheel torques, including the
        // wheel-gyroscopic compensation (shared by the governor and the hold)
        void apply_u(float u_1, float u_2, float u_3,
            float omega_x, float omega_y, float omega_z,
            float omega_w1, float omega_w2, float omega_w3);
        // Cycles spent in the current phase
        unsigned int cycles;
        // Pre-lean settle counter (L0 waits for the tap transient to die)
        unsigned int quiet;
        // No-progress bail tracking for the descent phases
        float best_ang;
        unsigned int stall;
};

#endif
