// Toggle over-the-air (WiFi) programming functionality (0 = disabled, 1 = enabled)
#define OTA 1

// Toggle between use of nonlinear or linear controller (0 = linear, 1 = nonlinear)
#define USE_NONLINEAR_CONTROLLER 1

// TickTwo provides ESP32-compatible timers
#include <TickTwo.h>

// Preferences provides flash (NVS) storage for the learned balance-point trim
#include <Preferences.h>

#if OTA
  // Libraries related to wireless functionality
  #include <WiFi.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif

// Cube controller library
#include "src/cube-controller.h"

#if OTA
  // WiFi settings
  const char* wifi_ssid = "YOUR_SSID";
  const char* wifi_password = "YOUR_PASSWORD";
#endif

// Instantiation of objects
Motor motor1(M1_ENABLE, M1_CURRENT), motor2(M2_ENABLE, M2_CURRENT), motor3(M3_ENABLE, M3_CURRENT);
WheelEstimator whe_est_1(M1_SPEED), whe_est_2(M2_SPEED), whe_est_3(M3_SPEED);
AttitudeEstimator att_est(IMU_SDA, IMU_SCL);
AttitudeWheelController cont(USE_NONLINEAR_CONTROLLER);
AttitudeTrajectory att_tra;
LandingController land;

// Run cube controller at the frequency as specified in parameter file
void controller();
TickTwo timer(controller, dt_us, 0, MICROS_MICROS);

// Status LED control: LED is either solid or blinking at 2.5 Hz
void control_led();
TickTwo timer_led(control_led, 200, 0, MILLIS);

#if OTA
  // Handle OTA flashing requests every 5 seconds
  void handle_ota();
  TickTwo timer_ota(handle_ota, 5000, 0, MILLIS);

  // Broadcast a one-line status beacon over UDP once per second
  WiFiUDP status_udp;

  // Command receiver socket and the control web page
  WiFiUDP cmd_udp;
  WebServer web(80);

  void handle_root();
  void handle_stop();
  void handle_land();
  void handle_spin();
  void handle_status();
  void send_status();
  TickTwo timer_status(send_status, 1000, 0, MILLIS);
#endif

// Quaternion and angle error
float qe0, qe1, qe2, qe3;
float phi;
float phi_lim = phi_min;

// Trajectory initialization flag
bool flag_tra = false;

// LED status when blinking
bool led_status = false;

// Security flags
bool flag_arm = false;
bool flag_terminate = false;

// Arming qualifier dwell counter
unsigned int arm_counter = 0;

// Wheel spin-down state (braking the wheels to rest after disarm or terminate)
bool flag_spindown = false;

// Soft-landing state (controlled descent onto a face after a tap)
bool flag_landing = false;

// Peak accel deviation since the last beacon (impact diagnostics)
float a_dev_max = 0;

// Network commands (received over UDP or the web page, consumed by the
// control cycle)
bool cmd_stop = false;
bool cmd_land = false;

// Pirouette (spin trajectory) toggle. spin_enabled is the user's wish
// (web-toggleable, persisted); spin_active is whether the trajectory is
// currently running - a disable only takes effect in a rest phase with the
// reference back at the base pose, so the reference never jumps
bool spin_enabled = true;
bool spin_active = false;
bool spin_save_pending = false;

// Deferred flash write of the learned trim (never write flash while torque
// control is active)
bool trim_save_pending = false;

// Auto-trim of the balance point: body-frame rotation vector (rad) learned
// while balancing and persisted in flash
float trim_x = 0, trim_y = 0, trim_z = 0;
Preferences prefs;

// Trimmed reference quaternion (trajectory reference with the trim composed)
float qt0 = 1, qt1 = 0, qt2 = 0, qt3 = 0;

// Hall re-zero rest dwell counter
unsigned int hall_rest = 0;

// Armed-phase attitude watchdog and fusion diagnostics
unsigned int att_sane_count = 0;
unsigned int fuse_count = 0;
unsigned int fuse_total = 0;

// Torques
float tau_1 = 0, tau_2 = 0, tau_3 = 0;

void setup() {
  // Note time since power-on: the Maxon ESCON drivers need about a second
  // after power-up before their outputs are reliable
  unsigned long t_start = millis();

  // Open serial connection
  Serial.begin(921600);
  Serial.println("This is the ESP32 cube controller.");

  // Load the learned balance-point trim from flash
  prefs.begin("cube", false);
  trim_x = prefs.getFloat("trim_x", 0.0);
  trim_y = prefs.getFloat("trim_y", 0.0);
  trim_z = prefs.getFloat("trim_z", 0.0);
  vec3_clamp_norm(trim_x, trim_y, trim_z, trim_max);
  spin_enabled = prefs.getBool("spin", true);

  #if OTA
    // Start the WiFi connection in the background. OTA is brought up by the
    // OTA timer callback once the connection is established, so WiFi never
    // delays the boot sequence.
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);

    // Command receiver ("STOP" / "LAND" as UDP payloads) and the web page
    cmd_udp.begin(47270);
    web.on("/", handle_root);
    web.on("/stop", handle_stop);
    web.on("/land", handle_land);
    web.on("/spin", handle_spin);
    web.on("/status", handle_status);
    web.begin();
    ArduinoOTA.setHostname("Cube ESP32");

    // If an OTA upload starts, disable the motors and latch the controller
    // off: the upload blocks the control loop for its whole duration
    ArduinoOTA.onStart([]() {
      flag_arm = false;
      flag_landing = false;
      flag_spindown = false;
      flag_terminate = true;
      flag_tra = true;
      att_est.set_correction_gain(lds_disarmed);
      motor1.set_torque(0.0);
      motor2.set_torque(0.0);
      motor3.set_torque(0.0);
    });
  #endif

  // Attitude estimator initialization (also calibrates the gyroscope and
  // seeds the attitude from the accelerometer, ~1.2 s)
  att_est.init();

  // Wait until the ESCON drivers have been powered for at least 1.5 s (the
  // IMU calibration above covers most of it)
  unsigned long elapsed = millis() - t_start;
  if(elapsed < 1500) {
    delay(1500 - elapsed);
  }

  // Wheel estimator initialization (also calibrates the hall sensor for each
  // wheel; runs before the motor test spins so the wheels have never moved)
  whe_est_1.init();
  whe_est_2.init();
  whe_est_3.init();

  // Motor setup (also spins each motor in positive direction very briefly)
  motor1.init();
  motor2.init();
  motor3.init();

  // Start controller and LED timers
  timer.start();
  timer_led.start();

  #if OTA
    // OTA handler and status beacon timers
    timer_ota.start();
    timer_status.start();
  #endif
}

void loop() {
  // Update controller timer
  timer.update();
  timer_led.update();

  #if OTA
    // Handle OTA updates, the status beacon, web requests and UDP commands
    timer_ota.update();
    timer_status.update();
    web.handleClient();
    if(cmd_udp.parsePacket() > 0) {
      char cbuf[8];
      int n = cmd_udp.read(cbuf, 7);
      if(n > 0) {
        cbuf[n] = 0;
        if(strncmp(cbuf, "STOP", 4) == 0) {
          cmd_stop = true;
        } else if(strncmp(cbuf, "LAND", 4) == 0) {
          cmd_land = true;
        }
      }
    }
  #endif

  // Persist the learned balance-point trim once the wheels are at rest
  // (flash writes block; never do them while torque control is active)
  if(trim_save_pending && !flag_arm && !flag_spindown) {
    prefs.putFloat("trim_x", trim_x);
    prefs.putFloat("trim_y", trim_y);
    prefs.putFloat("trim_z", trim_z);
    trim_save_pending = false;
  }
  if(spin_save_pending && !flag_arm && !flag_spindown) {
    prefs.putBool("spin", spin_enabled);
    spin_save_pending = false;
  }
}

#if OTA
  // Set once ArduinoOTA has been started (requires a WiFi connection)
  bool ota_started = false;

  // Number of OTA timer ticks without a WiFi connection (for a one-time log)
  unsigned int ota_ticks = 0;

  void handle_ota(){
    if(!ota_started) {
      if(WiFi.status() != WL_CONNECTED) {
        // One-time notice after roughly a minute without a connection
        if(++ota_ticks == 12) {
          Serial.println("WiFi not connected; OTA unavailable.");
        }
      } else if(!flag_arm) {
        // Only bring OTA up while the motors are off: ArduinoOTA.begin() sets
        // up mDNS and sockets and must not stall the control loop mid-balance
        ArduinoOTA.begin();
        ota_started = true;
        Serial.print("Connected to WiFi (local IP is ");
        Serial.print(WiFi.localIP());
        Serial.println("). OTA ready.");
      }
    } else {
      ArduinoOTA.handle();
    }
  }
#endif

#if OTA
  void handle_root() {
    web.send(200, "text/html",
      "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Cube</title><style>body{font-family:sans-serif;text-align:center;background:#111;color:#eee}"
      "button{font-size:2em;margin:0.4em;padding:0.8em 1.6em;border-radius:12px;border:none}"
      "#stop{background:#c62828;color:#fff}#land{background:#1565c0;color:#fff}#st{font-size:1.2em;margin:1em}</style>"
      "</head><body><h2>Balancing Cube</h2><div id='st'>...</div>"
      "<div><button id='stop' onclick=\"fetch('/stop')\">STOP</button></div>"
      "<div><button id='land' onclick=\"fetch('/land')\">LAND</button></div>"
      "<div><button id='spin' onclick=\"fetch('/spin')\" style='background:#6a1b9a;color:#fff'>SPIN ON/OFF</button></div>"
      "<script>setInterval(async()=>{try{document.getElementById('st').textContent="
      "await(await fetch('/status')).text()}catch(e){}},1000)</script></body></html>");
  }

  void handle_stop() {
    cmd_stop = true;
    web.send(200, "text/plain", "ok");
  }

  void handle_land() {
    cmd_land = true;
    web.send(200, "text/plain", "ok");
  }

  void handle_spin() {
    spin_enabled = !spin_enabled;
    spin_save_pending = true;
    web.send(200, "text/plain", spin_enabled ? "spin on" : "spin off");
  }

  void handle_status() {
    char state = flag_terminate ? 'T' : (flag_arm ? 'A' : (flag_landing ? 'L' : (flag_spindown ? 'S' : 'R')));
    char sbuf[80];
    snprintf(sbuf, sizeof(sbuf), "%c  phi %.1f deg  wheels %.0f/%.0f/%.0f  spin %s",
      state, phi * 180.0 / pi, whe_est_1.omega_w, whe_est_2.omega_w, whe_est_3.omega_w,
      spin_enabled ? "on" : "off");
    web.send(200, "text/plain", sbuf);
  }

  void send_status() {
    if(WiFi.status() != WL_CONNECTED) {
      return;
    }
    if(flag_arm && !status_while_armed) {
      return;
    }

    // One-line state summary: T terminated, A armed, L<phase> landing,
    // S spinning down, R ready
    char statebuf[4];
    if(flag_landing) {
      snprintf(statebuf, sizeof(statebuf), "L%d", land.phase);
    } else {
      statebuf[0] = flag_terminate ? 'T' : (flag_arm ? 'A' : (flag_spindown ? 'S' : 'R'));
      statebuf[1] = 0;
    }
    char buf[220];
    snprintf(buf, sizeof(buf),
      "S=%s PHI=%.2f TRIM=%.3f,%.3f,%.3f W=%.1f,%.1f,%.1f AM=%.2f DEV=%.1f LAND=%.1f,%.1f FUSE=%u TH=%.0f",
      statebuf, phi * 180.0 / pi, trim_x * 180.0 / pi, trim_y * 180.0 / pi, trim_z * 180.0 / pi,
      whe_est_1.omega_w, whe_est_2.omega_w, whe_est_3.omega_w, att_est.a_mag, a_dev_max,
      land.peak_omega, land.peak_dev, fuse_total > 0 ? 100 * fuse_count / fuse_total : 0,
      (whe_est_1.theta_w + whe_est_2.theta_w + whe_est_3.theta_w) / 3.0);
    a_dev_max = 0;
    fuse_count = 0;
    fuse_total = 0;
    status_udp.beginPacket("255.255.255.255", 47269);
    status_udp.write((uint8_t*) buf, strlen(buf));
    status_udp.endPacket();
  }
#endif

void controller() {
  // Estimate wheel velocities
  whe_est_1.estimate(tau_1);
  whe_est_2.estimate(tau_2);
  whe_est_3.estimate(tau_3);

  // Estimate cube attitude
  att_est.estimate();

  // Apply the learned balance-point trim to the trajectory reference. The
  // trim is a body-frame (right-side) composition, so it stays correct
  // through the spin trajectory.
  quat_compose_body(att_tra.qr0, att_tra.qr1, att_tra.qr2, att_tra.qr3, trim_x, trim_y, trim_z,
    qt0, qt1, qt2, qt3);

  // While disarmed, pin the estimator's unobservable yaw onto the (trimmed)
  // reference yaw, so that the error angle below measures pure tilt and the
  // cube arms at any heading
  if(!flag_arm) {
    att_est.pin_yaw(qt0, qt1, qt2, qt3);
  } else {
    // While armed, leak the unobservable yaw toward the reference with a slow
    // time constant: absorbs secular gyro bias drift (which otherwise walks
    // phi into the error limit over long sessions) without disturbing real
    // yaw dynamics or the spin trajectory, whose timescales are much faster
    att_est.pin_yaw_partial(qt0, qt1, qt2, qt3, dt / yaw_leak_tau);
  }

  // Fusion duty diagnostics for the beacon
  fuse_total++;
  if(att_est.fused) {
    fuse_count++;
  }

  // Armed attitude watchdog: if a clean gravity measurement disagrees with
  // the estimated up direction for att_sane_cycles, the estimator is blind -
  // force a terminate so a fallen cube can never keep its motors energized
  if(flag_arm) {
    bool clean = att_est.a_mag >= acc_arm_lo && att_est.a_mag <= acc_arm_hi;
    if(clean) {
      float ux, uy, uz;
      quat_body_up(att_est.q0, att_est.q1, att_est.q2, att_est.q3, ux, uy, uz);
      float ang = vec3_angle(att_est.ax(), att_est.ay(), att_est.az(), ux, uy, uz);
      att_sane_count = ang > att_sane_ang ? att_sane_count + 1 : 0;
    }
    if(att_sane_count > (unsigned int) att_sane_cycles) {
      flag_arm = false;
      flag_terminate = true;
      flag_spindown = true;
      att_sane_count = 0;
      att_est.set_correction_gain(lds_disarmed);
    }
  } else {
    att_sane_count = 0;
  }

  // Calculate rotation quaternion error (against the trimmed reference)
  qe0 = att_est.q0 * qt0 + att_est.q1 * qt1 + att_est.q2 * qt2 + att_est.q3 * qt3;
  qe1 = att_est.q0 * qt1 - att_est.q1 * qt0 - att_est.q2 * qt3 + att_est.q3 * qt2;
  qe2 = att_est.q0 * qt2 - att_est.q2 * qt0 + att_est.q1 * qt3 - att_est.q3 * qt1;
  qe3 = att_est.q0 * qt3 - att_est.q1 * qt2 + att_est.q2 * qt1 - att_est.q3 * qt0;

  // Normalize rotation quaternion error (real part only since we don't need the rest)
  qe0 /= sqrt(qe0 * qe0 + qe1 * qe1 + qe2 * qe2 + qe3 * qe3);

  // Calculate error angle (clamped: float rounding beyond 1.0 would yield NaN
  // and silently prevent arming forever)
  phi = 2.0 * acos(constrain(qe0, -1.0, 1.0));

  // Track the peak accel deviation for the beacon diagnostics
  float a_dev = abs(att_est.a_mag - g);
  if(a_dev > a_dev_max) {
    a_dev_max = a_dev;
  }

  // Network commands. STOP: graceful disarm from any active state (spin the
  // wheels down, restore the pre-arm state, return to ready). LAND: start
  // the soft landing (only from quiet balancing).
  if(cmd_stop) {
    cmd_stop = false;
    if(flag_arm || flag_landing) {
      flag_arm = false;
      flag_landing = false;
      flag_spindown = true;
      phi_lim = phi_min;
      arm_counter = 0;
      att_est.set_correction_gain(lds_disarmed);
      flag_tra = false;
      att_tra.reset();
      trim_save_pending = true;
    }
  }
  if(cmd_land) {
    cmd_land = false;
    if(flag_arm && abs(phi) < land_phi_max) {
      flag_arm = false;
      flag_landing = true;
      land.start();
    }
  }

  // Arming qualifier: near the balance orientation, held still, and with a
  // plausible gravity reading, for arm_dwell consecutive cycles. This
  // prevents arming while the cube is still being handled, which would slam
  // the motors to saturation in the user's hand.
  if(!flag_arm && !flag_terminate && !flag_spindown && !flag_landing) {
    float omega_mag = sqrt(att_est.omega_x * att_est.omega_x + att_est.omega_y * att_est.omega_y +
      att_est.omega_z * att_est.omega_z);
    if(abs(phi) <= phi_lim && omega_mag <= omega_still && att_est.a_mag >= acc_arm_lo &&
      att_est.a_mag <= acc_arm_hi) {
      arm_counter++;
    } else {
      arm_counter = 0;
    }
  }

  // The controller engages once the arming qualifier has held for arm_dwell
  // cycles, and stays engaged while the error is within phi_lim. Exceeding
  // phi_lim once armed disables the controller until the chip is reset.
  bool engage = flag_arm ? (abs(phi) <= phi_lim) : (arm_counter >= arm_dwell && !flag_terminate &&
    !flag_spindown && !flag_landing);

  if(flag_landing) {
    // Soft landing: the phase machine decides between balance control (L0
    // lean) and the descent rate governor (L1/L2). Yaw pinning above stays
    // active (it never changes body-up, and it keeps the L0 yaw error zero).
    land.update(att_est.q0, att_est.q1, att_est.q2, att_est.q3,
      att_est.omega_x, att_est.omega_y, att_est.omega_z, abs(att_est.a_mag - g),
      whe_est_1.omega_w, whe_est_2.omega_w, whe_est_3.omega_w);

    if(land.use_balance_controller) {
      // L0: balance against the leaned reference (static reference, zero
      // reference rates)
      float ql0, ql1, ql2, ql3;
      quat_compose_body(qt0, qt1, qt2, qt3, land.lean_x, land.lean_y, land.lean_z, ql0, ql1, ql2, ql3);
      float theta_mean_l = (whe_est_1.theta_w + whe_est_2.theta_w + whe_est_3.theta_w) / 3.0;
      float omega_mean_l = (whe_est_1.omega_w + whe_est_2.omega_w + whe_est_3.omega_w) / 3.0;
      cont.control(ql0, ql1, ql2, ql3, att_est.q0, att_est.q1, att_est.q2, att_est.q3,
        0.0, 0.0, 0.0, att_est.omega_x, att_est.omega_y, att_est.omega_z,
        0.0, 0.0, 0.0, whe_est_1.theta_w - theta_mean_l, whe_est_2.theta_w - theta_mean_l,
        whe_est_3.theta_w - theta_mean_l,
        whe_est_1.omega_w - omega_mean_l, whe_est_2.omega_w - omega_mean_l,
        whe_est_3.omega_w - omega_mean_l);
      tau_1 = cont.tau_1;
      tau_2 = cont.tau_2;
      tau_3 = cont.tau_3;
    } else {
      tau_1 = land.tau_1;
      tau_2 = land.tau_2;
      tau_3 = land.tau_3;
    }

    // Landing ended (touchdown or abort): restore the pre-arm state and
    // brake the wheels to rest; the cube returns to ready either way
    if(land.done || land.aborted) {
      flag_landing = false;
      flag_spindown = true;
      phi_lim = phi_min;
      arm_counter = 0;
      att_est.set_correction_gain(lds_disarmed);
      flag_tra = false;
      att_tra.reset();

      // Persist the learned balance-point trim once the wheels are at rest
      trim_save_pending = true;
    }
  } else if(engage) {
    if(!flag_arm) {
      // Arming instant: widen the error limit, revert to the balancing
      // estimator gain and discard wheel angle accumulated from hall bias
      flag_arm = true;
      phi_lim = phi_max;
      att_est.set_correction_gain(lds);
      whe_est_1.reset_theta();
      whe_est_2.reset_theta();
      whe_est_3.reset_theta();
    }

    // Generate trajectory
    // Comment out the generate() method if the cube is balancing on a side instead of a corner
    if(!flag_tra) {
      flag_tra = true;
      att_tra.init();
      spin_active = spin_enabled;
    }
    if(spin_active) {
      att_tra.generate();

      // A pending disable is honored only in a rest phase with the reference
      // at the base pose (quaternion dot near +1: excludes the mid-cycle
      // rest, where the 2 pi rotation leaves qr at -qu and a snap back to
      // +qu would read as a 360 degree error and terminate)
      float orm_s = abs(att_tra.omega_r_x) + abs(att_tra.omega_r_y) + abs(att_tra.omega_r_z);
      float dot_s = qu0 * att_tra.qr0 + qu1 * att_tra.qr1 + qu2 * att_tra.qr2 + qu3 * att_tra.qr3;
      if(!spin_enabled && orm_s < 0.01 && dot_s > 0.9999) {
        att_tra.reset();
        spin_active = false;
      }
    } else if(spin_enabled) {
      // Re-enable starts a fresh trajectory (leading rest phase, no jump)
      att_tra.init();
      spin_active = true;
    }

    // Recompute the trimmed reference from the freshly generated trajectory
    quat_compose_body(att_tra.qr0, att_tra.qr1, att_tra.qr2, att_tra.qr3, trim_x, trim_y, trim_z,
      qt0, qt1, qt2, qt3);

    // Auto-trim: while balancing quietly (small error, no commanded motion),
    // slowly bleed the gravity-orthogonal part of the wheel angles into the
    // reference trim. A persistent wheel-angle offset is the signature of a
    // center-of-mass mismatch; removing the mean leaves the tilt-relevant
    // components and keeps the reference yaw untouched.
    float omega_r_mag = abs(att_tra.omega_r_x) + abs(att_tra.omega_r_y) + abs(att_tra.omega_r_z);
    float omega_mag_t = sqrt(att_est.omega_x * att_est.omega_x + att_est.omega_y * att_est.omega_y +
      att_est.omega_z * att_est.omega_z);
    if(abs(phi) < phi_quiet && omega_r_mag < 0.01 && abs(att_est.a_mag - g) < trim_quiet_acc &&
      omega_mag_t < trim_quiet_omega) {
      float theta_mean = (whe_est_1.theta_w + whe_est_2.theta_w + whe_est_3.theta_w) / 3.0;
      trim_x -= trim_rate * dt * (whe_est_1.theta_w - theta_mean);
      trim_y -= trim_rate * dt * (whe_est_2.theta_w - theta_mean);
      trim_z -= trim_rate * dt * (whe_est_3.theta_w - theta_mean);
      vec3_clamp_norm(trim_x, trim_y, trim_z, trim_max);
    }

    // Controller calculates motor torques based on cube and wheel states.
    // The wheel angles AND speeds are fed mean-removed: their common-mode
    // components are the wheels' collective yaw position/momentum, which
    // gravity-mediated shedding can never drain, and whose feedback through
    // the torque matrix is anti-damped (the common-mode speed feeds itself
    // exponentially). The controller reacts only to the differential parts;
    // common-mode momentum is left to bearing friction, which damps it.
    float theta_mean_c = (whe_est_1.theta_w + whe_est_2.theta_w + whe_est_3.theta_w) / 3.0;
    float omega_mean_c = (whe_est_1.omega_w + whe_est_2.omega_w + whe_est_3.omega_w) / 3.0;
    cont.control(qt0, qt1, qt2, qt3, att_est.q0, att_est.q1, att_est.q2, att_est.q3,
      att_tra.omega_r_x, att_tra.omega_r_y, att_tra.omega_r_z, att_est.omega_x, att_est.omega_y, att_est.omega_z,
      att_tra.alpha_r_x, att_tra.alpha_r_y, att_tra.alpha_r_z, whe_est_1.theta_w - theta_mean_c,
      whe_est_2.theta_w - theta_mean_c, whe_est_3.theta_w - theta_mean_c,
      whe_est_1.omega_w - omega_mean_c, whe_est_2.omega_w - omega_mean_c,
      whe_est_3.omega_w - omega_mean_c);

    // Get motor torques from controller
    tau_1 = cont.tau_1;
    tau_2 = cont.tau_2;
    tau_3 = cont.tau_3;
  } else {
    if(flag_spindown) {
      // Actively brake the wheels to rest instead of letting them freewheel
      // Staged braking: above omega_brake_max the back-EMF defeats the ESCON
      // (it cannot regenerate into the supply), so coast until braking can
      // actually bite, then brake hard
      float sign_1 = (0.0 < whe_est_1.omega_w) - (whe_est_1.omega_w < 0.0);
      float sign_2 = (0.0 < whe_est_2.omega_w) - (whe_est_2.omega_w < 0.0);
      float sign_3 = (0.0 < whe_est_3.omega_w) - (whe_est_3.omega_w < 0.0);
      float mag_1 = abs(whe_est_1.omega_w);
      float mag_2 = abs(whe_est_2.omega_w);
      float mag_3 = abs(whe_est_3.omega_w);
      tau_1 = mag_1 > omega_stop && mag_1 <= omega_brake_max ? -sign_1 * ia_brake * Km : 0.0;
      tau_2 = mag_2 > omega_stop && mag_2 <= omega_brake_max ? -sign_2 * ia_brake * Km : 0.0;
      tau_3 = mag_3 > omega_stop && mag_3 <= omega_brake_max ? -sign_3 * ia_brake * Km : 0.0;

      // Spin-down completes once all three wheels are near rest
      if(abs(whe_est_1.omega_w) <= omega_stop && abs(whe_est_2.omega_w) <= omega_stop &&
        abs(whe_est_3.omega_w) <= omega_stop) {
        flag_spindown = false;
        tau_1 = 0.0;
        tau_2 = 0.0;
        tau_3 = 0.0;
      }
    } else {
      // Not engaged: reset motor torques to zero
      tau_1 = 0.0;
      tau_2 = 0.0;
      tau_3 = 0.0;

      // With the wheels at rest and motors off for a sustained stretch,
      // slowly heal the hall bias against the actual readings (the boot
      // calibration can be slightly off; a residual bias ramps the wheel
      // angles while balancing and poisons the auto-trim)
      bool wheels_rest = abs(whe_est_1.omega()) < 3.0 && abs(whe_est_2.omega()) < 3.0 &&
        abs(whe_est_3.omega()) < 3.0;
      hall_rest = wheels_rest ? hall_rest + 1 : 0;
      if(hall_rest > (unsigned int) f) {
        whe_est_1.rezero_hall();
        whe_est_2.rezero_hall();
        whe_est_3.rezero_hall();
      }
    }

    // Disarm mechanism (safety terminate also brakes the wheels to rest)
    if(flag_arm) {
      flag_arm = false;
      flag_terminate = true;
      flag_spindown = true;

      // Restore the disarmed estimator gain to keep the disarmed-implies-fast-
      // gain invariant (harmless while terminated; matters if the latch is
      // ever removed)
      att_est.set_correction_gain(lds_disarmed);
    }
  }

  // Apply torques to motors
  // When balancing on the x side, comment out motor 2 and 3. When balancing on the y side, comment out 1 and 3.
  motor1.set_torque(tau_1);
  motor2.set_torque(tau_2);
  motor3.set_torque(tau_3);
}

void control_led() {
  if(flag_landing) {
    // Controlled descent in progress: blink blue
    if(led_status) {
      neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    } else {
      digitalWrite(RGB_BUILTIN, LOW);
    }
    led_status = !led_status;
  } else if(flag_spindown) {
    // Wheels are being braked to rest: blink so a spinning cube never shows
    // a "safe" solid state (green after a tap-disarm, red after a terminate)
    if(led_status) {
      if(flag_terminate) {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
      } else {
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
      }
    } else {
      digitalWrite(RGB_BUILTIN, LOW);
    }
    led_status = !led_status;
  } else if(flag_arm) {
    if(abs(phi) <= phi_lim - 10 * pi / 180) {
      // Cube is not close to error limit: solid green LED
      neopixelWrite(RGB_BUILTIN, 0, 255, 0);
    } else {
      // Cube is close to error limit: alternate between red and green. Cube should be manually put to rest.
      if(led_status) {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
      } else {
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
      }
      led_status = !led_status;
    }
  } else {
    if(flag_tra) {
      // A trajectory was already generated so the cube must now be halted due to a limit error
      if(led_status) {
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
      } else {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
      }
      led_status = !led_status;
    } else {
      // Show a solid red LED to indicate readiness to start
      neopixelWrite(RGB_BUILTIN, 255, 0, 0);
    }
  }
}
