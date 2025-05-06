#include <Arduino.h>
#include "EKF.hpp"
#include "matrixUtils.hpp"

using namespace matrix_utils;

// ------------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------------
const float dt              = 0.01f;               // seconds (100 Hz)
const float deg2rad         = PI / 180.0f;
const float pitchRateDeg    = 10.0f;               // deg/sec up‑tilt
const float targetPitchDeg  = 30.0f;               // target +30°
const int   holdSteps       = 500;                 // 500 samples = 5 s

// Noise levels (add a bit of realism)
const float gyroNoiseStd    = 0.02f * deg2rad;     // 0.02 deg/s
const float accelNoiseStd   = 0.01f;               // 0.01 g
const float magNoiseStd     = 0.005f;              // 0.005 units

// ------------------------------------------------------------------------
// Globals
// ------------------------------------------------------------------------
EKF      ekf(dt);
matrix<float> true_q(4,1);
Vector3  true_gyro(3,1);
Vector3  gravity_world(3,1), mag_world(3,1);

// Loop timing
unsigned long lastMs = 0;

// Hold state
bool holding    = false;
int  holdCount  = 0;

// Helper: uniform noise in [–σ..+σ]
float noise(float sigma) {
  return (random(0, 10001) / 10000.0f * 2.0f - 1.0f) * sigma;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // World vectors
  gravity_world.set_elt(0,0, 0.0f);
  gravity_world.set_elt(1,0, 0.0f);
  gravity_world.set_elt(2,0, -1.0f);

  mag_world.set_elt(0,0, 1.0f);
  mag_world.set_elt(1,0, 0.0f);
  mag_world.set_elt(2,0, 0.0f);

  // Initialize true quaternion to identity
  true_q.set_elt(0,0, 1.0f);
  true_q.set_elt(1,0, 0.0f);
  true_q.set_elt(2,0, 0.0f);
  true_q.set_elt(3,0, 0.0f);

  // Serial Plotter header
  Serial.println("EKF_Roll\tEKF_Pitch\tEKF_Yaw");
}

void loop() {
  // Run at 100 Hz
  if (millis() - lastMs < int(dt * 1000)) return;
  lastMs = millis();

  // 1) Determine true gyro: tilt up until target, then hold zero
  float true_roll, true_pitch, true_yaw;
  // compute current true Euler
  quaternionToEuler(true_q, true_roll, true_pitch, true_yaw);

  if (!holding) {
    // drive positive pitch
    true_gyro.set_elt(0,0, pitchRateDeg * deg2rad);
    true_gyro.set_elt(1,0, pitchRateDeg * deg2rad);
    true_gyro.set_elt(2,0, 0.0f);
    if (true_pitch >= targetPitchDeg) {
      holding = true;
    }
  } else {
    // hold zero rotation
    true_gyro.set_elt(0,0, 0.0f);
    true_gyro.set_elt(1,0, 0.0f);
    true_gyro.set_elt(2,0, 0.0f);
    holdCount++;
    if (holdCount > holdSteps) {
      // done holding: stop the loop
      while(true) {
        // do nothing
      }
    }
  }

  // 2) Integrate true quaternion
  auto dq = ekf.quaternionDerivative(true_q, true_gyro);
  for (int i = 0; i < 4; ++i) {
    true_q.set_elt(i,0, true_q(i,0) + dq(i,0) * dt);
  }
  matrix_utils::normalizeQuaternion(true_q);

  // 3) Simulate noisy sensors
  Vector3 accel_b = matrix_utils::rotateVector(true_q, gravity_world);
  Vector3 mag_b   = matrix_utils::rotateVector(true_q, mag_world);

  Vector3 gyro_n(3,1), accel_n(3,1), mag_n(3,1);
  for (int i = 0; i < 3; ++i) {
    gyro_n .set_elt(i,0, true_gyro(i,0) + noise(gyroNoiseStd));
    accel_n.set_elt(i,0, accel_b(i,0)  + noise(accelNoiseStd));
    mag_n  .set_elt(i,0, mag_b(i,0)    + noise(magNoiseStd));
  }

  // 4) EKF predict & update
  ekf.predict(gyro_n);
  ekf.update(accel_n, mag_n);

  // 5) Extract EKF Euler
  matrix<float> q_est = ekf.getQuaternion();
  float est_roll, est_pitch, est_yaw;
  quaternionToEuler(q_est, est_roll, est_pitch, est_yaw);

  // 6) Print to Serial Plotter
  Serial.print(est_roll,  4); Serial.print('\t');
  Serial.print(est_pitch, 4); Serial.print('\t');
  Serial.println(est_yaw,  4);
}
