#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iomanip>
#include "EKF.hpp"
#include "matrixUtils.hpp"

using namespace matrix_utils;

enum class MotionState { TiltingUp, TiltingDown, ReturningToZero, Holding };

float randomNoise(float level) {
    return level * (2.0f * (rand() / (float)RAND_MAX) - 1.0f);
}

int main() {
    // Seed random
    srand(static_cast<unsigned>(time(nullptr)));

    // Time step
    const float dt = 0.01f;  // 10 ms
    EKF ekf(dt);

    // World vectors
    Vector3 gravity_world(3, 1);
    gravity_world.set_elt(2, 0, -1.0f);
    Vector3 mag_world(3, 1);
    mag_world.set_elt(0, 0, 1.0f);

    // True states
    Vector3 true_gyro(3, 1);
    matrix<float> true_q(4, 1);
    true_q.set_elt(0, 0, 1.0f);

    // Control parameters
    const float deg_to_rad     = 3.14159265358979323846f / 180.0f;
    const float pitch_rate_deg = 10.0f;   // deg/sec
    const float maxPitch       = 30.0f;   // deg
    const float minPitch       = -30.0f;  // deg
    const float zeroThresh     = 0.5f;    // deg tolerance for “zero”
    const int   holdSteps      = 500;     // iterations to hold level
    int         holdCount      = 0;
    float       current_pitch  = 0.0f;

    // State machine
    MotionState state = MotionState::TiltingUp;

    // Main loop
    for (int i = 0; i < 5000; ++i) {
        // 1) Set true gyro based on state
        switch (state) {
            case MotionState::TiltingUp:
                true_gyro.set_elt(1, 0,  pitch_rate_deg * deg_to_rad);
                break;
            case MotionState::TiltingDown:
                true_gyro.set_elt(1, 0, -pitch_rate_deg * deg_to_rad);
                break;
            case MotionState::ReturningToZero:
                if (current_pitch > zeroThresh)
                    true_gyro.set_elt(1, 0, -pitch_rate_deg * deg_to_rad);
                else if (current_pitch < -zeroThresh)
                    true_gyro.set_elt(1, 0,  pitch_rate_deg * deg_to_rad);
                else
                    true_gyro.set_elt(1, 0, 0.0f);
                break;
            case MotionState::Holding:
                true_gyro.set_elt(1, 0, 0.0f);
                break;
        }

        // 2) Integrate true quaternion
        matrix<float> dq = ekf.quaternionDerivative(true_q, true_gyro);
        for (int j = 0; j < 4; ++j) {
            true_q.set_elt(j, 0, true_q(j, 0) + dq(j, 0) * dt);
        }
        matrix_utils::normalizeQuaternion(true_q);

        // 3) Extract true pitch
        {
            float dummy_roll, dummy_yaw;
            quaternionToEuler(true_q, dummy_roll, current_pitch, dummy_yaw);
        }

        // 4) State transitions
        if (state == MotionState::TiltingUp && current_pitch >= maxPitch) {
            state = MotionState::TiltingDown;
        }
        else if (state == MotionState::TiltingDown && current_pitch <= minPitch) {
            state = MotionState::ReturningToZero;
        }
        else if (state == MotionState::ReturningToZero
                 && std::abs(current_pitch) <= zeroThresh) {
            state = MotionState::Holding;
        }

        // 5) Rotate and add noise
        Vector3 accel_body = matrix_utils::rotateVector(true_q, gravity_world);
        Vector3 mag_body   = matrix_utils::rotateVector(true_q, mag_world);

        Vector3 gyro_noisy(3,1), accel_noisy(3,1), mag_noisy(3,1);
        for (int j = 0; j < 3; ++j) {
            gyro_noisy .set_elt(j,0, true_gyro(j,0) + randomNoise(0.5f * deg_to_rad));
            accel_noisy.set_elt(j,0, accel_body(j,0) + randomNoise(0.02f));
            mag_noisy  .set_elt(j,0, mag_body(j,0)   + randomNoise(0.01f));
        }

        // 6) EKF predict & update
        ekf.predict(gyro_noisy);
        ekf.update(accel_noisy, mag_noisy);

        // 7) Print Roll, Pitch, Yaw (unchanged)
        matrix<float> q_est = ekf.getQuaternion();
        float roll, pitch, yaw;
        quaternionToEuler(q_est, roll, pitch, yaw);

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Step " << i << ":\n";
        std::cout << "Roll: "  << roll  << " deg, "
                  << "Pitch: " << pitch << " deg, "
                  << "Yaw: "   << yaw   << " deg\n";
        std::cout << "---------------------------\n";

        // 8) After holding for holdSteps, exit
        if (state == MotionState::Holding && ++holdCount >= holdSteps) {
            break;
        }
    }

    return 0;
}
