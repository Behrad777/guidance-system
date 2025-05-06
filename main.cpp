#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iomanip>
#include "EKF.hpp"
#include "matrixUtils.hpp"

using namespace matrix_utils;

// Which axis we’re moving
enum class Axis { Roll, Pitch, Yaw };
// State machine for each axis
enum class MotionState { TiltingPos, TiltingNeg, ReturningToZero, Holding };

// Utility to pick gyro index by axis
inline int axisIndex(Axis a) {
    return (a == Axis::Roll  ? 0 :
            a == Axis::Pitch ? 1 :
                               2);
}

float randomNoise(float level) {
    return level * (2.0f * (rand() / (float)RAND_MAX) - 1.0f);
}

int main() {
    srand(static_cast<unsigned>(time(nullptr)));

    const float dt             = 0.01f;  // 10 ms
    const float deg_to_rad     = 3.14159265358979323846f / 180.0f;
    const float rate_deg       = 10.0f;  // 10 deg/sec
    const float maxAngle       = 30.0f;  // ±30°
    const float zeroThresh     = 0.5f;   // within ±0.5° considered “zero”
    const int   holdSteps      = 500;    // hold for 500 samples

    EKF ekf(dt);

    // World‐frame references
    Vector3 gravity_world(3,1);  gravity_world.set_elt(2,0,-1.0f);
    Vector3 mag_world(3,1);      mag_world.set_elt(0,0, 1.0f);

    // True and noisy states
    Vector3 true_gyro(3,1);
    matrix<float> true_q(4,1);   true_q.set_elt(0,0,1.0f);

    // Loop over each axis
    for (Axis axis : {Axis::Pitch, Axis::Roll, Axis::Yaw}) {
        // Reset filter and true orientation
        ekf = EKF(dt);
        true_q = matrix<float>(4,1);
        true_q.set_elt(0,0,1.0f);

        MotionState state = MotionState::TiltingPos;
        int holdCount = 0;
        float true_angle = 0.0f;

        std::cout << "\n--- Starting cycle on "
                  << (axis==Axis::Pitch? "PITCH" :
                      axis==Axis::Roll ? "ROLL"  :
                                          "YAW")
                  << " axis ---\n";

        for (int i = 0; i < 5000; ++i) {
            // 1) Set true gyro about selected axis
            int idx = axisIndex(axis);
            switch (state) {
                case MotionState::TiltingPos:
                    true_gyro.set_elt(idx,0,  rate_deg * deg_to_rad);
                    break;
                case MotionState::TiltingNeg:
                    true_gyro.set_elt(idx,0, -rate_deg * deg_to_rad);
                    break;
                case MotionState::ReturningToZero:
                    if      (true_angle >  zeroThresh) true_gyro.set_elt(idx,0, -rate_deg*deg_to_rad);
                    else if (true_angle < -zeroThresh) true_gyro.set_elt(idx,0,  rate_deg*deg_to_rad);
                    else                                true_gyro.set_elt(idx,0,  0.0f);
                    break;
                case MotionState::Holding:
                    true_gyro.set_elt(idx,0, 0.0f);
                    break;
            }

            // 2) Integrate true quaternion
            auto dq = ekf.quaternionDerivative(true_q, true_gyro);
            for (int j = 0; j < 4; ++j)
                true_q.set_elt(j,0, true_q(j,0) + dq(j,0)*dt);
            matrix_utils::normalizeQuaternion(true_q);

            // 3) Extract the true Euler angle on that axis
            float r,p,y;
            quaternionToEuler(true_q, r,p,y);
            true_angle = (axis==Axis::Roll  ? r :
                          axis==Axis::Pitch ? p :
                                              y);

            // 4) State transitions
            if      (state==MotionState::TiltingPos 
                     && true_angle >=  maxAngle) state = MotionState::TiltingNeg;
            else if (state==MotionState::TiltingNeg
                     && true_angle <= -maxAngle) state = MotionState::ReturningToZero;
            else if (state==MotionState::ReturningToZero
                     && std::abs(true_angle) <= zeroThresh) state = MotionState::Holding;

            // 5) Build noisy measurements
            Vector3 accel_b = rotateVector(true_q, gravity_world);
            Vector3 mag_b   = rotateVector(true_q, mag_world);

            Vector3 g_noisy(3,1), a_noisy(3,1), m_noisy(3,1);
            for (int j = 0; j < 3; ++j) {
                g_noisy .set_elt(j,0, true_gyro(j,0) + randomNoise(0.5f*deg_to_rad));
                a_noisy.set_elt(j,0, accel_b(j,0) + randomNoise(0.02f));
                m_noisy.set_elt(j,0, mag_b   (j,0) + randomNoise(0.01f));
            }

            // 6) EKF predict + update
            ekf.predict(g_noisy);
            ekf.update(a_noisy, m_noisy);

            // 7) Print EKF estimate + raw true angle
            auto q_est = ekf.getQuaternion();
            float er,ep,ey;
            quaternionToEuler(q_est, er,ep,ey);

            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Step " << i << ":\n"
                      << " Roll: "  << er  << " deg, "
                      << "Pitch: " << ep  << " deg, "
                      << "Yaw: "   << ey  << " deg\n"
                      << "Raw " << (axis==Axis::Roll?"Roll":
                                   axis==Axis::Pitch?"Pitch":
                                                      "Yaw")
                      << ": " << true_angle << " deg\n"
                      << "---------------------------\n";

            // 8) Exit this axis after hold
            if (state==MotionState::Holding && ++holdCount>=holdSteps)
                break;
        }
    }

    return 0;
}
