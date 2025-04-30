#include "EKF.hpp"
#include "matrixUtils.hpp"
using namespace matrix_utils;

EKF::EKF(float dt): 
dt_(dt),
x_(7, 1),
P_(7, 7),
Q_(7, 7),
R_(6, 6),
F_(7, 7),
H_(6, 7)

{
    x_.set_elt(0, 0, 1.0);    
    // Initialize quaternion to identity rotation
    x_.set_elt(0, 0, 1.0f); // q0 = 1
    for (int i = 1; i < 7; ++i) {
        x_.set_elt(i, 0, 0.0f); // rest = 0
    }

    // Initialize P_ (state covariance) small diagonal
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            P_.set_elt(i, j, (i == j) ? 0.01f : 0.0f);
        }
    }

    // Initialize Q_ (process noise covariance) very small diagonal
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            Q_.set_elt(i, j, (i == j) ? 0.001f : 0.0f);
        }
    }

    // Initialize R_ (measurement noise covariance)
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            R_.set_elt(i, j, (i == j) ? 0.05f : 0.0f);
        }
    }

    // Initialize F_ (state transition Jacobian) as identity
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            F_.set_elt(i, j, (i == j) ? 1.0f : 0.0f);
        }
    }

    // Set identity only for first 6 quaternion-related terms
    for (int i = 0; i < 6; ++i) {
            H_.set_elt(i, i, 1.0f);
    }

    

}

void EKF::predict(const matrix<float>& gyro) {
    float bgx = x_(4, 0);
    float bgy = x_(5, 0);
    float bgz = x_(6, 0);

    matrix<float> omega(3, 1);
    omega.set_elt(0, 0, gyro(0, 0) - bgx);
    omega.set_elt(1, 0, gyro(1, 0) - bgy);
    omega.set_elt(2, 0, gyro(2, 0) - bgz);

    matrix<float> q(4, 1);
    for (int i = 0; i < 4; ++i)
        q.set_elt(i, 0, x_(i, 0));

    matrix<float> dq = quaternionDerivative(q, omega);

    for (int i = 0; i < 4; ++i)
        x_.set_elt(i, 0, x_(i, 0) + dt_ * dq(i, 0));

    normalizeQuaternion();

}


matrix<float> EKF::quaternionDerivative(const matrix<float>& q, const matrix<float>& omega) {
    float q0 = q(0, 0), q1 = q(1, 0), q2 = q(2, 0), q3 = q(3, 0);
    float wx = omega(0, 0), wy = omega(1, 0), wz = omega(2, 0);

    matrix<float> dq(4, 1);
    dq.set_elt(0, 0, 0.5f * (-q1*wx - q2*wy - q3*wz));
    dq.set_elt(1, 0, 0.5f * ( q0*wx + q2*wz - q3*wy));
    dq.set_elt(2, 0, 0.5f * ( q0*wy - q1*wz + q3*wx));
    dq.set_elt(3, 0, 0.5f * ( q0*wz + q1*wy - q2*wx));
    return dq;
}

void EKF::normalizeQuaternion(){
    float norm = sqrt(x_(0, 0) * x_(0, 0) + x_(1, 0) * x_(1, 0) + x_(2, 0) * x_(2, 0) + x_(3, 0) * x_(3, 0));
    if (norm > 0.0f) {
        x_.set_elt(0, 0, x_(0, 0) / norm);
        x_.set_elt(1, 0, x_(1, 0) / norm);
        x_.set_elt(2, 0, x_(2, 0) / norm);
        x_.set_elt(3, 0, x_(3, 0) / norm);
    }
}

matrix<float> EKF::expectedMeasurement(const matrix<float>& q) {
    matrix<float> z_pred(6, 1);

    matrix<float> g_ref(3, 1);  // Gravity: down
    g_ref.set_elt(0, 0, 0.0f);
    g_ref.set_elt(1, 0, 0.0f);
    g_ref.set_elt(2, 0, -1.0f);

    matrix<float> m_ref(3, 1);  // Magnetic field: pointing north
    m_ref.set_elt(0, 0, 1.0f);
    m_ref.set_elt(1, 0, 0.0f);
    m_ref.set_elt(2, 0, 0.0f);

    // Rotate global → body: q * v * q⁻¹
    matrix<float> accel_pred = rotateVector(q, g_ref);
    matrix<float> mag_pred   = rotateVector(q, m_ref);

    for (int i = 0; i < 3; ++i) {
        z_pred.set_elt(i,   0, accel_pred(i, 0));  
        z_pred.set_elt(i+3, 0, mag_pred(i, 0));    
    }

    return z_pred;
}

void EKF::update(const matrix<float>& accel, const matrix<float>& mag) {
    matrix<float> z(6, 1);
    for (int i = 0; i < 3; ++i) {
        z.set_elt(i,   0, accel(i, 0));
        z.set_elt(i+3, 0, mag(i, 0));
    }


    matrix<float> q(4, 1);
    for (int i = 0; i < 4; ++i)
        q.set_elt(i, 0, x_(i, 0));

    matrix<float> z_pred = expectedMeasurement(q);

    matrix<float> y = z - z_pred;


    
    for (int i = 0; i < 6; ++i) {
        H_.set_elt(i, i, 1.0f);
    }
    
    
    matrix<float> Ht = transpose(H_);

    matrix<float> S  = H_ * P_ * Ht + R_; //innovation covariance

    matrix<float> K  = P_ * Ht * inverse6x6(S); //gain

    x_ = x_ + K * y;
    normalizeQuaternion();

    matrix<float> I(7, 7);
    for (int i = 0; i < 7; ++i) {
        I.set_elt(i, i, 1.0f);
    }
    
    P_ = (I - K * H_) * P_;
}

matrix<float> EKF::getQuaternion() const {
    matrix<float> q(4, 1);
    for (int i = 0; i < 4; ++i) {
        q.set_elt(i, 0, x_(i, 0));
    }
    return q;
}

