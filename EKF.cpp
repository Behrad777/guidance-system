#include "EKF.hpp"
#include "matrixUtils.hpp"
#include <cmath>

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
    x_.set_elt(0,0, 1.0f);
    for(int i=1; i<7; ++i) x_.set_elt(i,0, 0.0f);

    for(int i=0;i<7;++i)
      for(int j=0;j<7;++j)
        P_.set_elt(i,j, (i==j? 0.01f : 0.0f));

    const float q_att  = 1e-6f;  // quaternion
    const float q_bias = 1e-5f;  // gyro bias
    for(int i=0;i<7;++i){
      float v = (i<4? q_att : q_bias);
      Q_.set_elt(i,i, v);
      for(int j=0;j<7;++j) if(i!=j) Q_.set_elt(i,j,0.0f);
    }

    const float r_acc = 0.01f;
    const float r_mag = 0.02f;
    for(int i=0;i<6;++i){
      float v = (i<3? r_acc : r_mag);
      R_.set_elt(i,i, v);
      for(int j=0;j<6;++j) if(i!=j) R_.set_elt(i,j,0.0f);
    }

    for(int i=0;i<7;++i) for(int j=0;j<7;++j) F_.set_elt(i,j,0.0f);
    for(int i=0;i<6;++i) for(int j=0;j<7;++j) H_.set_elt(i,j,0.0f);
}

void EKF::predict(const matrix<float>& gyro) {
    float bgx = x_(4,0), bgy = x_(5,0), bgz = x_(6,0);
    matrix<float> omega(3,1);
    omega.set_elt(0,0, gyro(0,0) - bgx);
    omega.set_elt(1,0, gyro(1,0) - bgy);
    omega.set_elt(2,0, gyro(2,0) - bgz);

    matrix<float> q(4,1);
    for(int i=0;i<4;++i) q.set_elt(i,0, x_(i,0));

    matrix<float> dq = quaternionDerivative(q, omega);
    for(int i=0;i<4;++i)
      x_.set_elt(i,0, x_(i,0) + dt_*dq(i,0));
    normalizeQuaternion();

    float q0=x_(0,0), q1=x_(1,0), q2=x_(2,0), q3=x_(3,0);
    float wx=omega(0,0), wy=omega(1,0), wz=omega(2,0);
    const float h = 0.5f;

    F_.set_elt(0,0,  0.0f);   F_.set_elt(0,1, -h*wx); F_.set_elt(0,2, -h*wy); F_.set_elt(0,3, -h*wz);
    F_.set_elt(1,0,  h*wx);   F_.set_elt(1,1,  0.0f); F_.set_elt(1,2,  h*wz); F_.set_elt(1,3, -h*wy);
    F_.set_elt(2,0,  h*wy);   F_.set_elt(2,1, -h*wz); F_.set_elt(2,2,  0.0f); F_.set_elt(2,3,  h*wx);
    F_.set_elt(3,0,  h*wz);   F_.set_elt(3,1,  h*wy); F_.set_elt(3,2, -h*wx); F_.set_elt(3,3,  0.0f);

    const float s = -0.5f;
    F_.set_elt(0,4, s*(-q1)); F_.set_elt(0,5, s*(-q2)); F_.set_elt(0,6, s*(-q3));
    F_.set_elt(1,4, s*( q0)); F_.set_elt(1,5, s*( q3)); F_.set_elt(1,6, s*(-q2));
    F_.set_elt(2,4, s*(-q3)); F_.set_elt(2,5, s*( q0)); F_.set_elt(2,6, s*( q1));
    F_.set_elt(3,4, s*( q2)); F_.set_elt(3,5, s*(-q1)); F_.set_elt(3,6, s*( q0));

    for(int i=4;i<7;++i)
      for(int j=0;j<7;++j)
        F_.set_elt(i,j, (i==j?1.0f:0.0f));

    P_ = F_ * P_ * transpose(F_) + Q_;
}

matrix<float> EKF::quaternionDerivative(
    const matrix<float>& q,
    const matrix<float>& omega)
{
    float q0=q(0,0), q1=q(1,0), q2=q(2,0), q3=q(3,0);
    float wx=omega(0,0), wy=omega(1,0), wz=omega(2,0);

    matrix<float> dq(4,1);
    dq.set_elt(0,0, 0.5f * (-q1*wx - q2*wy - q3*wz));
    dq.set_elt(1,0, 0.5f * ( q0*wx + q2*wz - q3*wy));
    dq.set_elt(2,0, 0.5f * ( q0*wy - q1*wz + q3*wx));
    dq.set_elt(3,0, 0.5f * ( q0*wz + q1*wy - q2*wx));
    return dq;
}

void EKF::normalizeQuaternion() {
    float n = std::sqrt(
        x_(0,0)*x_(0,0) + x_(1,0)*x_(1,0) +
        x_(2,0)*x_(2,0) + x_(3,0)*x_(3,0)
    );
    if(n>0){
      for(int i=0;i<4;++i)
        x_.set_elt(i,0, x_(i,0)/n);
    }
}

matrix<float> EKF::expectedMeasurement(const matrix<float>& q) {
    matrix<float> z(6,1);
    matrix<float> g(3,1), m(3,1);
    g.set_elt(0,0,0.0f); g.set_elt(1,0,0.0f); g.set_elt(2,0,-1.0f);
    m.set_elt(0,0,1.0f); m.set_elt(1,0,0.0f); m.set_elt(2,0,0.0f);

    matrix<float> a = rotateVector(q,g);
    matrix<float> b = rotateVector(q,m);
    for(int i=0;i<3;++i){
      z.set_elt(i,  0, a(i,0));
      z.set_elt(i+3,0, b(i,0));
    }
    return z;
}
void EKF::update(const matrix<float>& accel, const matrix<float>& mag) {
    matrix<float> z(6,1);
    for(int i=0;i<3;++i) {
        z.set_elt(i,   0, accel(i,0));
        z.set_elt(i+3, 0, mag(i,0));
    }

    matrix<float> q(4,1);
    for(int i=0;i<4;++i) q.set_elt(i,0, x_(i,0));
    matrix<float> z_pred = expectedMeasurement(q);

    matrix<float> y = z - z_pred;

    //measurement Jacobian H_
    float q0 = x_(0,0), q1 = x_(1,0), q2 = x_(2,0), q3 = x_(3,0);

    H_.set_elt(0,0, -2*q2);   H_.set_elt(0,1, -2*q3);   H_.set_elt(0,2, -2*q0);   H_.set_elt(0,3, -2*q1);
    H_.set_elt(1,0,  2*q1);   H_.set_elt(1,1,  2*q0);   H_.set_elt(1,2, -2*q3);   H_.set_elt(1,3, -2*q2);
    H_.set_elt(2,0, -2*q0);   H_.set_elt(2,1,  2*q1);   H_.set_elt(2,2,  2*q2);   H_.set_elt(2,3, -2*q3);

    H_.set_elt(3,0,  2*q0);   H_.set_elt(3,1,  2*q1);   H_.set_elt(3,2, -2*q2);   H_.set_elt(3,3, -2*q3);
    H_.set_elt(4,0,  2*q3);   H_.set_elt(4,1,  2*q2);   H_.set_elt(4,2,  2*q1);   H_.set_elt(4,3,  2*q0);
    H_.set_elt(5,0, -2*q2);   H_.set_elt(5,1,  2*q3);   H_.set_elt(5,2,  2*q0);   H_.set_elt(5,3, -2*q1);

    for(int i=0; i<6; ++i)
      for(int j=4; j<7; ++j)
        H_.set_elt(i,j, 0.0f);

    //kalman gain
    matrix<float> Ht = transpose(H_);
    matrix<float> S  = H_ * P_ * Ht + R_;
    matrix<float> K  = P_ * Ht * inverse6x6(S);

    x_ = x_ + K * y;
    normalizeQuaternion();

    matrix<float> I(7,7);
    for(int i=0;i<7;++i)
      for(int j=0;j<7;++j)
        I.set_elt(i,j, (i==j?1.0f:0.0f));
    P_ = (I - K * H_) * P_;
}


matrix<float> EKF::getQuaternion() const {
    matrix<float> q(4,1);
    for(int i=0;i<4;++i) q.set_elt(i,0, x_(i,0));
    return q;
}
