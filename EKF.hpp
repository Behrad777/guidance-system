#ifndef EKF_HPP
#define EKF_HPP
#include "fastmatrix.hpp"
using namespace fastmatrix;
using Vector3 = matrix<float>;


class EKF {
    public:
        EKF(float dt);
        void predict(const matrix<float>& gyro); //3x1 vector
        void update(const matrix<float>& accel, const matrix<float>& mag); //both 3x1 vectors 
        matrix<float> getBias() const;
        matrix<float> getQuaternion() const;
        //helpers
        void normalizeQuaternion();
        matrix<float> quaternionDerivative(const matrix<float>& q, const matrix<float>& omega);
        matrix<float> expectedMeasurement(const matrix<float>& q);  // For accel + mag


    private:
        float dt_;
        matrix<float> x_; //7x1
        matrix<float> P_; //7x7
        matrix<float> Q_; //7x7
        matrix<float> R_; //6x6
        matrix<float> F_;   // 7x7: Jacobian of predict model
        matrix<float> H_;   // 6x7: Jacobian of measurement model

        




};
#endif