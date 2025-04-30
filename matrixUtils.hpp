#include "EKF.hpp"
#include <iostream>
#include <cmath>
namespace matrix_utils {
    inline matrix<float> rotateVector(const matrix<float>& q, const matrix<float>& v) {
        float q0 = q(0, 0);
        float q1 = q(1, 0);
        float q2 = q(2, 0);
        float q3 = q(3, 0);

        matrix<float> result(3, 1);

        float t2 =   q0*q1;
        float t3 =   q0*q2;
        float t4 =   q0*q3;
        float t5 =  -q1*q1;
        float t6 =   q1*q2;
        float t7 =   q1*q3;
        float t8 =  -q2*q2;
        float t9 =   q2*q3;
        float t10 = -q3*q3;

        result.set_elt(0, 0, 2*( (t8 + t10)*v(0,0) + (t6 - t4)*v(1,0) + (t3 + t7)*v(2,0) ) + v(0,0));
        result.set_elt(1, 0, 2*( (t4 + t6)*v(0,0) + (t5 + t10)*v(1,0) + (t9 - t2)*v(2,0) ) + v(1,0));
        result.set_elt(2, 0, 2*( (t7 - t3)*v(0,0) + (t2 + t9)*v(1,0) + (t5 + t8)*v(2,0) ) + v(2,0));

        return result;
    } 
    
    inline matrix<float> transpose(const matrix<float>& m) {
        matrix<float> result(m.num_cols(), m.num_rows());
    
        for (int i = 0; i < m.num_rows(); ++i) {
            for (int j = 0; j < m.num_cols(); ++j) {
                result.set_elt(j, i, m(i, j));
            }
        }
    
        return result;
    }
    
    inline matrix<float> inverse6x6(const matrix<float>& m) {
        if (m.num_rows() != 6 || m.num_cols() != 6) {
        }
    
        matrix<float> A(6, 6);  
        matrix<float> I(6, 6);  
    
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                A.set_elt(i, j, m(i, j));
                I.set_elt(i, j, (i == j) ? 1.0f : 0.0f);
            }
        }
    
        // Gauss-Jordan elimination
        for (int col = 0; col < 6; ++col) {
            float pivot = A(col, col);
            if (fabs(pivot) < 1e-6f) {
                std::cout<<"Matrix is singular and cannot be inverted."<<std::endl;
            }
    
            for (int j = 0; j < 6; ++j) {
                A.set_elt(col, j, A(col, j) / pivot);
                I.set_elt(col, j, I(col, j) / pivot);
            }
    
            for (int row = 0; row < 6; ++row) {
                if (row != col) {
                    float factor = A(row, col);
                    for (int j = 0; j < 6; ++j) {
                        A.set_elt(row, j, A(row, j) - factor * A(col, j));
                        I.set_elt(row, j, I(row, j) - factor * I(col, j));
                    }
                }
            }
        }
    
        return I;
    }
    
    inline void quaternionToEuler(const matrix<float>& q, float& roll, float& pitch, float& yaw) {
        float q0 = q(0,0);
        float q1 = q(1,0);
        float q2 = q(2,0);
        float q3 = q(3,0);
    
        // Roll (x-axis)
        float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
        roll = atan2(sinr_cosp, cosr_cosp) * (180.0f / 3.14159265f);
    
        // Pitch (y-axis)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabs(sinp) >= 1.0f)
            pitch = copysign(90.0f, sinp); // out of range
        else
            pitch = asin(sinp) * (180.0f / 3.14159265f);
    
        // Yaw (z-axis)
        float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
        yaw = atan2(siny_cosp, cosy_cosp) * (180.0f / 3.14159265f);
    }
    inline void normalizeQuaternion(matrix<float>& q) {
        float norm = 0.0f;
        for (int i = 0; i < 4; ++i) {
            norm += q(i, 0) * q(i, 0);
        }
        norm = sqrt(norm);
        if (norm > 0.0f) {
            for (int i = 0; i < 4; ++i) {
                q.set_elt(i, 0, q(i, 0) / norm);
            }
        }
    }
    
    
};
