#ifndef MATRIX_H_
#define MATRIX_H_

#include <cmath>

class Matrix3 {
    public:
        Matrix3();
        Matrix3(float rx, float ry, float rz);
        static Matrix3 mul(Matrix3 *left, Matrix3 *right);
        void mul(Matrix3 *other);
        void mul_left(Matrix3 *other);
        void set(Matrix3 *other);
        void set_raw(float other[3][3]);
        void set_rot_3d(float rx, float ry, float rz);
        void set_rot_x(float rx);
        void set_rot_y(float ry);
        void set_rot_z(float rz);
        void set_diag(float diag_value);
        void clear();
        float data[3][3];
    private:

};

#endif