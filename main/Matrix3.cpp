#include "Matrix3.h"

Matrix3::Matrix3() {
    set_diag(1);
}

Matrix3::Matrix3(float rx, float ry, float rz) {
    set_rot_3d(rx, ry, rz);
}

void Matrix3::clear() {
    set_diag(0);
}

void Matrix3::set_diag(float diag_value) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            data[i][j] = i == j ? diag_value : 0;
        }
    }
}

void Matrix3::set_raw(float other[3][3]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            data[i][j] = other[i][j];
        }
    }
}

void Matrix3::set_rot_3d(float rx, float ry, float rz) {
    set_diag(1);
    Matrix3 tmp;
    tmp.set_rot_x(rx);
    mul(&tmp); 
    tmp.set_rot_y(ry);
    mul(&tmp); 
    tmp.set_rot_z(rz);
    mul(&tmp); 
}

void Matrix3::set_rot_x(float rx) {
    float sx = sin(rx);
    float cx = cos(rx);
    set_diag(1);
    data[1][1] = cx;
    data[1][2] = -sx;
    data[2][1] = sx;
    data[2][2] = cx;
}

void Matrix3::set_rot_y(float ry) {
    float sy = sin(ry);
    float cy = cos(ry);
    set_diag(1);
    data[0][0] = cy;
    data[2][0] = -sy;
    data[0][2] = sy;
    data[2][2] = cy;
}

void Matrix3::set_rot_z(float rz) {
    float sz = sin(rz);
    float cz = cos(rz);
    set_diag(1);
    data[0][0] = cz;
    data[0][1] = -sz;
    data[1][0] = sz;
    data[1][1] = cz;
}

void Matrix3::set(Matrix3 *other) {
    set_raw(other->data);
}

void Matrix3::mul(Matrix3 *other) {
    Matrix3 tmp = mul(this, other);
    set(&tmp);
}

void Matrix3::mul_left(Matrix3 *other) {
    Matrix3 tmp = mul(other, this);
    set(&tmp);
}

Matrix3 Matrix3::mul(Matrix3 *left, Matrix3 *right) {
    Matrix3 result;
    float data[3][3];
    float tmp;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            tmp = 0;
            for(int k = 0; k < 3; k++) {
                tmp += left->data[i][k] * right->data[k][j];
            }
            data[i][j] = tmp;
        }
    }
    result.set_raw(data);
    return result;
}