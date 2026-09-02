#ifndef QUATERNION_H
#define QUATERNION_H

#include "vector3.h"

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

Quaternion quaternion_conjugate(Quaternion q);
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);
Quaternion quaternion_from_axis_angle(Vector3 axis, float angle);
Vector3 quaternion_apply_rotation(Quaternion q, Vector3 v);
Vector3 quaternion_to_euler(Quaternion q);

#endif