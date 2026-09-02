#include "quaternion.h"
#include <math.h>

Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion q3 = {
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
    };

    return q3;
}

// Takes an axis as a unit vector and an angle and constructs a quaternion
Quaternion quaternion_from_axis_angle(Vector3 axis, float angle) {
    angle /= 2;

    Quaternion q = {
        cosf(angle),
        axis.x * sinf(angle),
        axis.y * sinf(angle),
        axis.z * sinf(angle)
    };

    return q;
}

Quaternion quaternion_conjugate(Quaternion q) {
    Quaternion conjugate = {
        q.w,
        -q.x,
        -q.y,
        -q.z
    };

    return conjugate;
}

// applies a rotation, q, to a vector, v
Vector3 quaternion_apply_rotation(Quaternion q, Vector3 v) {
    Quaternion v_quaternion = {
        0.0f,
        v.x,
        v.y,
        v.z
    };

    Quaternion result = quaternion_multiply(
        quaternion_multiply(q,v_quaternion),
        quaternion_conjugate(q)
    );

    Vector3 rotated = {
        result.x,
        result.y,
        result.z
    };

    return rotated;
}

// converts a quaternion to a Vector3 of euler angles - pitch, yaw, roll
// conventions for this project:
// Rotation about X = yaw
// Rotation about Y = roll
// Rotation about Z = pitch
Vector3 quaternion_to_euler(Quaternion q) {
    float yaw = -atan2f(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.x*q.x + q.y*q.y)
    );

    float value = 2 * (q.w * q.y - q.z * q.x);
    value = fmaxf(-1.0f, fminf(1.0f, value));

    float roll = asinf(value);

    float pitch = -atan2f(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y*q.y + q.z*q.z)
    );

    Vector3 euler = {
        pitch,
        yaw,
        roll
    };

    return euler;
}