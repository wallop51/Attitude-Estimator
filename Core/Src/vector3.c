#include "vector3.h"
#include <math.h>

float vector3_magnitude(Vector3 v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

Vector3 vector3_normalise(Vector3 v) {
    float magnitude = vector3_magnitude(v);

    if (magnitude == 0) return v;

    Vector3 v_normalised = {
        v.x / magnitude,
        v.y / magnitude,
        v.z / magnitude
    };

    return v_normalised;
}

float vector3_dot(Vector3 v1, Vector3 v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
} 

Vector3 vector3_cross(Vector3 v1, Vector3 v2) {
    Vector3 cross = {
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x
    };

    return cross;
}