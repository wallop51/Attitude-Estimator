#ifndef VECTOR3_H
#define VECTOR3_H

typedef struct {
    float x;
    float y;
    float z;
} Vector3;

float vector3_magnitude(Vector3 v);
Vector3 vector3_normalise(Vector3 v);
float vector3_dot(Vector3 v1, Vector3 v2);
Vector3 vector3_cross(Vector3 v1, Vector3 v2);

#endif