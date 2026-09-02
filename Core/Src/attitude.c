#include "mpu6050.h"
#include "stm32f4xx_hal.h"
#include "quaternion.h"
#include "vector3.h"

#define RAD_PER_DEG_MS (3.14159265f / 180000.0f)
#define RAD_TO_DEG (180.0f / 3.14159265f)
#define RAW_TO_DPS 131
#define RAW_TO_G 16384
#define KP 0.02f

/* CALIBRATION PARAMETERS 
obtained from the output of calibration.py
when the STM32 used to output raw sensor data.
*/
const Vector3 GYRO_BIAS = {-417.08f, 161.904f, 19.94f};
const Vector3 ACCEL_OFFSET = {573.654f, -126.384f, 680.564f};
const Vector3 ACCEL_SCALE = {1.008102f, 0.993418f, 0.989793f};

// Private function prototypes
HAL_StatusTypeDef imu_read_sample(Vector3_i *accel, Vector3_i *gyro);

void calibrate_sample(
    Vector3_i *accel,
    Vector3_i* gyro,
    Vector3 *accel_calibrated,
    Vector3 *gyro_calibrated
);

void apply_error_correction(Vector3 *v, Vector3 error);
Quaternion get_delta_quaternion(Vector3 gyros, uint32_t dt);

// INITIALISE TIMING
uint32_t t0;
uint32_t t1;
uint32_t dt;

void init_t0(void) {
    t0 = HAL_GetTick();
}

// SET GRAVITY VECTOR = 1,0,0
Vector3 gravity_vector = {
  1.0f,
  0.0f,
  0.0f
};

Vector3 predicted_gravity_vector;

Vector3_i accels_raw;
Vector3_i gyros_raw;

Vector3 accels;
Vector3 gyros;

Vector3 error;

Quaternion current_attitude_conjugate;

HAL_StatusTypeDef imu_read_sample(Vector3_i *accel, Vector3_i *gyro) {
    HAL_StatusTypeDef status;

    status = mpu6050_read_accel(&accel->x, &accel->y, &accel->z);
    if (status != HAL_OK) return status;

    status = mpu6050_read_gyro(&gyro->x, &gyro->y, &gyro->z);
    if(status != HAL_OK) return status;

    return HAL_OK;
}

// called in main() for every sample
HAL_StatusTypeDef attitude_update(Quaternion *current_attitude) {
    if(imu_read_sample(&accels_raw, &gyros_raw) != HAL_OK) {
        return HAL_ERROR; // failed to read sample
    }

    t1 = HAL_GetTick(); // TODO: use timer
    dt = t1 - t0;
    t0 = t1;

    calibrate_sample(&accels_raw, &gyros_raw, &accels, &gyros);
    gravity_vector = vector3_normalise(accels);
    current_attitude_conjugate = quaternion_conjugate(*current_attitude);
    predicted_gravity_vector = quaternion_apply_rotation(current_attitude_conjugate, (Vector3){1.0f, 0.0f, 0.0f});
    
    error = vector3_cross(gravity_vector, predicted_gravity_vector);
    apply_error_correction(&gyros, error);

    *current_attitude = quaternion_multiply(*current_attitude, get_delta_quaternion(gyros, dt));

    return HAL_OK; // successfully updated attitude
}

// return a quaternion representing the rotation between the current orientation and previous orientation
Quaternion get_delta_quaternion(Vector3 gyros, uint32_t dt) {
    Vector3 delta = {
        gyros.x * dt * RAD_PER_DEG_MS,
        gyros.y * dt * RAD_PER_DEG_MS,
        gyros.z * dt * RAD_PER_DEG_MS
    };

    float angle = vector3_magnitude(delta);
    Vector3 axis = vector3_normalise(delta);

    return quaternion_from_axis_angle(axis, angle);
}

void apply_error_correction(Vector3 *v, Vector3 error) {
    v->x += KP * error.x * RAD_TO_DEG;
    v->y += KP * error.y * RAD_TO_DEG;
    v->z += KP * error.z * RAD_TO_DEG;
}

// calibrate each sample using calibration data obtained from calibration.py
void calibrate_sample(Vector3_i *accel, Vector3_i* gyro, Vector3 *accel_calibrated, Vector3 *gyro_calibrated) {
    accel_calibrated->x = (accel->x - ACCEL_OFFSET.x) * ACCEL_SCALE.x / RAW_TO_G;
    accel_calibrated->y = (accel->y - ACCEL_OFFSET.y) * ACCEL_SCALE.y / RAW_TO_G;
    accel_calibrated->z = (accel->z - ACCEL_OFFSET.z) * ACCEL_SCALE.z / RAW_TO_G;

    gyro_calibrated->x = (gyro->x - GYRO_BIAS.x) / RAW_TO_DPS;
    gyro_calibrated->y = (gyro->y - GYRO_BIAS.y) / RAW_TO_DPS;
    gyro_calibrated->z = (gyro->z - GYRO_BIAS.z) / RAW_TO_DPS;
}