#ifndef _DRONE_TYPES_HPP_
#define _DRONE_TYPES_HPP_

#include <math.h>

#define DRONE_PROPELLER_NUM 4

typedef struct {
    double x;
    double y;
    double z;
} Vector3Type;

typedef struct {
    double w;
    double x;
    double y;
    double z;
} QuaternionType;

static inline void euler2Quaternion(const Vector3Type& e, QuaternionType& ret)
{
    // Compute the half angles
    double cy = cos(e.z * 0.5); // yaw
    double sy = sin(e.z * 0.5);
    double cp = cos(e.y * 0.5); // pitch
    double sp = sin(e.y * 0.5);
    double cr = cos(e.x * 0.5); // roll
    double sr = sin(e.x * 0.5);

    ret.w = cr * cp * cy + sr * sp * sy;
    ret.x = sr * cp * cy - cr * sp * sy;
    ret.y = cr * sp * cy + sr * cp * sy;
    ret.z = cr * cp * sy - sr * sp * cy;
}

static inline void vector3_plus(const Vector3Type& l, const Vector3Type& r, Vector3Type& ret)
{
    ret.x = l.x + r.x;
    ret.y = l.y + r.y;
    ret.z = l.z + r.z;
}
static inline void vector3_minus(const Vector3Type& l, const Vector3Type& r, Vector3Type& ret)
{
    ret.x = l.x - r.x;
    ret.y = l.y - r.y;
    ret.z = l.z - r.z;
}
static inline void vector3_div(const Vector3Type& l, double r, Vector3Type& ret)
{
    ret.x = l.x / r;
    ret.y = l.y / r;
    ret.z = l.z / r;
}

static inline double vector3_magnitude(Vector3Type& v) 
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// クォータニオンの共役を計算する関数
static inline QuaternionType QuaternionConjugate(QuaternionType q) 
{
    QuaternionType result = {q.w, -q.x, -q.y, -q.z};
    return result;
}

// クォータニオンの積を計算する関数
static inline QuaternionType QuaternionMultiply(QuaternionType q1, QuaternionType q2) 
{
    QuaternionType result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// クォータニオンによるベクトルの回転
static inline Vector3Type RotateVectorByQuaternion(QuaternionType q, Vector3Type v) 
{
    QuaternionType vec_quat = {0, v.x, v.y, v.z};
    QuaternionType result_quat = QuaternionMultiply(QuaternionMultiply(q, vec_quat), QuaternionConjugate(q));

    Vector3Type rotated_vector = {result_quat.x, result_quat.y, result_quat.z};
    return rotated_vector;
}

typedef struct {
    double      thrust;
    Vector3Type torque;
} DroneControlSignalType;

#endif /* _DRONE_TYPES_HPP_ */