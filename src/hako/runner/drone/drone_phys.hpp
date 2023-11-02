#ifndef _DRONE_PHYS_HPP_
#define _DRONE_PHYS_HPP_

#include "../../pdu/hako_pdu_data.hpp"

typedef struct {
    uint64 up_cycle_hil_sensor;
    uint64 up_cycle_hil_gps;
    uint64 up_cycle_hil_state_quaternion;
    Hako_HakoHilSensor hil_sensor;
    Hako_HakoHilGps hil_gps;
    Hako_HakoHilStateQuaternion hil_state_quaternion;
} DroneSensorType;

typedef struct {
    Hako_HakoHilActuatorControls hil_actuator_controls;
} DroneActuatorType;


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

#define DRONE_PROPELLER_NUM 4

typedef struct {
    double w[DRONE_PROPELLER_NUM];
} DronePropellerRotationRateType;


typedef struct {
    /*
     * ドローンの質量： 1 kg
     */
    double m;
    /*
     * 重力加速度：9.81 m/s^2
     */
    double gravity;
    /*
     * プロペラ回転中心から機体重心までの距離：0.3m
     */
    double l;
    /*
     * ρ と κ はプロペラの形状等で決まる空力特性に 依存するパラメータ
     */
    double p;
    double k;
} DronePhysParamType;

typedef struct {
    Vector3Type pos;
    Vector3Type vec;
    Vector3Type rot;
    Vector3Type rot_vec;
} DronePhysStateType;

#define SENSOR_MAX_COUNT_NUM    10
typedef struct {
    int inx;
    int max_count;
    int data_num;
    Vector3Type values[SENSOR_MAX_COUNT_NUM];
} DroneSensorAverageDataType;
static inline bool initAverageData(DroneSensorAverageDataType& data, int max_count)
{
    if (max_count > SENSOR_MAX_COUNT_NUM || max_count <= 0) {
        return false;  // 入力されたmax_countが範囲外であれば、失敗を返す
    }

    data.inx = 0;
    data.max_count = max_count;
    data.data_num = 0;
    for (int i = 0; i < SENSOR_MAX_COUNT_NUM; ++i) {
        data.values[i].x = 0.0;
        data.values[i].y = 0.0;
        data.values[i].z = 0.0;
    }

    return true;  // 初期化成功
}
static inline void addAverageData(DroneSensorAverageDataType& data, const Vector3Type value)
{
    // Add the new value to the values array at the current index
    data.values[data.inx] = value;

    // Update the index for the next data point
    data.inx = (data.inx + 1) % data.max_count;

    // Update the data count (but don't let it exceed SENSOR_AVERAGE_COUNT)
    if (data.data_num < data.max_count)
    {
        data.data_num++;
    }
}

static inline void calcAverage(DroneSensorAverageDataType& data, Vector3Type& result)
{
    result.x = 0;
    result.y = 0;
    result.z = 0;

    // Sum up all the values in the array
    for (int i = 0; i < data.data_num; i++)
    {
        result.x += data.values[i].x;
        result.y += data.values[i].y;
        result.z += data.values[i].z;
    }

    // Calculate the average
    if (data.data_num > 0)
    {
        result.x /= data.data_num;
        result.y /= data.data_num;
        result.z /= data.data_num;
    }
}

typedef struct {
    /*
     * constant values
     */
    double delta_t;
    DronePhysParamType param;
    DronePhysStateType initial_value;
    /*
     * update values
     */
    double current_time;
    DronePhysStateType current;
    DronePhysStateType next;


    /*
     * for other module communication data:
     */
    /*
     * Actuators
     */
    DroneActuatorType actuator;
    /*
     * Sensors
     */
    DroneSensorType sensor;
    DroneSensorAverageDataType sensor_acc;
    Vector3Type prev_vec;
} DronePhysType;


extern void drone_init(double delta_t, const DronePhysParamType&param, const DronePhysStateType& initial_value, DronePhysType& phys);
extern void drone_run(const DronePropellerRotationRateType& propeller, DronePhysType& phys);

#endif /* _DRONE_PHYS_HPP_ */