#ifndef _DRONE_PHYS_HPP_
#define _DRONE_PHYS_HPP_

typedef struct {
    double x;
    double y;
    double z;
} Vector3Type;

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
} DronePhysType;


extern void drone_init(double delta_t, const DronePhysParamType&param, const DronePhysStateType& initial_value, DronePhysType& phys);
extern void drone_run(const DronePropellerRotationRateType& propeller, DronePhysType& phys);

#endif /* _DRONE_PHYS_HPP_ */