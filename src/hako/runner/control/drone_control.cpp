#include "drone_control.hpp"
#include "../../pdu/hako_pdu_data.hpp"
#include <iostream>
#include <cstring> // memsetの代わりにC++スタイルのヘッダを使用

// PIDゲインの定数を定義
constexpr float DRONE_CONTROL_PID_POS_Z_KP = 2.3f;  // 比例ゲイン
constexpr float DRONE_CONTROL_PID_POS_Z_KI = 1.0f;  // 積分ゲイン
constexpr float DRONE_CONTROL_PID_POS_Z_KD = 0.7f;  // 微分ゲイン


constexpr float DRONE_CONTROL_PID_POS_Y_KP = 0.0f;  // 比例ゲイン
constexpr float DRONE_CONTROL_PID_POS_Y_KI = 0.0f;  // 積分ゲイン
constexpr float DRONE_CONTROL_PID_POS_Y_KD = 1.0f;  // 微分ゲイン

void drone_control_init(DroneControlType& ctrl, double delta_t)
{
    std::memset(&ctrl, 0, sizeof(ctrl));  // 構造体の全メンバを0で初期化
    ctrl.delta_t = delta_t;
    ctrl.target_pos.target.z = 6000.0f;   // 単位：mm
    //ctrl.target_pos.target.y = ((double) DRONE_GPS_INIT_LON) + 450.0f;
    //ctrl.target_rot.target.x = M_PI / 20.0;
    // Z軸用PIDコントローラの初期化
    initPID(ctrl.target_pos.pid_z, DRONE_CONTROL_PID_POS_Z_KP, DRONE_CONTROL_PID_POS_Z_KI, DRONE_CONTROL_PID_POS_Z_KD, ctrl.target_pos.target.z);
    initPID(ctrl.target_pos.pid_y, DRONE_CONTROL_PID_POS_Y_KP, DRONE_CONTROL_PID_POS_Y_KI, DRONE_CONTROL_PID_POS_Y_KD, ctrl.target_pos.target.y);
    // initPID関数がPIDControllerのインスタンスを適切に初期化することを想定しています。
}

void drone_control_run(DroneControlType& ctrl)
{
    Hako_HakoHilStateQuaternion hil_state_quaternion;

    if (hako_read_hil_state_quaternion(hil_state_quaternion) == false) {
        return;
    }
    // PID制御の計算
    ctrl.signal.thrust = updatePID(ctrl.target_pos.pid_z, hil_state_quaternion.alt, ctrl.delta_t) / 1000.0f;
    QuaternionType q;
    Vector3Type angle;
    q.w = hil_state_quaternion.attitude_quaternion[0];
    q.x = hil_state_quaternion.attitude_quaternion[1];
    q.y = hil_state_quaternion.attitude_quaternion[2];
    q.z = hil_state_quaternion.attitude_quaternion[3];
    quaternion2Euler(q, angle);
    std::cout << "angle.x = " << angle.x << std::endl;
    std::cout << "angle.y = " << angle.y << std::endl;
    std::cout << "angle.z = " << angle.z << std::endl;
    // roll power
    //ctrl.signal.torque.x = -updatePID(ctrl.target_pos.pid_y, hil_state_quaternion.lon, ctrl.delta_t) / 10000.0;
}
