#include "drone_control.hpp"
#include "../../pdu/hako_pdu_data.hpp"
#include <iostream>
#include <cstring> // memsetの代わりにC++スタイルのヘッダを使用

// PIDゲインの定数を定義
constexpr float DRONE_CONTROL_PID_POS_Z_KP = 4.3f;  // 比例ゲイン
constexpr float DRONE_CONTROL_PID_POS_Z_KI = 1.0f;  // 積分ゲイン
constexpr float DRONE_CONTROL_PID_POS_Z_KD = 1.5f;  // 微分ゲイン

constexpr float DRONE_CONTROL_PID_ROT_X_KP = 20.0f;  // 比例ゲイン
constexpr float DRONE_CONTROL_PID_ROT_X_KI = 1.0f;  // 積分ゲイン
constexpr float DRONE_CONTROL_PID_ROT_X_KD = 1.0f;  // 微分ゲイン


constexpr float DRONE_THRUST_MAX = 20.0f;
constexpr float DRONE_THRUST_MIN = 0.0f;
constexpr float DRONE_TORQUE_X_MAX_RATE = 1.0f;

static inline double get_value_with_limit(double value, double max, double min)
{
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else
        return value;
}
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

static void key_init(void)
{
    struct termios oldt, newt;
    int oldf;

    // 端末設定を取得
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // カノニカルモードとエコーをオフに設定
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // ファイル記述子の設定を非ブロッキングに
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);    
}

void drone_control_init(DroneControlType& ctrl, double delta_t)
{
    key_init();
    std::memset(&ctrl, 0, sizeof(ctrl));  // 構造体の全メンバを0で初期化
    ctrl.delta_t = delta_t;
    ctrl.target_pos.target.z = 6000.0f;   // 単位：mm

    ctrl.target_rot.target.x = 0;
    // Z軸用PIDコントローラの初期化
    initPID(ctrl.target_pos.pid_z, DRONE_CONTROL_PID_POS_Z_KP, DRONE_CONTROL_PID_POS_Z_KI, DRONE_CONTROL_PID_POS_Z_KD, ctrl.target_pos.target.z);

    //X軸の水平制御
    initPID(ctrl.target_rot.pid_roll, DRONE_CONTROL_PID_ROT_X_KP, DRONE_CONTROL_PID_ROT_X_KI, DRONE_CONTROL_PID_ROT_X_KD, ctrl.target_rot.target.x);

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
    std::cout << "ctrl.target_pos.pid_z.previous_error= " << ctrl.target_pos.pid_z.previous_error << std::endl;
    ctrl.signal.thrust = get_value_with_limit(ctrl.signal.thrust, DRONE_THRUST_MAX, DRONE_THRUST_MIN);
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
    std::cout << "hil_state_quaternion.lon= " << hil_state_quaternion.lon << std::endl;

    // roll power
    static double roll_power_for_move = 0;
    std::cout << "ctrl.target_rot.pid_roll.previous_error= " << ctrl.target_rot.pid_roll.previous_error << std::endl;
    int c = getc(stdin);
    if (c == 'j') {
        std::cout << "key=" << c << std::endl;
        roll_power_for_move = -1.0;
    }
    else if (c == ' ') {
        std::cout << "key=" << c << std::endl;
        roll_power_for_move = 0;
    }
    else if (c == 'l') {
        std::cout << "key=" << c << std::endl;
        roll_power_for_move = 1.0;
    }
    double roll_power_for_same = updatePID(ctrl.target_rot.pid_roll, angle.x, ctrl.delta_t);    

    std::cout << "roll_power_for_move= " << roll_power_for_move << std::endl;
    std::cout << "roll_power_for_same= " << roll_power_for_same << std::endl;
    double torque_x = roll_power_for_move + roll_power_for_same;
    double torque_x_max = ctrl.signal.thrust * DRONE_PARAM_L / (2.0 * DRONE_TORQUE_X_MAX_RATE);
    ctrl.signal.torque.x = get_value_with_limit(torque_x, torque_x_max, -torque_x_max);

}
