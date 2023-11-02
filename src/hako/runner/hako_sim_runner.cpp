#include "hako_sim_runner.hpp"
#include "../pdu/hako_pdu_data.hpp"
#include "drone/drone_phys.hpp"
#include "drone/drone_phys_sensor.hpp"
#include "../../threads/px4sim_thread_sender.hpp"
#include "control/drone_control.hpp"
#include "drone/drone_phys_control.hpp"
#include <iostream>
#include <unistd.h>
#include <memory.h>
#include "hako_capi.h"

#define HAKO_AVATOR_CHANNLE_ID_MOTOR   0
#define HAKO_AVATOR_CHANNLE_ID_POS   1


typedef struct {
    HakoSimRunnerArgType *arg;
    hako_time_t asset_time;
} HakoSimControlType;
HakoSimControlType hako_sim_control;

static DroneControlType drone_ctrl;
static DronePhysType drone_phys;
static DronePropellerRotationRateType drone_propeller;

#define DRONE_PHYS_DELTA_TIME 0.001 /* 1msec */
static void my_setup()
{
    std::cout << "INFO: setup start" << std::endl;
    DronePhysParamType param;
    DronePhysStateType initial_value;
    param.m = 1;
    param.l = 0.3;
    param.gravity = 9.81;
    param.k = 0.5;
    param.p = 0.25 ;
    memset(&initial_value, 0, sizeof(initial_value));
    memset(&drone_propeller, 0, sizeof(drone_propeller));
    initial_value.pos.z = 0; //10m
    drone_init(DRONE_PHYS_DELTA_TIME, param, initial_value, drone_phys);
    drone_sensor_init(drone_phys);
    drone_control_init(drone_ctrl, DRONE_PHYS_DELTA_TIME);
    std::cout << "INFO: setup done" << std::endl;
    return;
}

static void do_io_write()
{
    Hako_HakoHilActuatorControls hil_actuator_controls;
    Hako_Twist pos;

    memset(&hil_actuator_controls, 0, sizeof(hil_actuator_controls));
    for (int i = 0; i < 4; i++) {
        hil_actuator_controls.controls[i] = drone_propeller.w[i];
    }
    if (!hako_asset_runner_pdu_write(hako_sim_control.arg->robo_name, HAKO_AVATOR_CHANNLE_ID_MOTOR, (const char*)&hil_actuator_controls, sizeof(hil_actuator_controls))) {
        std::cerr << "ERROR: can not write pdu data: hil_actuator_controls" << std::endl;
    }
    pos.linear.x = drone_phys.current.pos.x;
    pos.linear.y = drone_phys.current.pos.y;
    pos.linear.z = drone_phys.current.pos.z;
    pos.angular.x = drone_phys.current.rot.x;
    pos.angular.y = drone_phys.current.rot.y;
    pos.angular.z = drone_phys.current.rot.z;
    if (!hako_asset_runner_pdu_write(hako_sim_control.arg->robo_name, HAKO_AVATOR_CHANNLE_ID_POS, (const char*)&pos, sizeof(pos))) {
        std::cerr << "ERROR: can not write pdu data: pos" << std::endl;
    }

    //TO sender shared data
    hako_write_hil_state_quaternion(drone_phys.sensor.hil_state_quaternion);
    hako_write_hil_sensor(drone_phys.sensor.hil_sensor);
    hako_write_hil_gps(drone_phys.sensor.hil_gps);
}
#define KEISU   10.0f
static void my_task()
{
    drone_run(drone_propeller, drone_phys);
    //std::cout << "time: " << drone_phys.current_time << std::endl;
    //std::cout << "rot.z = " << drone_phys.current.rot.z << std::endl;
    //std::cout << "pos.z = " << drone_phys.current.pos.z << std::endl;
    drone_sensor_run(drone_phys);

    do_io_write();

    //px4sim_sender_do_task();
    drone_control_run(drone_ctrl);
    convert2RotationRate(drone_ctrl.signal, drone_phys, drone_propeller);
    return;
}

static void my_reset()
{
    //nothing to do
}

static hako_asset_runner_callback_t my_callbacks = {
    my_setup,   // setup
    NULL,   // write_initial_pdu_data
    my_task,   // do_task
    my_reset    // reset
};

void *hako_sim_runner(void *argp)
{
    hako_asset_runner_register_callback(&my_callbacks);
    hako_sim_control.arg = static_cast<HakoSimRunnerArgType*>(argp);
    if (hako_asset_runner_init(hako_sim_control.arg->asset_name, hako_sim_control.arg->config_path, hako_sim_control.arg->delta_time_msec * 1000) == false) {
        std::cerr << "ERROR: " << "hako_asset_runner_init() error" << std::endl;
        return nullptr;
    }
    while (true) {
        hako_sim_control.asset_time = 0;
        std::cout << "INFO: start simulation" << std::endl;
        while (true) {
            if (hako_asset_runner_step(1) == false) {
                std::cout << "INFO: stopped simulation" << std::endl;
                break;
            }
            else {
                hako_sim_control.asset_time++;
                usleep(hako_sim_control.arg->delta_time_msec * 200);
            }
            //std::cout << "STEP" << std::endl;
        }
        //リセット発生した場合は最初からやり直す。
    }
    std::cout << "INFO: end simulation" << std::endl;
    hako_asset_runner_fin();

    return nullptr;
}

