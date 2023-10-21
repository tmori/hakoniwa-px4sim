#include "hako_px4_runner.hpp"
#include "hako_px4_runner_config.hpp"
#include "hako_asset_runner.h"
#include "../pdu/hako_pdu_data.hpp"
#include <iostream>

HakoPx4RunnerArgType *px4_arg = nullptr; 

static void my_setup()
{
    //nothing to do
}
static void my_task()
{
    // READ SHARING ACTUATOR AND WRITE PDU ACTUATOR
    Hako_HakoHilActuatorControls hil_actuator_controls;
    if (hako_read_hil_actuator_controls(hil_actuator_controls)) {
        if (!hako_asset_runner_pdu_write(px4_arg->robo_name, 
                HAKO_PX4_CHANNLE_ID_HIL_ACTUATOR_CONTROLS, 
                (const char*)&hil_actuator_controls, 
                sizeof(hil_actuator_controls))) {
                    std::cerr << "ERROR: can not write pdu data: hil_actuator_controls" << std::endl;
                }
    }
    // READ PDU SENSOR AND WRITE SHARING SENSOR
    Hako_HakoHilSensor hil_sensor;
    Hako_HakoHilGps hil_gps;
    Hako_HakoHilStateQuaternion hil_state_quaternion;
    if (hako_asset_runner_pdu_read(px4_arg->robo_name, HAKO_PX4_CHANNLE_ID_HIL_SENSOR, (char*)&hil_sensor, sizeof(hil_sensor))) {
        hako_write_hil_sensor(hil_sensor);
    }
    if (hako_asset_runner_pdu_read(px4_arg->robo_name, HAKO_PX4_CHANNLE_ID_HIL_GPS, (char*)&hil_gps, sizeof(hil_gps))) {
        hako_write_hil_gps(hil_gps);
    }
    if (hako_asset_runner_pdu_read(px4_arg->robo_name, HAKO_PX4_CHANNLE_ID_HIL_STATE_QUATERNION, (char*)&hil_state_quaternion, sizeof(hil_state_quaternion))) {
        hako_write_hil_state_quaternion(hil_state_quaternion);
    }
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

void *hako_px4_runner(void *argp)
{
    px4_arg = static_cast<HakoPx4RunnerArgType*>(argp);
    if (hako_asset_runner_init(px4_arg->asset_name, px4_arg->config_path, px4_arg->delta_time_msec) == false) {
        std::cerr << "ERROR: " << "hako_asset_runner_init() error" << std::endl;
        return nullptr;
    }
    hako_asset_runner_register_callback(&my_callbacks);
    while (true) {
        std::cout << "INFO: start simulation" << std::endl;
        while (true) {
            if (hako_asset_runner_step(1) == false) {
                std::cout << "INFO: stopped simulation" << std::endl;
                break;
            }
            //std::cout << "STEP" << std::endl;
        }
        //リセット発生した場合は最初からやり直す。
    }
    std::cout << "INFO: end simulation" << std::endl;
    hako_asset_runner_fin();

    return nullptr;
}