#include "drone_phys_sensor.hpp"
#define HIL_GPS_UP_CYCLE                60  /* msec*/
#define HIL_STATE_QUATERNION_UP_CYCLE    4  /* msec*/
#define HIL_SENSOR_UP_CYCLE              4 /* msec*/

void drone_sensor_init(DronePhysType &phys)
{
    phys.sensor.up_cycle_hil_sensor = HIL_SENSOR_UP_CYCLE;
    phys.sensor.up_cycle_hil_state_quaternion = HIL_STATE_QUATERNION_UP_CYCLE;
    phys.sensor.up_cycle_hil_gps = HIL_GPS_UP_CYCLE;
    memset(&phys.prev_vec, 0, sizeof(phys.prev_vec));
    memset(&phys.sensor.hil_gps, 0, sizeof(phys.sensor.hil_gps));
    memset(&phys.sensor.hil_sensor, 0, sizeof(phys.sensor.hil_sensor));
    memset(&phys.sensor.hil_state_quaternion, 0, sizeof(phys.sensor.hil_state_quaternion));
    initAverageData(phys.sensor_acc, HIL_SENSOR_UP_CYCLE);
}

static void drone_sensor_run_hil_state_quaternion(DronePhysType &phys);
static void drone_sensor_run_hil_sensor(DronePhysType &phys);
static void drone_sensor_run_hil_gps(DronePhysType &phys);

void droner_sensor_run(DronePhysType &phys)
{
    drone_sensor_run_hil_state_quaternion(phys);
    drone_sensor_run_hil_sensor(phys);
    drone_sensor_run_hil_gps(phys);
}

static void drone_sensor_run_hil_state_quaternion(DronePhysType &phys)
{
    //TODO
    phys.sensor.hil_state_quaternion.time_usec = 0;
    
    phys.sensor.hil_state_quaternion.rollspeed = phys.current.rot_vec.x;
    phys.sensor.hil_state_quaternion.pitchspeed = phys.current.rot_vec.y;
    phys.sensor.hil_state_quaternion.yawspeed = phys.current.rot_vec.z;

    phys.sensor.hil_state_quaternion.vx = (Hako_int16)(phys.current.vec.x * 100);
    phys.sensor.hil_state_quaternion.vy = (Hako_int16)(phys.current.vec.y * 100);
    phys.sensor.hil_state_quaternion.vz = (Hako_int16)(phys.current.vec.z * 100);

    // for acc
    Vector3Type acc_1;
    Vector3Type acc;
    Vector3Type ave_acc;
    vector3_minus(phys.current.vec, phys.prev_vec, acc_1);
    vector3_div(acc_1, phys.delta_t, acc);
    phys.prev_vec = phys.current.vec;
    addAverageData(phys.sensor_acc, acc);
    calcAverage(phys.sensor_acc, ave_acc);
    phys.sensor.hil_state_quaternion.xacc = ave_acc.x;
    phys.sensor.hil_state_quaternion.yacc = ave_acc.y;
    phys.sensor.hil_state_quaternion.zacc = ave_acc.z;

}
static void drone_sensor_run_hil_sensor(DronePhysType &phys)
{
    //TODO
    phys.sensor.hil_sensor.time_usec = 0;
}
static void drone_sensor_run_hil_gps(DronePhysType &phys)
{
    //TODO
    phys.sensor.hil_gps.time_usec = 0;
}
