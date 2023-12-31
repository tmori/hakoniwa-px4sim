#include "drone_phys_sensor.hpp"
#include <iostream>

#define HIL_GPS_UP_CYCLE                60  /* msec*/
#define HIL_STATE_QUATERNION_UP_CYCLE    4  /* msec*/
#define HIL_SENSOR_UP_CYCLE              8 /* msec*/

// Tokyo
#define REFERENCE_LATITUDE      35.6895
#define REFERENCE_LONGTITUDE    139.6917
#define REFERENCE_ALTITUDE      0
static Vector3Type TOKYO_MAGNETIC_NORTH = {0.5, 0, 0};

static int32_t CalculateLatitude(const Vector3Type &dronePosition, double referenceLatitude);
static int32_t CalculateLongitude(const Vector3Type &dronePosition, double referenceLongitude);
static int32_t CalculateAltitude(const Vector3Type &dronePosition, double referenceAltitude);
static Vector3Type CalcMAVLinkMagnet(const DronePhysType &phys);


void drone_sensor_init(DronePhysType &phys)
{
    phys.sensor.up_cycle_hil_sensor = HIL_SENSOR_UP_CYCLE;
    phys.sensor.up_cycle_hil_state_quaternion = HIL_STATE_QUATERNION_UP_CYCLE;
    phys.sensor.up_cycle_hil_gps = HIL_GPS_UP_CYCLE;
    memset(&phys.prev_vec, 0, sizeof(phys.prev_vec));
    memset(&phys.sensor.hil_gps, 0, sizeof(phys.sensor.hil_gps));
    memset(&phys.sensor.hil_sensor, 0, sizeof(phys.sensor.hil_sensor));
    memset(&phys.sensor.hil_state_quaternion, 0, sizeof(phys.sensor.hil_state_quaternion));
    initAverageData(phys.sensor_pos, HIL_SENSOR_UP_CYCLE);
    initAverageData(phys.sensor_vec, HIL_SENSOR_UP_CYCLE);
    initAverageData(phys.sensor_acc, HIL_SENSOR_UP_CYCLE);
}

static void drone_sensor_run_hil_state_quaternion(DronePhysType &phys);
static void drone_sensor_run_hil_sensor(DronePhysType &phys);
static void drone_sensor_run_hil_gps(DronePhysType &phys);

void drone_sensor_run(DronePhysType &phys)
{
    drone_sensor_run_hil_state_quaternion(phys);
    drone_sensor_run_hil_sensor(phys);
    drone_sensor_run_hil_gps(phys);
}

static void drone_sensor_run_hil_state_quaternion(DronePhysType &phys)
{
    phys.sensor.hil_state_quaternion.time_usec = 0;

    phys.sensor.hil_state_quaternion.rollspeed = phys.current.rot_vec.x;
    phys.sensor.hil_state_quaternion.pitchspeed = phys.current.rot_vec.y;
    phys.sensor.hil_state_quaternion.yawspeed = phys.current.rot_vec.z;

#if 0
    phys.sensor.hil_state_quaternion.vx = (Hako_int16)(phys.current.vec.x * 100);
    phys.sensor.hil_state_quaternion.vy = (Hako_int16)(phys.current.vec.y * 100);
    phys.sensor.hil_state_quaternion.vz = (Hako_int16)(phys.current.vec.z * 100);
#else
    Vector3Type ave_vec;
    addAverageData(phys.sensor_vec, phys.current.vec);
    calcAverage(phys.sensor_vec, ave_vec);
    phys.sensor.hil_state_quaternion.vx = (Hako_int16)(ave_vec.x * 100);
    phys.sensor.hil_state_quaternion.vy = (Hako_int16)(ave_vec.y * 100);
    phys.sensor.hil_state_quaternion.vz = (Hako_int16)(ave_vec.z * 100);
#endif
    // for acc
    Vector3Type acc_1;
    Vector3Type acc;
    Vector3Type ave_acc;
    vector3_minus(phys.current.vec, phys.prev_vec, acc_1);
    vector3_div(acc_1, phys.delta_t, acc);
    acc.z -= phys.param.gravity;
    phys.prev_vec = phys.current.vec;
    addAverageData(phys.sensor_acc, acc);
    calcAverage(phys.sensor_acc, ave_acc);
    phys.sensor.hil_state_quaternion.xacc = (Hako_int16)(ave_acc.x * 1000.0f);
    phys.sensor.hil_state_quaternion.yacc = (Hako_int16)(ave_acc.y * 1000.0f);
    phys.sensor.hil_state_quaternion.zacc = (Hako_int16)(ave_acc.z * 1000.0f);

    QuaternionType q;
    euler2Quaternion(phys.current.rot, q);
    phys.sensor.hil_state_quaternion.attitude_quaternion[0] = q.w;
    phys.sensor.hil_state_quaternion.attitude_quaternion[1] = q.x;
    phys.sensor.hil_state_quaternion.attitude_quaternion[2] = q.y;
    phys.sensor.hil_state_quaternion.attitude_quaternion[3] = q.z;

    phys.sensor.hil_state_quaternion.ind_airspeed = 0;
    phys.sensor.hil_state_quaternion.true_airspeed = 0;

#if 0
    phys.sensor.hil_state_quaternion.lat = CalculateLatitude(phys.current.pos, REFERENCE_LATITUDE);
    phys.sensor.hil_state_quaternion.lon = CalculateLongitude(phys.current.pos, REFERENCE_LONGTITUDE);
    phys.sensor.hil_state_quaternion.alt = CalculateAltitude(phys.current.pos, REFERENCE_ALTITUDE);
    //std::cout << "alt: " << phys.current.pos.z << std::endl;
#else
    Vector3Type ave_pos;
    addAverageData(phys.sensor_pos, phys.current.pos);
    calcAverage(phys.sensor_pos, ave_pos);
    phys.sensor.hil_state_quaternion.lat = CalculateLatitude(ave_pos, REFERENCE_LATITUDE);
    phys.sensor.hil_state_quaternion.lon = CalculateLongitude(ave_pos, REFERENCE_LONGTITUDE);
    phys.sensor.hil_state_quaternion.alt = CalculateAltitude(ave_pos, REFERENCE_ALTITUDE);
#endif

    return;
}
static void drone_sensor_run_hil_sensor(DronePhysType &phys)
{
    phys.sensor.hil_sensor.time_usec = 0;
    phys.sensor.hil_sensor.xacc = phys.sensor.hil_state_quaternion.xacc / 1000.0f;
    phys.sensor.hil_sensor.yacc = phys.sensor.hil_state_quaternion.yacc / 1000.0f;
    phys.sensor.hil_sensor.zacc = phys.sensor.hil_state_quaternion.zacc / 1000.0f;

    phys.sensor.hil_sensor.xgyro = phys.sensor.hil_state_quaternion.rollspeed;
    phys.sensor.hil_sensor.ygyro = phys.sensor.hil_state_quaternion.pitchspeed;
    phys.sensor.hil_sensor.zgyro = phys.sensor.hil_state_quaternion.yawspeed;

    Vector3Type mag = CalcMAVLinkMagnet(phys);
    phys.sensor.hil_sensor.xmag = mag.x;
    phys.sensor.hil_sensor.ymag = mag.y;
    phys.sensor.hil_sensor.zmag = mag.z;

    phys.sensor.hil_sensor.abs_pressure = 1013.25f;  // Standard atmospheric pressure at sea level
    phys.sensor.hil_sensor.diff_pressure = 0.0f;  // Differential pressure (used for airspeed calculation)
    phys.sensor.hil_sensor.pressure_alt = 0.0f;  // Pressure altitude
    phys.sensor.hil_sensor.temperature = 20.0f;  // Assume 20 degrees Celsius by default

    phys.sensor.hil_sensor.fields_updated = 0x1FFF;  // Bitmask indicating which fields are valid (assuming all fields are updated for simplicity)
    phys.sensor.hil_sensor.id = 0;  // Sensor instance ID (use default 0)
    return;
}
static void drone_sensor_run_hil_gps(DronePhysType &phys)
{
    phys.sensor.hil_gps.time_usec = 0;
    phys.sensor.hil_gps.fix_type = 3;
    phys.sensor.hil_gps.lat = phys.sensor.hil_state_quaternion.lat;
    phys.sensor.hil_gps.lon = phys.sensor.hil_state_quaternion.lon;
    phys.sensor.hil_gps.alt = phys.sensor.hil_state_quaternion.alt;

    phys.sensor.hil_gps.eph = 100;
    phys.sensor.hil_gps.epv = 100;

#if 0
    phys.sensor.hil_gps.vel = vector3_magnitude(phys.current.vec) * 100.0f;
    phys.sensor.hil_gps.vn = phys.current.vec.x;
    phys.sensor.hil_gps.ve = phys.current.vec.y;
    phys.sensor.hil_gps.vd = -phys.current.vec.z;
#else
    Vector3Type ave_vec;
    calcAverage(phys.sensor_vec, ave_vec);
    phys.sensor.hil_gps.vel = vector3_magnitude(ave_vec) * 100.0f;
    phys.sensor.hil_gps.vn = ave_vec.x;
    phys.sensor.hil_gps.ve = ave_vec.y;
    phys.sensor.hil_gps.vd = -ave_vec.z;
#endif
    phys.sensor.hil_gps.cog = 0;
    phys.sensor.hil_gps.satellites_visible = 10;
    phys.sensor.hil_gps.id = 0;
    phys.sensor.hil_gps.yaw = 0;
    return;
}


static int32_t CalculateAltitude(const Vector3Type& dronePosition, double referenceAltitude) 
{
    // ROS's Z axis represents altitude.
    // Assuming dronePosition.z is in meters and we need to return altitude in millimeters.
    int32_t altitude = (int32_t)((dronePosition.z + referenceAltitude) * 1000);
    return altitude;
}

static int32_t CalculateLongitude(const Vector3Type& dronePosition, double referenceLongitude) 
{
    // Convert ROS position (in meters) to change in longitude based on reference.
    double deltaLongitude = dronePosition.y / 111000.0; // ROS's Y axis represents East-West direction.
    int32_t longitude = (int32_t)((referenceLongitude + deltaLongitude) * 1e7); // Convert to 1e7 format used by MAVLink
    return longitude;
}

static int32_t CalculateLatitude(const Vector3Type& dronePosition, double referenceLatitude) 
{
    // Convert ROS position (in meters) to change in latitude based on reference.
    double deltaLatitude = dronePosition.x / 111000.0; // ROS's X axis represents North-South direction.
    int32_t latitude = (int32_t)((referenceLatitude + deltaLatitude) * 1e7); // Convert to 1e7 format used by MAVLink
    return latitude;
}
static Vector3Type CalcMAVLinkMagnet(const DronePhysType &phys)
{
    QuaternionType rotation;
    euler2Quaternion(phys.current.rot, rotation);
    Vector3Type adjustedMagneticNorth = RotateVectorByQuaternion(rotation, TOKYO_MAGNETIC_NORTH);
    return adjustedMagneticNorth;
}
