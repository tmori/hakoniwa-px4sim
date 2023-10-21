#ifndef _HAKO_PDU_DATA_HPP_
#define _HAKO_PDU_DATA_HPP_

#include "hako_mavlink_msgs/pdu_ctype_conv_mavlink_HakoHilSensor.hpp"
#include "hako_mavlink_msgs/pdu_ctype_conv_mavlink_HakoHilGps.hpp"
#include "hako_mavlink_msgs/pdu_ctype_conv_mavlink_HakoHilStateQuaternion.hpp"
#include "hako_mavlink_msgs/pdu_ctype_conv_mavlink_HakoHilActuatorControls.hpp"

extern bool hako_read_hil_sensor(Hako_HakoHilSensor &hil_sensor);
extern bool hako_read_hil_gps(Hako_HakoHilGps &hil_gps);
extern bool hako_read_hil_state_quaternion(Hako_HakoHilStateQuaternion &hil_state_quaternion);

extern void hako_write_hil_sensor(const Hako_HakoHilSensor &hil_sensor);
extern void hako_write_hil_gps(const Hako_HakoHilGps &hil_gps);
extern void hako_write_hil_state_quaternion(const Hako_HakoHilStateQuaternion &hil_state_quaternion);

extern bool hako_read_hil_actuator_controls(Hako_HakoHilActuatorControls &hil_actuator_controls);

extern void hako_write_hil_actuator_controls(const Hako_HakoHilActuatorControls &hil_actuator_controls);


extern Hako_uint64 hako_get_current_time_usec();


#endif /* _HAKO_PDU_DATA_HPP_ */