#ifndef _MAVLIN_MSG_TYPES_HPP_
#define _MAVLIN_MSG_TYPES_HPP_

#include "mavlink.h"

typedef enum {
    MAVLINK_MSG_TYPE_UNKNOWN,
    MAVLINK_MSG_TYPE_HEARTBEAT,
    MAVLINK_MSG_TYPE_LONG,
    MAVLINK_MSG_TYPE_ACK,
    MAVLINK_MSG_TYPE_HIL_SENSOR,
    MAVLINK_MSG_TYPE_NUM,
} MavlinkMsgType;

typedef struct {
    MavlinkMsgType type;
    union {
        mavlink_heartbeat_t heartbeat;
        mavlink_command_long_t command_long;
        mavlink_command_ack_t ack;
        mavlink_hil_sensor_t sensor;
    } data;
} MavlinkDecodedMessage;


#endif /* _MAVLIN_MSG_TYPES_HPP_ */
