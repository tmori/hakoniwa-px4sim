#include "mavlink_encoder.hpp"
#include <iostream>

int mavlink_get_packet(char* packet, int packet_len, const mavlink_message_t *msg) 
{
    if (!packet || !msg) {
        return -1;
    }

    // この関数は、エンコードされたMAVLinkメッセージのバイト数を返します。
    int len = mavlink_msg_to_send_buffer((uint8_t*)packet, msg);

    if (len <= packet_len) {
        return len;
    } else {
        return -1;
    }
}

bool mavlink_encode_message(mavlink_message_t *msg, const MavlinkDecodedMessage *message) 
{
    if (!msg || !message) {
        return false;
    }

    switch (message->type) {
        case MAVLINK_MSG_TYPE_HEARTBEAT:
            mavlink_msg_heartbeat_pack(
                MAVLINK_CONFIG_SYSTEM_ID, 
                MAVLINK_CONFIG_COMPONENT_ID, 
                msg, 
                message->data.heartbeat.type,
                message->data.heartbeat.autopilot, 
                message->data.heartbeat.base_mode,
                message->data.heartbeat.custom_mode, 
                message->data.heartbeat.system_status
            );
            return true;
        
        case MAVLINK_MSG_TYPE_LONG:
            mavlink_msg_command_long_pack(
                MAVLINK_CONFIG_SYSTEM_ID, 
                MAVLINK_CONFIG_COMPONENT_ID, 
                msg, 
                message->data.command_long.target_system,
                message->data.command_long.target_component, 
                message->data.command_long.command, 
                message->data.command_long.confirmation, 
                message->data.command_long.param1, 
                message->data.command_long.param2,
                message->data.command_long.param3, 
                message->data.command_long.param4, 
                message->data.command_long.param5, 
                message->data.command_long.param6, 
                message->data.command_long.param7
            );
            return true;
        
        default:
            std::cerr << "Unsupported message type for encoding: " << message->type << std::endl;
            return false;
    }
}
