#include "mavlink_decoder.hpp"
#include <iostream>

bool mavlink_decode(const char* packet, int packet_len, mavlink_message_t *msg)
{
    mavlink_status_t status;
    bool messageReceived = false;

    for(int i = 0; i < packet_len; i++) {
        uint8_t result = mavlink_parse_char(MAVLINK_CONFIG_CHAN_0, packet[i], msg, &status);
        if(result) {
            messageReceived = true;
        }
        //std::cout << "parse result[ " << i << " ] = " << result << std::endl;
    }

    return messageReceived;
}

bool mavlink_get_message(mavlink_message_t *msg, MavlinkDecodedMessage *message)
{
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: 
        {
            message->type = MAVLINK_MSG_TYPE_HEARTBEAT;
            mavlink_msg_heartbeat_decode(msg, &message->data.heartbeat);
            return true;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            message->type = MAVLINK_MSG_TYPE_LONG;
            mavlink_msg_command_long_decode(msg, &message->data.command_long);
            return true;
        }        default:
            message->type = MAVLINK_MSG_TYPE_UNKNOWN;
            return false;
    }
}
