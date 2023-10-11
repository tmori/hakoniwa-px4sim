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
