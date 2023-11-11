#include "px4sim_thread_receiver.hpp"
#include "mavlink.h"
#include "../mavlink/mavlink_decoder.hpp"
#include "../mavlink/mavlink_dump.hpp"
#include "../comm/tcp_connector.hpp"
#include "../hako/pdu/hako_pdu_data.hpp"

#include <iostream>

#include "../mavlink/mavlink_msg_types.hpp"

#include "hako_capi.h"

hako_time_t hako_asset_time = 0;
hako_time_t hako_px4_asset_time = 0;
static uint64_t px4_boot_time = 0;
static void hako_mavlink_write_data(MavlinkDecodedMessage &message)
{
    switch (message.type) {
        case MAVLINK_MSG_TYPE_HIL_ACTUATOR_CONTROLS:
            hako_mavlink_write_hil_actuator_controls(message.data.hil_actuator_controls);
            if (px4_boot_time == 0) {
                px4_boot_time = message.data.hil_actuator_controls.time_usec;
            }
            else {
                hako_px4_asset_time = message.data.hil_actuator_controls.time_usec - px4_boot_time;
                std::cout << "px4_asset_time : " << hako_px4_asset_time << std::endl;
                std::cout << "hako_asset_time: " << hako_asset_time << std::endl;
                std::cout << "diff_time      : " << (long long)(hako_asset_time - hako_px4_asset_time) << std::endl;
            }
            break;
        case MAVLINK_MSG_TYPE_HEARTBEAT:
            break;
        case MAVLINK_MSG_TYPE_LONG:
            break;
        case MAVLINK_MSG_TYPE_HIL_SENSOR:
            break;
        case MAVLINK_MSG_TYPE_SYSTEM_TIME:
            break;
        case MAVLINK_MSG_TYPE_HIL_GPS:
            break;
        default:
            std::cout << "  Unknown or unsupported MAVLink message type received." << std::endl;
            break;
    }    
}

void *px4sim_thread_receiver(void *arg)
{
    std::cout << "INFO: px4 reciver start" << std::endl;
    hako::px4::comm::ICommIO *clientConnector = static_cast<hako::px4::comm::ICommIO *>(arg);
    while (true) {
        char recvBuffer[1024];
        int recvDataLen;
        if (clientConnector->recv(recvBuffer, sizeof(recvBuffer), &recvDataLen)) 
        {
            std::cout << "Received data with length: " << recvDataLen << std::endl;
            mavlink_message_t msg;
            bool ret = mavlink_decode(MAVLINK_CONFIG_CHAN_0, recvBuffer, recvDataLen, &msg);
            if (ret)
            {
                MavlinkDecodedMessage message;
                ret = mavlink_get_message(&msg, &message);
                if (ret) {
                    mavlink_msg_dump(msg);
                    mavlink_message_dump(message);
                    hako_mavlink_write_data(message);
                }
            }
        } else {
            std::cerr << "Failed to receive data" << std::endl;
        }
    }
    return NULL;
}