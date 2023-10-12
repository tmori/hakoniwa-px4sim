#include "comm/udp_connector.hpp"
#include "mavlink/mavlink_decoder.hpp"
#include "mavlink/mavlink_encoder.hpp"
#include <iostream>
#include <cstdlib> // for std::atoi

static void send_heartbeat(hako::px4::comm::ICommConnector &clientConnector)
{
    // HEARTBEATメッセージの準備
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_HEARTBEAT;
    message.data.heartbeat.type = MAV_TYPE_QUADROTOR; // 例として
    message.data.heartbeat.autopilot = MAV_AUTOPILOT_PX4;
    message.data.heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    message.data.heartbeat.custom_mode = 0; // 任意のカスタムモード
    message.data.heartbeat.system_status = MAV_STATE_STANDBY; // 例として

    // メッセージをエンコード
    mavlink_message_t mavlinkMsg;
    if (mavlink_encode_message(&mavlinkMsg, &message)) 
    {
        int sentDataLen = 0;
        // パケットにエンコードされたメッセージを格納
        char packet[MAVLINK_MAX_PACKET_LEN];
        int packetLen = mavlink_get_packet(packet, sizeof(packet), &mavlinkMsg);
        if (packetLen > 0) 
        {
            if (clientConnector.send(packet, packetLen, &sentDataLen)) 
            {
                std::cout << "Sent MAVLink HEARTBEAT message with length: " << sentDataLen << std::endl;
            } 
            else 
            {
                std::cerr << "Failed to send MAVLink HEARTBEAT message" << std::endl;
            }
        }
    }

}

int main(int argc, char* argv[]) 
{
    if(argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <remote_ip> <remote_port> <server_ip> <server_port>" << std::endl;
        return -1;
    }

    const char* remoteIp = argv[1];
    int remotePort = std::atoi(argv[2]);
    const char* serverIp = argv[3];
    int serverPort = std::atoi(argv[4]);

    hako::px4::comm::IcommEndpointType remoteEndpoint = { remoteIp, remotePort };
    hako::px4::comm::IcommEndpointType serverEndpoint = { serverIp, serverPort };

    hako::px4::comm::UdpConnector clientConnector;

    if (!clientConnector.client_open(&serverEndpoint, &remoteEndpoint)) 
    {
        std::cerr << "Failed to open UDP client" << std::endl;
        return -1;
    }
    send_heartbeat(clientConnector);

    while (true) {
        char recvBuffer[1024];
        int recvDataLen;
        if (clientConnector.recv(recvBuffer, sizeof(recvBuffer), &recvDataLen)) 
        {
            std::cout << "Received data with length: " << recvDataLen << std::endl;
            mavlink_message_t msg;
            bool ret = mavlink_decode(recvBuffer, recvDataLen, &msg);
            if (ret) 
            {
                std::cout << "Decoded MAVLink message:" << std::endl;
                std::cout << "  Message ID: " << msg.msgid << std::endl;
                std::cout << "  System ID: " << static_cast<int>(msg.sysid) << std::endl;
                std::cout << "  Component ID: " << static_cast<int>(msg.compid) << std::endl;
                std::cout << "  Sequence: " << static_cast<int>(msg.seq) << std::endl;

                MavlinkDecodedMessage message;
                ret = mavlink_get_message(&msg, &message);
                if (ret) {
                    switch (message.type) {
                    case MAVLINK_MSG_TYPE_HEARTBEAT:
                        std::cout << "  Type: HEARTBEAT" << std::endl;
                        std::cout << "  Custom mode: " << message.data.heartbeat.custom_mode << std::endl;
                        std::cout << "  Base mode: " << static_cast<int>(message.data.heartbeat.base_mode) << std::endl;
                        std::cout << "  System status: " << static_cast<int>(message.data.heartbeat.system_status) << std::endl;
                        std::cout << "  MAVLink version: " << static_cast<int>(message.data.heartbeat.mavlink_version) << std::endl;
                        break;
                    
                    case MAVLINK_MSG_TYPE_LONG:
                        std::cout << "  Type: COMMAND_LONG" << std::endl;
                        std::cout << "  Target system: " << static_cast<int>(message.data.command_long.target_system) << std::endl;
                        std::cout << "  Target component: " << static_cast<int>(message.data.command_long.target_component) << std::endl;
                        std::cout << "  Command ID: " << message.data.command_long.command << std::endl;
                        std::cout << "  Confirmation: " << static_cast<int>(message.data.command_long.confirmation) << std::endl;
                        // ... 他のCOMMAND_LONGのパラメータも同様に表示することができます ...
                        break;
                    
                    default:
                        std::cout << "  Unknown or unsupported MAVLink message type received." << std::endl;
                        break;
                    }
                }
                send_heartbeat(clientConnector);
            }
        } else {
            std::cerr << "Failed to receive data" << std::endl;
        }
    }

    clientConnector.close();
    return 0;
}
