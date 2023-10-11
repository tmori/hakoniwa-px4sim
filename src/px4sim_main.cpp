#include "comm/udp_connector.hpp"
#include "mavlink/mavlink_decoder.hpp"
#include <iostream>
#include <cstdlib> // for std::atoi

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

    // Example: Sending data
    const char* sendData = "Hello from PX4 Sim!";
    int sentDataLen;
    if(clientConnector.send(sendData, strlen(sendData) + 1, &sentDataLen)) { // +1 to include null terminator
        std::cout << "Sent data: " << sendData << " with length: " << sentDataLen << std::endl;
    } else {
        std::cerr << "Failed to send data" << std::endl;
    }

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
            }
        } else {
            std::cerr << "Failed to receive data" << std::endl;
        }
    }

    clientConnector.close();
    return 0;
}
