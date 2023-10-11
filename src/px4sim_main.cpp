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
            }
        } else {
            std::cerr << "Failed to receive data" << std::endl;
        }
    }

    clientConnector.close();
    return 0;
}
