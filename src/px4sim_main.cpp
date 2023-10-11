#include "comm/udp_connector.hpp"
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

    // Example: Receiving data
    char recvBuffer[1024];
    int recvDataLen;
    if (clientConnector.recv(recvBuffer, sizeof(recvBuffer), &recvDataLen)) 
    {
        std::cout << "Received data: " << recvBuffer << " with length: " << recvDataLen << std::endl;
    } else {
        std::cerr << "Failed to receive data" << std::endl;
    }

    clientConnector.close();
    return 0;
}
