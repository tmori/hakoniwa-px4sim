
#include "comm/udp_connector.hpp"
#include <iostream>
#include <cstdlib> // for std::atoi

int main(int argc, char* argv[]) 
{
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <server_ip> <server_port>" << std::endl;
        return -1;
    }

    const char* serverIp = argv[1];
    int serverPort = std::atoi(argv[2]);

    // 自動採番されるポート番号を使う場合、0を指定します。
    //hako::px4::comm::IcommEndpointType localEndpoint = { nullptr, 0 };
    hako::px4::comm::IcommEndpointType serverEndpoint = { serverIp, serverPort };

    hako::px4::comm::UdpConnector udpConnector;

    if (!udpConnector.server_open(&serverEndpoint)) 
    {
        std::cerr << "Failed to open UDP server" << std::endl;
        return -1;
    }

    // Example: Receiving data
    char recvBuffer[1024];
    int recvDataLen;
    if (udpConnector.recv(recvBuffer, sizeof(recvBuffer), &recvDataLen)) 
    {
        std::cout << "Received data: " << recvBuffer << " with length: " << recvDataLen << std::endl;
    } else {
        std::cerr << "Failed to receive data" << std::endl;
    }

    udpConnector.close();
    return 0;
}
