#include "tcp_connector.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <iostream>  // Added for error output
#include <arpa/inet.h>  // for inet_pton

namespace hako::px4::comm {

TcpConnector::TcpConnector() {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cout << "Failed to create socket: " << strerror(errno) << std::endl;
    }
    memset(&local_addr, 0, sizeof(local_addr));
    memset(&remote_addr, 0, sizeof(remote_addr));
}

TcpConnector::~TcpConnector() {
    close();
}

bool TcpConnector::client_open(IcommEndpointType *src, IcommEndpointType *dst) {
    if (src == nullptr || dst == nullptr) {
        std::cout << "Source or destination endpoint is nullptr" << std::endl;
        return false;
    }

    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = inet_addr(src->ipaddr);
    local_addr.sin_port = htons(src->portno);
    if (bind(sockfd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cout << "Failed to bind socket: " << strerror(errno) << std::endl;
        return false;
    }

    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(dst->ipaddr);
    remote_addr.sin_port = htons(dst->portno);
    if (connect(sockfd, (struct sockaddr*)&remote_addr, sizeof(remote_addr)) < 0) {
        std::cout << "Failed to connect: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}
#define MAVLINK_HEADER_LEN  10
bool TcpConnector::recv(char* data, int datalen, int* recv_datalen) {
    // see: http://mavlink.io/en/guide/serialization.html

    char header[MAVLINK_HEADER_LEN];
    int received = 0;

    // Receive header
    while (received < MAVLINK_HEADER_LEN) {
        int len = read(sockfd, header + received, MAVLINK_HEADER_LEN - received);
        if (len > 0) {
            received += len;
        } else if (len == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
            std::cout << "Failed to receive MAVLink header: " << strerror(errno) << std::endl;
            return false;
        }
    }

    // Parse header to get packet length (assuming packet length is at offset 1)
    int packetlen = static_cast<unsigned char>(header[1]);

    // Check if datalen is sufficient to hold header and packet data
    if (datalen < MAVLINK_HEADER_LEN + packetlen) {
        std::cout << "Provided data buffer is too small to hold the MAVLink message." << std::endl;
        return false;
    }

    // Copy header data to output buffer
    memcpy(data, header, MAVLINK_HEADER_LEN);

    // Receive packet data
    received = 0;
    while (received < packetlen) {
        int len = read(sockfd, data + MAVLINK_HEADER_LEN + received, packetlen - received);
        if (len > 0) {
            received += len;
        } else if (len == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
            std::cout << "Failed to receive MAVLink data: " << strerror(errno) << std::endl;
            return false;
        }
    }

    *recv_datalen = MAVLINK_HEADER_LEN + packetlen;
    return true;
}


bool TcpConnector::send(const char* data, int datalen, int* send_datalen) {
    int total_sent = 0;
    while (total_sent < datalen) {
        int sent = write(sockfd, data + total_sent, datalen - total_sent);
        if (sent > 0) {
            total_sent += sent;
        } else if (sent == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
            std::cout << "Failed to send data: " << strerror(errno) << std::endl;
            break;
        }
    }
    *send_datalen = total_sent;
    return total_sent == datalen;
}

bool TcpConnector::close() {
    if (::close(sockfd) < 0) {
        std::cout << "Failed to close socket: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

}  // namespace hako::px4::comm
