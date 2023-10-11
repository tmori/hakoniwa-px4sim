#include "udp_connector.hpp"
#include <cstring>      // for std::memset
#include <unistd.h>     // for close
#include <arpa/inet.h>  // for inet_pton

namespace hako::px4::comm {

UdpConnector::UdpConnector() : sockfd(-1) {
    std::memset(&local_addr, 0, sizeof(local_addr));
    std::memset(&remote_addr, 0, sizeof(remote_addr));
}

UdpConnector::~UdpConnector() {
    close();
}

bool UdpConnector::client_open(IcommEndpointType *src, IcommEndpointType *dst) {
    if(!src || !dst) return false;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0) {
        return false;
    }

    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(src->portno);
    inet_pton(AF_INET, src->ipaddr, &(local_addr.sin_addr));

    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(dst->portno);
    inet_pton(AF_INET, dst->ipaddr, &(remote_addr.sin_addr));

    if(bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        close();
        return false;
    }

    return true;
}

bool UdpConnector::server_open(IcommEndpointType *src) {
    if(!src) return false;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0) {
        return false;
    }

    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(src->portno);
    inet_pton(AF_INET, src->ipaddr, &(local_addr.sin_addr));

    if(bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        close();
        return false;
    }

    return true;
}

bool UdpConnector::recv(char* data, int datalen, int* recv_datalen) {
    if(sockfd < 0 || !data || datalen <= 0) return false;

    socklen_t addr_len = sizeof(remote_addr);
    int bytes_received = recvfrom(sockfd, data, datalen, 0, (struct sockaddr*)&remote_addr, &addr_len);
    if(bytes_received < 0) {
        return false;
    }

    if(recv_datalen) {
        *recv_datalen = bytes_received;
    }

    return true;
}

bool UdpConnector::send(const char* data, int datalen, int* send_datalen) {
    if(sockfd < 0 || !data || datalen <= 0) return false;

    int bytes_sent = sendto(sockfd, data, datalen, 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
    if(bytes_sent < 0) {
        return false;
    }

    if(send_datalen) {
        *send_datalen = bytes_sent;
    }

    return true;
}

bool UdpConnector::close() {
    if(sockfd >= 0) {
        ::close(sockfd);
        sockfd = -1;
    }
    return true;
}

} // namespace hako::px4::comm
