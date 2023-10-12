#ifndef _TCPCONNECTOR_HPP_
#define _TCPCONNECTOR_HPP_

#include "icomm_connector.hpp"
#include <sys/socket.h>
#include <netinet/in.h>

namespace hako::px4::comm {

class TcpConnector : public ICommConnector {
private:
    int sockfd; // ソケットのディスクリプタ
    struct sockaddr_in local_addr; // ローカルのアドレス情報
    struct sockaddr_in remote_addr; // リモートのアドレス情報

public:
    TcpConnector();
    ~TcpConnector() override;

    bool client_open(IcommEndpointType *src, IcommEndpointType *dst) override;

    bool recv(char* data, int datalen, int* recv_datalen) override;
    bool send(const char* data, int datalen, int* send_datalen) override;

    bool close() override;

};

} // namespace hako::px4::comm

#endif /* _TCPCONNECTOR_HPP_ */
