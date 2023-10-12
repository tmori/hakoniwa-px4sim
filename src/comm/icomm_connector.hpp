#ifndef _ICOMM_CONNECTOR_HPP_
#define _ICOMM_CONNECTOR_HPP_

namespace hako::px4::comm {
    typedef struct {
        const char *ipaddr;
        int portno;
    } IcommEndpointType;
    class ICommConnector {
    public:
        virtual ~ICommConnector() = default;

        virtual bool client_open(IcommEndpointType *src, IcommEndpointType *dst) = 0;

        virtual bool recv(char* data, int datalen, int* recv_datalen) = 0;
        virtual bool send(const char* data, int datalen, int* send_datalen) = 0;

        virtual bool close() = 0;
    };
}

#endif /* _ICOMM_CONNECTOR_HPP_ */