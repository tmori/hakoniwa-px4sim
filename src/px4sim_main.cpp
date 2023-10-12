#include "comm/udp_connector.hpp"
#include "mavlink/mavlink_decoder.hpp"
#include "mavlink/mavlink_encoder.hpp"
#include <iostream>
#include <cstdlib> // for std::atoi
#include <pthread.h>
#include <unistd.h>

#include <chrono> // For obtaining the current timestamp

static void send_message(hako::px4::comm::ICommConnector &clientConnector, MavlinkDecodedMessage &message)
{
    mavlink_message_t mavlinkMsg;
    if (mavlink_encode_message(&mavlinkMsg, &message)) 
    {
        int sentDataLen = 0;
        char packet[MAVLINK_MAX_PACKET_LEN];
        int packetLen = mavlink_get_packet(packet, sizeof(packet), &mavlinkMsg);
        if (packetLen > 0) 
        {
            if (clientConnector.send(packet, packetLen, &sentDataLen)) 
            {
                std::cout << "Sent MAVLink message with length: " << sentDataLen << std::endl;
            } 
            else 
            {
                std::cerr << "Failed to send MAVLink message" << std::endl;
            }
        }
    }
}
static void send_heartbeat(hako::px4::comm::ICommConnector &clientConnector)
{
    // HEARTBEATメッセージの準備
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_HEARTBEAT;
    message.data.heartbeat.type = MAV_TYPE_GCS;
    message.data.heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    message.data.heartbeat.base_mode = MAV_MODE_FLAG_HIL_ENABLED;
    message.data.heartbeat.custom_mode = 0;
    message.data.heartbeat.system_status = MAV_STATE_STANDBY;

    send_message(clientConnector, message);
}
static auto start_time = std::chrono::system_clock::now();

static void send_sensor(hako::px4::comm::ICommConnector &clientConnector)
{
    static float test_gyro = 0.0f;
    // Static variables inside function scope
    static uint32_t fields_updated_rotation = 0x1;
    static auto prev_time = std::chrono::system_clock::now();

    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_HIL_SENSOR;

    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    uint64_t microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch).count();
    message.data.sensor.time_usec = microseconds_since_epoch;

    // Calculate the time difference between now and the last time this function was called
    auto delta_time = now - prev_time;
    auto delta_time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(delta_time);
    prev_time = now;

    // Rotate fields_updated bits based on a desired interval
    if(delta_time_microseconds.count() > 500000) { // for example, every 500ms
        fields_updated_rotation = (fields_updated_rotation << 1) | (fields_updated_rotation >> 15);  // 16-bit rotation
    }
    if(delta_time_microseconds.count() > 40000) { 
        fields_updated_rotation |= 0b111;
    }
    // 微小な変動を加える
    test_gyro += 0.001f;

    message.data.sensor.xacc = 0.0f;
    message.data.sensor.yacc = 0.0f;
    message.data.sensor.zacc = 0.0f;
    message.data.sensor.xgyro = test_gyro;
    message.data.sensor.ygyro = test_gyro;
    message.data.sensor.zgyro = test_gyro;
    message.data.sensor.xmag = 0.0f;
    message.data.sensor.ymag = 0.0f;
    message.data.sensor.zmag = 0.0f;
    message.data.sensor.abs_pressure = 1013.25f;
    message.data.sensor.diff_pressure = 0.0f;
    message.data.sensor.pressure_alt = 0.0f;
    message.data.sensor.temperature = 20.0f;

    message.data.sensor.fields_updated = fields_updated_rotation; 

    send_message(clientConnector, message);
}


static void send_ack(hako::px4::comm::ICommConnector &clientConnector, 
                     uint16_t command, 
                     uint8_t result, 
                     uint8_t target_system, 
                     uint8_t target_component)
{
    // ACKメッセージの準備
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_ACK;
    message.data.ack.command = command;
    message.data.ack.result = result;
    message.data.ack.progress = 100;
    message.data.ack.target_system = target_system;
    message.data.ack.target_component = target_component;
    message.data.ack.result_param2 = 0;

    send_message(clientConnector, message);

}
static void *receiver_thread(void *arg)
{
    hako::px4::comm::ICommConnector *clientConnector = static_cast<hako::px4::comm::ICommConnector *>(arg);
    while (true) {
        char recvBuffer[1024];
        int recvDataLen;
        if (clientConnector->recv(recvBuffer, sizeof(recvBuffer), &recvDataLen)) 
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
                        //send_ack(*clientConnector, message.data.command_long.command, MAV_RESULT_ACCEPTED, 
                        //    message.data.command_long.target_system, message.data.command_long.target_component);
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
    return NULL;
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

    pthread_t thread;
    if (pthread_create(&thread, NULL, receiver_thread, &clientConnector) != 0) {
        std::cerr << "Failed to create heartbeat sender thread!" << std::endl;
        return -1;
    }

    int count = 0;
    while (true) {
        if ((count % 10) == 0) {
            send_heartbeat(clientConnector);
        }
        if ((count % 2) == 0) { // 頻度を50Hzに変更してみてください。
            send_sensor(clientConnector);
        }

        usleep(20 * 1000);  // 50Hzに更新する場合、20msごとにsleepします。
        count++;
    }


    clientConnector.close();
    return 0;
}
