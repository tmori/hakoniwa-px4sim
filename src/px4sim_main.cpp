#include "comm/tcp_connector.hpp"
#include "mavlink/mavlink_decoder.hpp"
#include "mavlink/mavlink_encoder.hpp"
#include <iostream>
#include <cstdlib> // for std::atoi
#include <pthread.h>
#include <unistd.h>
#include <random>
#include <chrono> // For obtaining the current timestamp

static void send_message(hako::px4::comm::ICommIO &clientConnector, MavlinkDecodedMessage &message)
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
                //std::cout << "Sent MAVLink message with length: " << sentDataLen << std::endl;
            } 
            else 
            {
                std::cerr << "Failed to send MAVLink message" << std::endl;
            }
        }
    }
}
static void send_command_long(hako::px4::comm::ICommIO &clientConnector)
{
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_LONG;
    
    // Setting up the fields for COMMAND_LONG
    message.data.command_long.target_system = 0; // The system which should execute the command, for example, 1 for the first MAV
    message.data.command_long.target_component = 0; // The component which should execute the command, for example, 0 for a generic component
    message.data.command_long.command = 0x4246;
    message.data.command_long.confirmation = 0; // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
    message.data.command_long.param1 = 0x9c40; // Parameter 1, as defined by MAV_CMD enum
    message.data.command_long.param2 = 0x45; // Parameter 2, as defined by MAV_CMD enum
    message.data.command_long.param3 = 0; // Parameter 3, as defined by MAV_CMD enum
    message.data.command_long.param4 = 0; // Parameter 4, as defined by MAV_CMD enum
    message.data.command_long.param5 = 0; // Parameter 5, as defined by MAV_CMD enum
    message.data.command_long.param6 = 0; // Parameter 6, as defined by MAV_CMD enum
    message.data.command_long.param7 = 0; // Parameter 7, as defined by MAV_CMD enum
    
    send_message(clientConnector, message);
}

static void send_heartbeat(hako::px4::comm::ICommIO &clientConnector)
{
    // HEARTBEATメッセージの準備
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_HEARTBEAT;
    message.data.heartbeat.type = MAV_TYPE_GENERIC;
    message.data.heartbeat.autopilot = 0;
    message.data.heartbeat.base_mode = 0;
    message.data.heartbeat.custom_mode = 0;
    message.data.heartbeat.system_status = 0;

    send_message(clientConnector, message);
}
static void send_system_time(hako::px4::comm::ICommIO &clientConnector, uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    // SYSTEM_TIMEメッセージの準備
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_SYSTEM_TIME;
    message.data.system_time.time_unix_usec = time_unix_usec;
    message.data.system_time.time_boot_ms = time_boot_ms;

    send_message(clientConnector, message);
}
static void send_hil_state_quaternion(hako::px4::comm::ICommIO &clientConnector, uint64_t time_usec)
{
    // HIL_STATE_QUATERNIONメッセージの準備
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_HIL_STATE_QUATERNION;
    message.data.hil_state_quaternion.time_usec = time_usec;
    // 以下の値は固定値として設定
    message.data.hil_state_quaternion.attitude_quaternion[0] = 1.0;
    message.data.hil_state_quaternion.attitude_quaternion[1] = 0.0;
    message.data.hil_state_quaternion.attitude_quaternion[2] = 0.0;
    message.data.hil_state_quaternion.attitude_quaternion[3] = 0.0;
    message.data.hil_state_quaternion.rollspeed = 0.0;
    message.data.hil_state_quaternion.pitchspeed = 0.0;
    message.data.hil_state_quaternion.yawspeed = 0.0;
    message.data.hil_state_quaternion.lat = 463700; //4c	52	40	1c
    message.data.hil_state_quaternion.lon = 337732; //44	f4	17	5
    message.data.hil_state_quaternion.alt = 0;
    message.data.hil_state_quaternion.vx = 0;
    message.data.hil_state_quaternion.vy = 0;
    message.data.hil_state_quaternion.vz = 0;
    message.data.hil_state_quaternion.ind_airspeed = 0;
    message.data.hil_state_quaternion.true_airspeed = 0;
    message.data.hil_state_quaternion.xacc = 0;
    message.data.hil_state_quaternion.yacc = 0;
    message.data.hil_state_quaternion.zacc = 0;

    send_message(clientConnector, message);
}
static void send_hil_gps(hako::px4::comm::ICommIO &clientConnector, uint64_t time_usec)
{
    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution(0.0, 0.01); // 平均: 0, 標準偏差: 0.01

    // HIL_GPSメッセージの準備
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_HIL_GPS;
    message.data.hil_gps.time_usec = time_usec;
    // 以下の値は固定値として設定
    message.data.hil_gps.fix_type = 0;  // GPS_FIX_TYPE_3D_FIX
    message.data.hil_gps.lat = 473977418 + distribution(generator);
    message.data.hil_gps.lon = 85455939 + distribution(generator);
    message.data.hil_gps.alt = 488008 + distribution(generator);
    message.data.hil_gps.eph = 9778;
    message.data.hil_gps.epv = 9778;
    message.data.hil_gps.vel = 0;
    message.data.hil_gps.vn = 0;
    message.data.hil_gps.ve = 0;
    message.data.hil_gps.vd = 0;
    message.data.hil_gps.cog = 0;
    message.data.hil_gps.satellites_visible = 0;
    message.data.hil_gps.id = 0;
    message.data.hil_gps.yaw = 0;

    send_message(clientConnector, message);
}

static auto start_time = std::chrono::system_clock::now();

static void send_sensor(hako::px4::comm::ICommIO &clientConnector, uint64_t time_usec)
{
    // Random noise generator setup
    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution(0.0, 0.01); // mean: 0, std_dev: 0.01    
    MavlinkDecodedMessage message;
    message.type = MAVLINK_MSG_TYPE_HIL_SENSOR;
    message.data.sensor.time_usec = time_usec;
    message.data.sensor.xacc = -0.0785347 + distribution(generator);
    message.data.sensor.yacc = 0.00118181 + distribution(generator);
    message.data.sensor.zacc = -9.83317 + distribution(generator);
    message.data.sensor.xgyro = 0.00286057 + distribution(generator);
    message.data.sensor.ygyro = -0.00736865 + distribution(generator);
    message.data.sensor.zgyro = -0.00834229 + distribution(generator);
    message.data.sensor.xmag = 0.217065 + distribution(generator);
    message.data.sensor.ymag = 0.0063418 + distribution(generator);
    message.data.sensor.zmag = 0.422639 + distribution(generator);
    message.data.sensor.abs_pressure = 956.025 + distribution(generator);
    message.data.sensor.diff_pressure = 0.0f;
    message.data.sensor.pressure_alt = 0;
    message.data.sensor.temperature = 0.0f + distribution(generator);

    message.data.sensor.fields_updated = 7167; 
    message.data.sensor.id = 0;

    send_message(clientConnector, message);
}

#if 0
static void send_ack(hako::px4::comm::ICommIO &clientConnector, 
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
#endif

static void *receiver_thread(void *arg)
{
    hako::px4::comm::ICommIO *clientConnector = static_cast<hako::px4::comm::ICommIO *>(arg);
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
                    case MAVLINK_MSG_TYPE_HIL_ACTUATOR_CONTROLS:
                        std::cout << "  Type: HIL_ACTUATOR_CONTROLS" << std::endl;
                        std::cout << "  Time_usec: " << message.data.hil_actuator_controls.time_usec << std::endl;
                        std::cout << "  Mode: " << static_cast<int>(message.data.hil_actuator_controls.mode) << std::endl;
                        std::cout << "  Flags: " << message.data.hil_actuator_controls.flags << std::endl;
                        std::cout << "  Controls: ";
                        for(int i = 0; i < 8; ++i) {  // Assuming there are 8 controls
                            std::cout << message.data.hil_actuator_controls.controls[i] << " ";
                        }
                        std::cout << std::endl;
                        break;
                    case MAVLINK_MSG_TYPE_HIL_SENSOR:
                        std::cout << "  Type: HIL_SENSOR" << std::endl;
                        std::cout << "  Time stamp: " << message.data.sensor.time_usec << std::endl;
                        std::cout << "  Xacc: " << message.data.sensor.xacc << std::endl;
                        std::cout << "  Yacc: " << message.data.sensor.yacc << std::endl;
                        std::cout << "  Zacc: " << message.data.sensor.zacc << std::endl;
                        std::cout << "  Xgyro: " << message.data.sensor.xgyro << std::endl;
                        std::cout << "  Ygyro: " << message.data.sensor.ygyro << std::endl;
                        std::cout << "  Zgyro: " << message.data.sensor.zgyro << std::endl;
                        std::cout << "  Xmag: " << message.data.sensor.xmag << std::endl;
                        std::cout << "  Ymag: " << message.data.sensor.ymag << std::endl;
                        std::cout << "  Zmag: " << message.data.sensor.zmag << std::endl;
                        std::cout << "  Abs_pressure: " << message.data.sensor.abs_pressure << std::endl;
                        std::cout << "  Diff_pressure: " << message.data.sensor.diff_pressure << std::endl;
                        std::cout << "  temparature: " << message.data.sensor.temperature << std::endl;
                        std::cout << "  fileds_updated: " << message.data.sensor.fields_updated << std::endl;
                        //std::cout << "  id: " << message.data.sensor.id << std::endl;
                        printf(" id: 0x%x\n", message.data.sensor.id);
                        break;
                    case MAVLINK_MSG_TYPE_SYSTEM_TIME:
                        std::cout << "  Type: SYSTEM_TIME" << std::endl;
                        std::cout << "  Unix time: " << message.data.system_time.time_unix_usec << std::endl;
                        std::cout << "  Boot time: " << message.data.system_time.time_boot_ms << std::endl;
                        break;

                    case MAVLINK_MSG_TYPE_HIL_GPS:
                        std::cout << "  Type: HIL_GPS" << std::endl;
                        std::cout << "  Time stamp: " << message.data.hil_gps.time_usec << std::endl;
                        std::cout << "  fix_type: " << message.data.hil_gps.fix_type << std::endl;
                        std::cout << "  Latitude: " << message.data.hil_gps.lat << std::endl;
                        std::cout << "  Longitude: " << message.data.hil_gps.lon << std::endl;
                        std::cout << "  alt: " << message.data.hil_gps.alt << std::endl;
                        std::cout << "  eph: " << message.data.hil_gps.eph << std::endl;
                        std::cout << "  epv: " << message.data.hil_gps.epv << std::endl;
                        std::cout << "  vn: " << message.data.hil_gps.vn << std::endl;
                        std::cout << "  vel: " << message.data.hil_gps.vel << std::endl;
                        std::cout << "  vd: " << message.data.hil_gps.vd << std::endl;
                        std::cout << "  cog: " << message.data.hil_gps.cog << std::endl;
                        std::cout << "  satelites_visible: " << message.data.hil_gps.satellites_visible << std::endl;
                        //std::cout << "  id: " << message.data.hil_gps.id << std::endl;
                        printf(" id: 0x%x\n", message.data.hil_gps.id);
                        std::cout << "  yaw: " << message.data.hil_gps.yaw << std::endl;

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
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <server_ip> <server_port>" << std::endl;
        return -1;
    }

    const char* serverIp = argv[1];
    int serverPort = std::atoi(argv[2]);

    hako::px4::comm::IcommEndpointType serverEndpoint = { serverIp, serverPort };

#if 1
    hako::px4::comm::TcpServer server;
    auto comm_io = server.server_open(&serverEndpoint);
#else
    hako::px4::comm::TcpClient client;
    auto comm_io = client.client_open(nullptr, &serverEndpoint);
#endif
    if (comm_io == nullptr) 
    {
        std::cerr << "Failed to open TCP client" << std::endl;
        return -1;
    }
    //send_command_long(*comm_io);

    pthread_t thread;
    if (pthread_create(&thread, NULL, receiver_thread, comm_io) != 0) {
        std::cerr << "Failed to create heartbeat sender thread!" << std::endl;
        return -1;
    }

    int count = 0;
    while (true) {
        auto now = std::chrono::system_clock::now();
        auto duration_since_epoch = now.time_since_epoch();
        uint64_t time_usec = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch).count();

        if ((count % 1000) == 0) {
            send_heartbeat(*comm_io);
        }
        if ((count % 4000) == 0) {
            send_system_time(*comm_io, time_usec, count);
        }
        if ((count % 1) == 0) {  // 頻度を1kHzに変更
            send_sensor(*comm_io, time_usec);
        }
        if ((count % 52) == 0) { 
            send_hil_gps(*comm_io, time_usec);
            //send_hil_state_quaternion(*comm_io, time_usec);
        }
        usleep(1 * 1000);  // 1msec
        count++;
    }

    comm_io->close();
    return 0;
}
