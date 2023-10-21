#include "comm/tcp_connector.hpp"
#include "mavlink/mavlink_capture_replay.hpp"
#include "threads/px4sim_thread_receiver.hpp"
#include "threads/px4sim_thread_sender.hpp"
#include "threads/px4sim_thread_replay.hpp"
#include "threads/px4sim_thread_capture.hpp"
#include "hako/runner/hako_px4_runner.hpp"
#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <unistd.h>
#include <chrono>

typedef enum {
    REPLAY = 0,
    REPLAY_DUMP,
    CAPTURE,
    NORMAL
} ModeType;

int main(int argc, char* argv[]) 
{
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << "<server_ip> <server_port> <mode={replay|replay_dump|capture|normal}> "  << std::endl;
        return -1;
    }
    ModeType mode = NORMAL;
    const char* serverIp = argv[1];
    int serverPort = std::atoi(argv[2]);
    const char* arg_mode = argv[3];

    hako::px4::comm::IcommEndpointType serverEndpoint = { serverIp, serverPort };

    hako::px4::comm::ICommIO *comm_io  = nullptr;
    if (strcmp("replay_dump", arg_mode) == 0) {
        comm_io = nullptr;
        mode = REPLAY_DUMP;
    }
    else if (strcmp("replay", arg_mode) == 0) {
        hako::px4::comm::TcpServer server;
        comm_io = server.server_open(&serverEndpoint);
        if (comm_io == nullptr) 
        {
            std::cerr << "Failed to open TCP client" << std::endl;
            return -1;
        }
        mode = REPLAY;
    }
    else if (strcmp("capture", arg_mode) == 0) {
        hako::px4::comm::TcpClient client;
        comm_io = client.client_open(nullptr, &serverEndpoint);
        if (comm_io == nullptr) 
        {
            std::cerr << "Failed to open TCP client" << std::endl;
            return -1;
        }
        mode = CAPTURE;
    }
    else {
        hako::px4::comm::TcpClient client;
        comm_io = client.client_open(nullptr, &serverEndpoint);
        if (comm_io == nullptr) 
        {
            std::cerr << "Failed to open TCP client" << std::endl;
            return -1;
        }
        mode = NORMAL;
    }

    if (mode == REPLAY) {
        pthread_t thread_1;
        if (pthread_create(&thread_1, NULL, px4sim_thread_receiver, comm_io) != 0) {
            std::cerr << "Failed to create receiver thread!" << std::endl;
            return -1;
        }
        px4sim_thread_replay(comm_io);
    }
    else if (mode == REPLAY_DUMP) {
        px4sim_thread_replay_dump(nullptr);
    }
    else if (mode == CAPTURE) {
        px4sim_send_dummy_command_long(*comm_io);
        px4sim_send_dummy_heartbeat(*comm_io);
        px4sim_thread_capture(comm_io);
    }
    else if (mode == NORMAL) {
        pthread_t thread_1;
        px4sim_send_dummy_command_long(*comm_io);
        if (pthread_create(&thread_1, NULL, px4sim_thread_receiver, comm_io) != 0) {
            std::cerr << "Failed to create receiver thread!" << std::endl;
            return -1;
        }
        if (pthread_create(&thread_1, NULL, px4sim_thread_sender, comm_io) != 0) {
            std::cerr << "Failed to create sender thread!" << std::endl;
            return -1;
        }
        HakoPx4RunnerArgType arg;
        arg.asset_name = "px4sim";
        arg.config_path = "./custom.json";
        arg.delta_time_msec = 10;
        arg.robo_name = "drone";
        hako_px4_runner(&arg);
    }

    comm_io->close();
    return 0;
}
