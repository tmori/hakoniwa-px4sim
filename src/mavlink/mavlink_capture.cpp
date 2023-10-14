#include "mavlink_capture.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <cstring>
#include <iostream>
#include <vector>

bool mavlink_capture_create_controller(MavlinkCaptureControllerType &controller, const char* filepath) {
    controller.start_time = 0;
    controller.packet_num = 0;
    controller.total_size = 0;
    controller.memsize = MAVLINK_CAPTURE_INC_DATA_SIZE;
    controller.offset = 0;

    // Allocate initial memory for data cache
    controller.data = (uint8_t*) malloc(controller.memsize);
    if (controller.data == nullptr) {
        std::cerr << "Initial memory allocation failed." << std::endl;
        return false;
    }

    // Open the file for writing
    controller.save_file = open(filepath, O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR);
    if (controller.save_file == -1) {
        std::cerr << "Failed to open capture file for writing." << std::endl;
        free(controller.data);
        controller.data = nullptr;
        return false;
    }

    return true;
}

bool mavlink_capture_append_data(MavlinkCaptureControllerType &controller, uint32_t dataLength, const uint8_t *data) {
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    uint64_t time_usec = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch).count();
    if (controller.packet_num == 0) {
        controller.start_time = time_usec;
    }
    MavlinkCaptureDataType packet;
    packet.dataLength = dataLength;
    packet.relativeTimestamp = time_usec - controller.start_time;
    uint64_t packet_size = sizeof(packet.dataLength) + sizeof(packet.relativeTimestamp) + dataLength;

    if (controller.offset + packet_size > controller.memsize) {
        controller.memsize += MAVLINK_CAPTURE_INC_DATA_SIZE;
        uint8_t* newData = (uint8_t*) realloc(controller.data, controller.memsize);
        if (newData == nullptr) {
            std::cerr << "Memory allocation failed." << std::endl;
            return false;
        }
        controller.data = newData;
    }

    // Copy dataLength and data to cache
    memcpy(controller.data + controller.offset, &packet.dataLength, sizeof(packet.dataLength));
    controller.offset += sizeof(packet.dataLength);
    controller.total_size += sizeof(packet.dataLength);
    memcpy(controller.data + controller.offset, &packet.relativeTimestamp, sizeof(packet.relativeTimestamp));
    controller.offset += sizeof(packet.relativeTimestamp);
    controller.total_size += sizeof(packet.relativeTimestamp);
    memcpy(controller.data + controller.offset, data, dataLength);
    controller.offset += dataLength;
    controller.total_size += dataLength;

    controller.packet_num++;
    return true;
}

bool mavlink_capture_save(MavlinkCaptureControllerType &controller) {
    if (controller.save_file == -1 || controller.data == nullptr) {
        std::cerr << "Invalid file descriptor or data." << std::endl;
        return false;
    }

    // Write start_time, packet_num, and total_size to the start of the file
    lseek(controller.save_file, 0, SEEK_SET);
    write(controller.save_file, &controller.start_time, sizeof(controller.start_time));
    write(controller.save_file, &controller.packet_num, sizeof(controller.packet_num));
    write(controller.save_file, &controller.total_size, sizeof(controller.total_size));

    // Write cached data to the file
    ssize_t ret = write(controller.save_file, controller.data, controller.total_size);
    if (ret != (ssize_t)controller.total_size) {
        std::cerr << "Can not write file." << std::endl;
        return false;
    }

    // Close the file and free memory
    close(controller.save_file);
    free(controller.data);
    controller.data = nullptr;

    return true;
}
