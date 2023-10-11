#ifndef _MAVLINK_DECODER_HPP_
#define _MAVLINK_DECODER_HPP_

#include <mavlink.h>
#include "mavlink_config.hpp"

extern bool mavlink_decode(const char* packet, int packet_len, mavlink_message_t *msg);

#endif /* _MAVLINK_DECODER_HPP_ */
