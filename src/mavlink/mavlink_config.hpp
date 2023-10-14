#ifndef _MAVLINK_CONFIG_HPP_
#define _MAVLINK_CONFIG_HPP_

#define MAVLINK_CONFIG_CHAN_0           0

#define SIM_FOR_PX4 1
#if SIM_FOR_PX4 /* for px4 */
#define MAVLINK_CONFIG_SYSTEM_ID        0x1
#define MAVLINK_CONFIG_COMPONENT_ID     0x33
#else /* for jmavlink */
#define MAVLINK_CONFIG_SYSTEM_ID        0x0
#define MAVLINK_CONFIG_COMPONENT_ID     0x0
#endif

#define MAVLINK_CONFIG_VERSION          3

#endif /* _MAVLINK_CONFIG_HPP_ */