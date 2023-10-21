#ifndef _HAKO_PX4_RUNNER_HPP_
#define _HAKO_PX4_RUNNER_HPP_

#include "hako_asset_runner.h"

typedef struct {
    char* asset_name;
    char* robo_name;
    char* config_path;
    int delta_time_msec;
} HakoPx4RunnerArgType;

extern void *hako_px4_runner(void *argp);
extern hako_time_t hako_get_current_time_usec();

#endif /* _HAKO_PX4_RUNNER_HPP_ */