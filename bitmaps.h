#ifndef GRAINWAVES_BITMAPS
#define GRAINWAVES_BITMAPS

#include "daisysp.h"

static const uint8_t write_head_indicator_recording[] = {
    0,0,0,3,0,0,0,
    0,0,3,8,3,0,0,
    0,3,8,8,8,3,0,
    3,8,8,8,8,8,3
};
static const uint8_t write_head_indicator_not_recording[] = {
    0,0,0,1,0,0,0,
    0,0,1,2,1,0,0,
    0,1,2,1,2,1,0,
    1,2,1,0,1,2,1
};
const uint8_t write_head_indicator_height = 4;
const uint8_t write_head_indicator_width = 7;

#endif