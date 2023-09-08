#ifndef GRAINWAVES_GRAIN
#define GRAINWAVES_GRAIN

struct Grain {
    int length = 0;
    int spawn_position_index = 0;
    int spawn_position = 0;
    int step = 0;
    float pan = 0; // 0 is left, 1 is right.
    float playback_speed = 0;
    float pitch_shift_in_octaves = 0;
};

inline bool is_alive(Grain grain) {
    return grain.step <= grain.length;
}

#endif