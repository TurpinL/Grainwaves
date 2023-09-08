#ifndef GRAINWAVES_UTILS
#define GRAINWAVES_UTILS

#include "daisysp.h"

using namespace daisysp;

const float DEG_TO_TAU = PI_F * 2 / 360;

struct iVec2 {
    int x;
    int y;
};

inline float fwrap(float x, float min, float max) {
    if (max == min) return min;
    if (min > max) return fwrap(x, max, min);

    return (x >= 0 ? min : max) + fmodf(x, max - min);
}

inline int wrap(int x, int min, int max) {
    if (max == min) return min;
    if (min > max) return wrap(x, max, min);

    return (x >= 0 ? min : max) + x % (max - min);
}

inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

inline float randF(float min, float max) {
    return min + rand() * kRandFrac * (max - min);
}

inline float map_to_range(float fraction, float min, float max) {
    return min + fraction * (max - min);
}

inline float modf(float x) {
    static float junk;
    return modf(x, &junk);
}

inline iVec2 polarToCartesian(float radius, float degrees) {
    float tau = degrees * DEG_TO_TAU;
    int x = radius * cosf(tau) + 64;
    int y = radius * sinf(tau) + 64;

    return {x, y};
}

const float degs_per_pixel[] = {
    360.f, 45.f, 22.5f, 15.f, 11.25f, 9.f, 7.5f, 6.429f, 5.625f, 5.f, 4.5f, 4.091f, 3.75f, 3.462f, 3.214f, 3.f, 2.813f, 2.647f, 2.5f, 2.368f, 2.25f, 2.143f, 2.045f, 1.957f, 1.957f, 1.8f, 1.731f, 1.667f, 1.607f, 1.607f, 1.5f, 1.5f, 1.406f, 1.364f, 1.364f, 1.286f, 1.25f, 1.216f, 1.216f, 1.184f, 1.125f, 1.115f, 1.098f, 1.047f, 1.023f, 1.023f, 1.f, 0.978f, 0.938f, 0.957f, 0.9f, 0.882f, 0.882f, 0.882f, 0.849f, 0.818f, 0.865f, 0.818f, 0.789f, 0.776f, 0.75f, 0.763f, 0.776f, 0.738f, 0.703f, 0.692f, 0.703f, 0.682f, 0.672f, 0.652f, 0.692f, 0.675f, 0.652f, 0.625f, 0.625f, 0.634f, 0.608f, 0.608f, 0.577f, 0.584f, 0.6f, 0.563f, 0.563f, 0.556f, 0.542f, 0.549f, 0.536f, 0.563f, 0.529f, 0.511f, 0.511f, 0.506f, 0.506f, 0.509f, 0.495f, 0.484f, 0.474f, 0.474f, 0.469f, 0.489f, 0.484f, 0.456f, 0.469f, 0.459f, 0.459f, 0.459f, 0.446f, 0.429f, 0.441f, 0.425f, 0.429f, 0.429f, 0.425f, 0.425f, 0.421f, 0.404f, 0.409f, 0.413f, 0.398f, 0.398f, 0.388f, 0.398f, 0.385f, 0.383f, 0.381f, 0.372f, 0.375f, 0.375
};

#endif