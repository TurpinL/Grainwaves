#ifndef GRAINWAVES_UTILS
#define GRAINWAVES_UTILS

#include "daisysp.h"

using namespace daisysp;

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

#endif