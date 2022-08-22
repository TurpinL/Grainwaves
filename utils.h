#ifndef GRAINWAVES_UTILS
#define GRAINWAVES_UTILS

#include "daisysp.h"

using namespace daisysp;

float fwrap(float x, float min, float max) {
    if (max == min) return min;
    if (min > max) return fwrap(x, max, min);

    return (x >= 0 ? min : max) + fmodf(x, max - min);
}

int wrap(int x, int min, int max) {
    if (max == min) return min;
    if (min > max) return wrap(x, max, min);

    return (x >= 0 ? min : max) + x % (max - min);
}

float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

float randF(float min, float max) {
    return min + rand() * kRandFrac * (max - min);
}

float map_to_range(float fraction, float min, float max) {
    return min + fraction * (max - min);
}

float modf(float x) {
    static float junk;
    return modf(x, &junk);
}

#endif