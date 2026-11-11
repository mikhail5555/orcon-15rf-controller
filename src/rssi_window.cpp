// rssi_window.cpp — see rssi_window.h
#include "rssi_window.h"

namespace rssiwin {

static const size_t MAX_IN_WINDOW = 64;

static bool inWindow(uint32_t ms, uint32_t t0, uint32_t t1) {
    return (int32_t)(ms - t0) >= 0 && (int32_t)(t1 - ms) >= 0;
}

int16_t frameRssi(const Sample* ring, size_t cap, size_t head, size_t count,
                  uint32_t t0, uint32_t t1, uint32_t fallbackSpan) {
    int16_t inside[MAX_IN_WINDOW];
    size_t n = 0;
    int16_t fallbackMax = -127;
    bool haveFallback = false;
    size_t oldest = (head + cap - count) % cap;
    for (size_t k = 0; k < count; k++) {
        const Sample& s = ring[(oldest + k) % cap];
        if (inWindow(s.ms, t0, t1) && n < MAX_IN_WINDOW) inside[n++] = s.dbm;
        if (inWindow(s.ms, t1 - fallbackSpan, t1) && (!haveFallback || s.dbm > fallbackMax)) {
            fallbackMax = s.dbm;
            haveFallback = true;
        }
    }
    if (n < 3) return fallbackMax;
    for (size_t i = 1; i < n; i++) {            // insertion sort; n is small
        int16_t v = inside[i];
        size_t j = i;
        while (j > 0 && inside[j - 1] > v) { inside[j] = inside[j - 1]; j--; }
        inside[j] = v;
    }
    return inside[n / 2];
}

} // namespace rssiwin
