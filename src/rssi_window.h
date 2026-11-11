// rssi_window.h — attribute RSSI to a received frame from timestamped samples.
// Pure (no Arduino) so it can be host-tested.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace rssiwin {

struct Sample {
    uint32_t ms;
    int16_t  dbm;
};

// RSSI of a frame that was on air during [t0, t1] (ms), given a ring of samples.
// Median of the samples inside the window; when the sampler was starved (< 3
// samples inside), the max over [t1 - fallbackSpan, t1]; -127 when nothing at all.
int16_t frameRssi(const Sample* ring, size_t cap, size_t head, size_t count,
                  uint32_t t0, uint32_t t1, uint32_t fallbackSpan);

} // namespace rssiwin
