// orcon_log.h — in-RAM ring buffer of RAMSES frames (RX + TX), viewable over
// the web with no USB serial.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ramses_codec.h"

namespace rlog {

static const size_t  CAP = 48;          // ring-buffer depth

// Format a frame as a ramses-style string, e.g.
//   "I --- 29:012345 32:178990 --:------ 22F1 003 000307"
// `out` should be >= 48 + 2 * MAX_PAYLOAD bytes. Returns length.
size_t format(const ramses::Frame& f, char* out, size_t cap);

// Record one frame. dir = 'R' (received) or 'T' (transmitted).
void add(char dir, const ramses::Frame& f, int rssi);

// Serialize recent entries (newest last) as a JSON array of
//   {"t":<ms>,"d":"R","s":"<frame str>","r":<rssi>}
size_t toJson(char* out, size_t cap);

} // namespace rlog
