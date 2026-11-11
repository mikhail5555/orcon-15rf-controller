// peers.h — the other devices of this household: every sender that addresses
// our fan or a unit linked to it (remotes, CO2 sensors) plus anything those senders
// broadcast afterwards (e.g. a CO2 sensor's self-addressed 1298). Keeps the
// last frame of each so the dashboard can show what they said.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ramses_codec.h"

namespace peers {

static const size_t CAP = 8;

struct Peer {
    ramses::DeviceId id;
    uint32_t heardMs = 0;                    // 0 = known (from NVS) but not heard since boot
    int16_t  rssi    = 0;
    uint32_t count   = 0;
    char     last[48 + 2 * ramses::MAX_PAYLOAD] = "";
    // Latest reading of each kind this device reported; -1 = never.
    int32_t  co2      = -1;   // ppm (1298)
    int16_t  demand   = -1;   // 31E0 first element, 0..200 = 0..100 %
    int8_t   mode     = -1;   // 22F1 mode index (see orcon::Mode)
    int8_t   humidity = -1;   // % (12A0)
};

// Feed every received frame. A sender becomes a peer once it addresses `fan`
// or a unit linked to it (a second fan-class unit). Fan-class units themselves
// are never peers. When full, the longest-silent peer is replaced.
void observe(const ramses::Frame& f, const ramses::DeviceId& fan, int rssi, uint32_t nowMs);

size_t      count();
const Peer& at(size_t i);

// The id list changed (a peer was added/replaced) -> caller should persist it.
bool dirty();
void clearDirty();

// Restore ids persisted earlier (raw() form); also used to forget all (n = 0).
// Linked units are forgotten too; they are relearned from live traffic.
// Ids of `fanClass` are dropped (and dirty() set so the cleaned list is saved).
void restore(const uint32_t* rawIds, size_t n, uint8_t fanClass);

} // namespace peers
