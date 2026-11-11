// peers.cpp — see peers.h
#include "peers.h"
#include "orcon_log.h"

namespace peers {

static Peer   g_peers[CAP];
static size_t g_count = 0;
static bool   g_dirty = false;

// Fan-class units linked to our fan (e.g. a second unit that sensors in other
// rooms are bound to). Not shown as peers; only widen who counts as household.
static const size_t UNIT_CAP = 4;
static ramses::DeviceId g_units[UNIT_CAP];
static size_t g_unitCount = 0;

static Peer* find(const ramses::DeviceId& id) {
    for (size_t i = 0; i < g_count; i++) if (g_peers[i].id == id) return &g_peers[i];
    return nullptr;
}

static bool isLinkedUnit(const ramses::DeviceId& d) {
    for (size_t i = 0; i < g_unitCount; i++) if (g_units[i] == d) return true;
    return false;
}

static void linkUnit(const ramses::DeviceId& d) {
    if (isLinkedUnit(d)) return;
    if (g_unitCount < UNIT_CAP) g_units[g_unitCount++] = d;
}

static bool isHomeUnit(const ramses::DeviceId& d, const ramses::DeviceId& fan) {
    return d == fan || isLinkedUnit(d);
}

static bool addresses(const ramses::Frame& f, const ramses::DeviceId& d) {
    return (f.has_addr[1] && f.addr[1] == d) || (f.has_addr[2] && f.addr[2] == d);
}

static bool addressesHome(const ramses::Frame& f, const ramses::DeviceId& fan) {
    return (f.has_addr[1] && isHomeUnit(f.addr[1], fan)) ||
           (f.has_addr[2] && isHomeUnit(f.addr[2], fan));
}

// Our fan forwarding to a unit, or a unit reporting to our fan, links them.
static void learnUnits(const ramses::Frame& f, const ramses::DeviceId& fan) {
    if (!f.has_addr[0]) return;
    const ramses::DeviceId& sender = f.addr[0];
    if (sender == fan) {
        for (int i = 1; i < 3; i++)
            if (f.has_addr[i] && f.addr[i].cls == fan.cls && !(f.addr[i] == fan)) linkUnit(f.addr[i]);
    } else if (sender.cls == fan.cls && addresses(f, fan)) {
        linkUnit(sender);
    }
}

static void recordReading(Peer& p, const ramses::Frame& f) {
    const uint8_t* b = f.payload;
    switch (f.cmd) {
        case 0x1298:
            if (f.len >= 3) { uint16_t v = (uint16_t)((b[1] << 8) | b[2]); if (v != 0x7FFF) p.co2 = v; }
            break;
        case 0x31E0:   // 4-byte elements [idx][flags][demand 0..200][?]
            if (f.len >= 3 && b[2] <= 200) p.demand = b[2];
            break;
        case 0x22F1:
            if (f.len >= 2 && b[1] <= 7) p.mode = (int8_t)b[1];
            break;
        case 0x12A0:
            if (f.len >= 2 && b[1] <= 100) p.humidity = (int8_t)b[1];
            break;
        default: break;
    }
}

static Peer* add(const ramses::DeviceId& id) {
    size_t slot = g_count;
    if (g_count < CAP) {
        g_count++;
    } else {
        slot = 0;
        for (size_t i = 1; i < CAP; i++)
            if (g_peers[i].heardMs < g_peers[slot].heardMs) slot = i;
    }
    g_peers[slot] = Peer();
    g_peers[slot].id = id;
    g_dirty = true;
    return &g_peers[slot];
}

void observe(const ramses::Frame& f, const ramses::DeviceId& fan, int rssi, uint32_t nowMs) {
    learnUnits(f, fan);
    if (!f.has_addr[0]) return;
    const ramses::DeviceId& sender = f.addr[0];
    if (sender.cls == fan.cls) return;
    Peer* p = find(sender);
    if (!p) {
        if (!addressesHome(f, fan)) return;
        p = add(sender);
    }
    p->heardMs = nowMs ? nowMs : 1;
    p->rssi = (int16_t)rssi;
    p->count++;
    rlog::format(f, p->last, sizeof(p->last));
    if (f.type != ramses::MsgType::RQ) recordReading(*p, f);
}

size_t      count()        { return g_count; }
const Peer& at(size_t i)   { return g_peers[i]; }
bool        dirty()        { return g_dirty; }
void        clearDirty()   { g_dirty = false; }

void restore(const uint32_t* rawIds, size_t n, uint8_t fanClass) {
    g_count = 0;
    g_unitCount = 0;
    g_dirty = false;
    for (size_t i = 0; i < n && g_count < CAP; i++) {
        ramses::DeviceId id = ramses::DeviceId::fromRaw(rawIds[i]);
        if (id.cls == fanClass) { g_dirty = true; continue; }   // stored by an older build
        g_peers[g_count] = Peer();
        g_peers[g_count].id = id;
        g_count++;
    }
}

} // namespace peers
