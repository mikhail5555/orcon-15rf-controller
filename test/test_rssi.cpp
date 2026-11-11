#include "rssi_window.h"
#include <cstdio>
using namespace rssiwin;
static int failures=0;
#define CHECK(c,m) do{ if(!(c)){printf("  [FAIL] %s\n",m);failures++;} else printf("  [ ok ] %s\n",m);}while(0)

static const size_t CAP = 16;
static Sample ring[CAP];
static size_t head = 0, count = 0;
static void push(uint32_t ms, int16_t dbm) {
    ring[head] = { ms, dbm }; head = (head + 1) % CAP; if (count < CAP) count++;
}
static void reset() { head = 0; count = 0; }

int main(){
    printf("RSSI window tests\n");

    // noise, a strong neighbour burst, noise, then our weak frame (ms 110..118)
    reset();
    for (uint32_t t = 90; t < 95; t++) push(t, -105);
    for (uint32_t t = 95; t < 100; t++) push(t, -30);
    for (uint32_t t = 100; t < 110; t++) push(t, -104);
    // ring holds 16: ms 100..115 after this
    for (uint32_t t = 110; t < 116; t++) push(t, (t == 111) ? -70 : -55);
    CHECK(frameRssi(ring, CAP, head, count, 109, 115, 10) == -55,
          "median of in-frame samples; earlier burst and one dip don't leak in");

    reset();
    for (uint32_t t = 200; t < 210; t++) push(t, -100);
    push(230, -60);                                   // sampler starved during the frame
    CHECK(frameRssi(ring, CAP, head, count, 225, 232, 12) == -60,
          "starved sampler falls back to max near the frame end");

    reset();
    push(10, -90);
    CHECK(frameRssi(ring, CAP, head, count, 500, 510, 5) == -127, "no samples at all -> -127");

    reset();
    uint32_t base = 0xFFFFFFF0u;                      // millis() wrap-around
    for (uint32_t k = 0; k < 8; k++) push(base + k * 4, -48);
    CHECK(frameRssi(ring, CAP, head, count, base + 2, base + 30, 10) == -48, "works across millis() wrap");

    const char* v=failures?"FAILED":"ALL PASSED"; const char* pl=failures==1?"":"s";
    printf("\n%s (%d failure%s)\n",v,failures,pl);
    return failures?1:0;
}
