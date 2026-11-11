#include "ramses_codec.h"
#include "orcon_frames.h"
#include "peers.h"
#include <cstdio>
#include <cstring>
unsigned long millis() { return 0; }
using namespace ramses; using namespace orcon;
static int failures=0;
#define CHECK(c,m) do{ if(!(c)){printf("  [FAIL] %s\n",m);failures++;} else printf("  [ ok ] %s\n",m);}while(0)
int main(){
    printf("Peer tracker tests\n");
    DeviceId fan(0x20,144220), otherFan(0x20,135704);
    Ids remote;    remote.us=DeviceId(0x1D,179256);    remote.fan=fan;
    Ids co2;       co2.us=DeviceId(0x25,28129);        co2.fan=fan;
    Ids neighbour; neighbour.us=DeviceId(0x25,26649);  neighbour.fan=otherFan;

    peers::observe(make1298(co2, 600), fan, -60, 100);
    CHECK(peers::count()==0, "self-broadcast from an unknown sender is ignored");
    peers::observe(make31E0(neighbour, 30), fan, -90, 200);
    CHECK(peers::count()==0, "neighbour talking to another fan is ignored");

    peers::observe(make22F1(remote, Mode::High), fan, -55, 300);
    Frame sensorDemand = make31E0(co2, 0);           // real capture: 00 00 7E 00 01 00 AA 00
    const uint8_t real31E0[8] = {0x00,0x00,0x7E,0x00,0x01,0x00,0xAA,0x00};
    memcpy(sensorDemand.payload, real31E0, 8);
    peers::observe(sensorDemand, fan, -62, 400);
    CHECK(peers::count()==2 && peers::dirty(), "remote + CO2 sensor addressing our fan are learned");
    peers::clearDirty();

    peers::observe(make1298(co2, 639), fan, -61, 500);
    const peers::Peer& c = peers::at(1);
    CHECK(c.count==2 && strstr(c.last,"1298 003 00027F") && c.rssi==-61 && !peers::dirty(),
          "known sender's later broadcast updates its last message");
    CHECK(c.co2==639 && c.demand==0x7E, "keeps latest CO2 and demand (first 31E0 element) from different frames");
    CHECK(peers::at(0).mode==(int8_t)Mode::High && peers::at(0).co2==-1, "remote's mode recorded, no CO2");

    Frame fromFanStatus = make1298(remote, 0); fromFanStatus.addr[0]=fan; fromFanStatus.addr[2]=fan;
    peers::observe(fromFanStatus, fan, -20, 600);
    CHECK(peers::count()==2, "the fan itself is never a peer");
    Frame fromFan = make22F1(remote, Mode::Low); fromFan.addr[0]=fan; fromFan.addr[1]=DeviceId(0x20,155896);

    Ids secondUnit; secondUnit.us=DeviceId(0x20,155896); secondUnit.fan=fan;
    Ids roomSensor; roomSensor.us=DeviceId(0x25,145268); roomSensor.fan=secondUnit.us;
    peers::observe(make31E0(roomSensor, 85), fan, -75, 650);
    CHECK(peers::count()==2, "sender to an unlinked fan-class unit is ignored");
    peers::observe(make31E0(secondUnit, 85), fan, -27, 700);
    CHECK(peers::count()==2, "linked second unit is not listed as a peer");
    peers::observe(make31E0(roomSensor, 85), fan, -75, 750);
    CHECK(peers::count()==3, "sensor bound to the linked unit is learned");

    peers::restore(nullptr, 0, 0x20);
    peers::observe(fromFan, fan, -20, 760);           // our fan forwarding to the second unit
    peers::observe(make31E0(roomSensor, 85), fan, -75, 770);
    CHECK(peers::count()==1, "fan forwarding to a unit also links it");

    for (uint32_t i = 0; i < peers::CAP; i++) {
        Ids x; x.us=DeviceId(0x25,1000+i); x.fan=fan;
        peers::observe(make31E0(x, 10), fan, -70, 1000+i);
    }
    bool remoteEvicted = true;
    for (size_t i = 0; i < peers::count(); i++) if (peers::at(i).id==remote.us) remoteEvicted = false;
    CHECK(peers::count()==peers::CAP && remoteEvicted, "when full, the longest-silent peer is replaced");

    uint32_t raw[2] = { remote.us.raw(), co2.us.raw() };
    peers::restore(raw, 2, 0x20);
    CHECK(peers::count()==2 && peers::at(0).id==remote.us && peers::at(0).heardMs==0 && !peers::dirty(),
          "restore brings back ids as not-heard-yet");
    uint32_t stale[2] = { secondUnit.us.raw(), remote.us.raw() };
    peers::restore(stale, 2, 0x20);
    CHECK(peers::count()==1 && peers::at(0).id==remote.us && peers::dirty(),
          "restore drops a stored fan-class id and asks to re-save");

    const char* v=failures?"FAILED":"ALL PASSED"; const char* pl=failures==1?"":"s";
    printf("\n%s (%d failure%s)\n",v,failures,pl);
    return failures?1:0;
}
