#include "Arduino.h"
Monitor Serial;
#include "calibration/Ibus_Calibrate.hpp"
#include <cstdlib>
#include <iostream>

static unsigned assertions=0;
#define CHECK(x) do { ++assertions; if (!(x)) { std::cerr << "FAIL " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)
static std::vector<std::uint8_t> packet(int raw) {
    auto a=static_cast<std::uint8_t>(raw),b=static_cast<std::uint8_t>(raw>>8);
    auto crc=static_cast<std::uint16_t>(65535-6-0x40-a-b);
    return {6,0x40,a,b,static_cast<std::uint8_t>(crc),static_cast<std::uint8_t>(crc>>8)};
}
static void run(HardwareSerial &uart, unsigned expected) {
    clock_ms()=0; Serial=Monitor{}; Serial.stop=5000+static_cast<std::uint32_t>(uart.packets.size())*2+200;
    rc::calibrate::run_ibus(uart,18,-1,115200,1);
    auto at=Serial.output.find(" 0 |"); CHECK(at!=std::string::npos);
    int span=0; unsigned clusters=0; unsigned long samples=0;
    CHECK(std::sscanf(Serial.output.c_str()+at," 0 | %d | %u | %lu",&span,&clusters,&samples)==3);
    if(samples!=expected) std::cerr << "packets=" << uart.packets.size() << " samples=" << samples << " expected=" << expected << '\n';
    CHECK(samples==expected);
}
int main() {
    HardwareSerial one; one.packets={packet(1500)}; run(one,1);
    HardwareSerial none; run(none,0);
    HardwareSerial repeated; repeated.packets={packet(1000),packet(1500),packet(2000)}; run(repeated,3);
    HardwareSerial corrupt; auto bad=packet(1500); bad.back()^=1;
    corrupt.packets={packet(1000),bad,{6,0x40,1},packet(2000),packet(1500)};
    // Sliding recovery retains all three complete valid packets despite corruption/truncation.
    run(corrupt,3);
    HardwareSerial many; many.packets.assign(65540,packet(1000));
    for(int i=0;i<8;++i) many.packets.push_back(packet(2000));
    run(many,65548); CHECK(Serial.output.find("Switch - 2 pos")!=std::string::npos);
    // Histogram/cluster saturation must retain occupied bins, not wrap to zero.
    using namespace rc::calibrate;
    Stats s{}; s.minv[0]=1000; s.maxv[0]=2000;
    for(int b=bin_index(1000);b<=bin_index(2000);++b) s.hist[0][b]=3;
    int levels[]={1000,2000}; CHECK(classify_channel(s,0,1000,levels,2)==ChClass::Axis);
    s=Stats{}; CHECK(classify_channel(s,0,1000,levels,2)==ChClass::Switch2);
    CHECK(classify_channel(s,0,1000,levels,3)==ChClass::Switch3);
    CHECK(classify_channel(s,0,1000,levels,4)==ChClass::SwitchN);
    CHECK(classify_channel(s,0,0,levels,2)==ChClass::NoInput);
    CHECK(classify_channel(s,0,1300,levels,4)==ChClass::Axis);
    CHECK(estimate_axis_center(s,0,1000,2000)==1500);
    // Keep the compatibility helper's existing classification behaviour covered.
    CHECK(snap_2_or_3pos_if_common(levels,2)==ChClass::Switch2);
    std::cout << "PASS calibration: " << assertions << " assertions\n";
}
