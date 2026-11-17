#include "Arduino.h"
Monitor Serial;
#define ARDUINO_ARCH_ESP32 1
#define RCLINK_NO_LEGACY_GLOBAL_ALIASES
#include "RCLink.h"
#include "calibration/Ibus_Calibrate.hpp"
#include "calibration/Sbus_Calibrate.hpp"
#include "RcMacros.hpp"
#include "Config.hpp"
#include "Link.hpp"
#define CAL_ROLES(X) X(Ch1) X(Ch2) X(Ch3) X(Ch4) X(Ch5) X(Ch6) X(Ch7) X(Ch8) X(Ch9) X(Ch10) X(Ch11) X(Ch12) X(Ch13) X(Ch14) X(Ch15) X(Ch16)
RC_DECLARE_ROLES(CalRole, CAL_ROLES)
#include "Json.hpp"
#include <cstdlib>
#include <iostream>
#include <type_traits>

namespace caldetail = rc::calibrate::detail;
static unsigned assertions=0;
#define CHECK(x) do { ++assertions; if (!(x)) { std::cerr << "FAIL " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)

static std::vector<std::uint8_t> sbus(unsigned native, unsigned flags=0) {
    std::vector<std::uint8_t> f(25,0); f[0]=15; f[23]=static_cast<std::uint8_t>(flags);
    for(unsigned ch=0;ch<16;++ch) for(unsigned b=0;b<11;++b) {
        const unsigned bit=ch*11+b;
        if(native & (1u<<b)) f[1+bit/8] |= static_cast<std::uint8_t>(1u<<(bit%8));
    }
    return f;
}
static std::vector<std::uint8_t> ibus(unsigned raw) {
    std::vector<std::uint8_t> f(32,0); f[0]=32; f[1]=64;
    for(unsigned ch=0;ch<14;++ch) { f[2+ch*2]=static_cast<std::uint8_t>(raw); f[3+ch*2]=static_cast<std::uint8_t>(raw>>8); }
    unsigned sum=65535; for(unsigned i=0;i<30;++i) sum-=f[i];
    f[30]=static_cast<std::uint8_t>(sum); f[31]=static_cast<std::uint8_t>(sum>>8); return f;
}
static void session(HardwareSerial &uart, bool sbusMode, unsigned limit=255, std::uint32_t start=0, int txSpace=32) {
    clock_ms()=start; uart.next=start+5000; Serial=Monitor{}; Serial.capacity=txSpace;
    Serial.stop=uart.next+static_cast<std::uint32_t>(uart.packets.size())*uart.step+300;
    if(sbusMode) rc::calibrate::run_sbus(uart,18,-1,100000,static_cast<std::uint8_t>(limit));
    else rc::calibrate::run_ibus(uart,18,-1,115200,static_cast<std::uint8_t>(limit));
    CHECK(Serial.maxWrite<=32);
    CHECK(Serial.output.find("Press ANY key to BEGIN sampling. Press ANY key again to STOP.")!=std::string::npos);
    CHECK(Serial.output.find("centre")!=std::string::npos);
}
static unsigned long samples(int humanChannel, int expectedLo, int expectedHi) {
    const auto marker="CH"+std::to_string(humanChannel)+" |";
    auto at=Serial.output.find(marker); CHECK(at!=std::string::npos);
    int ch=0,lo=0,hi=0,span=0; unsigned clusters=0; unsigned long n=0;
    CHECK(std::sscanf(Serial.output.c_str()+at,"CH%d | %d | %d | %d | %u | %lu",&ch,&lo,&hi,&span,&clusters,&n)==6);
    CHECK(ch==humanChannel); CHECK(lo==expectedLo); CHECK(hi==expectedHi); CHECK(span==hi-lo);
    return n;
}
static JsonDocument json() {
    const auto heading=Serial.output.find("JSON suggestion (optional)"); CHECK(heading!=std::string::npos);
    const auto first=Serial.output.find('{',heading), last=Serial.output.find("\n}\n",first);
    CHECK(first!=std::string::npos && last!=std::string::npos);
    JsonDocument doc; const auto error=deserializeJson(doc,Serial.output.substr(first,last-first+2));
    if(error) std::cerr<<error.c_str()<<"\n"<<Serial.output.substr(first,last-first+2)<<"\n";
    CHECK(!error); return doc;
}
// Exercise the real beginner setup() functions as well as the helper API.
HardwareSerial Serial2;
namespace ibusExample {
#include "../../examples/02_IbusCalibration/02_IbusCalibration.ino"
}
namespace sbusExample {
#include "../../examples/03_SbusCalibration/03_SbusCalibration.ino"
}
int main() {
    using namespace rc::calibrate;
    static_assert(Stats::K==14,"preserve public iBUS capacity");
    static_assert(caldetail::CalibrationStats<16>::K==16,"all SBUS analogue channels");
    static_assert(std::is_trivially_destructible<Stats>::value,"fixed statistics storage");
    HardwareSerial none; session(none,true); CHECK(samples(16,0,0)==0);
    CHECK(Serial.output.find("No valid calibration samples")!=std::string::npos); CHECK(json()["map"].size()==0);
    CHECK(none.configuredBaud==100000 && none.configuredFraming==SERIAL_8E2 && none.configuredInverted && none.configuredTx==-1 && none.configuredRx==18);
    HardwareSerial partial; auto truncated=sbus(0); truncated.resize(7); partial.packets={truncated};
    session(partial,true); CHECK(samples(1,0,0)==0);
    HardwareSerial fault;
    auto batch=sbus(0,4), healthy=sbus(1811); batch.insert(batch.end(),healthy.begin(),healthy.end());
    fault.packets={sbus(992),sbus(0,4),sbus(2047,8),batch,sbus(172)};
    session(fault,true); CHECK(samples(1,1000,1500)==2); CHECK(samples(16,1000,1500)==2);
    CHECK(Serial.output.find("2 accepted lost-flag frames, 1 failsafe-flag frames")!=std::string::npos);
    CHECK(json()["map"].size()==0); // Sparse samples do not invent a classification.
    HardwareSerial recover; auto bad=sbus(0); bad.back()=0x99;
    recover.step=20; recover.packets={bad,truncated,sbus(172),sbus(1811)};
    session(recover,true); CHECK(samples(16,1000,2000)==2);
    HardwareSerial switches;
    for(int i=0;i<10;++i) switches.packets.push_back(sbus(240));
    for(int i=0;i<10;++i) switches.packets.push_back(sbus(1807));
    session(switches,true); CHECK(samples(16,1041,1997)==20);
    CHECK(Serial.output.find("CH16 (Switch - 2 pos)")!=std::string::npos);
    CHECK(Serial.output.find("raw_levels({1041, 1997})")!=std::string::npos);
    auto doc=json(); CHECK(doc["map"].size()==16); CHECK(doc["map"]["Ch16"]==15);
    CHECK(doc["switches"]["Ch16"]["raw_levels"][0]==1041);
    std::string configText; serializeJson(doc,configText); rc::RcConfig<CalRole> cfg; CHECK(rc::load_json(CalRole{},cfg,configText.c_str())); CHECK(rc::validate_config(cfg));
    // Equivalent protocol words lead to equivalent JSON, without protocol normalization.
    HardwareSerial isw,ssw;
    for(int raw : {1000,2000}) for(int i=0;i<10;++i) isw.packets.push_back(ibus(static_cast<unsigned>(raw)));
    for(int native : {172,1811}) for(int i=0;i<10;++i) ssw.packets.push_back(sbus(static_cast<unsigned>(native)));
    session(isw,false); CHECK(samples(14,1000,2000)==20); auto idoc=json(); CHECK(idoc["map"].size()==14);
    CHECK(Serial.output.find("CH15 | ")==std::string::npos);
    session(ssw,true,14); CHECK(samples(14,1000,2000)==20); auto sdoc=json(); CHECK(sdoc["map"].size()==14);
    std::string ij,sj; serializeJson(idoc,ij); serializeJson(sdoc,sj); CHECK(ij==sj);
    HardwareSerial ambiguous;
    for(int i=0;i<10;++i) ambiguous.packets.push_back(sbus(172));
    for(int i=0;i<10;++i) ambiguous.packets.push_back(sbus(517));
    for(int i=0;i<20;++i) ambiguous.packets.push_back(sbus(369));
    session(ambiguous,true);
    auto ambiguousDoc=json();
    CHECK(ambiguousDoc["map"].size()==0);
    CHECK(ambiguousDoc["switches"].size()==0);
    CHECK(Serial.output.find("raw_levels({")==std::string::npos);
    HardwareSerial ambiguousIbus;
    for(unsigned raw : {1000u,1210u}) for(int i=0;i<10;++i) ambiguousIbus.packets.push_back(ibus(raw));
    for(int i=0;i<20;++i) ambiguousIbus.packets.push_back(ibus(1120));
    session(ambiguousIbus,false); CHECK(json()["map"].size()==0);
    CHECK(Serial.output.find("ambiguous switch spacing")!=std::string::npos);
    HardwareSerial axis;
    for(unsigned n=240;n<=1807;n+=20) for(int i=0;i<3;++i) axis.packets.push_back(sbus(n));
    for(int i=0;i<40;++i) axis.packets.push_back(sbus(1024));
    session(axis,true,255,0xffffec00u,0); // Rollover and blocked TX must not stop acquisition.
    CHECK(samples(16,1041,1993)==axis.packets.size()); CHECK(Serial.writeCalls==0);
    auto adoc=json(); CHECK(adoc["axes"].size()==16); CHECK(adoc["map"]["Ch16"]==15);
    std::string axisText; serializeJson(adoc,axisText); rc::RcConfig<CalRole> acfg; CHECK(rc::load_json(CalRole{},acfg,axisText.c_str())); CHECK(rc::validate_config(acfg));
    CHECK(Serial.output.find("not measured mechanical centres")!=std::string::npos);
    // Repeated sessions use fresh state; no old extrema/JSON survive.
    HardwareSerial again; session(again,true); CHECK(samples(16,0,0)==0); CHECK(json()["axes"].size()==0);
    caldetail::CalibrationStats<16> stats{}; stats.minv[15]=32767; stats.maxv[15]=-32768; stats.last[15]=-99999;
    stats.samples[15]=UINT32_MAX; stats.hist[15][bin_index(1500)]=UINT16_MAX;
    stats.lcnt[15]=1; stats.levels[15][0]=1500; stats.occ[15][0]=UINT16_MAX;
    caldetail::add_sample(stats,15,1500,8);
    CHECK(stats.samples[15]==UINT32_MAX); CHECK(stats.hist[15][bin_index(1500)]==UINT16_MAX); CHECK(stats.occ[15][0]==UINT16_MAX);
    caldetail::add_sample(stats,-1,800,8); caldetail::add_sample(stats,16,800,8); CHECK(stats.samples[0]==0);
    CHECK(caldetail::classify_channel(stats,-1,1000,nullptr,0)==ChClass::NoInput);
    CHECK(!caldetail::is_axis_by_hist(stats,16,1000));
    auto legacyCenter = &estimate_axis_center;
    auto legacyClassify = &classify_channel;
    auto legacyAxis = &is_axis_by_hist;
    (void)legacyClassify; (void)legacyAxis;
    Stats narrow{}; narrow.hist[0][bin_index(1530)]=20;
    const int center=legacyCenter(narrow,0,1500,1540); CHECK(center==1520);
    CHECK(caldetail::suggested_deadband(19,20,true,1500,1540,center)==19);
    rc::RcConfig<CalRole> narrowcfg;
    for(int i=0;i<16;++i) { auto role=static_cast<CalRole>(i); narrowcfg.map(role,static_cast<std::uint8_t>(i)).axis(role).raw(1500,1540,static_cast<std::int16_t>(center)).deadband_us(19).done(); }
    CHECK(rc::validate_config(narrowcfg));
    for(bool sbusMode : {false,true}) {
        Serial2=HardwareSerial{}; Serial2.next=5050;
        for(unsigned value : {0u,1u}) for(int i=0;i<10;++i)
            Serial2.packets.push_back(sbusMode?sbus(value?1811:172):ibus(value?2000:1000));
        clock_ms()=0; Serial=Monitor{}; Serial.stop=5500;
        if(sbusMode) sbusExample::setup(); else ibusExample::setup();
        CHECK(Serial.baud==115200);
        CHECK(samples(sbusMode?16:14,1000,2000)==20);
        CHECK(json()["map"].size()==static_cast<std::size_t>(sbusMode?16:14));
    }
    std::cout<<"PASS shared/SBUS calibration: "<<assertions<<" assertions; stats14="<<sizeof(Stats)<<" stats16="<<sizeof(stats)<<"\n";
}
