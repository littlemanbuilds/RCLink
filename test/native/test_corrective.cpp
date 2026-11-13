#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>
#include "Link.hpp"
#include "transport/Ibus.hpp"

static unsigned assertions = 0;
#define CHECK(x) do { ++assertions; if (!(x)) { std::cerr << "FAIL " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)
enum class Role { Switch, Axis, Alias, Count };
struct Transport {
    int raw = 1500;
    int axis_raw = 2000;
    bool pending = false, fs = false, lost = false;
    std::uint32_t mask = 3, counter = 0;
    bool update(std::uint32_t) { bool p=pending; pending=false; if(p) ++counter; return p; }
    int channels() const { return 2; }
    std::uint32_t channelValidMask() const { return mask; }
    int readRaw(int ch) const { return ch == 0 ? raw : axis_raw; }
    bool protoFailsafe() const { return fs; }
    bool frameLost() const { return lost; }
    std::uint32_t frames() const { return counter; }
    rc::RcTransportCaps caps() const { return {}; }
    void begin(int &, std::uint32_t, int, int) { counter=0; pending=false; }
    void sample(int value) { raw=value; pending=true; }
};
static rc::RcConfig<Role> config() {
    rc::RcConfig<Role> c;
    c.map(Role::Switch,0).map(Role::Axis,1).map(Role::Alias,0);
    c.sw(Role::Switch).values({0,1}).auto_levels().learn_alpha(0.01f).done();
    c.sw(Role::Alias).values({0,1}).auto_levels().learn_alpha(0.01f).done();
    c.axis(Role::Axis).valid_raw(900,2100).out(-100,100).done();
    c.setAxisFilter(Role::Axis,0.5f);
    c.setFailsafePolicy(Role::Switch,rc::Failsafe::Mode::HoldLast);
    c.setFailsafePolicy(Role::Axis,rc::Failsafe::Mode::HoldLast);
    return c;
}
static void rejected_switches() {
    for(int fault=0; fault<8; ++fault) for(int learn=0; learn<2; ++learn) {
        Transport t; rc::RcLink<Transport,Role> l(t); CHECK(l.apply_config(config()));
        if(learn) CHECK(l.set_switch_learning_enabled(true));
        rc::RcFailsafeRule<3> sig; sig.check[0]=1; sig.expected[0]=1; sig.hold_ms=fault==7 ? 0 : 10; sig.tol=0;
        CHECK(l.set_failsafe_signature(sig)); l.apply_rxfs_outputs(true);
        t.sample(1490); l.update(1); CHECK(l.read(Role::Switch)==0);
        CHECK(l.read(Role::Axis)==50);
        std::int16_t before[2]{}; CHECK(l.copy_switch_levels(Role::Switch,before,2)==2);
        t.fs=fault==0; t.lost=fault==1; t.mask=fault==2 ? 1u : 3u;
        if(fault==6) t.axis_raw=2200;
        // Fault 3 invalidates the switch; fault 4 rejects by signature candidate;
        // fault 5 invalidates an aliased required role via its own plausibility rule.
        if(fault==5) {
            auto c=config(); c.sw(Role::Alias).valid_raw(900,2100).done();
            CHECK(l.apply_config(c)); if(learn) CHECK(l.set_switch_learning_enabled(true));
            t.sample(1490); l.update(1);
        }
        t.sample(fault==3 ? 0 : fault==5 ? 2200 : 2000); l.update(2);
        // A signature candidate may still be exposed before its hold time;
        // it must nevertheless be excluded from accepted selection/filter history.
        CHECK(l.read(Role::Switch)==(fault==4 ? 1 : 0));
        if(l.read(Role::Axis)!=(fault==4 ? 75 : 50)) std::cerr << "fault=" << fault << " learning=" << learn << " axis=" << l.read(Role::Axis) << '\n';
        CHECK(l.read(Role::Axis)==(fault==4 ? 75 : 50));
        std::int16_t after[2]{}; CHECK(l.copy_switch_levels(Role::Switch,after,2)==2);
        if(fault!=5) { CHECK(before[0]==after[0]); CHECK(before[1]==after[1]); }
        t.fs=false; t.lost=false; t.mask=3; t.axis_raw=2000;
        t.sample(1500); l.update(3); CHECK(l.healthy());
        CHECK(l.read(Role::Switch)==0); CHECK(l.read(Role::Alias)==0);
        CHECK(l.read(Role::Axis)==75);
        if(learn) CHECK(!l.finalize_switch_learning());
    }
}
static void hysteresis() {
    Transport t; rc::RcLink<Transport,Role> l(t); auto c=config();
    c.sw(Role::Switch).values({0,1}).auto_levels().hysteresis_us(200).done(); CHECK(l.apply_config(c));
    for(int us : {1000,1498,1502,1599,1600}) { t.sample(us); l.update(1); CHECK(l.read(Role::Switch)==0); }
    t.sample(1601); l.update(2); CHECK(l.read(Role::Switch)==1);
    for(int us : {1502,1498,1401,1400}) { t.sample(us); l.update(3); CHECK(l.read(Role::Switch)==1); }
    t.sample(1399); l.update(4); CHECK(l.read(Role::Switch)==0);
    c.sw(Role::Switch).values({0,1,2}).auto_levels().hysteresis_us(100).learn_alpha(1.0f).done();
    CHECK(l.apply_config(c)); CHECK(l.set_switch_learning_enabled(true));
    for(int us : {1000,1400,2000}) { t.sample(us); l.update(5); }
    CHECK(l.finalize_switch_learning()); // Learned unequal gaps: 400 and 600 us.
    t.sample(1000); l.update(6); CHECK(l.read(Role::Switch)==0);
    t.sample(1250); l.update(7); CHECK(l.read(Role::Switch)==0);
    t.sample(1251); l.update(8); CHECK(l.read(Role::Switch)==1);
    t.sample(1749); l.update(9); CHECK(l.read(Role::Switch)==1);
    t.sample(1751); l.update(10); CHECK(l.read(Role::Switch)==2);
    t.sample(1000); l.update(11); CHECK(l.read(Role::Switch)==0);
    t.sample(2000); l.update(12); CHECK(l.read(Role::Switch)==2);
    t.sample(1650); l.update(13); CHECK(l.read(Role::Switch)==2);
    t.sample(1649); l.update(14); CHECK(l.read(Role::Switch)==1);
    t.sample(1150); l.update(15); CHECK(l.read(Role::Switch)==1);
    t.sample(1149); l.update(16); CHECK(l.read(Role::Switch)==0);
}
static void explicit_levels() {
    for(const auto &levels : {std::vector<int>{800,1000}, {1000,2000}, {2000,2200}}) {
        Transport t; rc::RcLink<Transport,Role> l(t); auto c=config();
        c.sw(Role::Switch).values({0,1}).auto_levels(false).raw_levels({static_cast<std::int16_t>(levels[0]),static_cast<std::int16_t>(levels[1])}).done();
        CHECK(l.apply_config(c));
        for(int i=0;i<2;++i) { t.sample(levels[static_cast<std::size_t>(i)]); l.update(1); CHECK(l.read(Role::Switch)==i); }
        // Explicit levels retain nearest-level semantics without hysteresis.
        int mid=(levels[0]+levels[1])/2;
        t.sample(mid+1); l.update(2); CHECK(l.read(Role::Switch)==1);
        t.sample(mid-1); l.update(3); CHECK(l.read(Role::Switch)==0);
        c.sw(Role::Switch).valid_raw(900,2100).done();
        CHECK(static_cast<bool>(rc::validate_config(c))==(levels[0]>=900 && levels[1]<=2100));
    }
    for(const auto &levels : {std::vector<int>{0,2000},{1000,1000},{1000,1050}}) {
        auto c=config(); c.sw(Role::Switch).auto_levels(false).raw_levels({static_cast<std::int16_t>(levels[0]),static_cast<std::int16_t>(levels[1])}).done();
        CHECK(!rc::validate_config(c));
    }
}
static void restart() {
    Transport t; rc::RcLink<Transport,Role> l(t); int port=0; CHECK(l.apply_config(config()));
    for(int i=0;i<3;++i) {
        l.begin(port,115200,0,-1); CHECK(!l.healthy()); CHECK(!l.ok()); CHECK(!l.role_valid(Role::Switch));
        CHECK(l.status().frames==0); CHECK(!l.status().has_frame); CHECK(l.read(Role::Axis)==0);
        l.update(100); CHECK(!l.healthy());
        t.sample(2000); l.update(101); CHECK(l.healthy()); CHECK(l.read(Role::Switch)==1);
    }
}
struct Bytes : Stream {
    std::vector<std::uint8_t> bytes; std::size_t offset=0;
    int available() override { return static_cast<int>(bytes.size()-offset); }
    int read() override { return available() ? bytes[offset++] : -1; }
    void packet(int raw) {
        std::uint8_t a=static_cast<std::uint8_t>(raw), b=static_cast<std::uint8_t>(raw>>8);
        std::uint16_t crc=static_cast<std::uint16_t>(65535-6-0x40-a-b);
        for(auto v : {std::uint8_t(6),std::uint8_t(0x40),a,b,static_cast<std::uint8_t>(crc),static_cast<std::uint8_t>(crc>>8)}) bytes.push_back(v);
    }
};
static void batches() {
    Bytes bytes; rc::RcIbusTransport t; t.attach(bytes); rc::RcLink<rc::RcIbusTransport,Role> l(t);
    auto c=config(); c.optional(Role::Axis); CHECK(l.apply_config(c));
    bytes.packet(1000); bytes.packet(2000); bytes.packet(1500); l.update(100);
    CHECK(l.status().frames==3); CHECK(l.status().frame_sequence==1); CHECK(l.read(Role::Switch)==0);
    l.update(200); CHECK(l.status().frame_sequence==1);
    bytes.packet(2000); l.update(500); CHECK(l.status().frames==4); CHECK(l.status().frame_sequence==2); CHECK(l.status().fps==4);
}
static void diagnostics() {
    Transport t; rc::RcLink<Transport,Role> l(t); CHECK(l.apply_config(config())); CHECK(l.status().frames==0);
    t.sample(1500); l.update(1); CHECK(l.status().frames==1);
    CHECK(l.apply_config(config())); CHECK(l.status().frames==1); // Transport lifetime, not mapping lifetime.
    t.counter=0; CHECK(l.apply_config(config())); CHECK(l.status().frames==0);
    CHECK(l.status().frame_sequence==1); l.update(2); CHECK(l.status().frames==0);
    t.sample(1500); l.update(3); CHECK(l.status().frame_sequence==2); CHECK(l.status().frames==1);
}
int main(int argc, char **argv) {
    const char *group=argc>1 ? argv[1] : "all";
    if(!std::strcmp(group,"F01") || !std::strcmp(group,"all")) rejected_switches();
    if(!std::strcmp(group,"F02") || !std::strcmp(group,"all")) hysteresis();
    if(!std::strcmp(group,"F03") || !std::strcmp(group,"all")) explicit_levels();
    if(!std::strcmp(group,"F06") || !std::strcmp(group,"all")) restart();
    if(!std::strcmp(group,"F07") || !std::strcmp(group,"all")) batches();
    if(!std::strcmp(group,"F08") || !std::strcmp(group,"all")) diagnostics();
    if(assertions==0) { std::cerr << "Unknown corrective group: " << group << '\n'; return 2; }
    std::cout << "PASS corrective " << group << ": " << assertions << " assertions\n";
}
