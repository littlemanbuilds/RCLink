#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

#include "Config.hpp"
#include "Link.hpp"
#include "RcMacros.hpp"
#include "transport/detail/IbusDecoder.hpp"
#include "transport/detail/SbusDecoder.hpp"

static unsigned g_tests = 0;
static unsigned g_assertions = 0;
#define CHECK(x) do { ++g_assertions; if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)
#define TEST(name) do { ++g_tests; std::cout << "[test] " << name << "\n"; } while (0)

enum class Role : std::uint8_t { A, B, C, Count };

struct FakeTransport {
    int raw[16]{};
    int count{3};
    std::uint32_t mask{0x7u};
    bool pending{false};
    bool proto_fs{false};
    bool lost{false};
    std::uint32_t frame_count{0};
    std::uint32_t crc{0}, parse{0}, timeouts{0}, discarded{0};
    bool update(std::uint32_t) { if (!pending) return false; pending=false; ++frame_count; return true; }
    int readRaw(int ch) const { return (ch >= 0 && ch < count) ? raw[ch] : 0; }
    int channels() const { return count; }
    std::uint32_t channelValidMask() const { return mask; }
    bool protoFailsafe() const { return proto_fs; }
    bool frameLost() const { return lost; }
    std::uint32_t frames() const { return frame_count; }
    std::uint32_t crcErrors() const { return crc; }
    std::uint32_t parseErrors() const { return parse; }
    std::uint32_t parserTimeouts() const { return timeouts; }
    std::uint32_t discardedBytes() const { return discarded; }
    rc::RcTransportCaps caps() const { return rc::RcTransportCaps{}; }
    void frame(int a, int b=1500, int c=1500) { raw[0]=a; raw[1]=b; raw[2]=c; pending=true; }
};

struct NoCounterTransport {
    int raw[3]{1500, 1500, 1500};
    bool pending{false};
    bool update(std::uint32_t) { const bool result=pending; pending=false; return result; }
    int readRaw(int ch) const { return raw[ch]; }
    int channels() const { return 3; }
    std::uint32_t channelValidMask() const { return 0x7u; }
    bool protoFailsafe() const { return false; }
    bool frameLost() const { return false; }
    rc::RcTransportCaps caps() const { return rc::RcTransportCaps{}; }
    void frame(int a) { raw[0]=a; pending=true; }
};

static rc::RcConfig<Role> base_cfg() {
    rc::RcConfig<Role> c;
    c.map(Role::A,0).map(Role::B,1).map(Role::C,2);
    c.axis(Role::A).raw(1000,2000,1500).out(-100,100).done();
    c.axis(Role::B).raw(1000,2000,1500).out(-100,100).done();
    c.axis(Role::C).raw(1000,2000,1500).out(-100,100).done();
    return c;
}

static std::vector<std::uint8_t> make_ibus(const std::vector<int>& ch) {
    const std::size_t n = ch.size();
    std::vector<std::uint8_t> f(4u + 2u*n, 0u);
    f[0] = static_cast<std::uint8_t>(f.size()); f[1]=0x40u;
    for (std::size_t i=0;i<n;++i) { const std::uint16_t v=static_cast<std::uint16_t>(ch[i]); f[2u+2u*i]=static_cast<std::uint8_t>(v&0xFFu); f[3u+2u*i]=static_cast<std::uint8_t>((v>>8u)&0xFFu); }
    std::uint16_t sum=0xFFFFu; for (std::size_t i=0;i<f.size()-2u;++i) sum=static_cast<std::uint16_t>(sum-f[i]);
    f[f.size()-2u]=static_cast<std::uint8_t>(sum&0xFFu); f[f.size()-1u]=static_cast<std::uint8_t>((sum>>8u)&0xFFu); return f;
}

static std::vector<std::uint8_t> make_sbus(const std::uint16_t ch[16], std::uint8_t flags=0u, std::uint8_t footer=0u) {
    std::vector<std::uint8_t> f(25u,0u); f[0]=0x0Fu;
    std::uint32_t bit=0u;
    for (std::size_t i=0;i<16u;++i) {
        const std::uint16_t v=static_cast<std::uint16_t>(ch[i]&0x07FFu);
        for (unsigned b=0;b<11u;++b,++bit) if ((v&(1u<<b))!=0u) { const std::size_t by=1u+static_cast<std::size_t>(bit/8u); const unsigned off=bit%8u; f[by]=static_cast<std::uint8_t>(f[by] | static_cast<std::uint8_t>(1u<<off)); }
    }
    f[23]=flags; f[24]=footer; return f;
}

int main() {
    TEST("accepted-frame sequence deltas use natural uint32 wrap semantics");
    { CHECK(rc::detail::sequence_delta(1u, 0xFFFFFFFFu)==2u); }

    TEST("asymmetric centre mapping and monotonicity");
    { FakeTransport t; rc::RcLink<FakeTransport,Role> l(t); auto c=base_cfg(); c.axis(Role::A).raw(1000,2000,1600).out(-100,100).done(); CHECK(l.apply_config(c)); int last=-101; for(int x=1000;x<=2000;++x){t.frame(x);l.update(static_cast<std::uint32_t>(x));const int v=l.read(Role::A);CHECK(v>=last);last=v;if(x==1000)CHECK(v==-100);if(x==1600)CHECK(v==0);if(x==2000)CHECK(v==100);} }

    TEST("one-sided throttle preserves endpoint neutral");
    { FakeTransport t; rc::RcLink<FakeTransport,Role> l(t); auto c=base_cfg(); c.axis(Role::A).raw(1000,2000,1000).deadband_us(8).out(0,100).done(); CHECK(l.apply_config(c)); t.frame(1000);l.update(0);CHECK(l.read(Role::A)==0);t.frame(2000);l.update(10);CHECK(l.read(Role::A)==100); }

    TEST("internal unipolar centre defaults to output midpoint");
    { FakeTransport t; rc::RcLink<FakeTransport,Role> l(t); auto c=base_cfg(); c.axis(Role::A).raw(1000,2000,1500).out(0,100).done(); CHECK(l.apply_config(c)); t.frame(1500);l.update(1);CHECK(l.read(Role::A)==50); }

    TEST("inversion remains explicit");
    { FakeTransport t; rc::RcLink<FakeTransport,Role> l(t); auto c=base_cfg(); c.axis(Role::A).invert().done(); CHECK(l.apply_config(c)); t.frame(1000);l.update(1);CHECK(l.read(Role::A)==100); t.frame(2000);l.update(2);CHECK(l.read(Role::A)==-100); }

    TEST("endpoint-centred inversion is rejected while reversed one-sided output is coherent");
    { auto c=base_cfg();c.axis(Role::A).raw(1000,2000,1000).out(0,100).invert().done();auto r=rc::validate_config(c);CHECK(!r);CHECK(r.error==rc::RcConfigError::EndpointCenteredInversion);c=base_cfg();c.axis(Role::A).raw(1000,2000,2000).out(0,100).invert().done();r=rc::validate_config(c);CHECK(!r);CHECK(r.error==rc::RcConfigError::EndpointCenteredInversion);FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);c=base_cfg();c.axis(Role::A).raw(1000,2000,1000).out(100,0).done();CHECK(l.apply_config(c));t.frame(1000);l.update(1);CHECK(l.read(Role::A)==100);t.frame(2000);l.update(2);CHECK(l.read(Role::A)==0); }

    TEST("deadband follows calibrated asymmetric centre");
    { FakeTransport t; rc::RcLink<FakeTransport,Role> l(t); auto c=base_cfg(); c.axis(Role::A).raw(1000,2000,1600).deadband_us(20).done(); CHECK(l.apply_config(c)); for(int x=1580;x<=1620;++x){t.frame(x);l.update(static_cast<std::uint32_t>(x));CHECK(l.read(Role::A)==0);} }

    TEST("configuration rejects invalid boundaries and non-finite values");
    { auto c=base_cfg(); c.axis(Role::A).raw(2000,1000,1500).done(); CHECK(!rc::validate_config(c)); c=base_cfg(); c.axis(Role::A).raw(1000,2000,999).done(); CHECK(!rc::validate_config(c)); c=base_cfg(); c.axis(Role::A).raw(1000,2000,1010).done(); CHECK(!rc::validate_config(c)); c=base_cfg(); c.axis(Role::A).raw(1000,2000,1500).deadband_us(500).done(); CHECK(!rc::validate_config(c)); c=base_cfg(); c.axis(Role::A).out(100,-100).done(); CHECK(!rc::validate_config(c)); c=base_cfg(); c.axis(Role::A).expo(std::numeric_limits<float>::quiet_NaN()).done(); CHECK(!rc::validate_config(c)); c=base_cfg(); c.setAxisFilter(Role::A,std::numeric_limits<float>::infinity()); CHECK(!rc::validate_config(c)); }

    TEST("unmapped required roles fail closed while optional roles may be absent");
    { rc::RcConfig<Role> c; CHECK(c.role_to_channel[0]==rc::kInvalidChannel); c.map(Role::A,0).map(Role::B,1).optional(Role::C); c.axis(Role::A).done();c.axis(Role::B).done();c.axis(Role::C).done();CHECK(rc::validate_config(c)); c.require(Role::C,true);CHECK(!rc::validate_config(c)); }

    TEST("invalid role and channel builders are bounds checked");
    { auto c=base_cfg(); c.map(static_cast<Role>(99),0); CHECK(!rc::validate_config(c)); c=base_cfg(); c.map(Role::A,rc::kInvalidChannel);CHECK(!rc::validate_config(c)); FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));CHECK(l.read(static_cast<Role>(99))==0);CHECK(l.read_by_index(999)==0);std::int16_t v=7;CHECK(!l.try_read(static_cast<Role>(99),v));CHECK(v==7); }

    TEST("missing required channel is fresh but unhealthy and fails closed");
    { FakeTransport t; rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.setFailsafePolicy(Role::A,rc::Failsafe::Mode::Value,11);CHECK(l.apply_config(c));t.count=2;t.mask=0x3u;t.frame(1800,1500);l.update(10);CHECK(l.status().has_frame);CHECK(l.status().link_ok);CHECK(!l.status().required_roles_valid);CHECK(!l.healthy());CHECK(l.read(Role::A)==11); }

    TEST("missing optional role fails only that role");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.optional(Role::C);c.setFailsafePolicy(Role::C,rc::Failsafe::Mode::Value,33);CHECK(l.apply_config(c));t.count=2;t.mask=3;t.frame(2000,1000);l.update(10);CHECK(l.status().required_roles_valid);CHECK(l.healthy());CHECK(l.read(Role::A)==100);CHECK(l.read(Role::B)==-100);CHECK(l.read(Role::C)==33); }

    TEST("HoldLast preserves acceptable values across protocol failsafe and recovery");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.setFailsafePolicy(Role::A,rc::Failsafe::Mode::HoldLast);CHECK(l.apply_config(c));t.frame(1800);l.update(1);CHECK(l.read(Role::A)==60);t.proto_fs=true;t.frame(1000);l.update(2);CHECK(l.read(Role::A)==60);t.proto_fs=false;t.frame(2000);l.update(3);CHECK(l.read(Role::A)==100); }

    TEST("HoldLast treats frame-lost evidence as unhealthy without replacing history");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.setFailsafePolicy(Role::A,rc::Failsafe::Mode::HoldLast);CHECK(l.apply_config(c));t.frame(1700);l.update(1);CHECK(l.read(Role::A)==40);t.lost=true;t.frame(1000);l.update(2);CHECK(!l.healthy());CHECK(l.read(Role::A)==40);t.lost=false;t.frame(1900);l.update(3);CHECK(l.read(Role::A)==80); }

    TEST("HoldLast rejects frames with missing required roles and later recovers");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.setFailsafePolicy(Role::A,rc::Failsafe::Mode::HoldLast);CHECK(l.apply_config(c));t.frame(1600);l.update(1);CHECK(l.read(Role::A)==20);t.count=2;t.mask=0x3u;t.frame(2000);l.update(2);CHECK(l.read(Role::A)==20);t.count=3;t.mask=0x7u;t.frame(2000);l.update(3);CHECK(l.read(Role::A)==100); }

    TEST("HoldLast naturally retains history through a stale-link transition");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.setLinkTimeout(20);c.setFailsafePolicy(Role::A,rc::Failsafe::Mode::HoldLast);CHECK(l.apply_config(c));t.frame(1750);l.update(1);CHECK(l.read(Role::A)==50);l.update(22);CHECK(!l.status().link_ok);CHECK(l.read(Role::A)==50);t.frame(1250);l.update(23);CHECK(l.read(Role::A)==-50); }

    TEST("HoldLast excludes receiver-signature candidate payloads from history");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.setFailsafePolicy(Role::A,rc::Failsafe::Mode::HoldLast);CHECK(l.apply_config(c));rc::RcFailsafeRule<3> r;r.check[0]=1;r.expected[0]=-100;r.hold_ms=20;CHECK(l.set_failsafe_signature(r));l.apply_rxfs_outputs(true);t.frame(1800);l.update(1);CHECK(l.read(Role::A)==60);t.frame(1000);l.update(10);CHECK(!l.status().rx_failsafe_sig);t.frame(1000);l.update(30);CHECK(l.status().rx_failsafe_sig);CHECK(l.read(Role::A)==60);t.frame(2000);l.update(31);CHECK(!l.status().rx_failsafe_sig);CHECK(l.read(Role::A)==100); }

    TEST("first-frame state and timestamp wrap are explicit");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.setLinkTimeout(50);CHECK(l.apply_config(c));l.update(0);CHECK(!l.status().has_frame);CHECK(l.status().last_frame_age==rc::kInvalidFrameAgeMs);t.frame(1500);l.update(0xFFFFFFF0u);CHECK(l.status().link_ok);l.update(0x00000010u);CHECK(l.status().link_ok);l.update(0x00000030u);CHECK(!l.status().link_ok); }

    TEST("epsilon comparison cannot overflow int16");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.axis(Role::A).out(-32768.0f,32767.0f).done();c.setEpsilon(Role::A,1);CHECK(l.apply_config(c));t.frame(1000);l.update(1);CHECK(l.read(Role::A)==-32768);t.frame(2000);l.update(2);CHECK(l.read(Role::A)==32767); }

    TEST("failsafe signatures reject empty masks and honour timestamp zero");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));rc::RcFailsafeRule<3> empty;CHECK(!l.set_failsafe_signature(empty));rc::RcFailsafeRule<3> r;r.check[0]=1;r.expected[0]=0;r.hold_ms=20;CHECK(l.set_failsafe_signature(r));t.frame(1500);l.update(0);CHECK(!l.status().rx_failsafe_sig);t.frame(1500);l.update(20);CHECK(l.status().rx_failsafe_sig); }

    TEST("failsafe hold advances only across new valid frames");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));rc::RcFailsafeRule<3> r;r.check[0]=1;r.expected[0]=0;r.hold_ms=20;CHECK(l.set_failsafe_signature(r));t.frame(1500);l.update(1);l.update(50);CHECK(!l.status().rx_failsafe_sig);t.frame(1500);l.update(60);CHECK(l.status().rx_failsafe_sig); }

    TEST("failsafe tolerance is genuinely 16-bit");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.axis(Role::A).out(-1000,1000).done();CHECK(l.apply_config(c));rc::RcFailsafeRule<3> r;r.check[0]=1;r.expected[0]=0;r.tol=400;r.hold_ms=0;CHECK(l.set_failsafe_signature(r));t.frame(1650);l.update(1);CHECK(l.status().rx_failsafe_sig); }

    TEST("switch raw levels are deterministic and validated");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.sw(Role::A).values({0,1,2}).raw_levels({1000,1500,2000}).done();CHECK(l.apply_config(c));t.frame(1510);l.update(1);CHECK(l.read(Role::A)==1);auto bad=base_cfg();bad.sw(Role::A).values({0,1,2}).raw_levels({1000,1500}).done();CHECK(!rc::validate_config(bad));bad=base_cfg();bad.sw(Role::A).values({0,1}).raw_levels({1000,1050}).done();CHECK(!rc::validate_config(bad)); }

    TEST("oversized switch builders fail instead of truncating");
    { auto c=base_cfg();c.sw(Role::A).values({0,1,2,3,4,5,6,7,8}).done();auto r=rc::validate_config(c);CHECK(!r);CHECK(r.error==rc::RcConfigError::InvalidSwitchCount);c=base_cfg();c.sw(Role::A).values({0,1,2}).raw_levels({1000,1100,1200,1300,1400,1500,1600,1700,1800}).done();r=rc::validate_config(c);CHECK(!r);CHECK(r.error==rc::RcConfigError::InvalidSwitchLevels); }

    TEST("switch positions are based on raw geometry, not output labels");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.sw(Role::A).values({0,1}).done();CHECK(l.apply_config(c));t.frame(1400);l.update(1);CHECK(l.read(Role::A)==0);t.frame(1600);l.update(2);CHECK(l.read(Role::A)==1); }

    TEST("switch-learning finalization requires valid observations of every position");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.sw(Role::A).values({0,1,2}).auto_levels().learn_alpha(0.5f).min_sep_us(100).done();CHECK(l.apply_config(c));CHECK(l.set_switch_learning_enabled(true));CHECK(!l.finalize_switch_learning());CHECK(l.set_switch_learning_enabled(true));t.frame(1000);l.update(1);t.frame(1500);l.update(2);CHECK(!l.finalize_switch_learning());CHECK(l.set_switch_learning_enabled(true));t.mask=0x6u;t.frame(1000);l.update(3);t.mask=0x7u;t.frame(1500);l.update(4);t.frame(2000);l.update(5);CHECK(!l.finalize_switch_learning());CHECK(l.set_switch_learning_enabled(true));t.frame(1000);l.update(6);t.frame(1500);l.update(7);t.frame(2000);l.update(8);CHECK(l.finalize_switch_learning());std::int16_t learned[3]{};CHECK(l.copy_switch_levels(Role::A,learned,3)==3); }

    TEST("deterministic switches do not create learning finalization requirements");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);auto c=base_cfg();c.sw(Role::A).values({0,1}).raw_levels({1000,2000}).done();CHECK(l.apply_config(c));CHECK(l.set_switch_learning_enabled(true));CHECK(l.finalize_switch_learning()); }

    TEST("iBUS decodes standard and short channel frames exactly");
    { rc::detail::IbusDecoder d; std::vector<int> c14;for(int i=0;i<14;++i)c14.push_back(1000+i*10);auto f=make_ibus(c14);bool ok=false;for(auto b:f)ok=d.feed(b,1)||ok;CHECK(ok);CHECK(d.channels()==14);CHECK(d.channelValidMask()==0x3FFFu);for(int i=0;i<14;++i)CHECK(d.readRaw(i)==c14[static_cast<std::size_t>(i)]);auto f2=make_ibus({1234});ok=false;for(auto b:f2)ok=d.feed(b,2)||ok;CHECK(ok);CHECK(d.channels()==1);CHECK(d.readRaw(0)==1234); }

    TEST("iBUS recovers from bad CRC, truncation, noise and back-to-back frames");
    { rc::detail::IbusDecoder d;auto bad=make_ibus({1000,1500});bad.back()^=0x55u;for(auto b:bad)(void)d.feed(b,10);auto good=make_ibus({1111,1666});bool ok=false;for(auto b:good)ok=d.feed(b,11)||ok;CHECK(ok);CHECK(d.crcErrors()>=1u);CHECK(d.readRaw(1)==1666);auto trunc=make_ibus({1200,1300});for(std::size_t i=0;i<3;++i)(void)d.feed(trunc[i],20);d.tick(30);CHECK(d.parserTimeouts()>=1u);std::uint32_t seed=1;for(int i=0;i<5000;++i){seed=seed*1664525u+1013904223u;(void)d.feed(static_cast<std::uint8_t>(seed>>24u),40);}ok=false;for(auto b:good)ok=d.feed(b,50)||ok;for(auto b:good)ok=d.feed(b,50)||ok;CHECK(ok);CHECK(d.frames()>=3u); }

    TEST("iBUS timeout arithmetic survives millis wrap");
    { rc::detail::IbusDecoder d;auto f=make_ibus({1000});(void)d.feed(f[0],0xFFFFFFFEu);d.tick(1u);CHECK(d.parserTimeouts()==0u);d.tick(10u);CHECK(d.parserTimeouts()==1u); }

    TEST("SBUS decodes all channels and protocol flags");
    { rc::detail::SbusDecoder d;std::uint16_t ch[16];for(int i=0;i<16;++i)ch[i]=static_cast<std::uint16_t>(172+i*80);auto f=make_sbus(ch,0x0Cu);bool ok=false;for(auto b:f)ok=d.feed(b,1)||ok;CHECK(ok);CHECK(d.channels()==16);CHECK(d.protoFailsafe());CHECK(d.frameLost());CHECK(d.channelValidMask()==0xFFFFu);for(int i=0;i<16;++i)CHECK(d.readRaw(i)>=800 && d.readRaw(i)<=2200); }

    TEST("SBUS sliding recovery handles footer corruption, truncation, noise and back-to-back frames");
    { rc::detail::SbusDecoder d;std::uint16_t ch[16]{};for(int i=0;i<16;++i)ch[i]=992;auto bad=make_sbus(ch,0,0x99);for(auto b:bad)(void)d.feed(b,1);auto good=make_sbus(ch);bool ok=false;for(auto b:good)ok=d.feed(b,2)||ok;CHECK(ok);CHECK(d.crcErrors()>=1u);for(std::size_t i=0;i<5;++i)(void)d.feed(good[i],10);d.tick(20);CHECK(d.parserTimeouts()>=1u);std::uint32_t seed=7;for(int i=0;i<5000;++i){seed=seed*1103515245u+12345u;(void)d.feed(static_cast<std::uint8_t>(seed>>24u),30);}ok=false;for(auto b:good)ok=d.feed(b,31)||ok;for(auto b:good)ok=d.feed(b,31)||ok;CHECK(ok);CHECK(d.frames()>=3u); }

    TEST("PW_PVT ten-role configuration remains valid");
    {
        enum class CarRole : std::uint8_t
        {
            steering, direction, speed, indicators, volume, power,
            override_role, lights, mode, obstacle, Count
        };
        rc::RcConfig<CarRole> c;
        for (std::size_t i = 0u; i < static_cast<std::size_t>(CarRole::Count); ++i)
            c.map(static_cast<CarRole>(i), static_cast<std::uint8_t>(i));
        c.axis(CarRole::steering).raw(1000, 2000, 1500).deadband_us(8).out(-100.0f, 100.0f).done();
        c.axis(CarRole::direction).raw(1000, 2000, 1500).deadband_us(8).out(-100.0f, 100.0f).done();
        c.axis(CarRole::speed).raw(1000, 2000, 1000).deadband_us(8).out(0.0f, 100.0f).done();
        c.axis(CarRole::indicators).raw(1000, 2000, 1500).deadband_us(8).out(-100.0f, 100.0f).done();
        c.axis(CarRole::volume).raw(1000, 2000, 1500).deadband_us(4).out(0.0f, 100.0f).done();
        c.axis(CarRole::power).raw(1000, 2000, 1500).deadband_us(4).out(0.0f, 100.0f).done();
        c.sw(CarRole::override_role).raw_levels({1000, 2000}).values({0.0f, 1.0f}).done();
        c.sw(CarRole::lights).raw_levels({1000, 2000}).values({0.0f, 1.0f}).done();
        c.sw(CarRole::mode).raw_levels({1000, 1500, 2000}).values({0.0f, 1.0f, 2.0f}).done();
        c.sw(CarRole::obstacle).raw_levels({1000, 2000}).values({0.0f, 1.0f}).done();
        c.setLinkTimeout(50);
        CHECK(rc::validate_config(c));
    }

    TEST("transport diagnostics propagate to link status");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));t.crc=2;t.parse=3;t.timeouts=4;t.discarded=5;t.frame(1500);l.update(10);CHECK(l.status().frames==1);CHECK(l.status().frame_sequence==1);CHECK(l.status().crc_errors==2);CHECK(l.status().parse_errors==3);CHECK(l.status().parser_timeouts==4);CHECK(l.status().discarded_bytes==5); }

    TEST("RCLink owns consecutive accepted-frame sequence without a transport counter");
    { NoCounterTransport t;rc::RcLink<NoCounterTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));CHECK(l.status().frame_sequence==0u);t.frame(1500);l.update(1);CHECK(l.status().frame_sequence==1u);t.frame(1600);l.update(2);CHECK(l.status().frame_sequence==2u);CHECK(l.status().frames==0u); }

    TEST("transport diagnostic reset cannot move accepted-frame sequence backward");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));t.frame(1500);l.update(1);t.frame(1500);l.update(2);CHECK(l.status().frame_sequence==2u);t.frame_count=0u;t.frame(1500);l.update(3);CHECK(l.status().frames==1u);CHECK(l.status().frame_sequence==3u); }

    TEST("configuration reapply clears stale frame validity until a new frame");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));t.frame(2000);l.update(10);CHECK(l.status().has_frame);CHECK(l.status().link_ok);CHECK(l.read(Role::A)==100);auto c=base_cfg();c.setFailsafePolicy(Role::A,rc::Failsafe::Mode::Value,9);CHECK(l.apply_config(c));CHECK(!l.status().has_frame);CHECK(!l.status().link_ok);CHECK(l.status().last_frame_age==rc::kInvalidFrameAgeMs);CHECK(!l.status().required_roles_valid);CHECK(!l.role_valid(Role::A));CHECK(l.read(Role::A)==9);t.frame(1000);l.update(11);CHECK(l.status().has_frame);CHECK(l.status().link_ok);CHECK(l.read(Role::A)==-100); }

    TEST("fps accounting tolerates transport frame counter reset");
    { FakeTransport t;rc::RcLink<FakeTransport,Role> l(t);CHECK(l.apply_config(base_cfg()));t.frame(1500);l.update(100);t.frame(1500);l.update(200);t.frame(1500);l.update(300);l.update(500);CHECK(l.status().fps==6u);t.frame_count=0;t.frame(1500);l.update(1000);CHECK(l.status().fps==2u);CHECK(l.status().frame_sequence==4u); }

    std::cout << "PASS: " << g_tests << " tests / " << g_assertions << " assertions\n";
    return 0;
}
