#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <new>
#include <type_traits>

#define ARDUINO_ARCH_ESP32 1
#define SERIAL_8E2 0x42
#include "transport/Sbus_Esp32.hpp"
#include "Link.hpp"
#include "Config.hpp"

static std::size_t allocations = 0;
void *operator new(std::size_t size) {
    ++allocations;
    if (void *p = std::malloc(size == 0 ? 1 : size)) return p;
    throw std::bad_alloc();
}
void *operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }

#define CHECK(x) do { if (!(x)) { std::cerr << "FAIL " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)

// Fixed storage also lets the actual transport run without fixture allocations.
struct SerialFixture : Stream {
    std::uint8_t bytes[256]{};
    int head{0}, tail{0};
    bool configured{false};
    void begin(unsigned long baud, int framing, int rx, int tx, bool inverted) {
        configured = baud == 100000ul && framing == SERIAL_8E2 && rx == 18 && tx == -1 && inverted;
        head = tail = 0;
    }
    int available() override { return tail - head; }
    int read() override { return head < tail ? bytes[head++] : -1; }
    void packet(const std::uint16_t (&values)[16], std::uint8_t flags=0, std::uint8_t footer=0) {
        if (head == tail) head = tail = 0;
        CHECK(tail + 25 <= 256);
        std::uint8_t frame[25]{};
        frame[0] = 0x0f;
        // Independent bit-by-bit fixture, not the decoder's extraction expressions.
        for (unsigned ch=0; ch<16; ++ch)
            for (unsigned bit=0; bit<11; ++bit) {
                const unsigned offset=ch*11+bit;
                if ((values[ch] & (1u<<bit)) != 0)
                    frame[1+offset/8] |= static_cast<std::uint8_t>(1u<<(offset%8));
            }
        frame[23]=flags; frame[24]=footer;
        for (auto byte : frame) bytes[tail++]=byte;
    }
};

enum class Role : std::uint8_t { Axis, Count };

int main() {
    const auto initialAllocations = allocations;
    static_assert(std::is_trivially_destructible<rc::detail::SbusDecoder>::value, "fixed value storage");
    static_assert(noexcept(std::declval<const rc::RcSbusEsp32Transport &>().tryReadNative(0, std::declval<std::uint16_t &>())), "noexcept API");
    SerialFixture serial;
    rc::RcSbusEsp32Transport transport;
    std::uint16_t value=54321;
    CHECK(!transport.tryReadNative(0,value)); CHECK(value==54321);
    transport.begin(serial,100000,18,-1); CHECK(serial.configured);
    CHECK(!transport.tryReadNative(-1,value)); CHECK(!transport.tryReadNative(16,value));
    CHECK(!transport.tryReadNative(255,value)); CHECK(value==54321);
    std::uint16_t channels[16]{};
    // Neither a malformed initial packet nor a partial packet grants reception.
    serial.packet(channels,12,0x99); CHECK(!transport.update(0));
    CHECK(!transport.tryReadNative(0,value)); CHECK(value==54321);
    serial.packet(channels); serial.tail=7; CHECK(!transport.update(1));
    CHECK(!transport.tryReadNative(0,value)); CHECK(value==54321);
    CHECK(!transport.update(100)); CHECK(transport.parserTimeouts()==1);
    const std::uint16_t refs[]={0,172,992,1811,2047};
    const int converted[]={896,1000,1500,2000,2143};
    for (unsigned n=0;n<5;++n) {
        for (auto &ch:channels) ch=refs[n];
        serial.packet(channels); CHECK(transport.update(n+101));
        for (int ch=0;ch<16;++ch) {
            CHECK(transport.tryReadNative(ch,value)); CHECK(value==refs[n]);
            CHECK(transport.readRaw(ch)==converted[n]);
        }
    }
    // Exhaustive native-code round trips at every channel position.
    for (unsigned n=0;n<2048;++n) {
        for (unsigned ch=0;ch<16;++ch) channels[ch]=static_cast<std::uint16_t>((n+ch*127u)%2048u);
        serial.packet(channels); CHECK(transport.update(n+106));
        for (int ch=0;ch<16;++ch) {
            CHECK(transport.tryReadNative(ch,value)); CHECK(value==channels[ch]);
            CHECK(transport.readRaw(ch)==1000+((static_cast<int>(channels[ch])-172)*1000)/1639);
        }
    }
    const auto count=transport.frames();
    CHECK(!transport.update(2200)); CHECK(transport.tryReadNative(0,value));
    const auto saved=value;
    // Invalid footer and timed-out truncation cannot replace accepted values/status.
    for (auto &ch:channels) ch=172;
    serial.packet(channels,12,0x99); CHECK(!transport.update(2201));
    CHECK(transport.frames()==count); CHECK(transport.tryReadNative(0,value)); CHECK(value==saved);
    CHECK(!transport.protoFailsafe()); CHECK(transport.lostFrames()==0);
    serial.packet(channels); serial.tail=7; CHECK(!transport.update(2202));
    const auto timeouts=transport.parserTimeouts();
    CHECK(!transport.update(2300)); CHECK(transport.parserTimeouts()==timeouts+1);
    CHECK(transport.tryReadNative(0,value)); CHECK(value==saved);
    // Latest-value flags recover; interval counters preserve superseded evidence.
    for (auto &ch:channels) ch=172;
    serial.packet(channels,4); CHECK(transport.update(2301));
    CHECK(transport.frameLost()); CHECK(!transport.protoFailsafe());
    CHECK(transport.lostFrames()==1); CHECK(transport.failsafeFrames()==0);
    CHECK(transport.frames()==count+1); CHECK(transport.lastGoodMs()==2301);
    for (int ch=0;ch<16;++ch) {
        CHECK(transport.tryReadNative(ch,value)); CHECK(value==172);
        CHECK(transport.readRaw(ch)==1000);
    }
    for (auto &ch:channels) ch=992;
    serial.packet(channels,8); CHECK(transport.update(2302));
    CHECK(!transport.frameLost()); CHECK(transport.protoFailsafe());
    CHECK(transport.lostFrames()==1); CHECK(transport.failsafeFrames()==1);
    CHECK(transport.frames()==count+2); CHECK(transport.lastGoodMs()==2302);
    for (int ch=0;ch<16;++ch) {
        CHECK(transport.tryReadNative(ch,value)); CHECK(value==992);
        CHECK(transport.readRaw(ch)==1500);
    }
    for (auto &ch:channels) ch=0;
    serial.packet(channels,12);
    for (auto &ch:channels) ch=1811;
    serial.packet(channels);
    CHECK(transport.update(2303));
    CHECK(!transport.frameLost()); CHECK(!transport.protoFailsafe());
    CHECK(transport.lostFrames()==2); CHECK(transport.failsafeFrames()==2);
    CHECK(transport.frames()==count+4); CHECK(transport.lastGoodMs()==2303);
    CHECK(transport.tryReadNative(15,value)); CHECK(value==1811); CHECK(transport.readRaw(15)==2000);
    CHECK(!transport.tryReadNative(16,value)); CHECK(value==1811);
    // Calibrated values retain the legacy conversion, including stale-data policy.
    rc::RcLink<rc::RcSbusEsp32Transport,Role> link(transport);
    rc::RcConfig<Role> cfg;
    cfg.map(Role::Axis,0).axis(Role::Axis).raw(1000,2000,1500).out(-100,100).done();
    cfg.setLinkTimeout(50); CHECK(link.apply_config(cfg));
    serial.packet(channels); link.update(2400);
    CHECK(link.healthy()); CHECK(link.read(Role::Axis)==100);
    CHECK(link.status().frame_sequence==1); CHECK(link.status().frames==count+5);
    for (auto &ch:channels) ch=172;
    serial.packet(channels,12); serial.packet(channels); link.update(2401);
    CHECK(link.healthy()); CHECK(link.read(Role::Axis)==-100);
    CHECK(link.status().frame_sequence==2); CHECK(transport.lostFrames()==3);
    CHECK(transport.failsafeFrames()==3); CHECK(link.status().frames==count+7);
    link.update(2452); CHECK(!link.healthy()); CHECK(!link.status().link_ok);
    CHECK(transport.tryReadNative(0,value)); CHECK(value==172);
    link.begin(serial,100000,18,-1);
    CHECK(!transport.tryReadNative(0,value)); CHECK(value==172);
    CHECK(transport.frames()==0); CHECK(transport.lostFrames()==0); CHECK(transport.failsafeFrames()==0);
    CHECK(!link.status().has_frame); CHECK(!link.status().link_ok);
    CHECK(transport.readRaw(0)==0); CHECK(!transport.frameLost()); CHECK(!transport.protoFailsafe());
    serial.packet(channels); link.update(2500); CHECK(link.healthy());
    CHECK(transport.tryReadNative(0,value)); CHECK(value==172);
    const std::uint8_t footers[]={0x04,0x14,0x24,0x34};
    for (auto footer:footers) {
        serial.packet(channels,0,footer); CHECK(transport.update(2501));
        CHECK(transport.tryReadNative(15,value)); CHECK(value==172);
        CHECK(transport.readRaw(15)==1000);
    }
    CHECK(allocations==initialAllocations);
    std::cout << "S.BUS native/transport/coherence: PASS; decoder bytes=" << sizeof(rc::detail::SbusDecoder) << "\n";
}
