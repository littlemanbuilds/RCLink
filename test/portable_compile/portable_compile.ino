/**
 * @file portable_compile.ino
 *
 * @brief Minimal hardware-neutral RCLink compile smoke test.
 */

#include <Arduino.h>
#include <RCLink.h>

#define TEST_ROLES(X) \
    X(Throttle)       \
    X(Steering)       \
    X(Mode)
RC_DECLARE_ROLES(TestRole, TEST_ROLES)

struct NullTransport
{
    bool update(uint32_t) { return false; }
    int readRaw(int) const { return 1500; }
    int channels() const { return 3; }
    uint32_t channelValidMask() const { return 0x07u; }
    bool protoFailsafe() const { return false; }
    bool frameLost() const { return false; }
    uint32_t frames() const { return 0u; }
    uint32_t crcErrors() const { return 0u; }
    rc::RcTransportCaps caps() const { return rc::RcTransportCaps{}; }
};

NullTransport transport;
rc::RcLink<NullTransport, TestRole> rclink(transport);

void setup()
{
    RC_CONFIG(TestRole, cfg);
    RC_CFG_MAP_DEFAULT(TestRole, cfg);
    cfg.axis(TestRole::Throttle).raw(1000, 2000, 1000).out(0.0f, 100.0f).done();
    cfg.axis(TestRole::Steering).raw(1000, 2000, 1500).out(-100.0f, 100.0f).done();
    cfg.sw(TestRole::Mode).values({0.0f, 1.0f, 2.0f}).done();
    (void)rclink.apply_config(cfg);
}

void loop()
{
    rclink.update(millis());
}
