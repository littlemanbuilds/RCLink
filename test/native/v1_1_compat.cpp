#include <RCLink.h>

#define FLYSKY_ROLES(_) \
    _(Ch1_RH)           \
    _(Ch2_RV)           \
    _(Ch3_LV)
RC_DECLARE_ROLES(Flysky, FLYSKY_ROLES)

#ifndef RCLINK_NO_LEGACY_GLOBAL_ALIASES
RcIbusTransport ibusTransport;
RcLink<RcIbusTransport, Flysky> rclink(ibusTransport);

static_assert(role_count(Flysky{}) == 3u, "v1.1 role_count compatibility");
static_assert(role_by_index(Flysky{}, 1u) == Flysky::Ch2_RV, "v1.1 role_by_index compatibility");

int main()
{
    RC_CONFIG(Flysky, cfg);
    RC_CFG_MAP_DEFAULT(Flysky, cfg);
    cfg.axis(Flysky::Ch1_RH).done();
    cfg.axis(Flysky::Ch2_RV).done();
    cfg.axis(Flysky::Ch3_LV).done();
    (void)rclink.apply_config(cfg);

    const char *name = to_string(Flysky::Ch1_RH);
    const RcLinkStatus &link_status = rclink.status();
    FsRule<Flysky> all = make_signature_all(Flysky{}, 0, 3, 250);
    FsRule<Flysky> overrides = make_signature_with_overrides(
        Flysky{}, 0, 3, 250, {{Flysky::Ch3_LV, 10}});
    FsRule<Flysky> selected = make_signature_selected(
        Flysky{}, 3, 250, {{Flysky::Ch1_RH, 0}, {Flysky::Ch2_RV, 0}});
    (void)name;
    (void)link_status;
    (void)all;
    (void)overrides;
    (void)selected;
    return 0;
}
#else
int main()
{
    rc::RcIbusTransport ibusTransport;
    rc::RcLink<rc::RcIbusTransport, Flysky> rclink(ibusTransport);
    rc::FsRule<Flysky> signature = rc::make_signature_all(Flysky{}, 0, 3, 250);
    (void)rclink;
    (void)signature;
    (void)rc::to_string(Flysky::Ch1_RH);
    (void)rc::role_count(Flysky{});
    (void)rc::role_by_index(Flysky{}, 0u);
    return 0;
}
#endif
