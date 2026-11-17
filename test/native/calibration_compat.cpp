#include "Arduino.h"
Monitor Serial;
namespace rc { namespace calibrate { struct Stats; } }
#include "calibration/Ibus_Calibrate.hpp"
#ifdef RCLINK_CALIBRATION_STATS_MEMBERS
#error Private calibration schema macro leaked
#endif
int main()
{
    rc::calibrate::Stats stats{{1000}}; // Valid public C++14 aggregate syntax.
    auto center = &rc::calibrate::estimate_axis_center;
    auto axis = &rc::calibrate::is_axis_by_hist;
    auto classify = &rc::calibrate::classify_channel;
    (void)axis; (void)classify;
    return stats.minv[0] == 1000 && stats.minv[1] == 0 && stats.K == 14 &&
           center(stats, 0, 1000, 2000) == 1500 ? 0 : 1;
}
