use crate::Clock;

use crate::Event;

/// Hand each event to `f` at its `log_time` on `clock`. Pass events in time order
/// (`Log::by_time`) and a `ReplayClock` started at the first one's `log_time`.
pub fn replay<'a>(events: impl IntoIterator<Item = &'a Event>, clock: &dyn Clock, mut f: impl FnMut(&Event)) {
    for e in events {
        clock.sleep_until(e.log_time);
        f(e);
    }
}
