use contracts::Clock;

use crate::Envelope;

/// Hand each envelope to `f` at its `log_time` on `clock`. Pass envelopes in time order
/// (`Log::by_time`) and a `ReplayClock` started at the first one's `log_time`.
pub fn replay<'a>(envelopes: impl IntoIterator<Item = &'a Envelope>, clock: &dyn Clock, mut f: impl FnMut(&Envelope)) {
    for e in envelopes {
        clock.sleep_until(e.log_time);
        f(e);
    }
}
