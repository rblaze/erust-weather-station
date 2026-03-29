use fugit::{KilohertzU32, TimerDuration, TimerInstant};
use futures::{Future, FutureExt, select_biased};

// TODO: check if it is possible to import LSI_FREQ from HAL
const LSI_FREQ: KilohertzU32 = KilohertzU32::kHz(32);
const TIMER_FREQ: u32 = LSI_FREQ.to_Hz() / 128;

pub type TimerTicks = u64;
pub type Instant = TimerInstant<TimerTicks, TIMER_FREQ>;
pub type Duration = TimerDuration<TimerTicks, TIMER_FREQ>;

/// Runs provided future with timeout.
/// Returns `Some(x)` if future completes, None if timeout occurs.
pub async fn timeout<T, E, F>(duration: Duration, f: F) -> Result<Option<T>, E>
where
    F: Future<Output = Result<T, E>>,
{
    select_biased! {
        ret = f.fuse() => ret.map(|v| Some(v)),
        _ = sleep(duration).fuse() => Ok(None),
    }
}

/// Sleeps for the specified duration.
/// Clamps at Duration::MAX for u64->i64 conversion.
pub async fn sleep(duration: Duration) {
    async_scheduler::sleep(duration.ticks().try_into().map_or(
        async_scheduler::time::Duration::MAX,
        async_scheduler::time::Duration::new,
    ))
    .await;
}

/// Returns current time.
pub async fn now() -> Instant {
    Instant::from_ticks(async_scheduler::now().await.ticks().clamp(0, i64::MAX) as u64)
}
