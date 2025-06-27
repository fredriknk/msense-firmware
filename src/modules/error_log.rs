use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::Channel,
};
use static_cell::StaticCell;
use core::sync::atomic::{AtomicPtr, Ordering};
use defmt::Format;

/* ---------- configuration ---------- */

pub const ERR_CAP: usize = 32;

#[derive(Clone, Copy, Format)]
pub struct ErrLine {
    pub ts_ms: u64,
    pub msg:   &'static str,
}

/* ---------- storage ---------- */

// 1. real storage: placed once in Flash / RAM
static ERR_CELL: StaticCell<
    Channel<NoopRawMutex, ErrLine, ERR_CAP>
> = StaticCell::new();

// 2. pointer we can read later (zero-cost once initialised)
static ERR_PTR: AtomicPtr<Channel<NoopRawMutex, ErrLine, ERR_CAP>> =
    AtomicPtr::new(core::ptr::null_mut());

/* ---------- public API ---------- */

/// Call once from `main()` very early.
pub fn init() {
    let chan_ref = ERR_CELL.init(Channel::new());
    ERR_PTR.store(chan_ref as *const _ as *mut _, Ordering::Relaxed);
}

/// Push one error line; drops if queue full.
pub fn push(msg: &'static str) {
    let chan = channel();
    let _ = chan.try_send(ErrLine {
        ts_ms: embassy_time::Instant::now().as_millis(),
        msg,
    });
}

/// Try pop (non-blocking). Returns `None` if empty.
pub fn try_recv() -> Option<ErrLine> {
    channel().try_receive().ok()
}

/* ---------- helper ---------- */

#[inline(always)]
fn channel() -> &'static Channel<NoopRawMutex, ErrLine, ERR_CAP> {
    // Safety: set exactly once by `init()`, never modified thereafter
    let ptr = ERR_PTR.load(Ordering::Relaxed);
    debug_assert!(!ptr.is_null(), "error_log::init() not called");
    unsafe { &*ptr }
}
