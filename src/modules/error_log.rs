//! Light-weight “flight recorder” that queues the last `ERR_CAP` log lines.
//! Use `log_info!` or `log_err!` instead of `defmt::info!/error!` when you
//! also want the message forwarded to the cloud.

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::Channel,
};
use static_cell::StaticCell;
use core::sync::atomic::{AtomicPtr, Ordering};
use defmt::Format;
use crate::modules::config::ERR_CAP;

/* ---------- data types ---------- */

#[derive(Clone, Copy, Format)]
pub enum LogLevel {
    Debug  = 0,
    Info   = 1,
    Error  = 2,
    HwRev = 3,
    FwRev = 4,
    BuildUnix = 5,
    GitHash = 6,
    Status = 7,
}


#[derive(Clone, Copy, Format)]
pub struct LogLine {
    pub ts_ms: u64,
    pub lvl:   LogLevel,
    pub msg:   &'static str,
}

/* ---------- storage ---------- */

type LogChan = Channel<NoopRawMutex, LogLine, ERR_CAP>;

static LOG_CELL: StaticCell<LogChan> = StaticCell::new();
static LOG_PTR : AtomicPtr<LogChan>  = AtomicPtr::new(core::ptr::null_mut());

/* ---------- public API ---------- */

/// Call once from `main()` right after `embassy_nrf::init`.
pub fn init() {
    let r = LOG_CELL.init(Channel::new());
    LOG_PTR.store(r as *const _ as *mut _, Ordering::Relaxed);
}

/// Obtain a *receiver* so another task can drain & transmit the queue.
pub fn receiver<'a>() -> embassy_sync::channel::Receiver<
    'static, NoopRawMutex, LogLine, ERR_CAP
> {
    channel().receiver()
}

/* ---------- internal helpers ---------- */

#[inline(always)]
fn channel() -> &'static LogChan {
    let ptr = LOG_PTR.load(Ordering::Relaxed);
    debug_assert!(!ptr.is_null(), "error_log::init() not called");
    unsafe { &*ptr }
}

#[inline(always)]
fn push_inner(lvl: LogLevel, msg: &'static str) {
    let _ = channel().try_send(LogLine {
        ts_ms: embassy_time::Instant::now().as_millis(),
        lvl,
        msg,
    });
}

/* ---------- wrappers for defmt ---------- */

pub fn push_debug (msg: &'static str) { push_inner(LogLevel::Debug,  msg); }
pub fn push_info  (msg: &'static str) { push_inner(LogLevel::Info,   msg); }
pub fn push_err   (msg: &'static str) { push_inner(LogLevel::Error,  msg); }
pub fn push_level(lvl: LogLevel, msg: &'static str) { push_inner(lvl, msg);}

#[macro_export]
macro_rules! log_dbg {
    ($msg:literal $(, $arg:expr)* $(,)?) => {{
        defmt::debug!($msg $(, $arg)*);
        $crate::modules::error_log::push_debug($msg);
    }};
}

#[macro_export]
macro_rules! log_info {
    ($msg:literal $(, $arg:expr)* $(,)?) => {{
        defmt::info!($msg $(, $arg)*);
        $crate::modules::error_log::push_info($msg);
    }};
}

#[macro_export]
macro_rules! log_err {
    ($msg:literal $(, $arg:expr)* $(,)?) => {{
        defmt::error!($msg $(, $arg)*);
        $crate::modules::error_log::push_err($msg);
    }};
}

#[macro_export]
macro_rules! log_param {
    // $lvl must be one of the enum variants without the prefix
    ($lvl:ident, $msg:expr $(,)?) => {{
        use $crate::modules::error_log::{LogLevel, push_level};
        // queue for cloud upload
        push_level(LogLevel::$lvl, $msg);
    }};
}