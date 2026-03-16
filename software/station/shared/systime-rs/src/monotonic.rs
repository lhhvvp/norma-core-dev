#[cfg(target_os = "linux")]
use libc::{CLOCK_BOOTTIME, clock_gettime, timespec};

#[cfg(target_os = "macos")]
use libc::{CLOCK_MONOTONIC_RAW, clock_gettime, timespec};

pub fn get_monotonic_stamp_ns() -> u64 {
    #[cfg(target_os = "linux")]
    {
        unsafe {
            let mut ts = timespec {
                tv_sec: 0,
                tv_nsec: 0,
            };

            if clock_gettime(CLOCK_BOOTTIME, &mut ts as *mut _) != 0 {
                log::warn!(target: "systime", "clock_gettime failed: {}", std::io::Error::last_os_error());
                return 0;
            }

            (ts.tv_sec as u64)
                .saturating_mul(1_000_000_000)
                .saturating_add(ts.tv_nsec as u64)
        }
    }

    #[cfg(target_os = "macos")]
    {
        unsafe {
            let mut ts = timespec {
                tv_sec: 0,
                tv_nsec: 0,
            };

            if clock_gettime(CLOCK_MONOTONIC_RAW, &mut ts as *mut _) != 0 {
                log::warn!(target: "systime", "clock_gettime failed: {}", std::io::Error::last_os_error());
                return 0;
            }

            (ts.tv_sec as u64)
                .saturating_mul(1_000_000_000)
                .saturating_add(ts.tv_nsec as u64)
        }
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        panic!("get_monotonic_stamp() is not supported on this platform");
    }
}