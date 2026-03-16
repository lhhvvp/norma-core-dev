use std::time::SystemTime;

pub fn get_local_stamp_ns() -> u64 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .map_err(|err| {
            log::warn!(target: "systime", "SystemTime before UNIX EPOCH: {}", err);
            err
        })
        .unwrap_or(0)
}