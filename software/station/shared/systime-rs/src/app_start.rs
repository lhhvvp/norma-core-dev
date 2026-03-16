use std::time::{SystemTime, UNIX_EPOCH};

use once_cell::sync::Lazy;

static APP_START_TIME: Lazy<u64> = Lazy::new(generate_app_start_id);

fn generate_app_start_id() -> u64 {
    SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs()
}

pub fn get_app_start_id() -> u64 {
    *APP_START_TIME
}