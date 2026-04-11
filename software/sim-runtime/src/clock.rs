//! Clock helpers for the simulation runtime.
//!
//! `world_tick` is the canonical sim-time index; `sim_time_ns` is derived
//! from it by multiplying the physics timestep. `wall_ns` is the Station's
//! own `CLOCK_MONOTONIC`-equivalent reference — cross-process comparison
//! with the sim backend's wall clock is deliberately forbidden by the
//! architecture.

use uuid::Uuid;

pub fn new_session_id() -> String {
    Uuid::new_v4().to_string()
}

pub fn world_tick_to_sim_time_ns(tick: u64, timestep_ns: u64) -> u64 {
    tick.saturating_mul(timestep_ns)
}

pub fn current_wall_ns() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_session_id_unique() {
        let a = new_session_id();
        let b = new_session_id();
        assert_ne!(a, b);
    }

    #[test]
    fn test_tick_to_sim_time() {
        assert_eq!(world_tick_to_sim_time_ns(0, 2_000_000), 0);
        assert_eq!(world_tick_to_sim_time_ns(5, 2_000_000), 10_000_000);
    }
}
