mod monotonic;
mod local;
mod app_start;

pub use monotonic::get_monotonic_stamp_ns;
pub use local::get_local_stamp_ns;
pub use app_start::get_app_start_id;