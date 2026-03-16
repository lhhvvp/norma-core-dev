use std::{sync::Arc, path::PathBuf};
use station_iface::StationEngine;
use normfs::NormFS;

pub mod usbvideo_proto {
    pub mod frame {
        include!("proto/frame.rs");
    }
    pub mod usbvideo {
        include!("proto/usbvideo.rs");
    }
}

mod converters;
mod state;

pub mod pipeline;

// Re-export resize and jpeg conversion functions
pub use converters::{resize_rgb_bilinear, calculate_resize_dimensions};
pub use converters::mjpeg::{convert_mjpeg_to_rgb, convert_rgb_to_jpeg};

#[cfg(target_os = "macos")]
pub mod osx;

#[cfg(target_os = "linux")]
pub mod linux;

#[derive(Debug, Clone)]
pub struct USBVideoConfig {
    /// Target size for resizing frames (shortest dimension). Default: 224
    /// Set to 0 to disable resizing.
    pub resize_target: u32,
}

impl Default for USBVideoConfig {
    fn default() -> Self {
        Self {
            resize_target: 224,
        }
    }
}

#[cfg(target_os = "macos")]
pub async fn start_usbvideo<T: StationEngine>(
    normfs: Arc<NormFS>,
    station_engine: Arc<T>,
    base_path: PathBuf,
    config: USBVideoConfig,
) -> Arc<pipeline::USBVideoManager<osx::CameraMacDriver>>{
    Arc::new(
        pipeline::USBVideoManager::new(
            osx::CameraMacDriver::new(),
            normfs,
            station_engine,
            base_path,
            config,
        ).await
    )
}

#[cfg(target_os = "linux")]
pub async fn start_usbvideo<T: StationEngine>(
    normfs: Arc<NormFS>,
    station_engine: Arc<T>,
    base_path: PathBuf,
    config: USBVideoConfig,
) -> Arc<pipeline::USBVideoManager<linux::CameraLinuxDriver>>{
    Arc::new(
        pipeline::USBVideoManager::new(
            linux::CameraLinuxDriver::new(),
            normfs,
            station_engine,
            base_path,
            config,
        ).await
    )
}

/// Process main run loop briefly to handle AVFoundation notifications (macOS only)
/// Call this periodically (e.g. every 100ms) from your main thread
#[cfg(target_os = "macos")]
pub fn process_main_run_loop() {
    osx::process_main_run_loop();
}

#[cfg(not(target_os = "macos"))]
pub fn process_main_run_loop() {
    // No-op on non-macOS platforms
}
