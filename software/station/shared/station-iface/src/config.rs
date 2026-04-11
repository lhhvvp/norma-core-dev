use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Config {
    pub drivers: Drivers,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub inference: Option<Vec<Inference>>,

    #[serde(rename = "cloud-offload", skip_serializing_if = "Option::is_none")]
    pub cloud_offload: Option<CloudOffloadConfig>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            drivers: Drivers::default(),
            inference: Some(vec![Inference::default_normvla()]),
            cloud_offload: None,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CloudOffloadConfig {
    /// Cloud storage bucket name
    pub bucket: String,

    /// Region (e.g., "us-east-1")
    pub region: String,

    /// Access key ID
    pub access_key_id: String,

    /// Secret access key
    pub secret_access_key: String,

    /// Optional endpoint URL for S3-compatible services (e.g., MinIO)
    pub endpoint: Option<String>,
}


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Drivers {
    /// ST3215 servo bus configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub st3215: Option<St3215Config>,

    /// Enable or disable system info monitoring
    #[serde(rename = "system-info")]
    pub system_info: bool,

    #[serde(rename = "usb-video", skip_serializing_if = "Option::is_none")]
    pub usb_video: Option<UsbVideoConfig>,
}

/// ST3215 servo bus configuration
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct St3215Config {
    /// Enable or disable the ST3215 driver
    #[serde(default = "default_st3215_enabled")]
    pub enabled: bool,

    /// Default current threshold for mirroring. When target motor's current exceeds this,
    /// set goal to current position to prevent overload. 0 means disabled. Default is 100.
    #[serde(rename = "current-threshold", default = "default_current_threshold")]
    pub current_threshold: u16,

    /// Per-motor current threshold overrides. Key is motor ID (0-255).
    /// Optional - if not specified, all motors use the default current-threshold.
    /// Example in YAML:
    /// ```yaml
    /// motor-current-thresholds:
    ///   8: 40   # Motor 8 has stricter limit
    ///   5: 60   # Motor 5 has more relaxed limit
    /// ```
    #[serde(rename = "motor-current-thresholds", default, skip_serializing_if = "Option::is_none")]
    pub motor_current_thresholds: Option<std::collections::HashMap<u8, u16>>,

    /// Deadband for mirroring. Minimum distance between current position
    /// and goal to trigger movement. Default is 20.
    #[serde(default = "default_deadband")]
    pub deadband: u16,
}

fn default_st3215_enabled() -> bool {
    true
}

fn default_current_threshold() -> u16 {
    100
}

fn default_deadband() -> u16 {
    20
}

impl Default for St3215Config {
    fn default() -> Self {
        Self {
            enabled: true,
            current_threshold: 100,
            motor_current_thresholds: None,
            deadband: 20,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Inference {
    /// Queue ID for inference data (e.g., "inference/normvla")
    #[serde(rename = "queue-id")]
    pub queue_id: String,

    /// Shared memory path (e.g., "/var/run/normvla")
    pub shm: PathBuf,

    /// Shared memory size in megabytes (e.g., 12 for 12MB)
    #[serde(rename = "shm-size-mb")]
    pub shm_size_mb: u64,

    /// Output format (e.g., "normvla")
    pub format: String,

    /// ST3215 bus identifier (e.g., "5AB9068587" or "auto")
    /// Default: "auto" (automatically selects the single bus with torque enabled)
    #[serde(rename = "st3215-bus", default = "default_st3215_bus")]
    pub st3215_bus: String,

    /// Update interval for publishing (e.g., "100ms")
    #[serde(rename = "update-interval", with = "humantime_serde", default = "default_update_interval")]
    pub update_interval: std::time::Duration,
}

fn default_update_interval() -> std::time::Duration {
    std::time::Duration::from_millis(100)
}

fn default_st3215_bus() -> String {
    "auto".to_string()
}

impl Inference {
    /// Create a default normvla inference configuration
    pub fn default_normvla() -> Self {
        // Use OS-appropriate path: /dev/shm for Linux (tmpfs, world-writable), /tmp for macOS
        let shm_path = if cfg!(target_os = "linux") {
            PathBuf::from("/dev/shm/normvla")
        } else {
            PathBuf::from("/tmp/normvla")
        };

        Self {
            queue_id: "inference/normvla".to_string(),
            shm: shm_path,
            shm_size_mb: 12,
            format: "normvla".to_string(),
            st3215_bus: "auto".to_string(),
            update_interval: default_update_interval(),
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct UsbVideoConfig {
    pub enabled: bool,
    /// Target size for resizing frames (shortest dimension). Default: 224
    /// Set to 0 to disable resizing.
    #[serde(default = "default_resize_target")]
    pub resize_target: u32,
}

fn default_resize_target() -> u32 {
    224
}

impl Default for UsbVideoConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            resize_target: 224,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct HikvisionConfig {
    /// List of RTSP URLs for Hikvision cameras
    pub rtsp: Vec<String>,
}

impl Default for Drivers {
    fn default() -> Self {
        Self {
            st3215: Some(St3215Config::default()),
            system_info: true,
            usb_video: Some(UsbVideoConfig::default()),
        }
    }
}

impl Config {
    /// Load configuration from a YAML file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(path)?;
        let config: Config = serde_yaml::from_str(&contents)?;
        Ok(config)
    }

    /// Save configuration to a YAML file
    pub fn to_file<P: AsRef<Path>>(&self, path: P) -> Result<(), Box<dyn std::error::Error>> {
        let yaml = serde_yaml::to_string(self)?;
        std::fs::write(path, yaml)?;
        Ok(())
    }

    /// Load configuration from file or create default if file doesn't exist
    pub fn load_or_default<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let path = path.as_ref();
        if path.exists() {
            Self::from_file(path)
        } else {
            let config = Self::default();
            config.to_file(path)?;
            Ok(config)
        }
    }
}

// ---------------------------------------------------------------------------
// SimulationRuntime subsystem configuration
// ---------------------------------------------------------------------------
//
// These types live in station_iface (not sim-runtime) to avoid a circular
// dependency: sim-runtime depends on station_iface for the StationEngine
// trait, so station_iface cannot in turn depend on sim-runtime to embed
// config types in the top-level Config struct. The "config-layer" crate
// hosts them; sim-runtime re-exports them for convenience.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SimMode {
    Internal,
    External,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogCapture {
    #[default]
    File,
    Inherit,
    Null,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimRuntimeConfig {
    pub enabled: bool,
    pub mode: SimMode,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub launcher: Option<Vec<String>>,

    #[serde(rename = "socket-path", default, skip_serializing_if = "Option::is_none")]
    pub socket_path: Option<PathBuf>,

    #[serde(rename = "runtime-dir", default, skip_serializing_if = "Option::is_none")]
    pub runtime_dir: Option<PathBuf>,

    #[serde(rename = "startup-timeout-ms", default = "default_sim_startup_timeout")]
    pub startup_timeout_ms: u64,

    #[serde(rename = "shutdown-timeout-ms", default = "default_sim_shutdown_timeout")]
    pub shutdown_timeout_ms: u64,

    #[serde(rename = "log-capture", default)]
    pub log_capture: LogCapture,

    #[serde(rename = "log-file", default, skip_serializing_if = "Option::is_none")]
    pub log_file: Option<PathBuf>,
}

fn default_sim_startup_timeout() -> u64 {
    5000
}

fn default_sim_shutdown_timeout() -> u64 {
    2000
}

impl SimRuntimeConfig {
    /// Validate the config in isolation (no cross-field mutual-exclusion
    /// rules — those would re-introduce the sim vs real-driver coupling
    /// the v2 architecture explicitly rejected). Returns a static message
    /// for each individual-field precondition.
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.enabled {
            return Ok(());
        }
        match self.mode {
            SimMode::Internal => {
                if self.launcher.is_none() {
                    return Err("sim-runtime.mode=internal requires launcher");
                }
            }
            SimMode::External => {
                if self.socket_path.is_none() {
                    return Err("sim-runtime.mode=external requires socket-path");
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod sim_runtime_config_tests {
    use super::*;

    fn base() -> SimRuntimeConfig {
        SimRuntimeConfig {
            enabled: true,
            mode: SimMode::Internal,
            launcher: None,
            socket_path: None,
            runtime_dir: None,
            startup_timeout_ms: 5000,
            shutdown_timeout_ms: 2000,
            log_capture: LogCapture::File,
            log_file: None,
        }
    }

    #[test]
    fn test_validate_disabled_is_ok() {
        let mut c = base();
        c.enabled = false;
        assert!(c.validate().is_ok());
    }

    #[test]
    fn test_validate_internal_requires_launcher() {
        let c = base();
        assert_eq!(
            c.validate(),
            Err("sim-runtime.mode=internal requires launcher")
        );

        let mut c2 = base();
        c2.launcher = Some(vec!["python3".into(), "-m".into(), "norma_sim".into()]);
        assert!(c2.validate().is_ok());
    }

    #[test]
    fn test_validate_external_requires_socket_path() {
        let mut c = base();
        c.mode = SimMode::External;
        assert_eq!(
            c.validate(),
            Err("sim-runtime.mode=external requires socket-path")
        );

        let mut c2 = c.clone();
        c2.socket_path = Some(PathBuf::from("/tmp/sim.sock"));
        assert!(c2.validate().is_ok());
    }
}