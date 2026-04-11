//! `TempRuntimeDir` — RAII for the per-session runtime directory.
//!
//! A single temp directory (`/tmp/norma-sim-<pid>` by default, or a
//! caller-supplied path) holds the UDS socket the Station and sim
//! backend share. The directory is mode 0700 so only the launching
//! user can inspect it. `Drop` removes the whole tree.
//!
//! Replaces the v1 spec's sentinel-file + lockfile + stale-socket
//! state machine with the simpler "socket presence == readiness"
//! contract.

use std::fs;
use std::path::{Path, PathBuf};

pub(crate) struct TempRuntimeDir {
    path: PathBuf,
}

impl TempRuntimeDir {
    pub fn create(parent: Option<&Path>, station_pid: u32) -> std::io::Result<Self> {
        let path = match parent {
            Some(p) => p.to_path_buf(),
            None => PathBuf::from(format!("/tmp/norma-sim-{}", station_pid)),
        };
        fs::create_dir_all(&path)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&path, fs::Permissions::from_mode(0o700))?;
        }
        Ok(Self { path })
    }

    pub fn socket_path(&self) -> PathBuf {
        self.path.join("sim.sock")
    }

    #[allow(dead_code)] // exposed for MVP-2 diagnostics (e.g. /sim/health)
    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TempRuntimeDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_and_drop_removes_dir() {
        let dir = TempRuntimeDir::create(None, 999_999).unwrap();
        let p = dir.path().to_path_buf();
        assert!(p.exists());
        let sock = dir.socket_path();
        assert!(sock.starts_with(&p));
        assert_eq!(sock.file_name().unwrap(), "sim.sock");
        drop(dir);
        assert!(!p.exists(), "drop should remove runtime dir tree");
    }

    #[test]
    fn test_create_with_custom_parent() {
        let tmp = tempfile::tempdir().unwrap();
        let custom = tmp.path().join("sim-dir");
        let dir = TempRuntimeDir::create(Some(&custom), 0).unwrap();
        assert_eq!(dir.path(), custom.as_path());
        assert!(custom.exists());
    }

    #[cfg(unix)]
    #[test]
    fn test_permissions_0o700() {
        use std::os::unix::fs::PermissionsExt;
        let dir = TempRuntimeDir::create(None, 999_998).unwrap();
        let meta = fs::metadata(dir.path()).unwrap();
        let mode = meta.permissions().mode() & 0o777;
        assert_eq!(mode, 0o700, "runtime dir must be user-private");
    }
}
