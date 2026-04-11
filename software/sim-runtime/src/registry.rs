//! `WorldRegistry` — in-memory index of the WorldDescriptor returned
//! by the backend's handshake. Used by the runtime and downstream
//! bridges to look up robots and actuators by ID.
//!
//! MVP-1 stores the registry in `SimulationRuntime` but does not yet
//! expose it; the `#[allow(dead_code)]` attributes below are intentional
//! — the fields and accessors will be wired up when MVP-2 adds the
//! `SimulationRuntime::registry()` public getter consumed by compat
//! bridges that do not ship their own preset file. Removing them now
//! would delete scaffolding we are about to use.

use crate::proto::{RobotDescriptor, WorldDescriptor};
use std::collections::HashMap;

#[allow(dead_code)]
pub(crate) struct WorldRegistry {
    pub world_name: String,
    pub robots: HashMap<String, RobotDescriptor>,
}

impl WorldRegistry {
    pub fn from_descriptor(desc: &WorldDescriptor) -> Self {
        let mut robots = HashMap::new();
        for r in &desc.robots {
            robots.insert(r.robot_id.clone(), r.clone());
        }
        Self {
            world_name: desc.world_name.clone(),
            robots,
        }
    }

    #[allow(dead_code)]
    pub fn robot(&self, id: &str) -> Option<&RobotDescriptor> {
        self.robots.get(id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_registry_from_descriptor() {
        let desc = WorldDescriptor {
            world_name: "test".into(),
            robots: vec![RobotDescriptor {
                robot_id: "robot1".into(),
                actuators: vec![],
                sensors: vec![],
            }],
            initial_clock: None,
            publish_hz: 100,
            physics_hz: 500,
        };
        let reg = WorldRegistry::from_descriptor(&desc);
        assert_eq!(reg.world_name, "test");
        assert!(reg.robot("robot1").is_some());
        assert!(reg.robot("nonexistent").is_none());
    }

    #[test]
    fn test_registry_multi_robot() {
        let desc = WorldDescriptor {
            world_name: "two".into(),
            robots: vec![
                RobotDescriptor {
                    robot_id: "a".into(),
                    actuators: vec![],
                    sensors: vec![],
                },
                RobotDescriptor {
                    robot_id: "b".into(),
                    actuators: vec![],
                    sensors: vec![],
                },
            ],
            initial_clock: None,
            publish_hz: 100,
            physics_hz: 500,
        };
        let reg = WorldRegistry::from_descriptor(&desc);
        assert_eq!(reg.robots.len(), 2);
        assert!(reg.robot("a").is_some());
        assert!(reg.robot("b").is_some());
    }
}
