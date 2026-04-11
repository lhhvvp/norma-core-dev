// Task 2.1 stub — replaced verbatim from st3215/src/protocol/memory.rs in Task 2.2.
// IMPORTANT: NO #[repr(u16)] — the real memory.rs uses a define_register_enum!
// macro. Call sites (Chunk 7 Task 7.5) use `.address()` not `as u16`.

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EepromRegister {
    _Placeholder,
}

#[allow(dead_code)]
impl EepromRegister {
    pub const fn address(&self) -> u8 {
        0
    }
    pub const fn size(&self) -> u8 {
        0
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RamRegister {
    _Placeholder,
}

#[allow(dead_code)]
impl RamRegister {
    pub const fn address(&self) -> u8 {
        0
    }
    pub const fn size(&self) -> u8 {
        0
    }
}
