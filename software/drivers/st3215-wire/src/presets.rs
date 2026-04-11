// Task 2.1 stub — replaced in Task 2.6 with verified values.

#[derive(Debug, Clone)]
pub struct MotorModelSpec {
    pub model_number: u16,
    pub firmware_version: u8,
    pub baud_rate_code: u8,
    pub steps_per_rev: u32,
}

pub const ST3215_STANDARD: MotorModelSpec = MotorModelSpec {
    model_number: 777,
    firmware_version: 10,
    baud_rate_code: 0,
    steps_per_rev: 4096,
};
