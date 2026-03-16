use crate::usbvideo_proto::usbvideo;

pub fn fourcc_from_u32(fourcc: u32) -> [u8; 4] {
    fourcc.to_be_bytes()
}

pub fn fourcc_to_string(format: &[u8; 4]) -> String {
    String::from_utf8_lossy(format).into()
}

pub fn filter_and_sort_cameras_formats(
    formats: &[usbvideo::CameraFormat]
) -> Vec<usbvideo::CameraFormat> {
    let mut suitable_formats: Vec<usbvideo::CameraFormat> = formats
        .iter()
        .filter(|format| {
            let fourcc = format.fourcc;
            FourCCFormat::from_fourcc_u32(fourcc).is_some()
        })
        .cloned()
        .collect();

    suitable_formats.sort_by(|a, b| {
        let a_fourcc = FourCCFormat::from_fourcc_u32(a.fourcc).unwrap();
        let b_fourcc = FourCCFormat::from_fourcc_u32(b.fourcc).unwrap();
        
        let a_is_mjpeg = a_fourcc == FourCCFormat::Mjpeg;
        let b_is_mjpeg = b_fourcc == FourCCFormat::Mjpeg;

        // First priority: MJPEG format
        match (a_is_mjpeg, b_is_mjpeg) {
            (true, false) => return std::cmp::Ordering::Less,
            (false, true) => return std::cmp::Ordering::Greater,
            _ => {}
        }

        // Second priority: Maximum framerate (descending order)
        match b.frames_per_second.partial_cmp(&a.frames_per_second) {
            Some(std::cmp::Ordering::Equal) | None => {}
            Some(other) => return other,
        }

        // Third priority: Resolution (ascending order - prefer lower resolution)
        let a_resolution = a.width as u64 * a.height as u64;
        let b_resolution = b.width as u64 * b.height as u64;
        a_resolution.cmp(&b_resolution)
    });

    suitable_formats
}

/// Supported FourCC formats for conversion
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FourCCFormat {
    /// YUV 4:2:2 format (YUYV)
    Yuv2,
    /// YUV 4:2:0 format (three-plane: I420, YV12)
    Yuv420,
    /// Apple's YUV 4:2:0 format (two-plane: 420v)
    Yuv420v,
    
    /// RGB - Red, Green, Blue (3 bytes per pixel)
    Rgb,
    /// BGR - Blue, Green, Red (3 bytes per pixel)
    Bgr,
    /// RGBA - Red, Green, Blue, Alpha (4 bytes per pixel)
    Rgba,
    /// BGRA - Blue, Green, Red, Alpha (4 bytes per pixel)
    Bgra,
    /// ARGB - Alpha, Red, Green, Blue (4 bytes per pixel)
    Argb,
    /// ABGR - Alpha, Blue, Green, Red (4 bytes per pixel)
    Abgr,
    
    /// MJPEG - Motion JPEG (compressed JPEG frames)
    Mjpeg,
}

impl FourCCFormat {
    /// Create from FourCC bytes
    pub fn from_fourcc(fourcc: &[u8; 4]) -> Option<Self> {
        match fourcc {
            b"YUY2" | b"YUYV" => Some(FourCCFormat::Yuv2),
            b"420v" => Some(FourCCFormat::Yuv420v),
            b"I420" | b"YV12" => Some(FourCCFormat::Yuv420),
            b"RGB3" | b"RGB " => Some(FourCCFormat::Rgb),
            b"BGR3" | b"BGR " => Some(FourCCFormat::Bgr),
            b"RGBA" => Some(FourCCFormat::Rgba),
            b"BGRA" => Some(FourCCFormat::Bgra),
            b"ARGB" => Some(FourCCFormat::Argb),
            b"ABGR" => Some(FourCCFormat::Abgr),
            b"MJPG" | b"JPEG" => Some(FourCCFormat::Mjpeg),
            _ => None,
        }
    }

    /// Create from FourCC u32 (big-endian)
    pub fn from_fourcc_u32(fourcc: u32) -> Option<Self> {
        let bytes = fourcc.to_be_bytes();
        Self::from_fourcc(&bytes)
    }

    /// Get the FourCC string representation
    pub fn as_str(&self) -> &'static str {
        match self {
            FourCCFormat::Yuv2 => "YUY2",
            FourCCFormat::Yuv420 => "I420",
            FourCCFormat::Yuv420v => "420v",
            FourCCFormat::Rgb => "RGB3",
            FourCCFormat::Bgr => "BGR3",
            FourCCFormat::Rgba => "RGBA",
            FourCCFormat::Bgra => "BGRA",
            FourCCFormat::Argb => "ARGB",
            FourCCFormat::Abgr => "ABGR",
            FourCCFormat::Mjpeg => "MJPG",
        }
    }
}