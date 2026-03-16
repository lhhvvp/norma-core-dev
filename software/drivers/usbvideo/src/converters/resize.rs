use bytes::{Bytes, BytesMut};

/// Calculate new dimensions preserving aspect ratio to fit within target_size
pub fn calculate_resize_dimensions(width: u32, height: u32, target_size: u32) -> (u32, u32) {
    if width == 0 || height == 0 {
        return (target_size, target_size);
    }

    // If already smaller or equal, keep original
    if width <= target_size && height <= target_size {
        return (width, height);
    }

    let aspect_ratio = width as f32 / height as f32;

    let (new_width, new_height) = if width < height {
        // Portrait: width is the shortest dimension
        let new_width = target_size;
        let new_height = (target_size as f32 / aspect_ratio).round() as u32;
        (new_width, new_height)
    } else {
        // Landscape or square: height is the shortest dimension (or equal)
        let new_height = target_size;
        let new_width = (target_size as f32 * aspect_ratio).round() as u32;
        (new_width, new_height)
    };

    (new_width.max(1), new_height.max(1))
}

/// Resize RGB image using bilinear interpolation
/// Input: RGB24 data (3 bytes per pixel, width * height * 3 bytes total)
/// Output: Resized RGB24 data (new_width * new_height * 3 bytes)
pub fn resize_rgb_bilinear(
    data: &Bytes,
    src_width: u32,
    src_height: u32,
    dst_width: u32,
    dst_height: u32,
) -> Bytes {
    // If dimensions match, no resize needed
    if src_width == dst_width && src_height == dst_height {
        return data.clone();
    }

    let src_width = src_width as usize;
    let src_height = src_height as usize;
    let dst_width = dst_width as usize;
    let dst_height = dst_height as usize;

    let mut output = BytesMut::with_capacity(dst_width * dst_height * 3);
    output.resize(dst_width * dst_height * 3, 0);

    let x_ratio = (src_width - 1) as f32 / dst_width as f32;
    let y_ratio = (src_height - 1) as f32 / dst_height as f32;

    for dst_y in 0..dst_height {
        for dst_x in 0..dst_width {
            let src_x = dst_x as f32 * x_ratio;
            let src_y = dst_y as f32 * y_ratio;

            let x0 = src_x.floor() as usize;
            let y0 = src_y.floor() as usize;
            let x1 = (x0 + 1).min(src_width - 1);
            let y1 = (y0 + 1).min(src_height - 1);

            let dx = src_x - x0 as f32;
            let dy = src_y - y0 as f32;

            // Bilinear interpolation for each RGB channel
            for c in 0..3 {
                let p00 = data[(y0 * src_width + x0) * 3 + c] as f32;
                let p10 = data[(y0 * src_width + x1) * 3 + c] as f32;
                let p01 = data[(y1 * src_width + x0) * 3 + c] as f32;
                let p11 = data[(y1 * src_width + x1) * 3 + c] as f32;

                let interpolated = (p00 * (1.0 - dx) * (1.0 - dy)
                    + p10 * dx * (1.0 - dy)
                    + p01 * (1.0 - dx) * dy
                    + p11 * dx * dy)
                    .round()
                    .clamp(0.0, 255.0) as u8;

                output[(dst_y * dst_width + dst_x) * 3 + c] = interpolated;
            }
        }
    }

    output.freeze()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_resize_dimensions_landscape() {
        // 640x480 -> shortest dimension (height=480) should be 224
        let (w, h) = calculate_resize_dimensions(640, 480, 224);
        assert_eq!(h, 224);
        assert_eq!(w, 299); // 224 * (640/480) = 299
    }

    #[test]
    fn test_calculate_resize_dimensions_portrait() {
        // 480x640 -> shortest dimension (width=480) should be 224
        let (w, h) = calculate_resize_dimensions(480, 640, 224);
        assert_eq!(w, 224);
        assert_eq!(h, 299); // 224 / (480/640) = 299
    }

    #[test]
    fn test_calculate_resize_dimensions_square() {
        // 640x640 -> should be 224x224
        let (w, h) = calculate_resize_dimensions(640, 640, 224);
        assert_eq!(w, 224);
        assert_eq!(h, 224);
    }

    #[test]
    fn test_calculate_resize_dimensions_already_small() {
        // 200x100 -> should stay same (already smaller than 224)
        let (w, h) = calculate_resize_dimensions(200, 100, 224);
        assert_eq!(w, 200);
        assert_eq!(h, 100);
    }

    #[test]
    fn test_resize_rgb_same_size() {
        let data = Bytes::from(vec![255u8; 100 * 100 * 3]);
        let resized = resize_rgb_bilinear(&data, 100, 100, 100, 100);
        assert_eq!(resized.len(), 100 * 100 * 3);
    }

    #[test]
    fn test_resize_rgb_downscale() {
        // Create a simple 4x4 image
        let data = Bytes::from(vec![128u8; 4 * 4 * 3]);
        let resized = resize_rgb_bilinear(&data, 4, 4, 2, 2);
        assert_eq!(resized.len(), 2 * 2 * 3);
        // All values should be close to 128
        for &byte in resized.iter() {
            assert!((byte as i16 - 128).abs() < 5);
        }
    }
}
