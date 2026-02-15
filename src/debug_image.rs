use crate::solid::Solid;
use std::fs::File;
use std::io::BufWriter;

/// Write a cross-sectional PNG of an SDF on the XY plane (z=0) to "output.png".
/// Covers [-2, 2] x [-2, 2] at 512x512 pixels.
pub fn write_sdf_cross_section(solid: &dyn Solid) {
    write_sdf_image(solid, "output.png", 512, 2.0, 0.0);
}

/// Write a cross-sectional PNG with configurable parameters.
pub fn write_sdf_image(solid: &dyn Solid, path: &str, resolution: usize, half_extent: f32, z: f32) {
    let w = resolution;
    let h = resolution;
    let pixel_size = 2.0 * half_extent / w as f32;
    let contour_spacing = half_extent / 10.0;

    let mut pixels = vec![0u8; w * h * 3];

    for row in 0..h {
        for col in 0..w {
            let x = -half_extent + (col as f32 + 0.5) * pixel_size;
            let y = half_extent - (row as f32 + 0.5) * pixel_size;
            let d = solid.sdf([x, y, z]);

            // Surface highlight: white line where |d| < ~1 pixel
            let (r, g, b) = if d.abs() < 0.75 * pixel_size {
                (255u8, 255u8, 255u8)
            } else {
                // Contour bands
                let phase = (d / contour_spacing * std::f32::consts::PI).cos();
                let contour = 0.88 + 0.12 * phase.max(0.0);
                // Fade with distance
                let fade = 1.0 / (1.0 + 0.5 * (d.abs() / half_extent));
                let f = fade * contour;

                if d < 0.0 {
                    // Inside: blue
                    ((60.0 * f) as u8, (110.0 * f) as u8, (230.0 * f) as u8)
                } else {
                    // Outside: orange
                    ((230.0 * f) as u8, (130.0 * f) as u8, (50.0 * f) as u8)
                }
            };

            let i = (row * w + col) * 3;
            pixels[i] = r;
            pixels[i + 1] = g;
            pixels[i + 2] = b;
        }
    }

    let file = File::create(path).expect("Failed to create PNG file");
    let buf = BufWriter::new(file);
    let mut encoder = png::Encoder::new(buf, w as u32, h as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().expect("Failed to write PNG header");
    writer
        .write_image_data(&pixels)
        .expect("Failed to write PNG data");
}
