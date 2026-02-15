use super::simd::Vec3x8;
use super::Solid;
use crate::math::Vec3;
use wide::f32x8;

pub struct Scale {
    pub inner: Box<dyn Solid>,
    pub factor: Vec3,
}

impl Scale {
    pub fn new(inner: impl Solid + 'static, factor: Vec3) -> Self {
        Self {
            inner: Box::new(inner),
            factor,
        }
    }

    pub fn new_boxed(inner: Box<dyn Solid>, factor: Vec3) -> Self {
        Self { inner, factor }
    }

    pub fn uniform(inner: impl Solid + 'static, factor: f32) -> Self {
        Self {
            inner: Box::new(inner),
            factor: [factor, factor, factor],
        }
    }
}

impl Solid for Scale {
    fn sdf(&self, point: Vec3) -> f32 {
        let inner_pt = [
            point[0] / self.factor[0],
            point[1] / self.factor[1],
            point[2] / self.factor[2],
        ];
        let min_s = self.factor[0]
            .abs()
            .min(self.factor[1].abs())
            .min(self.factor[2].abs());
        self.inner.sdf(inner_pt) * min_s
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let min_s = self.factor[0]
            .abs()
            .min(self.factor[1].abs())
            .min(self.factor[2].abs());
        let inner_center = [
            center[0] / self.factor[0],
            center[1] / self.factor[1],
            center[2] / self.factor[2],
        ];
        let (min, max) = self.inner.sdf_bounds(inner_center, radius / min_s);
        (min * min_s, max * min_s)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let fx = f32x8::splat(self.factor[0]);
        let fy = f32x8::splat(self.factor[1]);
        let fz = f32x8::splat(self.factor[2]);
        let min_s = self.factor[0]
            .abs()
            .min(self.factor[1].abs())
            .min(self.factor[2].abs());
        let inner_pts = Vec3x8 {
            x: points.x / fx,
            y: points.y / fy,
            z: points.z / fz,
        };
        self.inner.sdf_batch(&inner_pts) * f32x8::splat(min_s)
    }
}

pub struct Rotate {
    pub inner: Box<dyn Solid>,
    /// Unit axis, sin(angle), cos(angle) — precomputed from constructor
    axis: Vec3,
    sin_a: f32,
    cos_a: f32,
}

impl Rotate {
    pub fn new(inner: impl Solid + 'static, axis: Vec3, degrees: f32) -> Self {
        let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let axis = [axis[0] / len, axis[1] / len, axis[2] / len];
        let radians = degrees * std::f32::consts::PI / 180.0;
        Self {
            inner: Box::new(inner),
            axis,
            sin_a: radians.sin(),
            cos_a: radians.cos(),
        }
    }

    /// Apply inverse rotation (rotate by -angle) using Rodrigues' formula
    fn inv_rotate(&self, v: Vec3) -> Vec3 {
        let k = self.axis;
        let cross = [
            k[1] * v[2] - k[2] * v[1],
            k[2] * v[0] - k[0] * v[2],
            k[0] * v[1] - k[1] * v[0],
        ];
        let dot = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
        let one_minus_cos = 1.0 - self.cos_a;
        // Inverse rotation: negate sin term
        [
            v[0] * self.cos_a - cross[0] * self.sin_a + k[0] * dot * one_minus_cos,
            v[1] * self.cos_a - cross[1] * self.sin_a + k[1] * dot * one_minus_cos,
            v[2] * self.cos_a - cross[2] * self.sin_a + k[2] * dot * one_minus_cos,
        ]
    }

    /// Rodrigues' inverse rotation on 8 points at once
    fn inv_rotate_batch(&self, v: &Vec3x8) -> Vec3x8 {
        let kx = f32x8::splat(self.axis[0]);
        let ky = f32x8::splat(self.axis[1]);
        let kz = f32x8::splat(self.axis[2]);
        let k = Vec3x8 { x: kx, y: ky, z: kz };

        let cross = k.cross(*v);
        let dot = k.dot(*v);

        let cos_a = f32x8::splat(self.cos_a);
        let sin_a = f32x8::splat(self.sin_a);
        let one_minus_cos = f32x8::splat(1.0 - self.cos_a);

        // v * cos - cross * sin + k * (k·v) * (1-cos)
        Vec3x8 {
            x: v.x * cos_a - cross.x * sin_a + kx * dot * one_minus_cos,
            y: v.y * cos_a - cross.y * sin_a + ky * dot * one_minus_cos,
            z: v.z * cos_a - cross.z * sin_a + kz * dot * one_minus_cos,
        }
    }
}

impl Solid for Rotate {
    fn sdf(&self, point: Vec3) -> f32 {
        self.inner.sdf(self.inv_rotate(point))
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        self.inner.sdf_bounds(self.inv_rotate(center), radius)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let rotated = self.inv_rotate_batch(points);
        self.inner.sdf_batch(&rotated)
    }
}

pub struct Translate {
    pub inner: Box<dyn Solid>,
    pub offset: Vec3,
}

impl Translate {
    pub fn new(inner: impl Solid + 'static, offset: Vec3) -> Self {
        Self {
            inner: Box::new(inner),
            offset,
        }
    }

    pub fn new_boxed(inner: Box<dyn Solid>, offset: Vec3) -> Self {
        Self { inner, offset }
    }
}

impl Solid for Translate {
    fn sdf(&self, point: Vec3) -> f32 {
        self.inner.sdf([
            point[0] - self.offset[0],
            point[1] - self.offset[1],
            point[2] - self.offset[2],
        ])
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        self.inner.sdf_bounds(
            [
                center[0] - self.offset[0],
                center[1] - self.offset[1],
                center[2] - self.offset[2],
            ],
            radius,
        )
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let offset = Vec3x8::splat(self.offset);
        let shifted = points.sub(offset);
        self.inner.sdf_batch(&shifted)
    }
}

/// Finite repetition of a solid along each axis.
/// `spacing` is the distance between copies on each axis.
/// `copies` is the number of copies on each axis (1 = no repetition).
/// Copies are centered at origin.
/// The inner solid should fit within the spacing to avoid overlapping copies.
pub struct Repeat {
    pub inner: Box<dyn Solid>,
    pub spacing: Vec3,
    pub copies: [u32; 3],
}

impl Repeat {
    pub fn new(inner: impl Solid + 'static, spacing: Vec3, copies: [u32; 3]) -> Self {
        Self {
            inner: Box::new(inner),
            spacing,
            copies,
        }
    }

    pub fn new_boxed(inner: Box<dyn Solid>, spacing: Vec3, copies: [u32; 3]) -> Self {
        Self {
            inner,
            spacing,
            copies,
        }
    }
}

impl Repeat {
    fn fold_axis(p: f32, spacing: f32, copies: u32) -> f32 {
        let max_idx = (copies - 1) as f32;
        let half = max_idx * 0.5;
        let id = (p / spacing + half).round().clamp(0.0, max_idx);
        p - spacing * (id - half)
    }
}

impl Solid for Repeat {
    fn sdf(&self, point: Vec3) -> f32 {
        let p = [
            Self::fold_axis(point[0], self.spacing[0], self.copies[0]),
            Self::fold_axis(point[1], self.spacing[1], self.copies[1]),
            Self::fold_axis(point[2], self.spacing[2], self.copies[2]),
        ];
        self.inner.sdf(p)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let sx = f32x8::splat(self.spacing[0]);
        let sy = f32x8::splat(self.spacing[1]);
        let sz = f32x8::splat(self.spacing[2]);

        let max_x = f32x8::splat((self.copies[0] - 1) as f32);
        let max_y = f32x8::splat((self.copies[1] - 1) as f32);
        let max_z = f32x8::splat((self.copies[2] - 1) as f32);

        let half_x = max_x * f32x8::splat(0.5);
        let half_y = max_y * f32x8::splat(0.5);
        let half_z = max_z * f32x8::splat(0.5);

        let zero = f32x8::splat(0.0);

        let id_x = (points.x / sx + half_x).round().max(zero).min(max_x);
        let id_y = (points.y / sy + half_y).round().max(zero).min(max_y);
        let id_z = (points.z / sz + half_z).round().max(zero).min(max_z);

        let folded = Vec3x8 {
            x: points.x - sx * (id_x - half_x),
            y: points.y - sy * (id_y - half_y),
            z: points.z - sz * (id_z - half_z),
        };
        self.inner.sdf_batch(&folded)
    }
}

/// Mirror a solid across coordinate planes.
/// Each true element in `axes` mirrors across that axis (takes abs of that coordinate).
/// The inner solid should be defined in the positive quadrant of mirrored axes.
pub struct Mirror {
    pub inner: Box<dyn Solid>,
    pub axes: [bool; 3],
}

impl Mirror {
    pub fn new(inner: impl Solid + 'static, axes: [bool; 3]) -> Self {
        Self {
            inner: Box::new(inner),
            axes,
        }
    }
}

impl Solid for Mirror {
    fn sdf(&self, point: Vec3) -> f32 {
        self.inner.sdf([
            if self.axes[0] {
                point[0].abs()
            } else {
                point[0]
            },
            if self.axes[1] {
                point[1].abs()
            } else {
                point[1]
            },
            if self.axes[2] {
                point[2].abs()
            } else {
                point[2]
            },
        ])
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        self.inner.sdf_bounds(
            [
                if self.axes[0] {
                    center[0].abs()
                } else {
                    center[0]
                },
                if self.axes[1] {
                    center[1].abs()
                } else {
                    center[1]
                },
                if self.axes[2] {
                    center[2].abs()
                } else {
                    center[2]
                },
            ],
            radius,
        )
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let mirrored = Vec3x8 {
            x: if self.axes[0] { points.x.abs() } else { points.x },
            y: if self.axes[1] { points.y.abs() } else { points.y },
            z: if self.axes[2] { points.z.abs() } else { points.z },
        };
        self.inner.sdf_batch(&mirrored)
    }
}
