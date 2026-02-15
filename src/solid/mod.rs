mod csg;
mod primitives;
pub mod simd;
mod transforms;

use crate::math::Vec3;
pub use simd::Vec3x8;
use wide::f32x8;

/// A solid defined by a signed distance function.
/// Negative values are inside, positive values are outside, zero is the surface.
pub trait Solid: Send + Sync {
    /// Returns the signed distance from the point to the surface.
    /// Negative = inside, positive = outside, zero = on surface.
    fn sdf(&self, point: Vec3) -> f32;

    /// Returns the bounds (min, max) of the SDF within a sphere.
    /// Used for culling cells that don't intersect the surface.
    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let center_sdf = self.sdf(center);
        (center_sdf - radius, center_sdf + radius)
    }

    /// Evaluate the SDF at 8 points simultaneously using SIMD.
    /// Default implementation falls back to 8 scalar calls.
    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let xs: [f32; 8] = points.x.into();
        let ys: [f32; 8] = points.y.into();
        let zs: [f32; 8] = points.z.into();
        let mut results = [0.0f32; 8];
        for i in 0..8 {
            results[i] = self.sdf([xs[i], ys[i], zs[i]]);
        }
        f32x8::new(results)
    }
}

impl Solid for Box<dyn Solid> {
    fn sdf(&self, point: Vec3) -> f32 {
        (**self).sdf(point)
    }
    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        (**self).sdf_bounds(center, radius)
    }
    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        (**self).sdf_batch(points)
    }
}

pub use csg::{Difference, Intersection, SmoothDifference, SmoothIntersection, SmoothUnion, Union};
pub use primitives::{
    Capsule, Cube, InfiniteCone, InfiniteCylinder, Plane, RoundedBox, Sphere, Torus,
};
pub use transforms::{Mirror, Repeat, Rotate, Scale, Translate};
