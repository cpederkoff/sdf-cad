use super::simd::{clamp_f32x8, Vec3x8};
use super::Solid;
use crate::math::Vec3;
use wide::f32x8;

pub struct Sphere {
    pub radius: f32,
}

impl Sphere {
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }
}

impl Solid for Sphere {
    fn sdf(&self, point: Vec3) -> f32 {
        let dist = (point[0] * point[0] + point[1] * point[1] + point[2] * point[2]).sqrt();
        dist - self.radius
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let dist = (center[0] * center[0] + center[1] * center[1] + center[2] * center[2]).sqrt();
        let center_sdf = dist - self.radius;
        (center_sdf - radius, center_sdf + radius)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let r = f32x8::splat(self.radius);
        points.length() - r
    }
}

pub struct Cube {
    pub size: f32,
}

impl Cube {
    pub fn new(size: f32) -> Self {
        Self { size }
    }
}

impl Solid for Cube {
    fn sdf(&self, point: Vec3) -> f32 {
        let half_size = self.size / 2.0;
        let dx = point[0].abs() - half_size;
        let dy = point[1].abs() - half_size;
        let dz = point[2].abs() - half_size;

        // Distance to box surface
        let outside_dist = dx.max(0.0).hypot(dy.max(0.0)).hypot(dz.max(0.0));
        let inside_dist = dx.max(dy).max(dz).min(0.0);
        outside_dist + inside_dist
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let zero = f32x8::ZERO;
        let half = f32x8::splat(self.size / 2.0);
        let dx = points.x.abs() - half;
        let dy = points.y.abs() - half;
        let dz = points.z.abs() - half;

        // outside: sqrt(max(dx,0)^2 + max(dy,0)^2 + max(dz,0)^2)
        let ox = dx.max(zero);
        let oy = dy.max(zero);
        let oz = dz.max(zero);
        let outside = (ox * ox + oy * oy + oz * oz).sqrt();
        // inside: min(max(dx, dy, dz), 0)
        let inside = dx.max(dy).max(dz).min(zero);
        outside + inside
    }
}

/// Half-space defined by a plane. The normal points "outside" (positive SDF).
/// The plane passes through `normal * distance` from the origin.
pub struct Plane {
    normal: Vec3,
    distance: f32,
}

impl Plane {
    pub fn new(normal: Vec3, distance: f32) -> Self {
        let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        Self {
            normal: [normal[0] / len, normal[1] / len, normal[2] / len],
            distance,
        }
    }
}

impl Solid for Plane {
    fn sdf(&self, point: Vec3) -> f32 {
        point[0] * self.normal[0] + point[1] * self.normal[1] + point[2] * self.normal[2]
            - self.distance
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let d = self.sdf(center);
        (d - radius, d + radius)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let nx = f32x8::splat(self.normal[0]);
        let ny = f32x8::splat(self.normal[1]);
        let nz = f32x8::splat(self.normal[2]);
        let d = f32x8::splat(self.distance);
        points.x * nx + points.y * ny + points.z * nz - d
    }
}

/// Capsule (line segment with radius) along the Z axis, centered at the origin.
/// Extends from z = -height/2 to z = +height/2 with spherical caps.
pub struct Capsule {
    pub radius: f32,
    half_height: f32,
}

impl Capsule {
    pub fn new(radius: f32, height: f32) -> Self {
        Self {
            radius,
            half_height: height / 2.0,
        }
    }
}

impl Solid for Capsule {
    fn sdf(&self, point: Vec3) -> f32 {
        let cz = point[2].clamp(-self.half_height, self.half_height);
        let dx = point[0];
        let dy = point[1];
        let dz = point[2] - cz;
        (dx * dx + dy * dy + dz * dz).sqrt() - self.radius
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let d = self.sdf(center);
        (d - radius, d + radius)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let r = f32x8::splat(self.radius);
        let lo = f32x8::splat(-self.half_height);
        let hi = f32x8::splat(self.half_height);
        let cz = clamp_f32x8(points.z, lo, hi);
        let dx = points.x;
        let dy = points.y;
        let dz = points.z - cz;
        (dx * dx + dy * dy + dz * dz).sqrt() - r
    }
}

/// Torus in the XY plane, centered at the origin.
/// `major_radius` is the distance from center to the tube center.
/// `minor_radius` is the tube radius.
pub struct Torus {
    pub major_radius: f32,
    pub minor_radius: f32,
}

impl Torus {
    pub fn new(major_radius: f32, minor_radius: f32) -> Self {
        Self {
            major_radius,
            minor_radius,
        }
    }
}

impl Solid for Torus {
    fn sdf(&self, point: Vec3) -> f32 {
        let q = (point[0] * point[0] + point[1] * point[1]).sqrt() - self.major_radius;
        (q * q + point[2] * point[2]).sqrt() - self.minor_radius
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let center_sdf = self.sdf(center);
        (center_sdf - radius, center_sdf + radius)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let major = f32x8::splat(self.major_radius);
        let minor = f32x8::splat(self.minor_radius);
        let q = (points.x * points.x + points.y * points.y).sqrt() - major;
        (q * q + points.z * points.z).sqrt() - minor
    }
}

/// Box with rounded edges, centered at origin.
/// `size` is the full outer dimension, `radius` is the edge rounding.
pub struct RoundedBox {
    half_inner: f32,
    radius: f32,
}

impl RoundedBox {
    pub fn new(size: f32, radius: f32) -> Self {
        Self {
            half_inner: size / 2.0 - radius,
            radius,
        }
    }
}

impl Solid for RoundedBox {
    fn sdf(&self, point: Vec3) -> f32 {
        let dx = point[0].abs() - self.half_inner;
        let dy = point[1].abs() - self.half_inner;
        let dz = point[2].abs() - self.half_inner;
        let outside = dx.max(0.0).hypot(dy.max(0.0)).hypot(dz.max(0.0));
        let inside = dx.max(dy).max(dz).min(0.0);
        outside + inside - self.radius
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let zero = f32x8::ZERO;
        let half = f32x8::splat(self.half_inner);
        let r = f32x8::splat(self.radius);
        let dx = points.x.abs() - half;
        let dy = points.y.abs() - half;
        let dz = points.z.abs() - half;
        let ox = dx.max(zero);
        let oy = dy.max(zero);
        let oz = dz.max(zero);
        let outside = (ox * ox + oy * oy + oz * oz).sqrt();
        let inside = dx.max(dy).max(dz).min(zero);
        outside + inside - r
    }
}

/// Infinite cylinder along the Z axis.
pub struct InfiniteCylinder {
    pub radius: f32,
}

impl InfiniteCylinder {
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }
}

impl Solid for InfiniteCylinder {
    fn sdf(&self, point: Vec3) -> f32 {
        (point[0] * point[0] + point[1] * point[1]).sqrt() - self.radius
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let d = self.sdf(center);
        (d - radius, d + radius)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let r = f32x8::splat(self.radius);
        (points.x * points.x + points.y * points.y).sqrt() - r
    }
}

/// Infinite double cone along the Z axis, tip at origin.
/// `half_angle` (degrees) is the angle between the axis and the surface.
/// Intersect with `Plane::new([0,0,-1], 0)` to get a single upward-opening cone.
pub struct InfiniteCone {
    sin_a: f32,
    cos_a: f32,
}

impl InfiniteCone {
    pub fn new(half_angle_degrees: f32) -> Self {
        let r = half_angle_degrees * std::f32::consts::PI / 180.0;
        Self {
            sin_a: r.sin(),
            cos_a: r.cos(),
        }
    }
}

impl Solid for InfiniteCone {
    fn sdf(&self, point: Vec3) -> f32 {
        let q = (point[0] * point[0] + point[1] * point[1]).sqrt();
        q * self.cos_a - point[2].abs() * self.sin_a
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let d = self.sdf(center);
        (d - radius, d + radius)
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let cos_a = f32x8::splat(self.cos_a);
        let sin_a = f32x8::splat(self.sin_a);
        let q = (points.x * points.x + points.y * points.y).sqrt();
        q * cos_a - points.z.abs() * sin_a
    }
}
