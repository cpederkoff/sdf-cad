use super::simd::{smooth_max_x8, smooth_min_x8, Vec3x8};
use super::Solid;
use crate::math::Vec3;
use wide::f32x8;

pub struct Difference {
    pub left: Box<dyn Solid>,
    pub right: Box<dyn Solid>,
}

impl Difference {
    pub fn new(left: impl Solid + 'static, right: impl Solid + 'static) -> Self {
        Self {
            left: Box::new(left),
            right: Box::new(right),
        }
    }

    pub fn new_boxed(left: Box<dyn Solid>, right: Box<dyn Solid>) -> Self {
        Self { left, right }
    }
}

impl Solid for Difference {
    fn sdf(&self, point: Vec3) -> f32 {
        let left_sdf = self.left.sdf(point);
        let right_sdf = self.right.sdf(point);
        left_sdf.max(-right_sdf)
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let (left_min, left_max) = self.left.sdf_bounds(center, radius);
        if left_min > 0.0 {
            return (left_min, left_max);
        }
        let (right_min, right_max) = self.right.sdf_bounds(center, radius);
        (left_min.max(-right_max), left_max.max(-right_min))
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let l = self.left.sdf_batch(points);
        let r = self.right.sdf_batch(points);
        l.max(-r)
    }
}

pub struct Union {
    pub left: Box<dyn Solid>,
    pub right: Box<dyn Solid>,
}

impl Union {
    pub fn new(left: impl Solid + 'static, right: impl Solid + 'static) -> Self {
        Self {
            left: Box::new(left),
            right: Box::new(right),
        }
    }

    pub fn new_boxed(left: Box<dyn Solid>, right: Box<dyn Solid>) -> Self {
        Self { left, right }
    }
}

impl Solid for Union {
    fn sdf(&self, point: Vec3) -> f32 {
        let left_sdf = self.left.sdf(point);
        let right_sdf = self.right.sdf(point);
        left_sdf.min(right_sdf)
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let (left_min, left_max) = self.left.sdf_bounds(center, radius);
        if left_max < 0.0 {
            return (left_min, left_max);
        }
        let (right_min, right_max) = self.right.sdf_bounds(center, radius);
        if right_max < 0.0 {
            return (right_min, right_max);
        }
        (left_min.min(right_min), left_max.min(right_max))
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let l = self.left.sdf_batch(points);
        let r = self.right.sdf_batch(points);
        l.min(r)
    }
}

pub struct Intersection {
    pub left: Box<dyn Solid>,
    pub right: Box<dyn Solid>,
}

impl Intersection {
    pub fn new(left: impl Solid + 'static, right: impl Solid + 'static) -> Self {
        Self {
            left: Box::new(left),
            right: Box::new(right),
        }
    }

    pub fn new_boxed(left: Box<dyn Solid>, right: Box<dyn Solid>) -> Self {
        Self { left, right }
    }
}

impl Solid for Intersection {
    fn sdf(&self, point: Vec3) -> f32 {
        let left_sdf = self.left.sdf(point);
        let right_sdf = self.right.sdf(point);
        left_sdf.max(right_sdf)
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let (left_min, left_max) = self.left.sdf_bounds(center, radius);
        if left_min > 0.0 {
            return (left_min, left_max);
        }
        let (right_min, right_max) = self.right.sdf_bounds(center, radius);
        if right_min > 0.0 {
            return (right_min, right_max);
        }
        (left_min.max(right_min), left_max.max(right_max))
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let l = self.left.sdf_batch(points);
        let r = self.right.sdf_batch(points);
        l.max(r)
    }
}

/// Polynomial smooth min: `min(a, b)` with blending radius `k`.
fn smooth_min(a: f32, b: f32, k: f32) -> f32 {
    let h = (k - (a - b).abs()).max(0.0) / k;
    a.min(b) - h * h * k * 0.25
}

/// Polynomial smooth max: `max(a, b)` with blending radius `k`.
fn smooth_max(a: f32, b: f32, k: f32) -> f32 {
    -smooth_min(-a, -b, k)
}

pub struct SmoothUnion {
    pub left: Box<dyn Solid>,
    pub right: Box<dyn Solid>,
    pub k: f32,
}

impl SmoothUnion {
    pub fn new(left: impl Solid + 'static, right: impl Solid + 'static, k: f32) -> Self {
        Self {
            left: Box::new(left),
            right: Box::new(right),
            k,
        }
    }
}

impl Solid for SmoothUnion {
    fn sdf(&self, point: Vec3) -> f32 {
        smooth_min(self.left.sdf(point), self.right.sdf(point), self.k)
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let (left_min, left_max) = self.left.sdf_bounds(center, radius);
        if left_max < 0.0 {
            return (left_min - self.k * 0.25, left_max);
        }
        let (right_min, right_max) = self.right.sdf_bounds(center, radius);
        if right_max < 0.0 {
            return (right_min - self.k * 0.25, right_max);
        }
        (
            left_min.min(right_min) - self.k * 0.25,
            left_max.min(right_max),
        )
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let l = self.left.sdf_batch(points);
        let r = self.right.sdf_batch(points);
        smooth_min_x8(l, r, f32x8::splat(self.k))
    }
}

pub struct SmoothIntersection {
    pub left: Box<dyn Solid>,
    pub right: Box<dyn Solid>,
    pub k: f32,
}

impl SmoothIntersection {
    pub fn new(left: impl Solid + 'static, right: impl Solid + 'static, k: f32) -> Self {
        Self {
            left: Box::new(left),
            right: Box::new(right),
            k,
        }
    }
}

impl Solid for SmoothIntersection {
    fn sdf(&self, point: Vec3) -> f32 {
        smooth_max(self.left.sdf(point), self.right.sdf(point), self.k)
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let (left_min, left_max) = self.left.sdf_bounds(center, radius);
        if left_min > 0.0 {
            return (left_min, left_max + self.k * 0.25);
        }
        let (right_min, right_max) = self.right.sdf_bounds(center, radius);
        if right_min > 0.0 {
            return (right_min, right_max + self.k * 0.25);
        }
        (
            left_min.max(right_min),
            left_max.max(right_max) + self.k * 0.25,
        )
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let l = self.left.sdf_batch(points);
        let r = self.right.sdf_batch(points);
        smooth_max_x8(l, r, f32x8::splat(self.k))
    }
}

pub struct SmoothDifference {
    pub left: Box<dyn Solid>,
    pub right: Box<dyn Solid>,
    pub k: f32,
}

impl SmoothDifference {
    pub fn new(left: impl Solid + 'static, right: impl Solid + 'static, k: f32) -> Self {
        Self {
            left: Box::new(left),
            right: Box::new(right),
            k,
        }
    }
}

impl Solid for SmoothDifference {
    fn sdf(&self, point: Vec3) -> f32 {
        smooth_max(self.left.sdf(point), -self.right.sdf(point), self.k)
    }

    fn sdf_bounds(&self, center: Vec3, radius: f32) -> (f32, f32) {
        let (left_min, left_max) = self.left.sdf_bounds(center, radius);
        if left_min > 0.0 {
            return (left_min, left_max + self.k * 0.25);
        }
        let (right_min, right_max) = self.right.sdf_bounds(center, radius);
        (
            left_min.max(-right_max),
            left_max.max(-right_min) + self.k * 0.25,
        )
    }

    fn sdf_batch(&self, points: &Vec3x8) -> f32x8 {
        let l = self.left.sdf_batch(points);
        let r = self.right.sdf_batch(points);
        smooth_max_x8(l, -r, f32x8::splat(self.k))
    }
}
