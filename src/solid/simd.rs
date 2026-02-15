use wide::f32x8;

/// 8 3D points in SoA (Structure of Arrays) layout for SIMD batch evaluation.
#[derive(Clone, Copy, Debug)]
pub struct Vec3x8 {
    pub x: f32x8,
    pub y: f32x8,
    pub z: f32x8,
}

impl Vec3x8 {
    pub fn splat(point: [f32; 3]) -> Self {
        Self {
            x: f32x8::splat(point[0]),
            y: f32x8::splat(point[1]),
            z: f32x8::splat(point[2]),
        }
    }

    /// Load up to 8 points from a slice, padding unused lanes with the last point.
    pub fn from_slice(points: &[[f32; 3]]) -> Self {
        debug_assert!(!points.is_empty() && points.len() <= 8);
        let pad = points.last().unwrap();
        let get = |i: usize| {
            if i < points.len() {
                &points[i]
            } else {
                pad
            }
        };
        Self {
            x: f32x8::new([
                get(0)[0], get(1)[0], get(2)[0], get(3)[0],
                get(4)[0], get(5)[0], get(6)[0], get(7)[0],
            ]),
            y: f32x8::new([
                get(0)[1], get(1)[1], get(2)[1], get(3)[1],
                get(4)[1], get(5)[1], get(6)[1], get(7)[1],
            ]),
            z: f32x8::new([
                get(0)[2], get(1)[2], get(2)[2], get(3)[2],
                get(4)[2], get(5)[2], get(6)[2], get(7)[2],
            ]),
        }
    }

    /// Load exactly 8 points from an array.
    pub fn from_array(points: &[[f32; 3]; 8]) -> Self {
        Self {
            x: f32x8::new([
                points[0][0], points[1][0], points[2][0], points[3][0],
                points[4][0], points[5][0], points[6][0], points[7][0],
            ]),
            y: f32x8::new([
                points[0][1], points[1][1], points[2][1], points[3][1],
                points[4][1], points[5][1], points[6][1], points[7][1],
            ]),
            z: f32x8::new([
                points[0][2], points[1][2], points[2][2], points[3][2],
                points[4][2], points[5][2], points[6][2], points[7][2],
            ]),
        }
    }

    pub fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    pub fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    pub fn scale(self, s: f32x8) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        }
    }

    pub fn dot(self, other: Self) -> f32x8 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn length_sq(self) -> f32x8 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn length(self) -> f32x8 {
        self.length_sq().sqrt()
    }

    pub fn abs(self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
        }
    }

    /// Extract all 8 lane results as f32 array.
    pub fn extract_results(results: f32x8) -> [f32; 8] {
        results.into()
    }
}

/// Component-wise max(v, scalar)
pub fn max_f32x8(a: f32x8, b: f32x8) -> f32x8 {
    a.max(b)
}

/// Component-wise min(v, scalar)
pub fn min_f32x8(a: f32x8, b: f32x8) -> f32x8 {
    a.min(b)
}

/// Clamp each lane to [lo, hi]
pub fn clamp_f32x8(v: f32x8, lo: f32x8, hi: f32x8) -> f32x8 {
    v.max(lo).min(hi)
}

/// Polynomial smooth min for f32x8: `min(a, b)` with blending radius `k`.
pub fn smooth_min_x8(a: f32x8, b: f32x8, k: f32x8) -> f32x8 {
    let zero = f32x8::ZERO;
    let diff = a - b;
    let h = (k - diff.abs()).max(zero) / k;
    a.min(b) - h * h * k * f32x8::splat(0.25)
}

/// Polynomial smooth max for f32x8.
pub fn smooth_max_x8(a: f32x8, b: f32x8, k: f32x8) -> f32x8 {
    -smooth_min_x8(-a, -b, k)
}

/// Blend/select: for each lane, pick `a` if mask is true, else `b`.
/// `wide` uses bitwise ops for this — mask lanes are all-1s or all-0s.
pub fn blend_f32x8(mask: f32x8, a: f32x8, b: f32x8) -> f32x8 {
    mask.blend(a, b)
}
