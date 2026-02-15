pub type Vec3 = [f32; 3];

pub fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn vec3_lerp(a: Vec3, b: Vec3, t: f32) -> Vec3 {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

pub trait Vec3Ext {
    fn add(self, other: Vec3) -> Vec3;
    fn sub(self, other: Vec3) -> Vec3;
    fn dot(self, other: Vec3) -> f32;
    fn cross(self, other: Vec3) -> Vec3;
    fn scale(self, s: f32) -> Vec3;
    fn length_sq(self) -> f32;
    fn length(self) -> f32;
    fn dist_sq(self, other: Vec3) -> f32;
    fn normalize(self) -> Vec3;
}

impl Vec3Ext for Vec3 {
    fn add(self, b: Vec3) -> Vec3 {
        [self[0] + b[0], self[1] + b[1], self[2] + b[2]]
    }
    fn sub(self, b: Vec3) -> Vec3 {
        [self[0] - b[0], self[1] - b[1], self[2] - b[2]]
    }
    fn dot(self, b: Vec3) -> f32 {
        self[0] * b[0] + self[1] * b[1] + self[2] * b[2]
    }
    fn cross(self, b: Vec3) -> Vec3 {
        [
            self[1] * b[2] - self[2] * b[1],
            self[2] * b[0] - self[0] * b[2],
            self[0] * b[1] - self[1] * b[0],
        ]
    }
    fn scale(self, s: f32) -> Vec3 {
        [self[0] * s, self[1] * s, self[2] * s]
    }
    fn length_sq(self) -> f32 {
        self[0] * self[0] + self[1] * self[1] + self[2] * self[2]
    }
    fn length(self) -> f32 {
        self.length_sq().sqrt()
    }
    fn dist_sq(self, b: Vec3) -> f32 {
        self.sub(b).length_sq()
    }
    fn normalize(self) -> Vec3 {
        let len = self.length();
        if len > 0.0 {
            self.scale(1.0 / len)
        } else {
            self
        }
    }
}

/// Integer lattice coordinate. All structural vertices (corners, BCC centers,
/// edge midpoints, face centers) are exact integer positions in the lattice.
#[derive(Clone, Copy, Debug, Hash, Eq, PartialEq, Ord, PartialOrd)]
pub struct LatticePoint {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl LatticePoint {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// Midpoint of two lattice points (both coordinates must have the same parity).
    pub fn midpoint(a: Self, b: Self) -> Self {
        debug_assert!(
            (a.x + b.x) % 2 == 0 && (a.y + b.y) % 2 == 0 && (a.z + b.z) % 2 == 0,
            "midpoint of {:?} and {:?} not on lattice",
            a,
            b
        );
        Self {
            x: (a.x + b.x) / 2,
            y: (a.y + b.y) / 2,
            z: (a.z + b.z) / 2,
        }
    }

    /// Center of 4 lattice points (sum must be divisible by 4 per axis).
    pub fn center4(a: Self, b: Self, c: Self, d: Self) -> Self {
        debug_assert!(
            (a.x + b.x + c.x + d.x) % 4 == 0
                && (a.y + b.y + c.y + d.y) % 4 == 0
                && (a.z + b.z + c.z + d.z) % 4 == 0,
            "center4 of {:?}, {:?}, {:?}, {:?} not on lattice",
            a,
            b,
            c,
            d
        );
        Self {
            x: (a.x + b.x + c.x + d.x) / 4,
            y: (a.y + b.y + c.y + d.y) / 4,
            z: (a.z + b.z + c.z + d.z) / 4,
        }
    }

    /// Convert lattice coordinates to world-space floating point.
    pub fn to_world(self, unit_size: f32, origin: Vec3) -> Vec3 {
        [
            self.x as f32 * unit_size + origin[0],
            self.y as f32 * unit_size + origin[1],
            self.z as f32 * unit_size + origin[2],
        ]
    }
}
