//! Octree cell addressing and spatial queries.
//!
//! Cells are identified by lattice coordinates (`LatticePoint`) and a step size.
//! The step size halves at each subdivision level. Provides neighbor finding,
//! parent/child conversion, and geometric queries (center, corner, circumradius).

use crate::math::{LatticePoint, Vec3};

#[derive(Clone, Copy, Debug, Hash, Eq, PartialEq)]
pub struct LeveledCell {
    pub key: LatticePoint,
    pub step: i32,
}

impl LeveledCell {
    pub fn new(x: i32, y: i32, z: i32, step: i32) -> Self {
        Self {
            key: LatticePoint::new(x, y, z),
            step,
        }
    }

    /// Lattice coordinate of a cell's BCC center.
    pub fn center(&self) -> LatticePoint {
        let half = self.step / 2;
        LatticePoint::new(self.key.x + half, self.key.y + half, self.key.z + half)
    }
}

#[derive(Clone, Copy, Debug, Hash, Eq, PartialEq)]
pub struct Direction(pub i32, pub i32, pub i32);

impl Direction {
    /// All 6 face-sharing neighbors.
    pub const FACES: [Direction; 6] = [
        Direction(-1, 0, 0),
        Direction(1, 0, 0),
        Direction(0, -1, 0),
        Direction(0, 1, 0),
        Direction(0, 0, -1),
        Direction(0, 0, 1),
    ];

    /// All 12 edge-sharing neighbors.
    pub const EDGES: [Direction; 12] = [
        Direction(-1, -1, 0),
        Direction(-1, 1, 0),
        Direction(1, -1, 0),
        Direction(1, 1, 0),
        Direction(-1, 0, -1),
        Direction(-1, 0, 1),
        Direction(1, 0, -1),
        Direction(1, 0, 1),
        Direction(0, -1, -1),
        Direction(0, -1, 1),
        Direction(0, 1, -1),
        Direction(0, 1, 1),
    ];

    /// All 18 neighbors (6 face + 12 edge).
    pub const ALL: [Direction; 18] = [
        Direction(-1, 0, 0),
        Direction(1, 0, 0),
        Direction(0, -1, 0),
        Direction(0, 1, 0),
        Direction(0, 0, -1),
        Direction(0, 0, 1),
        Direction(-1, -1, 0),
        Direction(-1, 1, 0),
        Direction(1, -1, 0),
        Direction(1, 1, 0),
        Direction(-1, 0, -1),
        Direction(-1, 0, 1),
        Direction(1, 0, -1),
        Direction(1, 0, 1),
        Direction(0, -1, -1),
        Direction(0, -1, 1),
        Direction(0, 1, -1),
        Direction(0, 1, 1),
    ];

    pub fn add(self, other: Direction) -> Direction {
        Direction(self.0 + other.0, self.1 + other.1, self.2 + other.2)
    }
}

impl From<Face> for Direction {
    fn from(f: Face) -> Self {
        match f {
            Face::NegX => Direction(-1, 0, 0),
            Face::PosX => Direction(1, 0, 0),
            Face::NegY => Direction(0, -1, 0),
            Face::PosY => Direction(0, 1, 0),
            Face::NegZ => Direction(0, 0, -1),
            Face::PosZ => Direction(0, 0, 1),
        }
    }
}

/// The 6 faces of a cube.
#[derive(Clone, Copy, Debug, Hash, Eq, PartialEq)]
#[repr(usize)]
pub enum Face {
    NegZ = 0, // bottom
    PosZ = 1, // top
    NegY = 2, // front
    PosY = 3, // back
    NegX = 4, // left
    PosX = 5, // right
}

impl Face {
    pub const ALL: [Face; 6] = [
        Face::NegZ,
        Face::PosZ,
        Face::NegY,
        Face::PosY,
        Face::NegX,
        Face::PosX,
    ];

    pub fn opposite(self) -> Face {
        match self {
            Face::NegX => Face::PosX,
            Face::PosX => Face::NegX,
            Face::NegY => Face::PosY,
            Face::PosY => Face::NegY,
            Face::NegZ => Face::PosZ,
            Face::PosZ => Face::NegZ,
        }
    }

    /// Returns (axis_index, is_positive) for this face.
    pub fn axis_info(self) -> (usize, bool) {
        match self {
            Face::NegX => (0, false),
            Face::PosX => (0, true),
            Face::NegY => (1, false),
            Face::PosY => (1, true),
            Face::NegZ => (2, false),
            Face::PosZ => (2, true),
        }
    }
}

/// Get the neighbor cell position in the given direction (same step level).
pub fn neighbor_key(cell: LatticePoint, dir: Direction, step: i32) -> LatticePoint {
    LatticePoint::new(
        cell.x + dir.0 * step,
        cell.y + dir.1 * step,
        cell.z + dir.2 * step,
    )
}

/// Snap a cell position to a coarser grid with the given step size.
pub fn cell_at_step(cell: LatticePoint, new_step: i32) -> LatticePoint {
    LatticePoint::new(
        cell.x - cell.x.rem_euclid(new_step),
        cell.y - cell.y.rem_euclid(new_step),
        cell.z - cell.z.rem_euclid(new_step),
    )
}

/// Get the 4 child cell keys that share a given face.
/// Children have step/2 and are offset by half from the parent position.
pub fn children_on_face(parent: LatticePoint, step: i32, face: Face) -> [LatticePoint; 4] {
    let half = step / 2;
    let (axis, positive) = face.axis_info();
    let (va, vb) = match axis {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    let mut base = [parent.x, parent.y, parent.z];
    if positive {
        base[axis] += half;
    }
    std::array::from_fn(|i| {
        let mut c = base;
        if i & 1 != 0 {
            c[va] += half;
        }
        if i & 2 != 0 {
            c[vb] += half;
        }
        LatticePoint::new(c[0], c[1], c[2])
    })
}

pub fn cell_center(cell: LatticePoint, step: i32, origin: Vec3, unit_size: f32) -> Vec3 {
    let half = step as f32 * unit_size / 2.0;
    [
        origin[0] + cell.x as f32 * unit_size + half,
        origin[1] + cell.y as f32 * unit_size + half,
        origin[2] + cell.z as f32 * unit_size + half,
    ]
}

pub fn cell_circumradius(cell_size: f32) -> f32 {
    cell_size * (3.0_f32).sqrt() / 2.0
}

/// Subdivide a cell into 8 children at half the step size.
/// Children are at offsets of (0 or half) in each axis from the parent position.
pub fn subdivide_cell(cell: LatticePoint, step: i32) -> [LatticePoint; 8] {
    let half = step / 2;
    [
        LatticePoint::new(cell.x, cell.y, cell.z),
        LatticePoint::new(cell.x + half, cell.y, cell.z),
        LatticePoint::new(cell.x, cell.y + half, cell.z),
        LatticePoint::new(cell.x + half, cell.y + half, cell.z),
        LatticePoint::new(cell.x, cell.y, cell.z + half),
        LatticePoint::new(cell.x + half, cell.y, cell.z + half),
        LatticePoint::new(cell.x, cell.y + half, cell.z + half),
        LatticePoint::new(cell.x + half, cell.y + half, cell.z + half),
    ]
}

/// Compute the initial (root) step size for a given max depth.
pub fn initial_step(max_depth: u32) -> i32 {
    1i32 << (max_depth + 1)
}

/// Global integer lattice for exact vertex coordinates.
///
/// All structural vertices (corners, BCC centers, edge midpoints, face centers)
/// are exact integer coordinates. Float conversion only happens at SDF evaluation
/// and final mesh output.
#[derive(Clone, Copy)]
pub struct LatticeGrid {
    pub unit_size: f32,
    pub origin: Vec3,
}

impl LatticeGrid {
    pub fn new(origin: Vec3, initial_size: f32, initial_step: i32) -> Self {
        let unit_size = initial_size / initial_step as f32;
        Self { unit_size, origin }
    }

    /// Convert a lattice point to world coordinates.
    pub fn to_world(&self, lp: LatticePoint) -> Vec3 {
        lp.to_world(self.unit_size, self.origin)
    }
}

#[cfg(test)]
#[path = "tests/octree_test.rs"]
mod octree_test;
