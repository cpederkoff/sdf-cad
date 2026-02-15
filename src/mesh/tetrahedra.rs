//! Tet generation for BCC meshing.
//!
//! Generates marching tetrahedra from octree leaf cells. Each cube face
//! produces a face-center fan of tets with the cell's BCC center as apex.
//! Where a coarse cell has finer neighbors, transition tets use subdivided
//! sub-quads to prevent T-junctions at level boundaries.

use super::adaptive::{BalancedOctree, NeighborStatus};
use super::builder::MeshBuilder;
use super::cache::SdfCache;
use super::marching::Tet;
use super::octree::{Direction, Face, LatticeGrid, LeveledCell};
use crate::math::LatticePoint;
use crate::solid::Solid;

// ── Cube and face geometry ──────────────────────────────────────────────

/// Get the 8 corner vertices of a cube in lattice coordinates.
/// `corner` is the min-corner, `step` is the cell size in lattice units.
pub(crate) fn cube_corners(corner: LatticePoint, step: i32) -> [LatticePoint; 8] {
    let s = step;
    [
        corner,                                                      // 000
        LatticePoint::new(corner.x + s, corner.y, corner.z),         // 100
        LatticePoint::new(corner.x, corner.y + s, corner.z),         // 010
        LatticePoint::new(corner.x + s, corner.y + s, corner.z),     // 110
        LatticePoint::new(corner.x, corner.y, corner.z + s),         // 001
        LatticePoint::new(corner.x + s, corner.y, corner.z + s),     // 101
        LatticePoint::new(corner.x, corner.y + s, corner.z + s),     // 011
        LatticePoint::new(corner.x + s, corner.y + s, corner.z + s), // 111
    ]
}

/// Get the 4 corner vertices of a face (in CCW order from outside).
pub(crate) fn face_corners(cube_corners: &[LatticePoint; 8], face: Face) -> [LatticePoint; 4] {
    match face {
        Face::NegZ => [cube_corners[0], cube_corners[1], cube_corners[3], cube_corners[2]],
        Face::PosZ => [cube_corners[4], cube_corners[6], cube_corners[7], cube_corners[5]],
        Face::NegY => [cube_corners[0], cube_corners[4], cube_corners[5], cube_corners[1]],
        Face::PosY => [cube_corners[2], cube_corners[3], cube_corners[7], cube_corners[6]],
        Face::NegX => [cube_corners[0], cube_corners[2], cube_corners[6], cube_corners[4]],
        Face::PosX => [cube_corners[1], cube_corners[5], cube_corners[7], cube_corners[3]],
    }
}

/// Subdivide a face quad into 4 sub-quads using edge midpoints and face center.
fn subdivide_face(fc: [LatticePoint; 4]) -> [[LatticePoint; 4]; 4] {
    let m01 = LatticePoint::midpoint(fc[0], fc[1]);
    let m12 = LatticePoint::midpoint(fc[1], fc[2]);
    let m23 = LatticePoint::midpoint(fc[2], fc[3]);
    let m30 = LatticePoint::midpoint(fc[3], fc[0]);
    let center = LatticePoint::center4(fc[0], fc[1], fc[2], fc[3]);
    [
        [fc[0], m01, center, m30],
        [m01, fc[1], m12, center],
        [center, m12, fc[2], m23],
        [m30, center, m23, fc[3]],
    ]
}

// ── T-junction prevention ───────────────────────────────────────────────

/// Check if a lattice edge (given by its two endpoints) needs a midpoint.
/// Checks all 4 cells sharing the edge for any Finer face touching it.
fn lattice_edge_needs_midpoint(
    a: LatticePoint,
    b: LatticePoint,
    octree: &BalancedOctree,
) -> bool {
    let dx = (b.x - a.x).abs();
    let dy = (b.y - a.y).abs();
    let dz = (b.z - a.z).abs();
    let step = dx.max(dy).max(dz);

    if step < 4 {
        return false;
    }

    // Determine the two axes perpendicular to the edge
    let (perp1, perp2) = if dx == step {
        (1, 2)
    } else if dy == step {
        (0, 2)
    } else {
        (0, 1)
    };

    let min_corner = [a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)];

    // Check all 4 cells sharing this edge
    for &d1 in &[0i32, -step] {
        for &d2 in &[0i32, -step] {
            let mut cc = min_corner;
            cc[perp1] += d1;
            cc[perp2] += d2;
            let cell = LeveledCell {
                key: LatticePoint::new(cc[0], cc[1], cc[2]),
                step,
            };

            if !octree.contains(&cell) {
                continue;
            }

            // d_offset==0 → edge at cell's min boundary → face points negative
            // d_offset==-step → edge at cell's max boundary → face points positive
            let sign1 = if d1 == 0 { -1 } else { 1 };
            let sign2 = if d2 == 0 { -1 } else { 1 };

            let mut dir1 = Direction(0, 0, 0);
            match perp1 {
                0 => dir1.0 = sign1,
                1 => dir1.1 = sign1,
                _ => dir1.2 = sign1,
            }
            let mut dir2 = Direction(0, 0, 0);
            match perp2 {
                0 => dir2.0 = sign2,
                1 => dir2.1 = sign2,
                _ => dir2.2 = sign2,
            }
            let dir3 = dir1.add(dir2);

            if octree.neighbor_status(cell, dir1) == NeighborStatus::Finer
                || octree.neighbor_status(cell, dir2) == NeighborStatus::Finer
                || octree.neighbor_status(cell, dir3) == NeighborStatus::Finer
            {
                return true;
            }
        }
    }

    false
}

/// Build a polygon from face corners, iteratively inserting midpoints on any
/// edge that needs splitting. Handles multi-level T-junctions.
pub(crate) fn build_refined_polygon(
    fc: [LatticePoint; 4],
    octree: &BalancedOctree,
) -> Vec<LatticePoint> {
    let mut polygon: Vec<LatticePoint> = fc.to_vec();

    loop {
        let mut new_polygon = Vec::with_capacity(polygon.len() * 2);
        let mut changed = false;

        for i in 0..polygon.len() {
            let a = polygon[i];
            let b = polygon[(i + 1) % polygon.len()];
            new_polygon.push(a);
            if lattice_edge_needs_midpoint(a, b, octree) {
                new_polygon.push(LatticePoint::midpoint(a, b));
                changed = true;
            }
        }

        polygon = new_polygon;
        if !changed {
            break;
        }
    }

    polygon
}

// ── Face processing ─────────────────────────────────────────────────────

/// Process a face quad: refine edges, build face-center fan, march each tet.
fn process_face<S: Solid + ?Sized>(
    fc: [LatticePoint; 4],
    apex: LatticePoint,
    octree: &BalancedOctree,
    solid: &S,
    grid: &LatticeGrid,
    interpolate: bool,
    cache: &SdfCache,
    mesh: &mut MeshBuilder,
) {
    let polygon = build_refined_polygon(fc, octree);
    let face_center = LatticePoint::center4(fc[0], fc[1], fc[2], fc[3]);
    let n = polygon.len();
    for i in 0..n {
        let tet = Tet {
            vertices: [polygon[i], polygon[(i + 1) % n], face_center, apex],
        };
        tet.process(solid, grid, interpolate, cache, mesh);
    }
}

/// Process a single cell, generating tets for each face.
pub(crate) fn process_cell<S: Solid + ?Sized>(
    cell: LeveledCell,
    octree: &BalancedOctree,
    solid: &S,
    grid: &LatticeGrid,
    interpolate: bool,
    cache: &SdfCache,
    mesh: &mut MeshBuilder,
) {
    let step = cell.step;
    let center = cell.center();
    let corners = cube_corners(cell.key, step);

    let statuses: [NeighborStatus; 6] = Face::ALL.map(|f| octree.neighbor_status(cell, f.into()));

    for (face_idx, &face) in Face::ALL.iter().enumerate() {
        let fc = face_corners(&corners, face);

        if statuses[face_idx] == NeighborStatus::Finer {
            for sub_fc in &subdivide_face(fc) {
                process_face(*sub_fc, center, octree, solid, grid, interpolate, cache, mesh);
            }
        } else {
            process_face(fc, center, octree, solid, grid, interpolate, cache, mesh);
        }
    }
}

#[cfg(test)]
#[path = "tests/tetrahedra_test.rs"]
mod tests;
