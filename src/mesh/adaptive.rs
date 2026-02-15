//! Adaptive octree construction for BCC meshing.
//!
//! Builds a balanced octree refined by SDF interpolation error, ensuring no two
//! adjacent cells differ by more than 1 level and all leaf cells intersect the
//! surface.

use super::octree::{
    cell_at_step, cell_center, cell_circumradius, neighbor_key, subdivide_cell,
    Direction, LeveledCell,
};
use crate::math::{LatticePoint, Vec3};
use crate::solid::simd::Vec3x8;
use crate::solid::Solid;
use rustc_hash::FxHashSet;
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct BccMeshParams {
    pub min_depth: u32,
    pub max_depth: u32,
    /// Maximum allowed SDF interpolation error before subdividing.
    /// Measured at face centers vs. bilinear interpolation from corners.
    pub sdf_error_threshold: f32,
    /// Whether to interpolate edge crossings using SDF values (true) or
    /// place them at edge midpoints (false).
    pub interpolate: bool,
}

impl Default for BccMeshParams {
    fn default() -> Self {
        Self {
            min_depth: 2,
            max_depth: 8,
            sdf_error_threshold: 0.01,
            interpolate: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum NeighborStatus {
    None,      // No neighbor (boundary or outside surface)
    SameLevel, // Neighbor at same level
    Coarser,   // Neighbor is coarser (we are finer)
    Finer,     // Neighbor is finer (we are coarser)
}

/// Balanced adaptive octree for BCC meshing
pub(crate) struct BalancedOctree {
    cells: FxHashSet<LeveledCell>,
    pub(crate) initial_step: i32,
}

impl BalancedOctree {
    pub(crate) fn new(initial_step: i32) -> Self {
        Self {
            cells: FxHashSet::default(),
            initial_step,
        }
    }

    pub(crate) fn insert(&mut self, cell: LeveledCell) {
        self.cells.insert(cell);
    }

    pub(crate) fn contains(&self, cell: &LeveledCell) -> bool {
        self.cells.contains(cell)
    }

    fn remove(&mut self, cell: &LeveledCell) {
        self.cells.remove(cell);
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = &LeveledCell> {
        self.cells.iter()
    }

    /// Find the neighbor status for a cell in a given direction
    pub(crate) fn neighbor_status(&self, cell: LeveledCell, dir: Direction) -> NeighborStatus {
        let neighbor_pos = neighbor_key(cell.key, dir, cell.step);
        let neighbor_same = LeveledCell {
            key: neighbor_pos,
            step: cell.step,
        };

        // Same level neighbor?
        if self.contains(&neighbor_same) {
            return NeighborStatus::SameLevel;
        }

        // Coarser neighbor? (one level up = double step)
        let coarser_step = cell.step * 2;
        if coarser_step <= self.initial_step {
            let coarser_key = cell_at_step(neighbor_pos, coarser_step);
            if self.contains(&LeveledCell {
                key: coarser_key,
                step: coarser_step,
            }) {
                return NeighborStatus::Coarser;
            }
        }

        // Finer neighbors? (check if any of the 8 children exist)
        let child_step = cell.step / 2;
        let child_keys = subdivide_cell(neighbor_pos, cell.step);
        for &child_key in &child_keys {
            if self.contains(&LeveledCell {
                key: child_key,
                step: child_step,
            }) {
                return NeighborStatus::Finer;
            }
        }

        NeighborStatus::None
    }

}

pub(crate) fn cell_intersects_surface<S: Solid + ?Sized>(
    solid: &S,
    center: Vec3,
    circumradius: f32,
) -> bool {
    let (min_sdf, max_sdf) = solid.sdf_bounds(center, circumradius);
    min_sdf <= 0.0 && max_sdf >= 0.0
}

fn should_subdivide<S: Solid + ?Sized>(
    solid: &S,
    center: Vec3,
    cell_size: f32,
    step: i32,
    min_step: i32,
    finest_step: i32,
    sdf_error_threshold: f32,
) -> bool {
    if step > min_step {
        return true;
    }
    if step <= finest_step {
        return false;
    }

    // Measure SDF nonlinearity by comparing the center value to the average of
    // 4 tetrahedral corners (alternating vertices of the cube). These capture
    // all quadratic and bilinear error terms identically to the full 8 corners,
    // differing only in the negligible trilinear (xyz) term.
    //
    // Pack 5 points (4 corners + center) into one SIMD batch call,
    // reducing 5 vtable traversals to 1.
    let h = cell_size * 0.5;
    let c = center;

    let batch = Vec3x8::from_slice(&[
        [c[0] - h, c[1] - h, c[2] - h], // corner 0
        [c[0] + h, c[1] + h, c[2] - h], // corner 1
        [c[0] + h, c[1] - h, c[2] + h], // corner 2
        [c[0] - h, c[1] + h, c[2] + h], // corner 3
        center,                           // center
    ]);
    let results = Vec3x8::extract_results(solid.sdf_batch(&batch));

    let corner_sum = results[0] + results[1] + results[2] + results[3];
    let center_interp = corner_sum * 0.25;
    let center_sdf = results[4];

    (center_sdf - center_interp).abs() > sdf_error_threshold
}

/// Build balanced octree with SDF error-based refinement.
///
/// `root_step` defines the lattice coordinate system. It must be at least
/// `initial_step(params.max_depth)` so the finest cells are representable.
pub(crate) fn build_octree<S: Solid + ?Sized>(
    solid: &S,
    origin: Vec3,
    initial_size: f32,
    params: &BccMeshParams,
    root_step: i32,
) -> BalancedOctree {
    let unit_size = initial_size / root_step as f32;
    let min_step = root_step >> params.min_depth;
    let finest_step = root_step >> params.max_depth;

    let mut octree = BalancedOctree::new(root_step);

    // Phase 1: SDF error-based refinement
    let mut queue: VecDeque<LeveledCell> = VecDeque::new();
    queue.push_back(LeveledCell {
        key: LatticePoint::new(0, 0, 0),
        step: root_step,
    });

    while let Some(cell) = queue.pop_front() {
        let cell_size = cell.step as f32 * unit_size;
        let circumradius = cell_circumradius(cell_size);
        let center = cell_center(cell.key, cell.step, origin, unit_size);

        if !cell_intersects_surface(solid, center, circumradius) {
            continue;
        }

        if should_subdivide(
            solid,
            center,
            cell_size,
            cell.step,
            min_step,
            finest_step,
            params.sdf_error_threshold,
        ) {
            let child_step = cell.step / 2;
            for sub_key in subdivide_cell(cell.key, cell.step) {
                queue.push_back(LeveledCell {
                    key: sub_key,
                    step: child_step,
                });
            }
        } else {
            octree.insert(cell);
        }
    }

    // Phase 2: Balance (ensure max 1 level difference)
    balance_octree(&mut octree, solid, origin, unit_size);
    check_octree_balance(&octree);

    octree
}

/// Ensure no neighbors differ by more than 1 level
fn balance_octree<S: Solid + ?Sized>(
    octree: &mut BalancedOctree,
    solid: &S,
    origin: Vec3,
    unit_size: f32,
) {
    let initial_step = octree.initial_step;
    let mut queue: VecDeque<LeveledCell> = octree.iter().copied().collect();

    while let Some(cell) = queue.pop_front() {
        if !octree.contains(&cell) {
            continue;
        }

        for dir in Direction::ALL {
            let neighbor_pos = neighbor_key(cell.key, dir, cell.step);

            let mut check_step = cell.step * 4;
            while check_step <= initial_step {
                let coarser_key = cell_at_step(neighbor_pos, check_step);
                let coarser = LeveledCell {
                    key: coarser_key,
                    step: check_step,
                };
                if octree.contains(&coarser) {
                    octree.remove(&coarser);
                    let child_step = coarser.step / 2;
                    for sub_key in subdivide_cell(coarser.key, coarser.step) {
                        let sub_cell = LeveledCell {
                            key: sub_key,
                            step: child_step,
                        };
                        let sub_size = child_step as f32 * unit_size;
                        let center = cell_center(sub_key, child_step, origin, unit_size);
                        let circumradius = cell_circumradius(sub_size);
                        if cell_intersects_surface(solid, center, circumradius) {
                            octree.insert(sub_cell);
                            queue.push_back(sub_cell);
                        }
                    }
                    break;
                }
                check_step *= 2;
            }
        }
    }
}

fn check_octree_balance(octree: &BalancedOctree) {
    for cell in octree.iter() {
        for dir in Direction::ALL {
            let status = octree.neighbor_status(*cell, dir);
            if status == NeighborStatus::Coarser {
                // Check for 2+ level difference
                let neighbor_pos = neighbor_key(cell.key, dir, cell.step);
                let mut check_step = cell.step * 4;
                while check_step <= octree.initial_step {
                    let coarser_key = cell_at_step(neighbor_pos, check_step);
                    let coarser = LeveledCell {
                        key: coarser_key,
                        step: check_step,
                    };
                    if octree.contains(&coarser) {
                        panic!(
                            "Unbalanced octree: Cell ({},{},{}) step {} has coarser neighbor step {}",
                            cell.key.x,
                            cell.key.y,
                            cell.key.z,
                            cell.step,
                            check_step,
                        );
                    }
                    check_step *= 2;
                }
            }
        }
    }
}

#[cfg(test)]
#[path = "tests/adaptive_test.rs"]
mod adaptive_test;
