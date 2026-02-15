//! Adaptive BCC (body-centered cubic) mesh generation.
//!
//! Builds a balanced octree refined by surface curvature, then generates 12
//! marching tetrahedra per leaf cell (two per cube face, apex at cell center).
//! Where a coarse cell neighbors finer cells, transition tets use face-center
//! fans to prevent T-junctions at level boundaries.

use super::adaptive::build_octree;
pub use super::adaptive::BccMeshParams;
use super::builder::MeshBuilder;
pub use super::cache::SdfCache;
pub use super::octree::initial_step;
use super::octree::LatticeGrid;
use super::tetrahedra::process_cell;
use crate::math::Vec3;
use crate::solid::Solid;
use rayon::prelude::*;

/// Generate a seamless BCC mesh with adaptive refinement.
///
/// The `cache` pins the lattice coordinate system via its `root_step`. For
/// progressive refinement (increasing `max_depth`), create a single cache with
/// `SdfCache::new(initial_step(finest_depth))` and reuse it across calls.
pub fn generate_bcc_mesh<S: Solid + ?Sized>(
    solid: &S,
    bounding_center: Vec3,
    bounding_radius: f32,
    params: &BccMeshParams,
    cache: &SdfCache,
) -> MeshBuilder {
    let initial_size = bounding_radius * 2.0;
    // Small irrational offset so the octree grid never aligns exactly with
    // axis-aligned SDF surfaces (which would cause SDF=0 at grid vertices,
    // leading to ambiguous marching configurations and non-manifold edges).
    let jitter = initial_size * 1.2345678e-5;
    let origin = [
        bounding_center[0] - bounding_radius + jitter,
        bounding_center[1] - bounding_radius + jitter,
        bounding_center[2] - bounding_radius + jitter,
    ];

    let root_step = cache.root_step();
    let octree = build_octree(solid, origin, initial_size, params, root_step);
    let grid = LatticeGrid::new(origin, initial_size, root_step);
    cache.set_grid(grid);

    // Sort cells for deterministic ordering (HashSet iteration is unordered).
    let mut cells: Vec<_> = octree.iter().copied().collect();
    cells.sort_unstable_by_key(|c| (c.step, c.key.x, c.key.y, c.key.z));

    let mut mesh = cells
        .par_iter()
        .fold(
            || MeshBuilder::new(),
            |mut local_mesh, &cell| {
                process_cell(
                    cell,
                    &octree,
                    solid,
                    &grid,
                    params.interpolate,
                    cache,
                    &mut local_mesh,
                );
                local_mesh
            },
        )
        .reduce(
            || MeshBuilder::new(),
            |mut acc, partial| {
                acc.merge(partial);
                acc
            },
        );

    mesh.canonicalize();
    mesh
}

#[cfg(test)]
#[path = "tests/bcc_test.rs"]
mod bcc_test;
