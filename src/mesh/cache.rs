//! SDF value cache keyed by lattice point.
//!
//! Eliminates redundant SDF evaluations for shared vertices across tetrahedra.
//! Uses `DashMap` for thread-safe concurrent access during rayon parallel processing.
//!
//! The cache pins a `root_step` that defines the lattice coordinate system.
//! Callers sharing a cache across progressive refinement runs (increasing
//! `max_depth`) must create the cache with `root_step = initial_step(finest_depth)`
//! so that lattice points remain stable across runs.

use std::sync::OnceLock;

use dashmap::DashMap;
use rustc_hash::FxHashMap;

use super::octree::LatticeGrid;
use crate::math::{LatticePoint, Vec3};
use crate::solid::Solid;

#[inline]
fn trilinear(
    c000: f32, c100: f32, c010: f32, c110: f32,
    c001: f32, c101: f32, c011: f32, c111: f32,
    tx: f32, ty: f32, tz: f32,
) -> f32 {
    let c00 = c000 * (1.0 - tx) + c100 * tx;
    let c10 = c010 * (1.0 - tx) + c110 * tx;
    let c01 = c001 * (1.0 - tx) + c101 * tx;
    let c11 = c011 * (1.0 - tx) + c111 * tx;
    let c0 = c00 * (1.0 - ty) + c10 * ty;
    let c1 = c01 * (1.0 - ty) + c11 * ty;
    c0 * (1.0 - tz) + c1 * tz
}

pub struct SdfCache {
    map: DashMap<LatticePoint, f32>,
    root_step: i32,
    grid: OnceLock<LatticeGrid>,
}

impl SdfCache {
    pub fn new(root_step: i32) -> Self {
        Self {
            map: DashMap::new(),
            root_step,
            grid: OnceLock::new(),
        }
    }

    pub fn root_step(&self) -> i32 {
        self.root_step
    }

    /// Store the lattice grid used during mesh generation.
    /// Called once by `generate_bcc_mesh`; subsequent calls are ignored.
    pub fn set_grid(&self, grid: LatticeGrid) {
        let _ = self.grid.set(grid);
    }

    pub fn grid(&self) -> Option<&LatticeGrid> {
        self.grid.get()
    }

    pub fn get_or_eval<S: Solid + ?Sized>(
        &self,
        point: LatticePoint,
        solid: &S,
        grid: &LatticeGrid,
    ) -> f32 {
        *self
            .map
            .entry(point)
            .or_insert_with(|| solid.sdf(grid.to_world(point)))
    }

    /// Look up a cached value, or evaluate + insert on miss.
    /// Uses a fast read-lock first, falling back to write-lock only on miss.
    fn get_or_insert(&self, lp: LatticePoint, solid: &dyn Solid, grid: &LatticeGrid) -> f32 {
        if let Some(v) = self.map.get(&lp) {
            return *v;
        }
        *self.map.entry(lp).or_insert_with(|| solid.sdf(grid.to_world(lp)))
    }

    /// Decompose a world-space point into lattice cell corner + fractional offsets.
    /// Returns `None` only if the grid hasn't been set.
    fn lattice_coords(&self, world: Vec3) -> Option<(&LatticeGrid, i32, i32, i32, f32, f32, f32)> {
        let grid = self.grid.get()?;
        let inv = 1.0 / grid.unit_size;
        let fx = (world[0] - grid.origin[0]) * inv;
        let fy = (world[1] - grid.origin[1]) * inv;
        let fz = (world[2] - grid.origin[2]) * inv;
        let x0 = fx.floor() as i32;
        let y0 = fy.floor() as i32;
        let z0 = fz.floor() as i32;
        Some((grid, x0, y0, z0, fx - x0 as f32, fy - y0 as f32, fz - z0 as f32))
    }

    /// Trilinearly interpolate SDF values at a world-space point.
    ///
    /// Looks up the 8 surrounding lattice points. Missing entries are evaluated
    /// and inserted on demand. Returns `None` only if the grid hasn't been set.
    pub fn interpolate_sdf(&self, world: Vec3, solid: &dyn Solid) -> Option<f32> {
        let (grid, x0, y0, z0, tx, ty, tz) = self.lattice_coords(world)?;

        let get = |dx, dy, dz| self.get_or_insert(LatticePoint::new(x0 + dx, y0 + dy, z0 + dz), solid, grid);

        let c000 = get(0, 0, 0);
        let c100 = get(1, 0, 0);
        let c010 = get(0, 1, 0);
        let c110 = get(1, 1, 0);
        let c001 = get(0, 0, 1);
        let c101 = get(1, 0, 1);
        let c011 = get(0, 1, 1);
        let c111 = get(1, 1, 1);

        Some(trilinear(
            c000, c100, c010, c110, c001, c101, c011, c111, tx, ty, tz,
        ))
    }

    /// Trilinearly interpolate SDF value AND compute the analytical gradient,
    /// all from one set of 8 lattice lookups.
    ///
    /// Returns `(value, world_space_gradient)`. Missing lattice entries are
    /// evaluated and inserted on demand. Returns `None` only if the grid
    /// hasn't been set.
    pub fn interpolate_sdf_with_gradient(
        &self,
        world: Vec3,
        solid: &dyn Solid,
    ) -> Option<(f32, Vec3)> {
        let (grid, x0, y0, z0, tx, ty, tz) = self.lattice_coords(world)?;

        let get = |dx, dy, dz| self.get_or_insert(LatticePoint::new(x0 + dx, y0 + dy, z0 + dz), solid, grid);

        let c000 = get(0, 0, 0);
        let c100 = get(1, 0, 0);
        let c010 = get(0, 1, 0);
        let c110 = get(1, 1, 0);
        let c001 = get(0, 0, 1);
        let c101 = get(1, 0, 1);
        let c011 = get(0, 1, 1);
        let c111 = get(1, 1, 1);

        let value = trilinear(
            c000, c100, c010, c110, c001, c101, c011, c111, tx, ty, tz,
        );

        // Analytical partial derivatives of trilinear interpolation
        // w.r.t. fractional coords (tx, ty, tz), then convert to world space.
        let inv_unit = 1.0 / grid.unit_size;

        let ntx = 1.0 - tx;
        let nty = 1.0 - ty;
        let ntz = 1.0 - tz;

        let df_dtx = (c100 - c000) * nty * ntz
            + (c110 - c010) * ty * ntz
            + (c101 - c001) * nty * tz
            + (c111 - c011) * ty * tz;

        let df_dty = (c010 - c000) * ntx * ntz
            + (c110 - c100) * tx * ntz
            + (c011 - c001) * ntx * tz
            + (c111 - c101) * tx * tz;

        let df_dtz = (c001 - c000) * ntx * nty
            + (c101 - c100) * tx * nty
            + (c011 - c010) * ntx * ty
            + (c111 - c110) * tx * ty;

        let gradient = [df_dtx * inv_unit, df_dty * inv_unit, df_dtz * inv_unit];

        Some((value, gradient))
    }

    /// Create a read-only snapshot for single-threaded use (no locking overhead).
    /// Returns `None` if the grid hasn't been set.
    pub fn snapshot(&self) -> Option<CacheSnapshot> {
        let grid = *self.grid.get()?;
        let map: FxHashMap<LatticePoint, f32> =
            self.map.iter().map(|r| (*r.key(), *r.value())).collect();
        Some(CacheSnapshot { map, grid })
    }
}

/// Lock-free read-only snapshot of an `SdfCache` for single-threaded optimization.
/// All lookups are plain `FxHashMap::get` — no shard locks, no write paths.
pub struct CacheSnapshot {
    map: FxHashMap<LatticePoint, f32>,
    grid: LatticeGrid,
}

impl CacheSnapshot {
    #[inline]
    fn get(&self, lp: LatticePoint) -> Option<f32> {
        self.map.get(&lp).copied()
    }

    #[inline]
    fn lattice_coords(&self, world: Vec3) -> (i32, i32, i32, f32, f32, f32) {
        let inv = 1.0 / self.grid.unit_size;
        let fx = (world[0] - self.grid.origin[0]) * inv;
        let fy = (world[1] - self.grid.origin[1]) * inv;
        let fz = (world[2] - self.grid.origin[2]) * inv;
        let x0 = fx.floor() as i32;
        let y0 = fy.floor() as i32;
        let z0 = fz.floor() as i32;
        (x0, y0, z0, fx - x0 as f32, fy - y0 as f32, fz - z0 as f32)
    }

    /// Fetch the 8 corner values for trilinear interpolation.
    /// Returns `None` if any corner is missing from the snapshot.
    #[inline]
    fn fetch_corners(&self, x0: i32, y0: i32, z0: i32) -> Option<[f32; 8]> {
        Some([
            self.get(LatticePoint::new(x0, y0, z0))?,
            self.get(LatticePoint::new(x0 + 1, y0, z0))?,
            self.get(LatticePoint::new(x0, y0 + 1, z0))?,
            self.get(LatticePoint::new(x0 + 1, y0 + 1, z0))?,
            self.get(LatticePoint::new(x0, y0, z0 + 1))?,
            self.get(LatticePoint::new(x0 + 1, y0, z0 + 1))?,
            self.get(LatticePoint::new(x0, y0 + 1, z0 + 1))?,
            self.get(LatticePoint::new(x0 + 1, y0 + 1, z0 + 1))?,
        ])
    }

    /// Trilinearly interpolate cached SDF at a world-space point.
    /// Returns `None` if any surrounding lattice point is missing.
    pub fn interpolate_sdf(&self, world: Vec3) -> Option<f32> {
        let (x0, y0, z0, tx, ty, tz) = self.lattice_coords(world);
        let [c000, c100, c010, c110, c001, c101, c011, c111] =
            self.fetch_corners(x0, y0, z0)?;
        Some(trilinear(c000, c100, c010, c110, c001, c101, c011, c111, tx, ty, tz))
    }

    /// Trilinearly interpolate SDF value + analytical gradient from 8 lookups.
    /// Returns `None` if any surrounding lattice point is missing.
    pub fn interpolate_sdf_with_gradient(&self, world: Vec3) -> Option<(f32, Vec3)> {
        let (x0, y0, z0, tx, ty, tz) = self.lattice_coords(world);
        let [c000, c100, c010, c110, c001, c101, c011, c111] =
            self.fetch_corners(x0, y0, z0)?;

        let value = trilinear(c000, c100, c010, c110, c001, c101, c011, c111, tx, ty, tz);

        let inv_unit = 1.0 / self.grid.unit_size;
        let ntx = 1.0 - tx;
        let nty = 1.0 - ty;
        let ntz = 1.0 - tz;

        let df_dtx = (c100 - c000) * nty * ntz
            + (c110 - c010) * ty * ntz
            + (c101 - c001) * nty * tz
            + (c111 - c011) * ty * tz;

        let df_dty = (c010 - c000) * ntx * ntz
            + (c110 - c100) * tx * ntz
            + (c011 - c001) * ntx * tz
            + (c111 - c101) * tx * tz;

        let df_dtz = (c001 - c000) * ntx * nty
            + (c101 - c100) * tx * nty
            + (c011 - c010) * ntx * ty
            + (c111 - c110) * tx * ty;

        Some((value, [df_dtx * inv_unit, df_dty * inv_unit, df_dtz * inv_unit]))
    }
}
