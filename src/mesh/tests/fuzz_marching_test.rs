//! Exhaustive fuzz test for marching tetrahedra configurations.
//!
//! Tests all 512 possible inside/outside configurations of a single cube's
//! 9 SDF evaluation points (8 corners + BCC center), surrounded by empty
//! neighbor cubes on all sides, at a single uniform level.

#[cfg(test)]
mod tests {
    use crate::math::{LatticePoint, Vec3};
    use crate::mesh::adaptive::BalancedOctree;
    use crate::mesh::builder::MeshBuilder;
    use crate::mesh::cache::SdfCache;
    use crate::mesh::octree::{initial_step, LatticeGrid, LeveledCell};
    use crate::mesh::tetrahedra::{cube_corners, process_cell};
    use crate::solid::Solid;
    use rustc_hash::FxHashMap;

    /// Custom SDF returning prescribed values at specific lattice points.
    /// Points not in the map return +1.0 (outside).
    struct PointSdf {
        values: FxHashMap<LatticePoint, f32>,
        origin: Vec3,
        unit_size: f32,
    }

    impl Solid for PointSdf {
        fn sdf(&self, point: Vec3) -> f32 {
            let lx = ((point[0] - self.origin[0]) / self.unit_size).round() as i32;
            let ly = ((point[1] - self.origin[1]) / self.unit_size).round() as i32;
            let lz = ((point[2] - self.origin[2]) / self.unit_size).round() as i32;
            let lp = LatticePoint::new(lx, ly, lz);
            *self.values.get(&lp).unwrap_or(&1.0)
        }
    }

    fn make_octree(x_range: std::ops::Range<i32>, step: i32, root_step: i32) -> BalancedOctree {
        let mut octree = BalancedOctree::new(root_step);
        for ix in x_range {
            for iy in 0..3i32 {
                for iz in 0..3i32 {
                    octree.insert(LeveledCell::new(ix * step, iy * step, iz * step, step));
                }
            }
        }
        octree
    }

    fn process_grid(
        x_range: std::ops::Range<i32>,
        step: i32,
        root_step: i32,
        octree: &BalancedOctree,
        sdf: &PointSdf,
        grid: &LatticeGrid,
        interpolate: bool,
    ) -> MeshBuilder {
        let cache = SdfCache::new(root_step);
        let mut mesh = MeshBuilder::new();
        for ix in x_range {
            for iy in 0..3i32 {
                for iz in 0..3i32 {
                    let cell = LeveledCell::new(ix * step, iy * step, iz * step, step);
                    process_cell(cell, octree, sdf, grid, interpolate, &cache, &mut mesh);
                }
            }
        }
        mesh
    }

    /// Collect unique lattice points from multiple cubes (corners + BCC centers).
    fn collect_unique_points(
        keys: &[LatticePoint],
        step: i32,
    ) -> Vec<LatticePoint> {
        let mut points = Vec::new();
        for &key in keys {
            for &c in &cube_corners(key, step) {
                if !points.contains(&c) {
                    points.push(c);
                }
            }
            let cell = LeveledCell {
                key,
                step,
            };
            let center = cell.center();
            if !points.contains(&center) {
                points.push(center);
            }
        }
        points
    }

    /// Check mesh quality, returning an error description or None.
    fn check_mesh(mesh: &MeshBuilder) -> Option<String> {
        let degenerate = mesh.count_degenerate_triangles();
        let (watertight, boundary, non_manifold) = mesh.is_watertight();
        let winding = mesh.check_consistent_winding();

        if degenerate > 0 || !watertight || winding > 0 {
            Some(format!(
                "{} tris, {} degenerate, watertight={}, {} boundary, {} non-manifold, {} dup half-edges",
                mesh.triangles.len(),
                degenerate,
                watertight,
                boundary.len(),
                non_manifold.len(),
                winding,
            ))
        } else {
            None
        }
    }

    /// Test all 512 inside/outside configurations of a single cube (2^9).
    ///
    /// Places a single cube at the center of a 3x3x3 grid (all at the same
    /// level), assigns the 8 corners and BCC center of the center cube to
    /// inside or outside per the configuration bits, and all other points to
    /// outside. Processes all 27 cells, removes opposing faces, checks mesh.
    #[test]
    fn exhaustive_single_cube_configs() {
        let depth: u32 = 2;
        let initial_size = 4.0;
        let origin = [0.0, 0.0, 0.0];
        let root_step = initial_step(depth);
        let step = root_step >> depth;
        let grid = LatticeGrid::new(origin, initial_size, root_step);

        let keys = [LatticePoint::new(1 * step, 1 * step, 1 * step)];
        let points = collect_unique_points(&keys, step);
        assert_eq!(points.len(), 9); // 8 corners + 1 BCC center

        let octree = make_octree(0..3, step, root_step);
        let total: u32 = 1 << points.len();

        let mut fail_count = 0;
        let mut empty_count = 0;

        for config in 0..total {
            let mut values = FxHashMap::default();
            for (i, &pt) in points.iter().enumerate() {
                let inside = (config >> i) & 1 == 1;
                values.insert(pt, if inside { -1.0 } else { 1.0 });
            }

            let sdf = PointSdf {
                values,
                origin,
                unit_size: grid.unit_size,
            };
            let mesh = process_grid(0..3, step, root_step, &octree, &sdf, &grid, false);

            if mesh.triangles.is_empty() {
                empty_count += 1;
                continue;
            }

            if let Some(err) = check_mesh(&mesh) {
                fail_count += 1;
                eprintln!("FAIL config {:09b} ({config}): {err}", config);
            }
        }

        eprintln!(
            "\n=== Single Cube Exhaustive: {total} configs, {empty_count} empty, {} with mesh ===",
            total - empty_count,
        );

        if fail_count > 0 {
            panic!(
                "{fail_count} / {} configurations failed mesh quality checks",
                total - empty_count,
            );
        }
    }

    /// Deterministic PRNG (xorshift64).
    struct Rng(u64);

    impl Rng {
        fn new(seed: u64) -> Self {
            Self(seed.max(1))
        }

        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn bool(&mut self) -> bool {
            self.next() % 2 == 0
        }

        fn range(&mut self, lo: f32, hi: f32) -> f32 {
            let t = (self.next() & 0xFFFF_FFFF) as f32 / 0xFFFF_FFFF_u64 as f32;
            lo + t * (hi - lo)
        }
    }

    /// Fuzz test with random SDF magnitudes and interpolation enabled.
    ///
    /// Same structure as the exhaustive test, but uses random SDF values
    /// (not just ±1) and enables interpolation to test edge crossing placement.
    #[test]
    fn fuzz_single_cube_interpolated() {
        let depth: u32 = 2;
        let initial_size = 4.0;
        let origin = [0.0, 0.0, 0.0];
        let root_step = initial_step(depth);
        let step = root_step >> depth;
        let grid = LatticeGrid::new(origin, initial_size, root_step);

        let keys = [LatticePoint::new(1 * step, 1 * step, 1 * step)];
        let points = collect_unique_points(&keys, step);

        let octree = make_octree(0..3, step, root_step);
        let mut rng = Rng::new(0xDEAD_BEEF);

        let num_cases = 2000;
        let mut fail_count = 0;
        let mut empty_count = 0;

        for case in 0..num_cases {
            let mut values = FxHashMap::default();
            for &pt in &points {
                let inside = rng.bool();
                let magnitude = rng.range(0.01, 2.0);
                values.insert(pt, if inside { -magnitude } else { magnitude });
            }

            let sdf = PointSdf {
                values,
                origin,
                unit_size: grid.unit_size,
            };
            let mesh = process_grid(0..3, step, root_step, &octree, &sdf, &grid, true);

            if mesh.triangles.is_empty() {
                empty_count += 1;
                continue;
            }

            if let Some(err) = check_mesh(&mesh) {
                fail_count += 1;
                eprintln!("FAIL case {case}: {err}");
            }
        }

        eprintln!(
            "\n=== Single Cube Fuzz (interpolated): {num_cases} cases, {empty_count} empty, {} with mesh ===",
            num_cases - empty_count,
        );

        if fail_count > 0 {
            panic!(
                "{fail_count} / {} cases failed mesh quality checks",
                num_cases - empty_count,
            );
        }
    }

    /// Test all 16384 inside/outside configurations of two X-adjacent cubes (2^14).
    ///
    /// Two cubes share a face (4 corners), giving 14 unique SDF evaluation
    /// points. Surrounded by a 4x3x3 grid of uniform-level cells (1 layer of
    /// padding on all sides). Tests that opposing internal faces cancel
    /// correctly and the surface closes across the shared face.
    ///
    /// Run: cargo test exhaustive_two_cube -- --ignored --nocapture
    #[test]
    #[ignore]
    fn exhaustive_two_cube_configs() {
        let depth: u32 = 2;
        let initial_size = 4.0;
        let origin = [0.0, 0.0, 0.0];
        let root_step = initial_step(depth);
        let step = root_step >> depth;
        let grid = LatticeGrid::new(origin, initial_size, root_step);

        let keys = [
            LatticePoint::new(1 * step, 1 * step, 1 * step),
            LatticePoint::new(2 * step, 1 * step, 1 * step),
        ];
        let points = collect_unique_points(&keys, step);
        assert_eq!(points.len(), 14); // 12 unique corners + 2 BCC centers

        // 4x3x3 grid: 1 layer of padding around both cubes
        let octree = make_octree(0..4, step, root_step);
        let total: u32 = 1 << points.len(); // 16384

        let mut fail_count = 0;
        let mut empty_count = 0;

        for config in 0..total {
            let mut values = FxHashMap::default();
            for (i, &pt) in points.iter().enumerate() {
                let inside = (config >> i) & 1 == 1;
                values.insert(pt, if inside { -1.0 } else { 1.0 });
            }

            let sdf = PointSdf {
                values,
                origin,
                unit_size: grid.unit_size,
            };
            let mesh = process_grid(0..4, step, root_step, &octree, &sdf, &grid, false);

            if mesh.triangles.is_empty() {
                empty_count += 1;
                continue;
            }

            if let Some(err) = check_mesh(&mesh) {
                fail_count += 1;
                eprintln!("FAIL config {config}: {err}");
            }
        }

        eprintln!(
            "\n=== Two Cube Exhaustive: {total} configs, {empty_count} empty, {} with mesh ===",
            total - empty_count,
        );

        if fail_count > 0 {
            panic!(
                "{fail_count} / {} configurations failed mesh quality checks",
                total - empty_count,
            );
        }
    }
}
