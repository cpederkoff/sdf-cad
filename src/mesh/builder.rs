use crate::math::{LatticePoint, Vec3};
use crate::solid::Solid;
use rustc_hash::FxHashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Canonical edge identity: pair of lattice endpoints in sorted order.
/// Two tet edges that share the same lattice endpoints produce the same
/// interpolated vertex, so this replaces float-quantization-based dedup.
#[derive(Clone, Copy, Debug, Hash, Eq, PartialEq, Ord, PartialOrd)]
pub struct EdgeKey(pub LatticePoint, pub LatticePoint);

impl EdgeKey {
    /// Create canonically ordered edge key (smaller point first).
    pub fn new(a: LatticePoint, b: LatticePoint) -> Self {
        if a <= b {
            Self(a, b)
        } else {
            Self(b, a)
        }
    }
}

/// Mesh builder that deduplicates vertices by exact edge identity.
#[derive(Clone)]
pub struct MeshBuilder {
    pub vertices: Vec<Vec3>,
    edge_map: FxHashMap<EdgeKey, usize>,
    edge_keys: Vec<EdgeKey>,
    pub triangles: Vec<[usize; 3]>,
}

impl MeshBuilder {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            edge_map: FxHashMap::default(),
            edge_keys: Vec::new(),
            triangles: Vec::new(),
        }
    }

    /// Create a MeshBuilder from raw vertex positions and triangle indices.
    /// Uses dummy EdgeKeys since dedup is not needed for pre-built meshes.
    pub fn from_raw(vertices: Vec<Vec3>, triangles: Vec<[usize; 3]>) -> Self {
        let origin = LatticePoint::new(0, 0, 0);
        let mut edge_map = FxHashMap::default();
        let mut edge_keys = Vec::with_capacity(vertices.len());
        for i in 0..vertices.len() {
            let ek = EdgeKey::new(LatticePoint::new(i as i32, 0, 0), origin);
            edge_keys.push(ek);
            edge_map.insert(ek, i);
        }
        Self {
            vertices,
            edge_map,
            edge_keys,
            triangles,
        }
    }

    fn add_vertex_by_edge(&mut self, ek: EdgeKey, pos: Vec3) -> usize {
        let vertices = &mut self.vertices;
        let edge_keys = &mut self.edge_keys;
        *self.edge_map.entry(ek).or_insert_with(|| {
            let idx = vertices.len();
            vertices.push(pos);
            edge_keys.push(ek);
            idx
        })
    }

    pub fn add_triangle_by_edge(
        &mut self,
        ek0: EdgeKey,
        p0: Vec3,
        ek1: EdgeKey,
        p1: Vec3,
        ek2: EdgeKey,
        p2: Vec3,
    ) {
        let i0 = self.add_vertex_by_edge(ek0, p0);
        let i1 = self.add_vertex_by_edge(ek1, p1);
        let i2 = self.add_vertex_by_edge(ek2, p2);

        // Skip degenerate triangles (where any two vertices are the same)
        if i0 == i1 || i1 == i2 || i2 == i0 {
            return;
        }

        self.triangles.push([i0, i1, i2]);
    }

    /// Reorder vertices and triangles into a deterministic canonical order.
    /// Vertices are sorted by edge key; triangles are sorted by their
    /// (remapped) vertex indices.
    pub fn canonicalize(&mut self) {
        if self.vertices.is_empty() {
            return;
        }

        // Build (edge_key, old_index) pairs and sort by edge key
        let mut order: Vec<(usize, EdgeKey)> = self
            .edge_keys
            .iter()
            .enumerate()
            .map(|(i, &ek)| (i, ek))
            .collect();
        order.sort_unstable_by_key(|&(_, ek)| ek);

        // Build old->new index mapping
        let mut remap = vec![0usize; self.vertices.len()];
        let mut new_vertices = Vec::with_capacity(self.vertices.len());
        let mut new_edge_keys = Vec::with_capacity(self.edge_keys.len());
        for (new_idx, &(old_idx, ek)) in order.iter().enumerate() {
            remap[old_idx] = new_idx;
            new_vertices.push(self.vertices[old_idx]);
            new_edge_keys.push(ek);
        }
        self.vertices = new_vertices;
        self.edge_keys = new_edge_keys;

        // Rebuild edge_map
        self.edge_map.clear();
        for (i, &ek) in self.edge_keys.iter().enumerate() {
            self.edge_map.insert(ek, i);
        }

        // Remap and sort triangles
        for tri in &mut self.triangles {
            tri[0] = remap[tri[0]];
            tri[1] = remap[tri[1]];
            tri[2] = remap[tri[2]];
        }
        self.triangles.sort_unstable_by_key(|t| (t[0], t[1], t[2]));
    }

    /// Remove unreferenced vertices and rebuild edge_map/edge_keys.
    /// Called after decimation to clean up orphaned vertices.
    pub fn compact(&mut self) {
        let nv = self.vertices.len();
        let mut used = vec![false; nv];
        for tri in &self.triangles {
            for &vi in tri {
                used[vi] = true;
            }
        }

        let mut remap = vec![0usize; nv];
        let mut new_vertices = Vec::new();
        let mut new_edge_keys = Vec::new();
        for i in 0..nv {
            if used[i] {
                remap[i] = new_vertices.len();
                new_vertices.push(self.vertices[i]);
                new_edge_keys.push(self.edge_keys[i]);
            }
        }

        for tri in &mut self.triangles {
            tri[0] = remap[tri[0]];
            tri[1] = remap[tri[1]];
            tri[2] = remap[tri[2]];
        }

        self.edge_map.clear();
        for (i, &ek) in new_edge_keys.iter().enumerate() {
            self.edge_map.insert(ek, i);
        }

        self.vertices = new_vertices;
        self.edge_keys = new_edge_keys;
    }

    /// Merge another MeshBuilder into this one, re-deduplicating vertices by edge key.
    pub fn merge(&mut self, other: MeshBuilder) {
        for tri in &other.triangles {
            let ek0 = other.edge_keys[tri[0]];
            let ek1 = other.edge_keys[tri[1]];
            let ek2 = other.edge_keys[tri[2]];
            let p0 = other.vertices[tri[0]];
            let p1 = other.vertices[tri[1]];
            let p2 = other.vertices[tri[2]];
            self.add_triangle_by_edge(ek0, p0, ek1, p1, ek2, p2);
        }
    }

    /// Check mesh for quality issues, returns count of degenerate triangles found
    pub fn count_degenerate_triangles(&self) -> usize {
        self.triangles
            .iter()
            .filter(|t| t[0] == t[1] || t[1] == t[2] || t[2] == t[0])
            .count()
    }

    /// Check if mesh is watertight (every edge shared by exactly 2 triangles)
    /// Returns (is_watertight, boundary_edges, non_manifold_edges)
    pub fn is_watertight(&self) -> (bool, Vec<(usize, usize)>, Vec<(usize, usize)>) {
        let mut edge_count: FxHashMap<(usize, usize), i32> = FxHashMap::default();

        for tri in &self.triangles {
            // Add edges in canonical order (smaller index first)
            for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let edge = if a < b { (a, b) } else { (b, a) };
                *edge_count.entry(edge).or_insert(0) += 1;
            }
        }

        let mut boundary_edges = Vec::new();
        let mut non_manifold_edges = Vec::new();

        for (edge, count) in &edge_count {
            match count {
                1 => boundary_edges.push(*edge),
                2 => {} // Perfect - manifold edge
                _ => non_manifold_edges.push(*edge),
            }
        }

        let is_watertight = boundary_edges.is_empty() && non_manifold_edges.is_empty();
        (is_watertight, boundary_edges, non_manifold_edges)
    }

    /// Check that winding is consistent: every directed half-edge (a,b) should appear
    /// exactly once. On a correctly-oriented watertight manifold, each undirected edge
    /// has one (a,b) and one (b,a). Flipped normals cause some half-edges to appear twice.
    /// Returns the number of duplicate directed half-edges.
    pub fn check_consistent_winding(&self) -> usize {
        let mut half_edge_count: FxHashMap<(usize, usize), usize> = FxHashMap::default();
        for tri in &self.triangles {
            for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                *half_edge_count.entry((a, b)).or_insert(0) += 1;
            }
        }
        half_edge_count.values().filter(|&&c| c > 1).count()
    }

    /// Compute SDF error statistics: (mean, max, p95) of |SDF(centroid)| across all triangles.
    pub fn sdf_error_stats<S: Solid + ?Sized>(&self, solid: &S) -> (f32, f32, f32) {
        let n = self.triangles.len();
        if n == 0 {
            return (0.0, 0.0, 0.0);
        }
        let mut errors: Vec<f32> = self
            .triangles
            .iter()
            .map(|tri| {
                let v0 = self.vertices[tri[0]];
                let v1 = self.vertices[tri[1]];
                let v2 = self.vertices[tri[2]];
                let centroid = [
                    (v0[0] + v1[0] + v2[0]) / 3.0,
                    (v0[1] + v1[1] + v2[1]) / 3.0,
                    (v0[2] + v1[2] + v2[2]) / 3.0,
                ];
                solid.sdf(centroid).abs()
            })
            .collect();
        errors.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mean = errors.iter().sum::<f32>() / n as f32;
        let max = errors[n - 1];
        let p95 = errors[(n as f32 * 0.95) as usize];
        (mean, max, p95)
    }

    fn triangle_area(&self, tri: &[usize; 3]) -> f32 {
        let v0 = self.vertices[tri[0]];
        let v1 = self.vertices[tri[1]];
        let v2 = self.vertices[tri[2]];
        let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
        let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
        let cross = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt()
    }

    /// Evaluate SDF at triangle centroids to diagnose mesh accuracy and refinement efficiency.
    pub fn check_sdf_accuracy<S: Solid + ?Sized>(&self, solid: &S) {
        let n = self.triangles.len();
        if n == 0 {
            println!("SDF accuracy: no triangles");
            return;
        }

        let mut errors: Vec<(usize, f32, f32, Vec3)> = Vec::with_capacity(n);
        let mut total_area: f32 = 0.0;

        for (i, tri) in self.triangles.iter().enumerate() {
            let v0 = self.vertices[tri[0]];
            let v1 = self.vertices[tri[1]];
            let v2 = self.vertices[tri[2]];
            let centroid = [
                (v0[0] + v1[0] + v2[0]) / 3.0,
                (v0[1] + v1[1] + v2[1]) / 3.0,
                (v0[2] + v1[2] + v2[2]) / 3.0,
            ];
            let sdf_err = solid.sdf(centroid).abs();
            let area = self.triangle_area(tri);
            total_area += area;
            errors.push((i, sdf_err, area, centroid));
        }

        // Compute stats
        let mut sdf_errors: Vec<f32> = errors.iter().map(|e| e.1).collect();
        sdf_errors.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mean_err: f32 = sdf_errors.iter().sum::<f32>() / n as f32;
        let max_err = sdf_errors[n - 1];
        let p95_err = sdf_errors[(n as f32 * 0.95) as usize];

        let mut areas: Vec<f32> = errors.iter().map(|e| e.2).collect();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let min_area = areas[0];
        let max_area = areas[n - 1];
        let median_area = areas[n / 2];

        println!("\n=== SDF Accuracy Report ===");
        println!("Triangles: {}  Total area: {:.6}", n, total_area);
        println!(
            "Centroid SDF error — max: {:.6}  mean: {:.6}  p95: {:.6}",
            max_err, mean_err, p95_err
        );
        println!(
            "Triangle area — min: {:.2e}  max: {:.2e}  median: {:.2e}",
            min_area, max_area, median_area
        );

        // Top 10 worst by SDF error
        let mut by_error = errors.clone();
        by_error.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        println!("\nTop 10 worst triangles (highest centroid SDF error):");
        println!(
            "  {:>5}  {:>10}  {:>10}  location",
            "tri", "sdf_err", "area"
        );
        for entry in by_error.iter().take(10) {
            let (i, err, area, c) = entry;
            println!(
                "  {:>5}  {:>10.6}  {:>10.2e}  ({:.3}, {:.3}, {:.3})",
                i, err, area, c[0], c[1], c[2]
            );
        }

        // Top 10 smallest triangles (over-refinement candidates)
        let mut by_area = errors;
        by_area.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
        println!("\nTop 10 smallest triangles (over-refinement candidates):");
        println!(
            "  {:>5}  {:>10}  {:>10}  location",
            "tri", "area", "sdf_err"
        );
        for entry in by_area.iter().take(10) {
            let (i, err, area, c) = entry;
            println!(
                "  {:>5}  {:>10.2e}  {:>10.6}  ({:.3}, {:.3}, {:.3})",
                i, area, err, c[0], c[1], c[2]
            );
        }
    }

    pub fn save(&self, path: &str) -> Result<(), String> {
        let ext = Path::new(path)
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_lowercase();
        match ext.as_str() {
            "obj" => self.write_obj(path).map_err(|e| format!("{e}")),
            "stl" => self.write_stl(path).map_err(|e| format!("{e}")),
            other => Err(format!("Unsupported format: .{other}")),
        }
    }

    fn write_obj(&self, path: &str) -> std::io::Result<()> {
        let mut file = BufWriter::new(File::create(path)?);
        for v in &self.vertices {
            writeln!(file, "v {} {} {}", v[0], v[1], v[2])?;
        }
        for tri in &self.triangles {
            // OBJ uses 1-based indices
            writeln!(file, "f {} {} {}", tri[0] + 1, tri[1] + 1, tri[2] + 1)?;
        }
        Ok(())
    }

    fn write_stl(&self, path: &str) -> std::io::Result<()> {
        let mut file = BufWriter::new(File::create(path)?);
        // 80-byte header
        file.write_all(&[0u8; 80])?;
        // Triangle count
        file.write_all(&(self.triangles.len() as u32).to_le_bytes())?;
        for tri in &self.triangles {
            let v0 = self.vertices[tri[0]];
            let v1 = self.vertices[tri[1]];
            let v2 = self.vertices[tri[2]];
            let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
            let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
            let n = [
                e1[1] * e2[2] - e1[2] * e2[1],
                e1[2] * e2[0] - e1[0] * e2[2],
                e1[0] * e2[1] - e1[1] * e2[0],
            ];
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            let n = if len > 0.0 {
                [n[0] / len, n[1] / len, n[2] / len]
            } else {
                [0.0f32, 0.0, 0.0]
            };
            // Normal
            for c in &n {
                file.write_all(&c.to_le_bytes())?;
            }
            // 3 vertices
            for &vi in tri {
                let v = self.vertices[vi];
                for c in &v {
                    file.write_all(&c.to_le_bytes())?;
                }
            }
            // Attribute byte count
            file.write_all(&0u16.to_le_bytes())?;
        }
        Ok(())
    }
}
