//! Coplanar vertex decimation for triangle meshes.
//!
//! Removes interior mesh vertices whose triangle neighborhoods are coplanar,
//! re-triangulating the surrounding polygon via ear clipping. Each removed
//! vertex eliminates exactly 2 triangles (N adjacent → N-2 ear-clipped).
//! Particularly effective on BCC marching-tet output where face-center fans
//! produce many coplanar triangles on flat surface patches.

use super::builder::MeshBuilder;
use crate::math::{Vec3, Vec3Ext};
use rustc_hash::{FxHashMap, FxHashSet};

/// Remove interior mesh vertices whose triangle neighborhoods are coplanar,
/// re-triangulating the surrounding polygon to reduce triangle count.
///
/// `normal_dot_threshold` is the minimum dot product between adjacent triangle
/// normals for a vertex to be considered flat (e.g. 0.999 ≈ 2.6°).
pub fn decimate_flat(mesh: &mut MeshBuilder, normal_dot_threshold: f32) {
    loop {
        let removed = decimate_pass(mesh, normal_dot_threshold);
        if removed == 0 {
            break;
        }
    }
    mesh.compact();
}

/// Collapse short edges to eliminate sliver triangles along creases.
///
/// `min_edge_ratio` is the minimum allowed ratio of an edge's length to the longest
/// edge in its triangle (e.g. 0.3 means edges shorter than 30% of their triangle's
/// longest edge are collapsed). Each collapse merges two vertices and removes 2 triangles.
pub fn collapse_short_edges(mesh: &mut MeshBuilder, min_edge_ratio: f32) {
    let ratio_sq = min_edge_ratio * min_edge_ratio;
    loop {
        let collapsed = collapse_pass(mesh, ratio_sq);
        if collapsed == 0 {
            break;
        }
    }
    mesh.compact();
}

fn collapse_pass(mesh: &mut MeshBuilder, ratio_sq: f32) -> usize {
    let nv = mesh.vertices.len();
    let nt = mesh.triangles.len();

    // Build vertex → triangle adjacency
    let mut vert_tris: Vec<Vec<usize>> = vec![Vec::new(); nv];
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        for &vi in tri {
            vert_tris[vi].push(ti);
        }
    }

    // Collect candidate short edges with their squared length
    let mut candidates: Vec<(usize, usize, f32)> = Vec::new();
    let mut seen: FxHashSet<(usize, usize)> = FxHashSet::default();

    for tri in mesh.triangles.iter() {
        let pairs = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
        let lens_sq = [
            mesh.vertices[pairs[0].0].sub(mesh.vertices[pairs[0].1]).length_sq(),
            mesh.vertices[pairs[1].0].sub(mesh.vertices[pairs[1].1]).length_sq(),
            mesh.vertices[pairs[2].0].sub(mesh.vertices[pairs[2].1]).length_sq(),
        ];
        let max_sq = lens_sq[0].max(lens_sq[1]).max(lens_sq[2]);

        for (i, &(a, b)) in pairs.iter().enumerate() {
            if lens_sq[i] < max_sq * ratio_sq {
                let e = edge(a, b);
                if seen.insert(e) {
                    candidates.push((e.0, e.1, lens_sq[i]));
                }
            }
        }
    }

    // Process shortest edges first
    candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    let mut dead = vec![false; nt];
    let mut touched = vec![false; nv]; // vertices involved in a collapse this pass
    let mut count = 0;

    for (u, v, _) in candidates {
        if touched[u] || touched[v] {
            continue;
        }

        // Skip if either vertex has dead adjacent triangles (from earlier collapse)
        if vert_tris[u].iter().any(|&ti| dead[ti]) {
            continue;
        }
        if vert_tris[v].iter().any(|&ti| dead[ti]) {
            continue;
        }

        // Link condition: u and v must share exactly 2 neighbor vertices (the wing vertices).
        // Violating this creates non-manifold topology.
        let u_neighbors: FxHashSet<usize> = vert_tris[u]
            .iter()
            .flat_map(|&ti| mesh.triangles[ti].iter().copied())
            .filter(|&vi| vi != u)
            .collect();
        let shared: Vec<usize> = vert_tris[v]
            .iter()
            .flat_map(|&ti| mesh.triangles[ti].iter().copied())
            .filter(|&vi| vi != v && vi != u && u_neighbors.contains(&vi))
            .collect::<FxHashSet<_>>()
            .into_iter()
            .collect();
        if shared.len() != 2 {
            continue;
        }

        // Check that collapsing v→u doesn't invert any triangle normal
        if !collapse_is_safe(u, v, &mesh.vertices, &mesh.triangles, &vert_tris[v], &dead) {
            // Try the other direction: u→v
            if !collapse_is_safe(v, u, &mesh.vertices, &mesh.triangles, &vert_tris[u], &dead) {
                continue;
            }
            // Collapse u→v instead
            perform_collapse(
                v, u, &mut mesh.triangles, &vert_tris[u], &mut dead,
            );
        } else {
            perform_collapse(
                u, v, &mut mesh.triangles, &vert_tris[v], &mut dead,
            );
        }

        touched[u] = true;
        touched[v] = true;
        count += 1;
    }

    // Remove dead triangles
    if count > 0 {
        let mut write = 0;
        for read in 0..mesh.triangles.len() {
            if dead[read] {
                continue;
            }
            mesh.triangles[write] = mesh.triangles[read];
            write += 1;
        }
        mesh.triangles.truncate(write);
    }

    count
}

/// Check that remapping `src→dst` (keeping dst's position) doesn't invert any triangle.
fn collapse_is_safe(
    dst: usize,
    src: usize,
    vertices: &[Vec3],
    triangles: &[[usize; 3]],
    src_tris: &[usize],
    dead: &[bool],
) -> bool {
    for &ti in src_tris {
        if dead[ti] {
            continue;
        }
        let tri = triangles[ti];
        // Triangles containing both vertices become degenerate — skip
        if tri[0] == dst || tri[1] == dst || tri[2] == dst {
            continue;
        }

        let mut new_tri = tri;
        for vi in &mut new_tri {
            if *vi == src {
                *vi = dst;
            }
        }

        let old_n = triangle_normal(vertices, &tri);
        let new_n = triangle_normal(vertices, &new_tri);

        // Normal must not flip or degenerate
        if old_n.length_sq() < 1e-10 {
            continue;
        }
        if new_n.length_sq() < 1e-10 || old_n.dot(new_n) < 0.5 {
            return false;
        }
    }
    true
}

/// Remap src→dst in all of src's triangles; mark shared triangles as dead.
fn perform_collapse(
    dst: usize,
    src: usize,
    triangles: &mut [[usize; 3]],
    src_tris: &[usize],
    dead: &mut [bool],
) {
    for &ti in src_tris {
        if dead[ti] {
            continue;
        }
        let tri = &mut triangles[ti];
        let has_dst = tri[0] == dst || tri[1] == dst || tri[2] == dst;
        if has_dst {
            // Shared triangle becomes degenerate
            dead[ti] = true;
        } else {
            for vi in tri.iter_mut() {
                if *vi == src {
                    *vi = dst;
                }
            }
        }
    }
}

fn triangle_normal(vertices: &[Vec3], tri: &[usize; 3]) -> Vec3 {
    let v0 = vertices[tri[0]];
    let v1 = vertices[tri[1]];
    let v2 = vertices[tri[2]];
    v1.sub(v0).cross(v2.sub(v0)).normalize()
}

/// Canonical undirected edge (smaller index first).
fn edge(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn decimate_pass(mesh: &mut MeshBuilder, threshold: f32) -> usize {
    let nv = mesh.vertices.len();
    let nt = mesh.triangles.len();

    // Build vertex → triangle adjacency
    let mut vert_tris: Vec<Vec<usize>> = vec![Vec::new(); nv];
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        for &vi in tri {
            vert_tris[vi].push(ti);
        }
    }

    let mut dead = vec![false; nt];
    let mut removed_count = 0;
    // Track edges created by ear-clip this pass (diagonals from earlier removals
    // can collide with later removals, causing non-manifold edges).
    let mut pass_edges: FxHashSet<(usize, usize)> = FxHashSet::default();

    for vi in 0..nv {
        let adj = &vert_tris[vi];
        if adj.len() < 3 {
            continue;
        }

        // Skip if any adjacent triangle was already removed this pass
        if adj.iter().any(|&ti| dead[ti]) {
            continue;
        }

        // Compute normals of adjacent triangles
        let normals: Vec<Vec3> = adj
            .iter()
            .map(|&ti| triangle_normal(&mesh.vertices, &mesh.triangles[ti]))
            .collect();

        // All normals must agree within threshold
        let ref_normal = normals[0];
        if ref_normal.length_sq() < 1e-10 {
            continue;
        }
        if !normals[1..].iter().all(|n| n.dot(ref_normal) >= threshold) {
            continue;
        }

        // Find ordered vertex ring around vi
        let ring = match find_vertex_ring(vi, adj, &mesh.triangles) {
            Some(r) => r,
            None => continue,
        };

        // Average normal for projection plane
        let avg_normal = normals
            .iter()
            .fold([0.0f32, 0.0, 0.0], |acc, &n| acc.add(n))
            .normalize();

        // Ear-clip the ring polygon
        let new_tris = ear_clip(&ring, &mesh.vertices, avg_normal);
        if new_tris.len() >= adj.len() {
            continue;
        }

        // Check that no diagonal edge from ear clip conflicts with existing mesh edges
        // or edges created by earlier removals this pass.
        let ring_edges: FxHashSet<(usize, usize)> = {
            let n = ring.len();
            (0..n).map(|i| edge(ring[i], ring[(i + 1) % n])).collect()
        };
        let adj_set: FxHashSet<usize> = adj.iter().copied().collect();

        let mut has_conflict = false;
        'outer: for tri in &new_tris {
            for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let e = edge(a, b);
                if ring_edges.contains(&e) {
                    continue; // ring edge, always safe
                }
                // Diagonal — check against original mesh edges
                for &ti in &vert_tris[e.0] {
                    if adj_set.contains(&ti) || dead[ti] {
                        continue;
                    }
                    let t = mesh.triangles[ti];
                    if t[0] == e.1 || t[1] == e.1 || t[2] == e.1 {
                        has_conflict = true;
                        break 'outer;
                    }
                }
                // Diagonal — check against edges from earlier removals this pass
                if pass_edges.contains(&e) {
                    has_conflict = true;
                    break 'outer;
                }
            }
        }
        if has_conflict {
            continue;
        }

        // Mark old triangles as dead, append new ones
        for &ti in adj {
            dead[ti] = true;
        }
        for tri in &new_tris {
            for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                pass_edges.insert(edge(a, b));
            }
        }
        for tri in new_tris {
            mesh.triangles.push(tri);
        }
        removed_count += 1;
    }

    // Compact: remove dead triangles
    if removed_count > 0 {
        let mut write = 0;
        for read in 0..mesh.triangles.len() {
            if read < dead.len() && dead[read] {
                continue;
            }
            mesh.triangles[write] = mesh.triangles[read];
            write += 1;
        }
        mesh.triangles.truncate(write);
    }

    removed_count
}

/// Walk half-edges around `vertex` to find the ordered ring of neighbor vertices.
/// Returns None for boundary vertices or non-manifold topology.
fn find_vertex_ring(
    vertex: usize,
    adj_tris: &[usize],
    triangles: &[[usize; 3]],
) -> Option<Vec<usize>> {
    // For each triangle containing vertex, map outgoing → incoming neighbor.
    // If tri[p] == vertex: outgoing = tri[(p+1)%3], incoming = tri[(p+2)%3].
    // map[outgoing] = incoming produces a CCW ring.
    let mut next_map: FxHashMap<usize, usize> = FxHashMap::default();

    for &ti in adj_tris {
        let tri = triangles[ti];
        let p = if tri[0] == vertex {
            0
        } else if tri[1] == vertex {
            1
        } else {
            2
        };
        let outgoing = tri[(p + 1) % 3];
        let incoming = tri[(p + 2) % 3];
        if next_map.insert(outgoing, incoming).is_some() {
            return None; // non-manifold
        }
    }

    // Walk the chain to build the ring
    let start = *next_map.keys().next()?;
    let mut ring = vec![start];
    let mut current = *next_map.get(&start)?;

    while current != start {
        if ring.len() > adj_tris.len() {
            return None;
        }
        ring.push(current);
        current = *next_map.get(&current)?; // None → boundary vertex
    }

    if ring.len() != adj_tris.len() {
        return None;
    }

    Some(ring)
}

/// Triangulate a planar polygon via ear clipping.
/// `ring` contains vertex indices in CCW order; `normal` is the face normal for projection.
fn ear_clip(ring: &[usize], vertices: &[Vec3], normal: Vec3) -> Vec<[usize; 3]> {
    let n = ring.len();
    if n < 3 {
        return vec![];
    }
    if n == 3 {
        return vec![[ring[0], ring[1], ring[2]]];
    }

    // Build orthonormal basis for 2D projection
    let (u_axis, v_axis) = make_basis(normal);
    let pts: Vec<[f32; 2]> = ring
        .iter()
        .map(|&vi| {
            let p = vertices[vi];
            [p.dot(u_axis), p.dot(v_axis)]
        })
        .collect();

    let mut indices: Vec<usize> = (0..n).collect();
    let mut result = Vec::with_capacity(n - 2);

    while indices.len() > 3 {
        let len = indices.len();
        let mut best_ear: Option<(usize, f32)> = None; // (index to remove, quality)

        for i in 0..len {
            let pi = indices[(i + len - 1) % len];
            let ci = indices[i];
            let ni = indices[(i + 1) % len];

            let area2 = cross_2d(pts[pi], pts[ci], pts[ni]);
            if area2 <= 0.0 {
                continue;
            }

            // Check no other vertex is inside this ear triangle
            let mut is_ear = true;
            for j in 0..len {
                let ji = indices[j];
                if ji == pi || ji == ci || ji == ni {
                    continue;
                }
                if point_in_triangle_2d(pts[ji], pts[pi], pts[ci], pts[ni]) {
                    is_ear = false;
                    break;
                }
            }

            if is_ear {
                // Quality: area / (longest edge)^2 — prefers well-shaped triangles
                let e1 = dist_sq_2d(pts[pi], pts[ci]);
                let e2 = dist_sq_2d(pts[ci], pts[ni]);
                let e3 = dist_sq_2d(pts[ni], pts[pi]);
                let max_edge_sq = e1.max(e2).max(e3);
                let quality = if max_edge_sq > 0.0 { area2 / max_edge_sq } else { 0.0 };

                if best_ear.map_or(true, |(_, q)| quality > q) {
                    best_ear = Some((i, quality));
                }
            }
        }

        if let Some((i, _)) = best_ear {
            let pi = indices[(i + indices.len() - 1) % indices.len()];
            let ci = indices[i];
            let ni = indices[(i + 1) % indices.len()];
            result.push([ring[pi], ring[ci], ring[ni]]);
            indices.remove(i);
        } else {
            // Fallback: fan triangulation from first vertex
            for i in 1..indices.len() - 1 {
                result.push([ring[indices[0]], ring[indices[i]], ring[indices[i + 1]]]);
            }
            return result;
        }
    }

    if indices.len() == 3 {
        result.push([ring[indices[0]], ring[indices[1]], ring[indices[2]]]);
    }

    result
}

fn make_basis(normal: Vec3) -> (Vec3, Vec3) {
    let up: Vec3 = if normal[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u = up.cross(normal).normalize();
    let v = normal.cross(u);
    (u, v)
}

fn dist_sq_2d(a: [f32; 2], b: [f32; 2]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    dx * dx + dy * dy
}

/// Signed 2x area of triangle (a, b, c) in 2D.
fn cross_2d(a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> f32 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Test if point p is inside triangle (a, b, c) in 2D (inclusive of edges).
fn point_in_triangle_2d(p: [f32; 2], a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> bool {
    let d1 = cross_2d(a, b, p);
    let d2 = cross_2d(b, c, p);
    let d3 = cross_2d(c, a, p);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::builder::MeshBuilder;

    /// Build a flat mesh: 4 triangles around a center vertex on the XY plane.
    fn make_flat_fan() -> MeshBuilder {
        let vertices = vec![
            [0.0, 0.0, 0.0], // 0: center
            [1.0, 0.0, 0.0], // 1
            [0.0, 1.0, 0.0], // 2
            [-1.0, 0.0, 0.0], // 3
            [0.0, -1.0, 0.0], // 4
        ];
        let triangles = vec![[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1]];
        MeshBuilder::from_raw(vertices, triangles)
    }

    #[test]
    fn test_decimate_flat_fan() {
        let mut mesh = make_flat_fan();
        assert_eq!(mesh.triangles.len(), 4);
        decimate_flat(&mut mesh, 0.999);
        // Center vertex removed: 4 → 2 triangles
        assert_eq!(mesh.triangles.len(), 2);
        assert_eq!(mesh.vertices.len(), 4); // center vertex compacted away
    }

    #[test]
    fn test_no_decimate_curved() {
        // Pyramid: center vertex is elevated, normals differ
        let vertices = vec![
            [0.0, 0.0, 1.0], // 0: apex (elevated)
            [1.0, 0.0, 0.0], // 1
            [0.0, 1.0, 0.0], // 2
            [-1.0, 0.0, 0.0], // 3
            [0.0, -1.0, 0.0], // 4
        ];
        let triangles = vec![[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1]];
        let mut mesh = MeshBuilder::from_raw(vertices, triangles);
        decimate_flat(&mut mesh, 0.999);
        assert_eq!(mesh.triangles.len(), 4); // no change
    }

    #[test]
    fn test_find_vertex_ring_boundary() {
        // Two triangles sharing an edge — vertex 1 is on the boundary
        let triangles = vec![[0, 1, 2], [1, 3, 2]];
        let adj = vec![0, 1];
        assert!(find_vertex_ring(1, &adj, &triangles).is_none());
    }

    #[test]
    fn test_ear_clip_quad() {
        let vertices = vec![
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ];
        let ring = vec![0, 1, 2, 3];
        let normal = [0.0, 0.0, 1.0];
        let tris = ear_clip(&ring, &vertices, normal);
        assert_eq!(tris.len(), 2);
    }
}
