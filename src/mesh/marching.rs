//! Marching tetrahedra isosurface extraction.
//!
//! Each tetrahedron has 4 vertices and 6 edges. SDF values at the vertices
//! produce a 4-bit configuration index into a 16-entry triangulation table.
//! Surface crossings are interpolated along edges where the SDF changes sign.
//!
//! ## Tetrahedron vertex and edge layout
//!
//! ```text
//!       (2)
//!      /   \
//!     1  5  3
//!    /  (3)  \
//!   /  2   4  \
//! (0)----0----(1)
//! ```
//!
//! Edges 0-2 connect vertex 0 to 1,2,3. Edges 3-5 connect remaining pairs.

use super::builder::{EdgeKey, MeshBuilder};
use super::cache::SdfCache;
use super::octree::LatticeGrid;
use crate::math::{vec3_lerp, LatticePoint, Vec3};
use crate::solid::Solid;

/// Maps tetrahedron edge index to the two vertex indices.
pub const TET_EDGE_VERTICES: [[usize; 2]; 6] = [
    [0, 1], // Edge 0
    [0, 2], // Edge 1
    [0, 3], // Edge 2
    [1, 2], // Edge 3
    [1, 3], // Edge 4
    [2, 3], // Edge 5
];

/// Triangulation table for all 16 tetrahedron configurations (2^4 vertices).
///
/// Each entry is a list of edge indices forming triangles.
/// -1 indicates end of the triangle list.
///
/// Configuration index is a bitmask where bit N indicates vertex N is inside.
pub const TET_TRIANGULATION_TABLE: [[i8; 6]; 16] = [
    [-1, -1, -1, -1, -1, -1], // Config  0: 0b0000 - no vertices inside (inverse of 15)
    [1, 2, 0, -1, -1, -1],    // Config  1: 0b0001 - vertex 0 inside (inverse of 14)
    [0, 4, 3, -1, -1, -1],    // Config  2: 0b0010 - vertex 1 inside (inverse of 13)
    [1, 2, 4, 1, 4, 3],       // Config  3: 0b0011 - vertices 0,1 inside (inverse of 12)
    [1, 3, 5, -1, -1, -1],    // Config  4: 0b0100 - vertex 2 inside (inverse of 11)
    [5, 2, 0, 3, 5, 0],       // Config  5: 0b0101 - vertices 0,2 inside (inverse of 10)
    [5, 1, 0, 5, 0, 4],       // Config  6: 0b0110 - vertices 1,2 inside (inverse of 9)
    [2, 4, 5, -1, -1, -1],    // Config  7: 0b0111 - vertices 0,1,2 inside (inverse of 8)
    [4, 2, 5, -1, -1, -1],    // Config  8: 0b1000 - vertex 3 inside (inverse of 7)
    [1, 5, 0, 0, 5, 4],       // Config  9: 0b1001 - vertices 0,3 inside (inverse of 6)
    [2, 5, 0, 5, 3, 0],       // Config 10: 0b1010 - vertices 1,3 inside (inverse of 5)
    [3, 1, 5, -1, -1, -1],    // Config 11: 0b1011 - vertices 0,1,3 inside (inverse of 4)
    [2, 1, 4, 4, 1, 3],       // Config 12: 0b1100 - vertices 2,3 inside (inverse of 3)
    [4, 0, 3, -1, -1, -1],    // Config 13: 0b1101 - vertices 0,2,3 inside (inverse of 2)
    [2, 1, 0, -1, -1, -1],    // Config 14: 0b1110 - vertices 1,2,3 inside (inverse of 1)
    [-1, -1, -1, -1, -1, -1], // Config 15: 0b1111 - all vertices inside (inverse of 0)
];

/// A tetrahedron with 4 vertices, in lattice coordinates.
pub struct Tet {
    pub vertices: [LatticePoint; 4],
}

impl Tet {
    /// Interpolate along an edge to find the SDF zero-crossing point (in world space).
    fn interpolate_edge(
        &self,
        edge_idx: usize,
        world_verts: &[Vec3; 4],
        sdf_values: &[f32; 4],
        interpolate: bool,
    ) -> Vec3 {
        let [v0, v1] = TET_EDGE_VERTICES[edge_idx];

        let t = if interpolate {
            let sdf0 = sdf_values[v0];
            let sdf1 = sdf_values[v1];

            if (sdf0 - sdf1).abs() < 1e-7 {
                0.5
            } else {
                sdf0 / (sdf0 - sdf1)
            }
        } else {
            0.5
        };

        vec3_lerp(world_verts[v0], world_verts[v1], t)
    }

    /// Get the EdgeKey for a tet edge (canonical pair of lattice endpoints).
    fn edge_key(&self, edge_idx: usize) -> EdgeKey {
        let [v0, v1] = TET_EDGE_VERTICES[edge_idx];
        EdgeKey::new(self.vertices[v0], self.vertices[v1])
    }

    /// Evaluate the SDF at each vertex, look up the triangulation, and emit
    /// interpolated triangles into the mesh.
    pub fn process<S: Solid + ?Sized>(
        &self,
        solid: &S,
        grid: &LatticeGrid,
        interpolate: bool,
        cache: &SdfCache,
        mesh: &mut MeshBuilder,
    ) {
        let world_verts: [Vec3; 4] = [
            grid.to_world(self.vertices[0]),
            grid.to_world(self.vertices[1]),
            grid.to_world(self.vertices[2]),
            grid.to_world(self.vertices[3]),
        ];

        let sdf_values = [
            cache.get_or_eval(self.vertices[0], solid, grid),
            cache.get_or_eval(self.vertices[1], solid, grid),
            cache.get_or_eval(self.vertices[2], solid, grid),
            cache.get_or_eval(self.vertices[3], solid, grid),
        ];

        let mut config: u8 = 0;
        for i in 0..4 {
            if sdf_values[i] < 0.0 {
                config |= 1 << i;
            }
        }

        let entry = &TET_TRIANGULATION_TABLE[config as usize];

        let mut i = 0;
        while i + 2 < entry.len() && entry[i] != -1 {
            let e0 = entry[i] as usize;
            let e1 = entry[i + 1] as usize;
            let e2 = entry[i + 2] as usize;
            let p0 = self.interpolate_edge(e0, &world_verts, &sdf_values, interpolate);
            let p1 = self.interpolate_edge(e1, &world_verts, &sdf_values, interpolate);
            let p2 = self.interpolate_edge(e2, &world_verts, &sdf_values, interpolate);
            mesh.add_triangle_by_edge(
                self.edge_key(e0),
                p0,
                self.edge_key(e1),
                p1,
                self.edge_key(e2),
                p2,
            );
            i += 3;
        }
    }
}
