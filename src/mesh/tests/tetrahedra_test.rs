use super::*;
use crate::math::LatticePoint;
use crate::mesh::adaptive::{BalancedOctree, NeighborStatus};
use crate::mesh::marching::Tet;
use crate::mesh::octree::{
    children_on_face, initial_step, neighbor_key, Direction, Face, LatticeGrid, LeveledCell,
};

fn lp(x: i32, y: i32, z: i32) -> LatticePoint {
    LatticePoint::new(x, y, z)
}

fn make_cell(ix: i32, iy: i32, iz: i32, step: i32) -> LeveledCell {
    LeveledCell::new(ix, iy, iz, step)
}

fn unit_face() -> [LatticePoint; 4] {
    [lp(0, 0, 0), lp(4, 0, 0), lp(4, 4, 0), lp(0, 4, 0)]
}

// ── Test helpers (moved from tetrahedra.rs) ─────────────────────────────

fn adjacent_faces(face: Face) -> [Face; 4] {
    match face {
        Face::NegZ => [Face::NegY, Face::PosX, Face::PosY, Face::NegX],
        Face::PosZ => [Face::NegX, Face::PosY, Face::PosX, Face::NegY],
        Face::NegY => [Face::NegX, Face::PosZ, Face::PosX, Face::NegZ],
        Face::PosY => [Face::NegZ, Face::PosX, Face::PosZ, Face::NegX],
        Face::NegX => [Face::NegZ, Face::PosY, Face::PosZ, Face::NegY],
        Face::PosX => [Face::NegY, Face::PosZ, Face::PosY, Face::NegZ],
    }
}

fn generate_face_tets(
    fc: [LatticePoint; 4],
    apex: LatticePoint,
    octree: &BalancedOctree,
) -> Vec<Tet> {
    let polygon = build_refined_polygon(fc, octree);
    let face_center = LatticePoint::center4(fc[0], fc[1], fc[2], fc[3]);
    let n = polygon.len();
    (0..n)
        .map(|i| Tet {
            vertices: [polygon[i], polygon[(i + 1) % n], face_center, apex],
        })
        .collect()
}

fn make_split_face_tets(
    fc: [LatticePoint; 4],
    apex: LatticePoint,
    needs_midpoint: [bool; 4],
) -> Vec<Tet> {
    let mut polygon: Vec<LatticePoint> = Vec::with_capacity(8);
    for i in 0..4 {
        polygon.push(fc[i]);
        if needs_midpoint[i] {
            polygon.push(LatticePoint::midpoint(fc[i], fc[(i + 1) % 4]));
        }
    }
    let face_center = LatticePoint::center4(fc[0], fc[1], fc[2], fc[3]);
    let n = polygon.len();
    (0..n)
        .map(|i| Tet {
            vertices: [polygon[i], polygon[(i + 1) % n], face_center, apex],
        })
        .collect()
}

fn edge_needs_midpoint(
    cell: LeveledCell,
    face: Face,
    edge_idx: usize,
    _statuses: &[NeighborStatus; 6],
    octree: &BalancedOctree,
) -> bool {
    let adj_dir = adjacent_faces(face)[edge_idx];
    let d1: Direction = face.into();
    let d2: Direction = adj_dir.into();
    let d3 = d1.add(d2);
    octree.neighbor_status(cell, d1) == NeighborStatus::Finer
        || octree.neighbor_status(cell, d2) == NeighborStatus::Finer
        || octree.neighbor_status(cell, d3) == NeighborStatus::Finer
}

fn make_tets(face_corners: [LatticePoint; 4], apex: LatticePoint) -> [Tet; 4] {
    let center = LatticePoint::center4(
        face_corners[0],
        face_corners[1],
        face_corners[2],
        face_corners[3],
    );
    [
        Tet { vertices: [face_corners[0], face_corners[1], center, apex] },
        Tet { vertices: [face_corners[1], face_corners[2], center, apex] },
        Tet { vertices: [face_corners[2], face_corners[3], center, apex] },
        Tet { vertices: [face_corners[3], face_corners[0], center, apex] },
    ]
}

fn triangle_area(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> f32 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cross = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt() * 0.5
}

// ── cube_corners ────────────────────────────────────────────────────────

#[test]
fn test_cube_corners_binary_indexing() {
    // Index bits: bit0=X, bit1=Y, bit2=Z
    let corner = lp(0, 0, 0);
    let step = 4;
    let corners = cube_corners(corner, step);
    for (i, c) in corners.iter().enumerate() {
        let bx = ((i >> 0) & 1) as i32;
        let by = ((i >> 1) & 1) as i32;
        let bz = ((i >> 2) & 1) as i32;
        let expected = lp(bx * step, by * step, bz * step);
        assert_eq!(*c, expected, "Corner {i}: expected {expected:?}, got {c:?}");
    }
}

// ── face_corners ────────────────────────────────────────────────────────

#[test]
fn test_face_corners_coplanar() {
    let corners = cube_corners(lp(0, 0, 0), 4);
    for &face in &Face::ALL {
        let fc = face_corners(&corners, face);
        let all_same_x = fc.iter().all(|p| p.x == fc[0].x);
        let all_same_y = fc.iter().all(|p| p.y == fc[0].y);
        let all_same_z = fc.iter().all(|p| p.z == fc[0].z);
        assert!(
            all_same_x || all_same_y || all_same_z,
            "Face {face:?} corners not coplanar: {fc:?}",
        );
    }
}

#[test]
fn test_face_corners_cover_all_cube_corners() {
    let corners = cube_corners(lp(0, 0, 0), 4);
    let mut used = [false; 8];
    for &face in &Face::ALL {
        let fc = face_corners(&corners, face);
        for fv in &fc {
            for (i, cv) in corners.iter().enumerate() {
                if fv == cv {
                    used[i] = true;
                }
            }
        }
    }
    for (i, &u) in used.iter().enumerate() {
        assert!(u, "Cube corner {i} not referenced by any face");
    }
}

// ── adjacent_faces ──────────────────────────────────────────────────────

#[test]
fn test_adjacent_faces_distinct() {
    for &face in &Face::ALL {
        let adj = adjacent_faces(face);
        for i in 0..4 {
            for j in (i + 1)..4 {
                assert_ne!(
                    adj[i], adj[j],
                    "Face {face:?}: adjacent faces {i} and {j} are the same ({:?})",
                    adj[i],
                );
            }
        }
        for &a in &adj {
            assert_ne!(a, face, "Face {face:?}: adjacent contains itself");
            assert_ne!(
                a,
                face.opposite(),
                "Face {face:?}: adjacent contains opposite",
            );
        }
    }
}

#[test]
fn test_adjacent_faces_symmetry() {
    for &face in &Face::ALL {
        let adj = adjacent_faces(face);
        for &g in &adj {
            let g_adj = adjacent_faces(g);
            assert!(
                g_adj.contains(&face),
                "Face {face:?} lists {g:?} as adjacent, but {g:?} doesn't list {face:?}",
            );
        }
    }
}

#[test]
fn test_adjacent_faces_share_edge_geometrically() {
    let corners = cube_corners(lp(0, 0, 0), 4);
    for &face in &Face::ALL {
        let fc = face_corners(&corners, face);
        let adj = adjacent_faces(face);
        for k in 0..4 {
            let v0 = fc[k];
            let v1 = fc[(k + 1) % 4];
            let adj_fc = face_corners(&corners, adj[k]);
            let has_v0 = adj_fc.iter().any(|v| *v == v0);
            let has_v1 = adj_fc.iter().any(|v| *v == v1);
            assert!(
                has_v0 && has_v1,
                "Face {face:?} edge {k} ({v0:?}-{v1:?}): adjacent {adj_k:?} missing vertex \
                 (has_v0={has_v0}, has_v1={has_v1})",
                adj_k = adj[k],
            );
        }
    }
}

// ── make_tets ───────────────────────────────────────────────────────────

#[test]
fn test_make_tets_vertices() {
    let fc = [lp(0, 0, 0), lp(4, 0, 0), lp(4, 4, 0), lp(0, 4, 0)];
    let apex = lp(2, 2, 4);
    let center = LatticePoint::center4(fc[0], fc[1], fc[2], fc[3]);
    let tets = make_tets(fc, apex);

    assert_eq!(tets.len(), 4);
    for (i, tet) in tets.iter().enumerate() {
        assert_eq!(tet.vertices[0], fc[i], "tet {i}: vertices[0]");
        assert_eq!(tet.vertices[1], fc[(i + 1) % 4], "tet {i}: vertices[1]");
        assert_eq!(tet.vertices[2], center, "tet {i}: vertices[2]");
        assert_eq!(tet.vertices[3], apex, "tet {i}: vertices[3]");
    }
}

#[test]
fn test_make_tets_positive_orientation() {
    let corners = cube_corners(lp(0, 0, 0), 4);
    let apex = lp(2, 2, 2);
    for &face in &Face::ALL {
        let fc = face_corners(&corners, face);
        let tets = make_tets(fc, apex);
        for (i, tet) in tets.iter().enumerate() {
            let v = tet.vertices;
            let d10 = [v[1].x - v[0].x, v[1].y - v[0].y, v[1].z - v[0].z];
            let d20 = [v[2].x - v[0].x, v[2].y - v[0].y, v[2].z - v[0].z];
            let d30 = [v[3].x - v[0].x, v[3].y - v[0].y, v[3].z - v[0].z];
            let det = d10[0] * (d20[1] * d30[2] - d20[2] * d30[1])
                - d10[1] * (d20[0] * d30[2] - d20[2] * d30[0])
                + d10[2] * (d20[0] * d30[1] - d20[1] * d30[0]);
            assert!(
                det > 0,
                "Face {face:?} tet {i}: determinant {det} should be positive",
            );
        }
    }
}

// ── make_split_face_tets ────────────────────────────────────────────────

#[test]
fn test_split_face_tet_count() {
    let fc = unit_face();
    let apex = lp(2, 2, 4);
    let cases: &[([bool; 4], usize)] = &[
        ([false, false, false, false], 4),
        ([true, false, false, false], 5),
        ([false, true, false, false], 5),
        ([false, false, true, false], 5),
        ([false, false, false, true], 5),
        ([true, true, false, false], 6),
        ([true, false, true, false], 6),
        ([false, true, false, true], 6),
        ([true, true, true, false], 7),
        ([true, true, true, true], 8),
    ];
    for (midpoints, expected) in cases {
        let result = make_split_face_tets(fc, apex, *midpoints);
        assert_eq!(
            result.len(),
            *expected,
            "Midpoints {midpoints:?}: expected {expected}, got {}",
            result.len(),
        );
    }
}

#[test]
fn test_split_face_tets_all_share_apex() {
    let fc = unit_face();
    let apex = lp(2, 2, 4);
    for midpoint_count in 1..=4 {
        let mut midpoints = [false; 4];
        for i in 0..midpoint_count {
            midpoints[i] = true;
        }
        let result = make_split_face_tets(fc, apex, midpoints);
        for (i, t) in result.iter().enumerate() {
            assert_eq!(
                t.vertices[3], apex,
                "Sub-tet {i} (midpoints {midpoints:?}): apex {:?} != {apex:?}",
                t.vertices[3],
            );
        }
    }
}

#[test]
fn test_split_face_tets_all_share_face_center() {
    let fc = unit_face();
    let apex = lp(2, 2, 4);
    let face_center = LatticePoint::center4(fc[0], fc[1], fc[2], fc[3]);
    let result = make_split_face_tets(fc, apex, [true, true, true, true]);
    for (i, t) in result.iter().enumerate() {
        assert_eq!(
            t.vertices[2], face_center,
            "Sub-tet {i}: vertices[2]={:?} != face_center={face_center:?}",
            t.vertices[2],
        );
    }
}

#[test]
fn test_split_face_tets_base_area_sums_to_face_area() {
    let root_step = initial_step(1);
    let grid = LatticeGrid::new([0.0, 0.0, 0.0], 4.0, root_step);
    let fc = unit_face();
    let apex = lp(2, 2, 4);
    let expected_area = 16.0; // 4x4 lattice units, unit_size=1.0

    let patterns: &[[bool; 4]] = &[
        [false, false, false, false],
        [true, false, false, false],
        [true, true, false, false],
        [true, false, true, false],
        [true, true, true, false],
        [true, true, true, true],
    ];
    for midpoints in patterns {
        let result = make_split_face_tets(fc, apex, *midpoints);
        let total_area: f32 = result
            .iter()
            .map(|t| {
                let w0 = grid.to_world(t.vertices[0]);
                let w1 = grid.to_world(t.vertices[1]);
                let w2 = grid.to_world(t.vertices[2]);
                triangle_area(w0, w1, w2)
            })
            .sum();
        assert!(
            (total_area - expected_area).abs() < 1e-5,
            "Midpoints {midpoints:?}: base area sum {total_area} != {expected_area}",
        );
    }
}

#[test]
fn test_split_face_tets_midpoint_vertices_present() {
    let fc = unit_face();
    let apex = lp(2, 2, 4);
    let midpoints = [true, true, true, true];
    let result = make_split_face_tets(fc, apex, midpoints);
    for k in 0..4 {
        let expected_mid = LatticePoint::midpoint(fc[k], fc[(k + 1) % 4]);
        let found = result
            .iter()
            .any(|t| t.vertices[0..3].iter().any(|v| *v == expected_mid));
        assert!(
            found,
            "Edge {k} midpoint {expected_mid:?} not found in any sub-tet",
        );
    }
}

// ── edge_needs_midpoint ─────────────────────────────────────────────────

#[test]
fn test_edge_no_midpoints_isolated_cell() {
    let root_step = initial_step(1);
    let mut octree = BalancedOctree::new(root_step);
    let cell = make_cell(0, 0, 0, root_step / 2);
    octree.insert(cell);

    let statuses: [NeighborStatus; 6] =
        Face::ALL.map(|f| octree.neighbor_status(cell, f.into()));

    for &face in &Face::ALL {
        for edge_idx in 0..4 {
            assert!(
                !edge_needs_midpoint(cell, face, edge_idx, &statuses, &octree),
                "Isolated cell: face {face:?} edge {edge_idx} should not need midpoint",
            );
        }
    }
}

#[test]
fn test_edge_no_midpoints_two_same_level() {
    let root_step = initial_step(1);
    let step = root_step / 2;
    let mut octree = BalancedOctree::new(root_step);
    let a = make_cell(0, 0, 0, step);
    let b = make_cell(step, 0, 0, step);
    octree.insert(a);
    octree.insert(b);

    let statuses: [NeighborStatus; 6] = Face::ALL.map(|f| octree.neighbor_status(a, f.into()));

    for &face in &Face::ALL {
        for edge_idx in 0..4 {
            assert!(
                !edge_needs_midpoint(a, face, edge_idx, &statuses, &octree),
                "Two same-level cells: face {face:?} edge {edge_idx} shouldn't need midpoint",
            );
        }
    }
}

#[test]
fn test_edge_midpoint_same_cell_finer() {
    let root_step = initial_step(2); // = 8
    let step = root_step / 2; // = 4
    let child_step = step / 2; // = 2
    let mut octree = BalancedOctree::new(root_step);
    let cell = make_cell(0, 0, 0, step);
    octree.insert(cell);

    let neighbor_pos = neighbor_key(cell.key, Face::PosX.into(), step);
    let fine_children = children_on_face(neighbor_pos, step, Face::NegX);
    for &ck in &fine_children {
        octree.insert(LeveledCell {
            key: ck,
            step: child_step,
        });
    }

    let statuses: [NeighborStatus; 6] =
        Face::ALL.map(|f| octree.neighbor_status(cell, f.into()));

    for &face in &Face::ALL {
        if face == Face::PosX {
            continue;
        }
        let adj = adjacent_faces(face);
        for edge_idx in 0..4 {
            let expected = adj[edge_idx] == Face::PosX;
            let result = edge_needs_midpoint(cell, face, edge_idx, &statuses, &octree);
            assert_eq!(
                result, expected,
                "Same-cell Finer on PosX: face {face:?} edge {edge_idx} \
                 (adj={:?}) expected={expected}, got={result}",
                adj[edge_idx],
            );
        }
    }
}

#[test]
fn test_edge_midpoint_cross_cell_via_face() {
    let root_step = initial_step(2); // = 8
    let step = root_step / 2; // = 4
    let child_step = step / 2; // = 2
    let mut octree = BalancedOctree::new(root_step);
    let a = make_cell(0, 0, 0, step);
    let b = make_cell(step, 0, 0, step);
    octree.insert(a);
    octree.insert(b);

    let b_posy_neighbor = neighbor_key(b.key, Face::PosY.into(), step);
    let fine_children = children_on_face(b_posy_neighbor, step, Face::NegY);
    for &ck in &fine_children {
        octree.insert(LeveledCell {
            key: ck,
            step: child_step,
        });
    }

    let statuses: [NeighborStatus; 6] = Face::ALL.map(|f| octree.neighbor_status(a, f.into()));

    assert!(
        edge_needs_midpoint(a, Face::PosX, 2, &statuses, &octree),
        "Cross-cell via face: A's PosX edge 2 (adj=PosY) should need midpoint \
         because B's PosY is Finer",
    );

    for edge_idx in [0, 1, 3] {
        assert!(
            !edge_needs_midpoint(a, Face::PosX, edge_idx, &statuses, &octree),
            "Cross-cell via face: A's PosX edge {edge_idx} should NOT need midpoint",
        );
    }
}

#[test]
fn test_edge_midpoint_cross_cell_via_adj() {
    let root_step = initial_step(2); // = 8
    let step = root_step / 2; // = 4
    let child_step = step / 2; // = 2
    let mut octree = BalancedOctree::new(root_step);
    let a = make_cell(0, 0, 0, step);
    let c = make_cell(0, step, 0, step);
    octree.insert(a);
    octree.insert(c);

    let c_posx_neighbor = neighbor_key(c.key, Face::PosX.into(), step);
    let fine_children = children_on_face(c_posx_neighbor, step, Face::NegX);
    for &ck in &fine_children {
        octree.insert(LeveledCell {
            key: ck,
            step: child_step,
        });
    }

    let statuses: [NeighborStatus; 6] = Face::ALL.map(|f| octree.neighbor_status(a, f.into()));

    assert!(
        edge_needs_midpoint(a, Face::PosX, 2, &statuses, &octree),
        "Cross-cell via adj: A's PosX edge 2 (adj=PosY) should need midpoint \
         because C's PosX is Finer",
    );
}

#[test]
fn test_edge_midpoint_diagonal() {
    let root_step = initial_step(3); // = 16
    let step = root_step / 4; // = 4 (level 2)
    let child_step = step / 2; // = 2 (level 3)
    let mut octree = BalancedOctree::new(root_step);
    let a = make_cell(20, 20, 20, step);
    let d = make_cell(24, 16, 20, step);
    octree.insert(a);
    octree.insert(d);

    octree.insert(make_cell(22, 16, 20, child_step));
    octree.insert(make_cell(22, 16, 22, child_step));

    let statuses: [NeighborStatus; 6] = Face::ALL.map(|f| octree.neighbor_status(a, f.into()));

    assert!(
        edge_needs_midpoint(a, Face::PosX, 0, &statuses, &octree),
        "Diagonal: A's PosX edge 0 (adj=NegY) should need midpoint via diagonal D",
    );

    for edge_idx in [1, 2, 3] {
        assert!(
            !edge_needs_midpoint(a, Face::PosX, edge_idx, &statuses, &octree),
            "Diagonal: A's PosX edge {edge_idx} should NOT need midpoint",
        );
    }
}

// ── generate_face_tets ──────────────────────────────────────────────────

#[test]
fn test_generate_face_tets_isolated() {
    let root_step = initial_step(1); // = 4
    let step = root_step / 2; // = 2
    let mut octree = BalancedOctree::new(root_step);
    let cell = make_cell(0, 0, 0, step);
    octree.insert(cell);

    let center = cell.center();
    let corners = cube_corners(cell.key, step);

    for &face in &Face::ALL {
        let fc = face_corners(&corners, face);
        let tets = generate_face_tets(fc, center, &octree);
        assert_eq!(
            tets.len(),
            4,
            "Isolated cell: face {face:?} should produce 4 tets, got {}",
            tets.len(),
        );
        for tet in &tets {
            assert_eq!(tet.vertices[3], center);
        }
    }
}

#[test]
fn test_generate_face_splits_when_finer_neighbor() {
    let root_step = initial_step(2); // = 8
    let step = root_step / 2; // = 4
    let child_step = step / 2; // = 2
    let mut octree = BalancedOctree::new(root_step);
    let cell = make_cell(0, 0, 0, step);
    octree.insert(cell);

    let neighbor_pos = neighbor_key(cell.key, Face::PosX.into(), step);
    let fine_children = children_on_face(neighbor_pos, step, Face::NegX);
    for &ck in &fine_children {
        octree.insert(LeveledCell {
            key: ck,
            step: child_step,
        });
    }

    let center = cell.center();
    let corners = cube_corners(cell.key, step);

    // NegX face has no adjacent edges touching PosX → 4 tets (unsplit center fan)
    let fc = face_corners(&corners, Face::NegX);
    let tets = generate_face_tets(fc, center, &octree);
    assert_eq!(
        tets.len(),
        4,
        "NegX (opposite of Finer face) should not be split"
    );

    // NegZ face has 1 edge adjacent to PosX (edge 1) → 5 sub-tets
    let fc = face_corners(&corners, Face::NegZ);
    let tets = generate_face_tets(fc, center, &octree);
    assert_eq!(
        tets.len(),
        5,
        "NegZ with 1 midpoint edge should produce 5 sub-tets",
    );
}
