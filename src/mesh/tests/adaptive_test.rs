use super::*;
use crate::math::LatticePoint;
use crate::mesh::octree::{children_on_face, initial_step, neighbor_key, Face, LeveledCell};
use crate::solid::Sphere;

fn fine_children_keys(cell: LeveledCell, face: Face) -> [LatticePoint; 4] {
    let neighbor_pos = neighbor_key(cell.key, face.into(), cell.step);
    children_on_face(neighbor_pos, cell.step, face.opposite())
}

#[test]
fn test_balance_invariant_sphere() {
    let sphere = Sphere::new(1.0);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 5,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let initial_size = 4.0f32;
    let origin = [-2.0f32, -2.0, -2.0];
    let root_step = initial_step(params.max_depth);
    let octree = build_octree(&sphere, origin, initial_size, &params, root_step);
    let unit_size = initial_size / root_step as f32;

    // Verify: no two adjacent cells differ by more than 1 level
    for cell in octree.iter() {
        for face in Face::ALL {
            let status = octree.neighbor_status(*cell, face.into());
            if status == NeighborStatus::Finer {
                // All 4 fine children should exist
                let children = fine_children_keys(*cell, face);
                let child_step = cell.step / 2;
                for &child_key in &children {
                    let child = LeveledCell {
                        key: child_key,
                        step: child_step,
                    };
                    assert!(
                        octree.contains(&child),
                        "Cell ({},{},{}) step {}: Finer on {:?} but child missing",
                        cell.key.x,
                        cell.key.y,
                        cell.key.z,
                        cell.step,
                        face,
                    );
                }
            }
        }

        // No neighbor should be 2+ levels away
        for face in Face::ALL {
            let neighbor_pos = neighbor_key(cell.key, face.into(), cell.step);
            let mut check_step = cell.step * 4;
            while check_step <= root_step {
                let coarser_key = cell_at_step(neighbor_pos, check_step);
                let coarser = LeveledCell {
                    key: coarser_key,
                    step: check_step,
                };
                assert!(
                    !octree.contains(&coarser),
                    "Cell step {} has neighbor step {} (>1 level difference)",
                    cell.step,
                    check_step,
                );
                check_step *= 2;
            }
        }
    }

    // Verify all cells intersect the surface
    for cell in octree.iter() {
        let cs = cell.step as f32 * unit_size;
        let center = cell_center(cell.key, cell.step, origin, unit_size);
        let cr = cell_circumradius(cs);
        assert!(
            cell_intersects_surface(&sphere, center, cr),
            "Cell ({},{},{}) step {} does not intersect surface",
            cell.key.x,
            cell.key.y,
            cell.key.z,
            cell.step,
        );
    }
}
