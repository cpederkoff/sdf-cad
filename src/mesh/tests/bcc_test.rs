use super::*;
use crate::solid::{Cube, Difference, Intersection, Sphere, Translate};

/// Helper: generate mesh with a fresh cache sized to the params.
fn bcc_mesh(solid: &dyn crate::solid::Solid, center: [f32; 3], radius: f32, params: &BccMeshParams) -> MeshBuilder {
    let cache = SdfCache::new(initial_step(params.max_depth));
    generate_bcc_mesh(solid, center, radius, params, &cache)
}

/// Assert that a mesh is watertight, has no degenerate triangles, and has
/// consistent winding (no duplicate directed half-edges).
fn assert_mesh_valid(mesh: &MeshBuilder, label: &str) {
    assert_eq!(
        mesh.count_degenerate_triangles(),
        0,
        "{label}: degenerate triangles"
    );

    let (is_watertight, boundary, non_manifold) = mesh.is_watertight();
    assert!(
        is_watertight,
        "{label}: not watertight ({} boundary, {} non-manifold edges)",
        boundary.len(),
        non_manifold.len(),
    );

    let dup = mesh.check_consistent_winding();
    assert_eq!(
        dup, 0,
        "{label}: {dup} duplicate half-edges (flipped normals)"
    );
}

#[test]
fn test_sphere_vertices_on_surface() {
    let sphere = Sphere::new(1.0);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 5,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&sphere, [0.0, 0.0, 0.0], 2.0, &params);

    for vertex in &mesh.vertices {
        let dist =
            (vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]).sqrt();
        assert!(
            (dist - 1.0).abs() < 0.02,
            "Vertex {:?} at distance {}, expected ~1.0",
            vertex,
            dist
        );
    }
}

#[test]
fn test_sphere_uniform() {
    let sphere = Sphere::new(1.0);
    let params = BccMeshParams {
        min_depth: 4,
        max_depth: 4,
        sdf_error_threshold: f32::INFINITY,
        ..Default::default()
    };
    let mesh = bcc_mesh(&sphere, [0.0, 0.0, 0.0], 2.0, &params);
    assert_mesh_valid(&mesh, "sphere uniform");
}

#[test]
fn test_sphere_adaptive() {
    let sphere = Sphere::new(1.0);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 5,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&sphere, [0.0, 0.0, 0.0], 2.0, &params);
    assert_mesh_valid(&mesh, "sphere adaptive");
}

#[test]
fn test_cube_uniform() {
    let cube = Cube::new(2.0);
    let params = BccMeshParams {
        min_depth: 4,
        max_depth: 4,
        sdf_error_threshold: f32::INFINITY,
        ..Default::default()
    };
    let mesh = bcc_mesh(&cube, [0.0, 0.0, 0.0], 5.0, &params);
    assert_mesh_valid(&mesh, "cube uniform");
}

#[test]
fn test_cube_adaptive() {
    let cube = Cube::new(2.0);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&cube, [0.0, 0.0, 0.0], 5.0, &params);
    assert_mesh_valid(&mesh, "cube adaptive");
}

#[test]
fn test_difference_csg() {
    let cube = Cube::new(2.0);
    let sphere = Translate::new(Sphere::new(3.0), [3.0, 0.0, 0.0]);
    let shape = Difference::new(cube, sphere);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 8,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 10.0, &params);
    assert_mesh_valid(&mesh, "difference CSG");
}

#[test]
fn test_capsule() {
    use crate::solid::Capsule;
    let shape = Capsule::new(1.0, 2.0);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "capsule");
}

#[test]
fn test_torus() {
    use crate::solid::Torus;
    let shape = Torus::new(1.0, 0.3);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "torus");
}

#[test]
fn test_rounded_box() {
    use crate::solid::RoundedBox;
    let shape = RoundedBox::new(2.0, 0.3);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "rounded box");
}

#[test]
fn test_infinite_cylinder() {
    use crate::solid::{InfiniteCylinder, Intersection};
    // Clip to a finite region with a sphere
    let shape = Intersection::new(InfiniteCylinder::new(1.0), Sphere::new(2.0));
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "infinite cylinder clipped");
}

#[test]
fn test_infinite_cone() {
    use crate::solid::{InfiniteCone, Intersection};
    // Clip double cone to a sphere
    let shape = Intersection::new(InfiniteCone::new(30.0), Sphere::new(2.0));
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "infinite cone clipped");
}

#[test]
fn test_mirror() {
    use crate::solid::Mirror;
    // Sphere offset in +X, mirrored across X → two spheres
    let shape = Mirror::new(
        Translate::new(Sphere::new(0.5), [1.0, 0.0, 0.0]),
        [true, false, false],
    );
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "mirror");
}

#[test]
fn test_repeat() {
    use crate::solid::Repeat;
    // 3x3x3 grid of spheres
    let shape = Repeat::new(Sphere::new(0.3), [1.0, 1.0, 1.0], [3, 3, 3]);
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&shape, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "repeat");
}

#[test]
fn test_cylinder_via_csg() {
    use crate::solid::{Capsule, InfiniteCylinder, Plane};
    // Cylinder built from capsule clipped by two planes (flat CSG surfaces
    // that previously caused non-manifold edges due to exact grid alignment).
    let capsule_clipped = Intersection::new(
        Intersection::new(Capsule::new(1.0, 4.0), Plane::new([0.0, 0.0, 1.0], 1.5)),
        Plane::new([0.0, 0.0, -1.0], 1.5),
    );
    let params = BccMeshParams {
        min_depth: 3,
        max_depth: 6,
        sdf_error_threshold: 0.01,
        ..Default::default()
    };
    let mesh = bcc_mesh(&capsule_clipped, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "capsule clipped by planes");

    // Infinite cylinder clipped by a cube (axis-aligned faces)
    let cyl_in_cube = Intersection::new(InfiniteCylinder::new(0.8), Cube::new(2.0));
    let mesh = bcc_mesh(&cyl_in_cube, [0.0, 0.0, 0.0], 4.0, &params);
    assert_mesh_valid(&mesh, "infinite cylinder in cube");
}
