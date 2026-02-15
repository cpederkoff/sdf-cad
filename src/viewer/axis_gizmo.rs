use three_d::*;

pub fn build_axis_gizmo(context: &Context) -> Vec<Gm<Mesh, ColorMaterial>> {
    let segments = 6u32;
    let shaft_r = 0.04f32;
    let shaft_len = 0.7f32;
    let tip_r = 0.12f32;
    let tip_len = 0.3f32;

    let mut positions = Vec::new();
    let mut indices = Vec::new();

    let circle: Vec<[f32; 2]> = (0..segments)
        .map(|i| {
            let a = 2.0 * std::f32::consts::PI * i as f32 / segments as f32;
            [a.cos(), a.sin()]
        })
        .collect();

    // Shaft bottom ring [0..segments)
    for c in &circle {
        positions.push(vec3(c[0] * shaft_r, 0.0, c[1] * shaft_r));
    }
    // Shaft top ring [segments..2*segments)
    for c in &circle {
        positions.push(vec3(c[0] * shaft_r, shaft_len, c[1] * shaft_r));
    }
    // Tip base ring [2*segments..3*segments)
    for c in &circle {
        positions.push(vec3(c[0] * tip_r, shaft_len, c[1] * tip_r));
    }
    // Tip apex
    let apex = 3 * segments;
    positions.push(vec3(0.0, shaft_len + tip_len, 0.0));
    // Bottom center
    let _bot = 3 * segments + 1;
    positions.push(vec3(0.0, 0.0, 0.0));

    // Shaft side quads
    for i in 0..segments {
        let j = (i + 1) % segments;
        indices.extend_from_slice(&[i, segments + i, j, j, segments + i, segments + j]);
    }
    // Cone sides
    for i in 0..segments {
        let j = (i + 1) % segments;
        indices.extend_from_slice(&[2 * segments + i, apex, 2 * segments + j]);
    }
    // Bottom cap
    for i in 1..segments - 1 {
        indices.extend_from_slice(&[0, i + 1, i]);
    }

    // Compute normals
    let mut normals = vec![vec3(0.0f32, 0.0, 0.0); positions.len()];
    for tri in indices.chunks(3) {
        let (i0, i1, i2) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
        let n = (positions[i1] - positions[i0]).cross(positions[i2] - positions[i0]);
        normals[i0] += n;
        normals[i1] += n;
        normals[i2] += n;
    }
    for n in &mut normals {
        let len = n.magnitude();
        if len > 0.0 {
            *n /= len;
        }
    }

    let base_mesh = CpuMesh {
        positions: Positions::F32(positions),
        indices: Indices::U32(indices),
        normals: Some(normals),
        ..Default::default()
    };

    let axes = [
        (
            Srgba::new(230, 60, 60, 255),
            Mat4::from_angle_z(degrees(-90.0)),
        ),
        (Srgba::new(60, 200, 60, 255), Mat4::identity()),
        (
            Srgba::new(60, 120, 230, 255),
            Mat4::from_angle_x(degrees(90.0)),
        ),
    ];

    axes.iter()
        .map(|(color, rot)| {
            let mut m = base_mesh.clone();
            m.transform(*rot).unwrap();
            Gm::new(
                Mesh::new(context, &m),
                ColorMaterial {
                    color: *color,
                    ..Default::default()
                },
            )
        })
        .collect()
}

pub fn render_gizmo(
    screen: &RenderTarget,
    gizmo: &[Gm<Mesh, ColorMaterial>],
    camera: &Camera,
    viewport: Viewport,
) {
    let gizmo_size = (viewport.height.min(viewport.width) / 6).max(80);
    let gizmo_viewport = Viewport {
        x: 10,
        y: 10,
        width: gizmo_size,
        height: gizmo_size,
    };
    let cam_dir = (camera.position() - camera.target()).normalize();
    let gizmo_cam = Camera::new_perspective(
        gizmo_viewport,
        cam_dir * 3.0,
        vec3(0.0, 0.0, 0.0),
        camera.up(),
        degrees(45.0),
        0.1,
        100.0,
    );
    screen.clear(ClearState {
        red: None,
        green: None,
        blue: None,
        alpha: None,
        depth: Some(1.0),
    });
    let no_lights: &[&dyn Light] = &[];
    for arrow in gizmo {
        screen.render(&gizmo_cam, arrow, no_lights);
    }
}
