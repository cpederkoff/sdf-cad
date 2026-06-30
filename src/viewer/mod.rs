mod axis_gizmo;
mod camera_control;

use crate::mesh::MeshBuilder;
use axis_gizmo::{build_axis_gizmo, render_gizmo};
use camera_control::CameraControl;
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender};
use three_d::*;

fn build_model(context: &Context, mesh: &MeshBuilder) -> Gm<Mesh, PhysicalMaterial> {
    let positions: Vec<Vector3<f32>> = mesh
        .vertices
        .iter()
        .map(|v| vec3(v[0], v[1], v[2]))
        .collect();

    let indices: Vec<u32> = mesh
        .triangles
        .iter()
        .flat_map(|t| [t[0] as u32, t[1] as u32, t[2] as u32])
        .collect();

    let mut normals = vec![vec3(0.0f32, 0.0, 0.0); positions.len()];
    for tri in &mesh.triangles {
        let v0 = &positions[tri[0]];
        let v1 = &positions[tri[1]];
        let v2 = &positions[tri[2]];
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let face_normal = e1.cross(e2);
        normals[tri[0]] += face_normal;
        normals[tri[1]] += face_normal;
        normals[tri[2]] += face_normal;
    }
    for n in &mut normals {
        let len = n.magnitude();
        if len > 0.0 {
            *n /= len;
        }
    }

    let cpu_mesh = CpuMesh {
        positions: Positions::F32(positions),
        indices: Indices::U32(indices),
        normals: Some(normals),
        ..Default::default()
    };

    Gm::new(
        Mesh::new(context, &cpu_mesh),
        PhysicalMaterial::new_opaque(
            context,
            &CpuMaterial {
                albedo: Srgba::new(180, 180, 200, 255),
                metallic: 0.0,
                roughness: 0.5,
                ..Default::default()
            },
        ),
    )
}

pub fn show_mesh(mesh: &MeshBuilder) {
    let (tx, rx) = std::sync::mpsc::channel();
    let _ = tx.send(mesh.clone());
    drop(tx);
    run_viewer(rx, None);
}

pub fn run_viewer(rx: Receiver<MeshBuilder>, open_file_tx: Option<Sender<PathBuf>>) {
    let window = Window::new(WindowSettings {
        title: "Spheres Viewer".to_string(),
        max_size: Some((1280, 720)),
        ..Default::default()
    })
    .unwrap();

    let context = window.gl();

    let key_light = DirectionalLight::new(&context, 2.0, Srgba::WHITE, vec3(-1.0, -1.0, -2.0));
    let fill_light = DirectionalLight::new(&context, 1.0, Srgba::WHITE, vec3(2.0, -1.0, 1.0));
    let back_light = DirectionalLight::new(&context, 0.5, Srgba::WHITE, vec3(0.0, 1.0, 0.0));
    let ambient = AmbientLight::new(&context, 0.25, Srgba::WHITE);

    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(5.0, 5.0, 5.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.1,
        1000.0,
    );

    let mut control = CameraControl::new(camera.target(), 1.0, 100.0);
    let mut model: Option<Gm<Mesh, PhysicalMaterial>> = None;
    let mut current_mesh: Option<MeshBuilder> = None;
    let axis_gizmo = build_axis_gizmo(&context);

    window.render_loop(move |mut frame_input| {
        // Check for new mesh (non-blocking)
        while let Ok(new_mesh) = rx.try_recv() {
            model = Some(build_model(&context, &new_mesh));
            current_mesh = Some(new_mesh);
        }

        let mut exit = false;
        for event in &frame_input.events {
            if let Event::KeyPress {
                kind, modifiers, ..
            } = event
            {
                if *kind == Key::Q && modifiers.ctrl {
                    exit = true;
                }
                if *kind == Key::S && modifiers.ctrl {
                    if let Some(ref mesh) = current_mesh {
                        if let Some(path) = tinyfiledialogs::save_file_dialog_with_filter(
                            "Save mesh",
                            "mesh.obj",
                            &["*.obj", "*.stl", "*.vtk"],
                            "Mesh files",
                        ) {
                            let before = mesh.triangles.len();
                            let export_mesh = mesh.clone();
                            // crate::mesh::decimate_flat(&mut export_mesh, 0.999);
                            // crate::mesh::collapse_short_edges(&mut export_mesh, 0.3);
                            eprintln!(
                                "decimation: {} → {} tris (−{:.1}%)",
                                before,
                                export_mesh.triangles.len(),
                                (1.0 - export_mesh.triangles.len() as f64 / before as f64) * 100.0
                            );
                            match export_mesh.save(&path) {
                                Ok(()) => println!(
                                    "Saved {} vertices, {} triangles to {path}",
                                    export_mesh.vertices.len(),
                                    export_mesh.triangles.len()
                                ),
                                Err(e) => eprintln!("Failed to save mesh: {e}"),
                            }
                        }
                    }
                }
                if *kind == Key::O && modifiers.ctrl {
                    if let Some(ref tx) = open_file_tx {
                        if let Some(path) = tinyfiledialogs::open_file_dialog(
                            "Open SCAD file",
                            "",
                            Some((&["*.scad"], "SCAD files")),
                        ) {
                            let _ = tx.send(PathBuf::from(path));
                        }
                    }
                }
            }
        }

        camera.set_viewport(frame_input.viewport);
        control.handle_events(&mut camera, &mut frame_input.events);

        let screen = frame_input.screen();
        let screen = screen.clear(ClearState::color_and_depth(0.1, 0.1, 0.1, 1.0, 1.0));

        if let Some(ref m) = model {
            screen.render(
                &camera,
                m,
                &[&key_light, &fill_light, &back_light, &ambient],
            );
        }

        render_gizmo(&screen, &axis_gizmo, &camera, frame_input.viewport);

        FrameOutput {
            exit,
            ..Default::default()
        }
    });
}
