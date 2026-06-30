use notify::{RecursiveMode, Watcher};
use sdf_cad::{generate_bcc_mesh, initial_step, BccMeshParams, MeshBuilder, SdfCache};
use std::path::{Path, PathBuf};
use std::sync::mpsc;

const SCAD_FILE: &str = "scene.scad";
const MIN_DEPTH: u32 = 2;
const MAX_DEPTH: u32 = 8;

fn main() {
    let path = Path::new(SCAD_FILE);

    let (mesh_tx, mesh_rx) = mpsc::channel::<MeshBuilder>();
    let (source_tx, source_rx) = mpsc::channel::<String>();
    let (file_tx, file_rx) = mpsc::channel::<PathBuf>();

    // Initial load
    match std::fs::read_to_string(path) {
        Ok(source) => {
            let _ = source_tx.send(source);
        }
        Err(e) => {
            eprintln!("Cannot read {}: {}", SCAD_FILE, e);
            eprintln!("Create a {} file to get started.", SCAD_FILE);
        }
    }

    // File watcher thread
    let source_tx_watch = source_tx.clone();
    let watch_path = path.to_path_buf();
    std::thread::spawn(move || {
        let (notify_tx, notify_rx) = mpsc::channel();
        let mut watcher = notify::recommended_watcher(move |res: notify::Result<notify::Event>| {
            if let Ok(event) = res {
                if event.kind.is_modify() || event.kind.is_create() {
                    let _ = notify_tx.send(());
                }
            }
        })
        .expect("failed to create file watcher");

        let mut watch_path = watch_path;
        let mut watch_dir = watch_path.parent().unwrap_or(Path::new(".")).to_path_buf();
        watcher
            .watch(&watch_dir, RecursiveMode::NonRecursive)
            .expect("failed to watch directory");

        eprintln!("Watching {} for changes...", watch_path.display());

        loop {
            // Wait for either a file change notification or a new file path
            std::thread::sleep(std::time::Duration::from_millis(50));

            // Check for new file to watch
            while let Ok(new_path) = file_rx.try_recv() {
                // Read and send the new file's source
                match std::fs::read_to_string(&new_path) {
                    Ok(source) => {
                        let _ = source_tx_watch.send(source);
                    }
                    Err(e) => {
                        eprintln!("Cannot read {}: {}", new_path.display(), e);
                        continue;
                    }
                }

                // Switch watcher to new file's directory
                let new_dir = new_path.parent().unwrap_or(Path::new(".")).to_path_buf();
                if new_dir != watch_dir {
                    let _ = watcher.unwatch(&watch_dir);
                    if let Err(e) = watcher.watch(&new_dir, RecursiveMode::NonRecursive) {
                        eprintln!("Failed to watch {}: {}", new_dir.display(), e);
                    }
                    watch_dir = new_dir;
                }
                watch_path = new_path;
                eprintln!("Watching {} for changes...", watch_path.display());
            }

            // Check for file modification notifications
            if notify_rx.try_recv().is_ok() {
                while notify_rx.try_recv().is_ok() {}

                match std::fs::read_to_string(&watch_path) {
                    Ok(source) => {
                        let _ = source_tx_watch.send(source);
                    }
                    Err(e) => eprintln!("Cannot read {}: {}", watch_path.display(), e),
                }
            }
        }
    });

    // Progressive meshing thread
    drop(source_tx);
    std::thread::spawn(move || {
        let mut current_source: Option<String> = None;

        loop {
            // Wait for first/next source
            let source = if let Some(s) = current_source.take() {
                s
            } else {
                match source_rx.recv() {
                    Ok(s) => s,
                    Err(_) => return,
                }
            };

            // Drain to latest source
            let mut source = source;
            while let Ok(newer) = source_rx.try_recv() {
                source = newer;
            }

            let solid = match sdf_cad::scad::parse(&source) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("Parse error: {}", e);
                    continue;
                }
            };

            // Progressive refinement: generate meshes at increasing depth.
            // Cache is shared across depths — lattice points are stable because
            // all runs use the same root_step (pinned to MAX_DEPTH).
            let cache = SdfCache::new(initial_step(MAX_DEPTH));
            let mut interrupted = false;
            for depth in MIN_DEPTH..=MAX_DEPTH {
                let params = BccMeshParams {
                    min_depth: MIN_DEPTH,
                    max_depth: depth,
                    sdf_error_threshold: 0.001,
                    ..Default::default()
                };
                let start = std::time::Instant::now();
                let mesh = generate_bcc_mesh(solid.as_ref(), [0.0, 0.0, 0.0], 20.0, &params, &cache);
                let elapsed = start.elapsed();
                let (is_watertight, _, _) = mesh.is_watertight();
                eprintln!(
                    "depth {}/{}: {} verts, {} tris in {:.0}ms (watertight: {})",
                    depth,
                    MAX_DEPTH,
                    mesh.vertices.len(),
                    mesh.triangles.len(),
                    elapsed.as_secs_f64() * 1000.0,
                    is_watertight
                );

                if mesh_tx.send(mesh).is_err() {
                    return; // viewer closed
                }

                // Check if a new source arrived while we were meshing
                if let Ok(newer) = source_rx.try_recv() {
                    current_source = Some(newer);
                    interrupted = true;
                    break;
                }
            }
            if !interrupted {
                eprintln!("Refinement complete.");
            }
        }
    });

    sdf_cad::viewer::run_viewer(mesh_rx, Some(file_tx));
}
