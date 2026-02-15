use super::*;
use crate::solid::{
    Capsule, Cube, Difference, InfiniteCone, InfiniteCylinder, Intersection, Mirror, Rotate,
    RoundedBox, Scale, Solid, SmoothDifference, SmoothIntersection, SmoothUnion, Sphere, Torus,
    Translate, Union,
};

/// Deterministic PRNG (xorshift64) — no external dependency needed.
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

    /// Uniform f32 in [lo, hi]
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let t = (self.next() & 0xFFFF_FFFF) as f32 / 0xFFFF_FFFF_u64 as f32;
        lo + t * (hi - lo)
    }

    /// Uniform usize in [0, n)
    fn usize(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

/// Generate a random primitive solid.
fn random_primitive(rng: &mut Rng) -> Box<dyn Solid> {
    match rng.usize(7) {
        0 => Box::new(Sphere::new(rng.range(0.3, 1.5))),
        1 => Box::new(Cube::new(rng.range(0.6, 3.0))),
        2 => Box::new(Capsule::new(rng.range(0.2, 1.0), rng.range(0.5, 3.0))),
        3 => Box::new(Torus::new(rng.range(0.5, 1.5), rng.range(0.1, 0.4))),
        4 => Box::new(RoundedBox::new(rng.range(1.0, 2.5), rng.range(0.05, 0.3))),
        5 => {
            // InfiniteCylinder clipped by a sphere
            let r = rng.range(0.3, 1.2);
            Box::new(Intersection::new(
                InfiniteCylinder::new(r),
                Sphere::new(r + rng.range(0.5, 2.0)),
            ))
        }
        _ => {
            // InfiniteCone clipped by a sphere
            Box::new(Intersection::new(
                InfiniteCone::new(rng.range(15.0, 60.0)),
                Sphere::new(rng.range(1.0, 2.0)),
            ))
        }
    }
}

/// Generate a random CSG tree of given depth.
fn random_csg(rng: &mut Rng, depth: u32) -> Box<dyn Solid> {
    if depth == 0 {
        return random_primitive(rng);
    }

    // Sometimes emit a primitive even at non-zero depth
    if rng.usize(4) == 0 {
        return random_primitive(rng);
    }

    // Pick a random combinator
    match rng.usize(10) {
        // Binary CSG (6 variants)
        op @ 0..=5 => {
            let left = random_csg(rng, depth - 1);
            let right_depth = if depth > 1 {
                rng.usize(depth as usize) as u32
            } else {
                0
            };
            let right = random_csg(rng, right_depth);
            // Optionally translate the right operand
            let right: Box<dyn Solid> = if rng.usize(3) > 0 {
                let offset = [
                    rng.range(-1.0, 1.0),
                    rng.range(-1.0, 1.0),
                    rng.range(-1.0, 1.0),
                ];
                Box::new(Translate::new(right, offset))
            } else {
                right
            };
            match op {
                0 => Box::new(Union::new(left, right)),
                1 => Box::new(Intersection::new(left, right)),
                2 => Box::new(Difference::new(left, right)),
                3 => Box::new(SmoothUnion::new(left, right, rng.range(0.05, 0.4))),
                4 => Box::new(SmoothIntersection::new(left, right, rng.range(0.05, 0.4))),
                _ => Box::new(SmoothDifference::new(left, right, rng.range(0.05, 0.4))),
            }
        }
        // Translate
        6 => {
            let inner = random_csg(rng, depth - 1);
            let offset = [
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
            ];
            Box::new(Translate::new(inner, offset))
        }
        // Rotate
        7 => {
            let inner = random_csg(rng, depth - 1);
            let axis = [
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
            ];
            Box::new(Rotate::new(inner, axis, rng.range(0.0, 360.0)))
        }
        // Scale (uniform)
        8 => {
            let inner = random_csg(rng, depth - 1);
            Box::new(Scale::uniform(inner, rng.range(0.5, 2.0)))
        }
        // Mirror
        _ => {
            // Mirror needs the inner solid offset so there's something to reflect
            let inner =
                Translate::new(random_csg(rng, depth - 1), [rng.range(0.2, 1.0), 0.0, 0.0]);
            Box::new(Mirror::new(inner, [true, rng.usize(2) == 0, false]))
        }
    }
}

/// Fuzz-test random CSG trees for mesh quality issues.
///
/// Run all cases:  FUZZ_N=500 cargo test fuzz_random_csg -- --ignored --nocapture
/// Reproduce one:  FUZZ_SEED=33 cargo test fuzz_random_csg -- --ignored --nocapture
#[test]
#[ignore]
fn fuzz_random_csg() {
    use std::time::Instant;

    // FUZZ_SEED=N → reproduce a single case; otherwise sweep 0..num_cases
    let single_seed: Option<u64> = std::env::var("FUZZ_SEED").ok().and_then(|s| s.parse().ok());
    let num_cases: u64 = std::env::var("FUZZ_N")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(200);

    let seeds: Vec<u64> = if let Some(s) = single_seed {
        eprintln!("Reproducing single seed {s}");
        vec![s]
    } else {
        (0..num_cases).collect()
    };

    let total = seeds.len();

    // Per-case stats
    struct CaseStats {
        seed: u64,
        tris: usize,
        time_ms: f64,
        watertight: bool,
        sdf_err_mean: f32,
        sdf_err_max: f32,
        sdf_err_p95: f32,
    }

    let mut stats: Vec<CaseStats> = Vec::new();
    let mut fail_count = 0usize;
    let mut empty_count = 0usize;
    let wall_start = Instant::now();

    for seed in &seeds {
        let mut rng = Rng::new(seed + 1);
        let csg_depth = 2 + rng.usize(3) as u32; // 2..=4
        let shape = random_csg(&mut rng, csg_depth);
        // Clip to a sphere smaller than the grid so the solid never extends
        // past the grid boundary (which would cause uncapped boundary edges).
        let shape: Box<dyn Solid> =
            Box::new(Intersection::new(shape, Sphere::new(4.5)));

        let min_depth = 2 + rng.usize(2) as u32; // 2..=3
        let max_depth = min_depth + 1 + rng.usize(3) as u32; // +1..=+3
        let sdf_error_threshold = rng.range(0.005, 0.05);
        let params = BccMeshParams {
            min_depth,
            max_depth,
            sdf_error_threshold,
            interpolate: true,
            ..Default::default()
        };

        let cache = crate::mesh::cache::SdfCache::new(
            crate::mesh::octree::initial_step(params.max_depth),
        );
        let t0 = Instant::now();
        let mesh = generate_bcc_mesh(&*shape, [0.0, 0.0, 0.0], 5.0, &params, &cache);
        let time_ms = t0.elapsed().as_secs_f64() * 1000.0;

        if mesh.triangles.is_empty() {
            empty_count += 1;
            continue;
        }

        // If single seed, write out mesh
        if single_seed.is_some() {
            let filename = format!("fuzz_case_{seed}.obj");
            if let Err(e) = mesh.save(&filename) {
                eprintln!("Failed to write {filename}: {e}");
            } else {
                eprintln!("Wrote {filename}");
            }
        }

        let (watertight, boundary, non_manifold) = mesh.is_watertight();
        let (sdf_err_mean, sdf_err_max, sdf_err_p95) = mesh.sdf_error_stats(&*shape);

        if !watertight {
            fail_count += 1;
            eprintln!(
                "FAIL seed {seed}: {} boundary + {} non-manifold edges ({} tris) [depth={}-{}, err={:.4}] — reproduce with: FUZZ_SEED={seed} cargo test fuzz_random_csg -- --ignored --nocapture",
                boundary.len(), non_manifold.len(), mesh.triangles.len(),
                params.min_depth, params.max_depth, params.sdf_error_threshold,
            );
            if single_seed.is_some() {
                for (a, b) in boundary.iter().chain(non_manifold.iter()) {
                    let va = mesh.vertices[*a];
                    let vb = mesh.vertices[*b];
                    eprintln!(
                        "  edge ({a},{b}): ({:.6},{:.6},{:.6})--({:.6},{:.6},{:.6})",
                        va[0], va[1], va[2], vb[0], vb[1], vb[2]
                    );
                }
            }
        }

        stats.push(CaseStats {
            seed: *seed,
            tris: mesh.triangles.len(),
            time_ms,
            watertight,
            sdf_err_mean,
            sdf_err_max,
            sdf_err_p95,
        });
    }

    let wall_ms = wall_start.elapsed().as_secs_f64() * 1000.0;

    // Aggregate report
    let n = stats.len() as f64;
    if stats.is_empty() {
        eprintln!("No non-empty meshes generated.");
        return;
    }

    let watertight_count = stats.iter().filter(|s| s.watertight).count();

    let times: Vec<f64> = stats.iter().map(|s| s.time_ms).collect();
    let time_mean = times.iter().sum::<f64>() / n;
    let time_max = times.iter().cloned().fold(0.0f64, f64::max);
    let time_min = times.iter().cloned().fold(f64::MAX, f64::min);

    let tri_counts: Vec<f64> = stats.iter().map(|s| s.tris as f64).collect();
    let tri_mean = tri_counts.iter().sum::<f64>() / n;
    let tri_max = stats.iter().map(|s| s.tris).max().unwrap();
    let tri_min = stats.iter().map(|s| s.tris).min().unwrap();

    let sdf_means: Vec<f32> = stats.iter().map(|s| s.sdf_err_mean).collect();
    let sdf_mean_of_means = sdf_means.iter().sum::<f32>() / n as f32;
    let sdf_maxes: Vec<f32> = stats.iter().map(|s| s.sdf_err_max).collect();
    let sdf_worst_max = sdf_maxes.iter().cloned().fold(0.0f32, f32::max);
    let sdf_p95s: Vec<f32> = stats.iter().map(|s| s.sdf_err_p95).collect();
    let sdf_mean_p95 = sdf_p95s.iter().sum::<f32>() / n as f32;

    // Top 5 slowest
    let mut by_time: Vec<&CaseStats> = stats.iter().collect();
    by_time.sort_by(|a, b| b.time_ms.partial_cmp(&a.time_ms).unwrap());

    // Top 5 worst SDF error
    let mut by_sdf: Vec<&CaseStats> = stats.iter().collect();
    by_sdf.sort_by(|a, b| b.sdf_err_max.partial_cmp(&a.sdf_err_max).unwrap());

    eprintln!(
        "\n=== Fuzz Report: {} cases ({} empty, {} non-empty) ===",
        total,
        empty_count,
        stats.len()
    );
    eprintln!("Wall time: {:.1}ms\n", wall_ms);

    eprintln!(
        "Watertight: {}/{} ({:.1}%)",
        watertight_count,
        stats.len(),
        watertight_count as f64 / n * 100.0
    );

    eprintln!(
        "\nMesh time (ms):  min={:.1}  mean={:.1}  max={:.1}",
        time_min, time_mean, time_max
    );
    eprintln!(
        "Triangles:       min={}  mean={:.0}  max={}",
        tri_min, tri_mean, tri_max
    );
    eprintln!(
        "SDF error mean:  avg={:.6}  worst_max={:.6}  avg_p95={:.6}",
        sdf_mean_of_means, sdf_worst_max, sdf_mean_p95
    );

    eprintln!("\nTop 5 slowest:");
    eprintln!(
        "  {:>6} {:>8} {:>6} {:>10} {:>10}",
        "seed", "ms", "tris", "sdf_max", "watertight"
    );
    for s in by_time.iter().take(5) {
        eprintln!(
            "  {:>6} {:>8.1} {:>6} {:>10.6} {:>10}",
            s.seed,
            s.time_ms,
            s.tris,
            s.sdf_err_max,
            if s.watertight { "yes" } else { "NO" }
        );
    }

    eprintln!("\nTop 5 worst SDF error:");
    eprintln!(
        "  {:>6} {:>10} {:>10} {:>10} {:>6}",
        "seed", "sdf_max", "sdf_p95", "sdf_mean", "tris"
    );
    for s in by_sdf.iter().take(5) {
        eprintln!(
            "  {:>6} {:>10.6} {:>10.6} {:>10.6} {:>6}",
            s.seed, s.sdf_err_max, s.sdf_err_p95, s.sdf_err_mean, s.tris
        );
    }

    if fail_count > 0 {
        eprintln!();
        panic!(
            "{fail_count} / {} cases not watertight (see FAIL lines above)",
            stats.len()
        );
    }
}
