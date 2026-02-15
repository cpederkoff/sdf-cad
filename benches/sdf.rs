use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use sdf_cad::{
    Capsule, Cube, Difference, Intersection, RoundedBox, SmoothDifference, SmoothUnion, Solid,
    Sphere, Torus, Translate, Union, Vec3, Vec3x8,
};

const NUM_POINTS: usize = 10_000;
const SEED: u64 = 42;

fn random_points(n: usize, range: f32) -> Vec<Vec3> {
    let mut rng = StdRng::seed_from_u64(SEED);
    (0..n)
        .map(|_| {
            [
                rng.gen_range(-range..range),
                rng.gen_range(-range..range),
                rng.gen_range(-range..range),
            ]
        })
        .collect()
}

fn make_solids() -> Vec<(&'static str, Box<dyn Solid>)> {
    vec![
        ("sphere", Box::new(Sphere::new(5.0))),
        ("cube", Box::new(Cube::new(10.0))),
        ("torus", Box::new(Torus::new(4.0, 1.0))),
        ("capsule", Box::new(Capsule::new(1.0, 6.0))),
        ("rounded_box", Box::new(RoundedBox::new(8.0, 1.0))),
        (
            "union_2",
            Box::new(Union::new(
                Sphere::new(5.0),
                Translate::new(Cube::new(8.0), [3.0, 0.0, 0.0]),
            )),
        ),
        (
            "difference_2",
            Box::new(Difference::new(
                Cube::new(10.0),
                Translate::new(Sphere::new(5.0), [-4.7, -5.0, -5.0]),
            )),
        ),
        (
            "intersection_2",
            Box::new(Intersection::new(Sphere::new(5.0), Cube::new(8.0))),
        ),
        (
            "smooth_union_2",
            Box::new(SmoothUnion::new(
                Sphere::new(4.0),
                Translate::new(Sphere::new(3.0), [5.0, 0.0, 0.0]),
                0.5,
            )),
        ),
        (
            "smooth_diff_2",
            Box::new(SmoothDifference::new(
                Cube::new(10.0),
                Translate::new(Sphere::new(6.0), [3.0, 3.0, 3.0]),
                0.3,
            )),
        ),
        ("csg_deep", Box::new(make_deep_csg())),
        ("union_many", make_union_many()),
    ]
}

/// A deeper CSG tree: difference of smooth-union and intersection, with transforms.
fn make_deep_csg() -> impl Solid {
    Difference::new(
        SmoothUnion::new(
            Translate::new(Sphere::new(4.0), [-2.0, 0.0, 0.0]),
            Translate::new(Capsule::new(1.5, 6.0), [2.0, 0.0, 0.0]),
            0.5,
        ),
        Intersection::new(
            Translate::new(Cube::new(6.0), [0.0, 0.0, 1.0]),
            Torus::new(5.0, 1.5),
        ),
    )
}

/// A 5x5x5 grid of translated spheres combined via balanced union tree.
fn make_union_many() -> Box<dyn Solid> {
    let mut items: Vec<Box<dyn Solid>> = Vec::new();
    let spacing = 3.0;
    for x in 0..5 {
        for y in 0..5 {
            for z in 0..5 {
                items.push(Box::new(Translate::new(
                    Sphere::new(1.0),
                    [x as f32 * spacing, y as f32 * spacing, z as f32 * spacing],
                )));
            }
        }
    }
    fn balanced_union(mut items: Vec<Box<dyn Solid>>) -> Box<dyn Solid> {
        if items.len() == 1 {
            return items.pop().unwrap();
        }
        let mid = items.len() / 2;
        let right = items.split_off(mid);
        let left = balanced_union(items);
        let right = balanced_union(right);
        Box::new(Union::new_boxed(left, right))
    }
    balanced_union(items)
}

fn bench_sdf(c: &mut Criterion) {
    let points = random_points(NUM_POINTS, 10.0);
    let solids = make_solids();

    let mut group = c.benchmark_group("sdf");
    for (name, solid) in &solids {
        group.bench_with_input(BenchmarkId::new("query", name), &points, |b, pts| {
            b.iter(|| {
                let mut sum = 0.0f32;
                for p in pts {
                    sum += black_box(solid.sdf(*p));
                }
                sum
            });
        });
    }
    group.finish();
}

fn bench_sdf_bounds(c: &mut Criterion) {
    let points = random_points(NUM_POINTS, 10.0);
    let solids = make_solids();

    let mut group = c.benchmark_group("sdf_bounds");
    for (name, solid) in &solids {
        group.bench_with_input(BenchmarkId::new("query", name), &points, |b, pts| {
            b.iter(|| {
                let mut sum = 0.0f32;
                for p in pts {
                    let (lo, hi) = black_box(solid.sdf_bounds(*p, 0.5));
                    sum += lo + hi;
                }
                sum
            });
        });
    }
    group.finish();
}

fn bench_sdf_batch(c: &mut Criterion) {
    let points = random_points(NUM_POINTS, 10.0);
    // Pre-pack points into Vec3x8 batches
    let batches: Vec<Vec3x8> = points
        .chunks(8)
        .map(|chunk| Vec3x8::from_slice(chunk))
        .collect();
    let solids = make_solids();

    let mut group = c.benchmark_group("sdf_batch");
    for (name, solid) in &solids {
        group.bench_with_input(BenchmarkId::new("query_x8", name), &batches, |b, bats| {
            b.iter(|| {
                let mut sum = wide::f32x8::ZERO;
                for batch in bats {
                    sum += black_box(solid.sdf_batch(batch));
                }
                sum
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_sdf, bench_sdf_bounds, bench_sdf_batch);
criterion_main!(benches);
