use super::*;
use crate::math::LatticePoint;

fn lp(x: i32, y: i32, z: i32) -> LatticePoint {
    LatticePoint::new(x, y, z)
}

#[test]
fn test_cell_center() {
    // Cell at lattice (0,0,0) step=4, unit_size=0.5 → center at [1,1,1]
    let center = cell_center(lp(0, 0, 0), 4, [0.0, 0.0, 0.0], 0.5);
    assert_eq!(center, [1.0, 1.0, 1.0]);

    // Cell at lattice (4,0,0) step=4, unit_size=0.5 → center at [3,1,1]
    let center = cell_center(lp(4, 0, 0), 4, [0.0, 0.0, 0.0], 0.5);
    assert_eq!(center, [3.0, 1.0, 1.0]);
}

#[test]
fn test_cell_circumradius() {
    let r = cell_circumradius(1.0);
    assert!((r - 3.0_f32.sqrt() / 2.0).abs() < 1e-6);

    let r = cell_circumradius(2.0);
    assert!((r - 3.0_f32.sqrt()).abs() < 1e-6);
}

#[test]
fn test_subdivide_cell() {
    // Cell (0,0,0) step=4 → 8 children at half=2
    let subcells = subdivide_cell(lp(0, 0, 0), 4);
    let expected = [
        lp(0, 0, 0),
        lp(2, 0, 0),
        lp(0, 2, 0),
        lp(2, 2, 0),
        lp(0, 0, 2),
        lp(2, 0, 2),
        lp(0, 2, 2),
        lp(2, 2, 2),
    ];
    assert_eq!(subcells, expected);
}

#[test]
fn test_cell_at_step() {
    // Snap (4,2,6) to step 4 → (4,0,4)
    let snapped = cell_at_step(lp(4, 2, 6), 4);
    assert_eq!(snapped, lp(4, 0, 4));

    // Snap (6,3,1) to step 8 → (0,0,0)
    let snapped = cell_at_step(lp(6, 3, 1), 8);
    assert_eq!(snapped, lp(0, 0, 0));
}
