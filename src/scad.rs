use crate::scad_parser::Node;
use crate::solid::*;

fn fold_csg(op: CsgOp, mut items: Vec<Box<dyn Solid>>) -> Box<dyn Solid> {
    assert!(!items.is_empty());
    if items.len() == 1 {
        return items.pop().unwrap();
    }
    let mid = items.len() / 2;
    let right_items = items.split_off(mid);
    let left = fold_csg(op, items);
    let right = fold_csg(op, right_items);
    match op {
        CsgOp::Union => Box::new(Union::new_boxed(left, right)),
        CsgOp::Difference => Box::new(Difference::new_boxed(left, right)),
        CsgOp::Intersection => Box::new(Intersection::new_boxed(left, right)),
    }
}

#[derive(Clone, Copy)]
enum CsgOp {
    Union,
    Difference,
    Intersection,
}

fn make_cylinder(h: f32, r1: f32, r2: f32, center: bool) -> Box<dyn Solid> {
    let half_h = h / 2.0;

    let body: Box<dyn Solid> = if (r1 - r2).abs() < 1e-6 {
        Box::new(Intersection::new(
            Intersection::new(
                InfiniteCylinder::new(r1),
                Plane::new([0.0, 0.0, 1.0], half_h),
            ),
            Plane::new([0.0, 0.0, -1.0], half_h),
        ))
    } else if r2 < 1e-6 {
        let angle = (r1 / h).atan().to_degrees();
        let cone = Translate::new(InfiniteCone::new(angle), [0.0, 0.0, half_h]);
        Box::new(Intersection::new(
            Intersection::new(cone, Plane::new([0.0, 0.0, 1.0], half_h)),
            Plane::new([0.0, 0.0, -1.0], half_h),
        ))
    } else if r1 < 1e-6 {
        let angle = (r2 / h).atan().to_degrees();
        let cone = Translate::new(InfiniteCone::new(angle), [0.0, 0.0, -half_h]);
        Box::new(Intersection::new(
            Intersection::new(cone, Plane::new([0.0, 0.0, 1.0], half_h)),
            Plane::new([0.0, 0.0, -1.0], half_h),
        ))
    } else {
        let apex_z = -half_h + h * r1 / (r1 - r2);
        let angle = (r1 / (apex_z - (-half_h)).abs()).atan().to_degrees();
        let cone = Translate::new(InfiniteCone::new(angle), [0.0, 0.0, apex_z]);
        Box::new(Intersection::new(
            Intersection::new(cone, Plane::new([0.0, 0.0, 1.0], half_h)),
            Plane::new([0.0, 0.0, -1.0], half_h),
        ))
    };

    if center {
        body
    } else {
        Box::new(Translate::new_boxed(body, [0.0, 0.0, half_h]))
    }
}

fn build(node: &Node) -> Box<dyn Solid> {
    match node {
        Node::Cube { size, center } => {
            let solid: Box<dyn Solid> = if (size[0] - size[1]).abs() < 1e-6
                && (size[1] - size[2]).abs() < 1e-6
            {
                Box::new(Cube::new(size[0]))
            } else {
                Box::new(Scale::new(Cube::new(1.0), *size))
            };
            if *center {
                solid
            } else {
                Box::new(Translate::new_boxed(
                    solid,
                    [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0],
                ))
            }
        }

        Node::Sphere { radius } => Box::new(Sphere::new(*radius)),

        Node::Cylinder { h, r1, r2, center } => make_cylinder(*h, *r1, *r2, *center),

        Node::Translate { offset, child } => {
            Box::new(Translate::new_boxed(build(child), *offset))
        }

        Node::RotateEuler { angles, child } => {
            let mut solid = build(child);
            if angles[0].abs() > 1e-6 {
                solid = Box::new(Rotate::new(solid, [1.0, 0.0, 0.0], angles[0]));
            }
            if angles[1].abs() > 1e-6 {
                solid = Box::new(Rotate::new(solid, [0.0, 1.0, 0.0], angles[1]));
            }
            if angles[2].abs() > 1e-6 {
                solid = Box::new(Rotate::new(solid, [0.0, 0.0, 1.0], angles[2]));
            }
            solid
        }

        Node::RotateAxisAngle { axis, angle, child } => {
            Box::new(Rotate::new(build(child), *axis, *angle))
        }

        Node::Scale { factor, child } => {
            Box::new(Scale::new_boxed(build(child), *factor))
        }

        Node::Mirror { axes, child } => {
            Box::new(Mirror::new(
                build(child),
                [axes[0] != 0.0, axes[1] != 0.0, axes[2] != 0.0],
            ))
        }

        Node::Repeat { spacing, copies, child } => {
            Box::new(Repeat::new_boxed(build(child), *spacing, *copies))
        }

        Node::Color { .. } => panic!("color() is not supported in SDF mode"),
        Node::Circle { .. } => panic!("circle() is not supported in SDF mode"),
        Node::Square { .. } => panic!("square() is not supported in SDF mode"),
        Node::LinearExtrude { .. } => panic!("linear_extrude() is not supported in SDF mode"),
        Node::RotateExtrude { .. } => panic!("rotate_extrude() is not supported in SDF mode"),
        Node::Polygon { .. } => panic!("polygon() is not supported in SDF mode"),
        Node::Polyhedron { .. } => panic!("polyhedron() is not supported in SDF mode"),

        Node::Union(children) => {
            let items: Vec<_> = children.iter().map(|c| build(c)).collect();
            if items.is_empty() {
                Box::new(Sphere::new(0.0))
            } else {
                fold_csg(CsgOp::Union, items)
            }
        }
        Node::Difference(children) => {
            let items: Vec<_> = children.iter().map(|c| build(c)).collect();
            if items.is_empty() {
                Box::new(Sphere::new(0.0))
            } else {
                fold_csg(CsgOp::Difference, items)
            }
        }
        Node::Intersection(children) => {
            let items: Vec<_> = children.iter().map(|c| build(c)).collect();
            if items.is_empty() {
                Box::new(Sphere::new(0.0))
            } else {
                fold_csg(CsgOp::Intersection, items)
            }
        }
        Node::Hull(_) => panic!("hull() is not supported in SDF mode"),
        Node::Minkowski(_) => panic!("minkowski() is not supported in SDF mode"),
    }
}

/// Parse an OpenSCAD subset into a Solid.
pub fn parse(input: &str) -> Result<Box<dyn Solid>, String> {
    let node = crate::scad_parser::parse(input)?;
    Ok(build(&node))
}
