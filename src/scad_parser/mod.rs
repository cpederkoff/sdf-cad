mod ast;
mod parser;
mod tokenizer;

pub use ast::Node;

/// Parse an OpenSCAD subset into an AST.
pub fn parse(input: &str) -> Result<Node, String> {
    let tokens = tokenizer::tokenize(input)?;
    let mut parser = parser::Parser::new(tokens);
    parser.parse_top_level()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── 3D Primitives ─────────────────────────────────────────────

    #[test]
    fn cube_uniform() {
        assert_eq!(
            parse("cube(2);").unwrap(),
            Node::Cube {
                size: [2.0, 2.0, 2.0],
                center: false,
            }
        );
    }

    #[test]
    fn cube_vector() {
        assert_eq!(
            parse("cube([1, 2, 3]);").unwrap(),
            Node::Cube {
                size: [1.0, 2.0, 3.0],
                center: false,
            }
        );
    }

    #[test]
    fn cube_centered() {
        assert_eq!(
            parse("cube(2, center=true);").unwrap(),
            Node::Cube {
                size: [2.0, 2.0, 2.0],
                center: true,
            }
        );
    }

    #[test]
    fn sphere_positional() {
        assert_eq!(parse("sphere(1);").unwrap(), Node::Sphere { radius: 1.0 });
    }

    #[test]
    fn sphere_named_r() {
        assert_eq!(parse("sphere(r=2.5);").unwrap(), Node::Sphere { radius: 2.5 });
    }

    #[test]
    fn sphere_named_d() {
        assert_eq!(parse("sphere(d=10);").unwrap(), Node::Sphere { radius: 5.0 });
    }

    #[test]
    fn cylinder_positional() {
        assert_eq!(
            parse("cylinder(4, 1);").unwrap(),
            Node::Cylinder {
                h: 4.0,
                r1: 1.0,
                r2: 1.0,
                center: false,
            }
        );
    }

    #[test]
    fn cylinder_cone() {
        assert_eq!(
            parse("cylinder(3, 2, 0);").unwrap(),
            Node::Cylinder {
                h: 3.0,
                r1: 2.0,
                r2: 0.0,
                center: false,
            }
        );
    }

    #[test]
    fn cylinder_named() {
        assert_eq!(
            parse("cylinder(h=4, r=1);").unwrap(),
            Node::Cylinder {
                h: 4.0,
                r1: 1.0,
                r2: 1.0,
                center: false,
            }
        );
    }

    #[test]
    fn cylinder_diameter() {
        assert_eq!(
            parse("cylinder(h=4, d=2);").unwrap(),
            Node::Cylinder {
                h: 4.0,
                r1: 1.0,
                r2: 1.0,
                center: false,
            }
        );
    }

    #[test]
    fn cylinder_centered() {
        assert_eq!(
            parse("cylinder(4, 1, center=true);").unwrap(),
            Node::Cylinder {
                h: 4.0,
                r1: 1.0,
                r2: 1.0,
                center: true,
            }
        );
    }

    #[test]
    fn polyhedron_basic() {
        let node = parse(
            "polyhedron(points=[[0,0,0],[1,0,0],[0,1,0],[0,0,1]], faces=[[0,1,2],[0,1,3],[0,2,3],[1,2,3]]);",
        ).unwrap();
        match node {
            Node::Polyhedron { points, faces } => {
                assert_eq!(points.len(), 4);
                assert_eq!(faces.len(), 4);
            }
            _ => panic!("expected Polyhedron"),
        }
    }

    // ── 2D Primitives ─────────────────────────────────────────────

    #[test]
    fn circle_positional() {
        assert_eq!(parse("circle(5);").unwrap(), Node::Circle { radius: 5.0 });
    }

    #[test]
    fn circle_diameter() {
        assert_eq!(parse("circle(d=10);").unwrap(), Node::Circle { radius: 5.0 });
    }

    #[test]
    fn square_uniform() {
        assert_eq!(
            parse("square(5);").unwrap(),
            Node::Square {
                size: [5.0, 5.0],
                center: false,
            }
        );
    }

    #[test]
    fn square_vector() {
        assert_eq!(
            parse("square([3, 4], center=true);").unwrap(),
            Node::Square {
                size: [3.0, 4.0],
                center: true,
            }
        );
    }

    #[test]
    fn polygon_basic() {
        let node = parse("polygon([[0,0],[10,0],[5,10]]);").unwrap();
        assert_eq!(
            node,
            Node::Polygon {
                points: vec![[0.0, 0.0], [10.0, 0.0], [5.0, 10.0]],
                paths: None,
            }
        );
    }

    #[test]
    fn polygon_with_paths() {
        let node =
            parse("polygon(points=[[0,0],[10,0],[10,10],[0,10]], paths=[[0,1,2,3]]);").unwrap();
        assert_eq!(
            node,
            Node::Polygon {
                points: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
                paths: Some(vec![vec![0, 1, 2, 3]]),
            }
        );
    }

    // ── Transforms ────────────────────────────────────────────────

    #[test]
    fn translate() {
        assert_eq!(
            parse("translate([5, 0, 0]) sphere(1);").unwrap(),
            Node::Translate {
                offset: [5.0, 0.0, 0.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn rotate_euler() {
        assert_eq!(
            parse("rotate([0, 0, 90]) sphere(1);").unwrap(),
            Node::RotateEuler {
                angles: [0.0, 0.0, 90.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn rotate_axis_angle() {
        assert_eq!(
            parse("rotate(90, v=[0, 0, 1]) sphere(1);").unwrap(),
            Node::RotateAxisAngle {
                axis: [0.0, 0.0, 1.0],
                angle: 90.0,
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn rotate_default_z() {
        assert_eq!(
            parse("rotate(90) sphere(1);").unwrap(),
            Node::RotateAxisAngle {
                axis: [0.0, 0.0, 1.0],
                angle: 90.0,
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn scale_uniform() {
        assert_eq!(
            parse("scale(2) sphere(1);").unwrap(),
            Node::Scale {
                factor: [2.0, 2.0, 2.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn scale_vector() {
        assert_eq!(
            parse("scale([2, 1, 1]) sphere(1);").unwrap(),
            Node::Scale {
                factor: [2.0, 1.0, 1.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn mirror_basic() {
        assert_eq!(
            parse("mirror([1, 0, 0]) cube(1);").unwrap(),
            Node::Mirror {
                axes: [1.0, 0.0, 0.0],
                child: Box::new(Node::Cube {
                    size: [1.0, 1.0, 1.0],
                    center: false,
                }),
            }
        );
    }

    #[test]
    fn color_named() {
        assert_eq!(
            parse(r#"color("red") sphere(1);"#).unwrap(),
            Node::Color {
                rgba: [1.0, 0.0, 0.0, 1.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn color_rgba_vector() {
        assert_eq!(
            parse("color([1, 0, 0, 0.5]) sphere(1);").unwrap(),
            Node::Color {
                rgba: [1.0, 0.0, 0.0, 0.5],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn color_named_with_alpha() {
        let node = parse(r#"color("blue", 0.5) sphere(1);"#).unwrap();
        match node {
            Node::Color { rgba, .. } => {
                assert_eq!(rgba[3], 0.5);
            }
            _ => panic!("expected Color"),
        }
    }

    // ── Extrusions ────────────────────────────────────────────────

    #[test]
    fn linear_extrude_basic() {
        assert_eq!(
            parse("linear_extrude(10) circle(5);").unwrap(),
            Node::LinearExtrude {
                height: 10.0,
                center: false,
                twist: 0.0,
                slices: None,
                child: Box::new(Node::Circle { radius: 5.0 }),
            }
        );
    }

    #[test]
    fn linear_extrude_named() {
        assert_eq!(
            parse("linear_extrude(height=10, center=true, twist=90) circle(5);").unwrap(),
            Node::LinearExtrude {
                height: 10.0,
                center: true,
                twist: 90.0,
                slices: None,
                child: Box::new(Node::Circle { radius: 5.0 }),
            }
        );
    }

    #[test]
    fn rotate_extrude_basic() {
        assert_eq!(
            parse("rotate_extrude() circle(5);").unwrap(),
            Node::RotateExtrude {
                angle: 360.0,
                child: Box::new(Node::Circle { radius: 5.0 }),
            }
        );
    }

    #[test]
    fn rotate_extrude_angle() {
        assert_eq!(
            parse("rotate_extrude(angle=180) square(5);").unwrap(),
            Node::RotateExtrude {
                angle: 180.0,
                child: Box::new(Node::Square {
                    size: [5.0, 5.0],
                    center: false,
                }),
            }
        );
    }

    // ── CSG operations ────────────────────────────────────────────

    #[test]
    fn union_block() {
        assert_eq!(
            parse("union() { cube(1); sphere(1); }").unwrap(),
            Node::Union(vec![
                Node::Cube {
                    size: [1.0, 1.0, 1.0],
                    center: false,
                },
                Node::Sphere { radius: 1.0 },
            ])
        );
    }

    #[test]
    fn difference_block() {
        assert_eq!(
            parse("difference() { cube(4, center=true); sphere(1); }").unwrap(),
            Node::Difference(vec![
                Node::Cube {
                    size: [4.0, 4.0, 4.0],
                    center: true,
                },
                Node::Sphere { radius: 1.0 },
            ])
        );
    }

    #[test]
    fn intersection_block() {
        assert_eq!(
            parse("intersection() { cube(2, center=true); sphere(1); }").unwrap(),
            Node::Intersection(vec![
                Node::Cube {
                    size: [2.0, 2.0, 2.0],
                    center: true,
                },
                Node::Sphere { radius: 1.0 },
            ])
        );
    }

    #[test]
    fn hull_block() {
        assert_eq!(
            parse("hull() { sphere(1); translate([5,0,0]) sphere(1); }").unwrap(),
            Node::Hull(vec![
                Node::Sphere { radius: 1.0 },
                Node::Translate {
                    offset: [5.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 1.0 }),
                },
            ])
        );
    }

    #[test]
    fn minkowski_block() {
        assert_eq!(
            parse("minkowski() { cube(10); sphere(1); }").unwrap(),
            Node::Minkowski(vec![
                Node::Cube {
                    size: [10.0, 10.0, 10.0],
                    center: false,
                },
                Node::Sphere { radius: 1.0 },
            ])
        );
    }

    #[test]
    fn implicit_union() {
        assert_eq!(
            parse("cube(2); sphere(1);").unwrap(),
            Node::Union(vec![
                Node::Cube {
                    size: [2.0, 2.0, 2.0],
                    center: false,
                },
                Node::Sphere { radius: 1.0 },
            ])
        );
    }

    #[test]
    fn single_statement_no_wrapping_union() {
        assert_eq!(parse("sphere(1);").unwrap(), Node::Sphere { radius: 1.0 });
    }

    // ── Operators & expressions ───────────────────────────────────

    #[test]
    fn arithmetic_precedence() {
        assert_eq!(
            parse("cube(2 + 3 * 4, center=true);").unwrap(),
            Node::Cube {
                size: [14.0, 14.0, 14.0],
                center: true,
            }
        );
    }

    #[test]
    fn arithmetic_parens() {
        assert_eq!(
            parse("cube((2 + 3) * 4, center=true);").unwrap(),
            Node::Cube {
                size: [20.0, 20.0, 20.0],
                center: true,
            }
        );
    }

    #[test]
    fn arithmetic_unary_minus() {
        assert_eq!(
            parse("translate([0, 0, -1 - 2]) sphere(1);").unwrap(),
            Node::Translate {
                offset: [0.0, 0.0, -3.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn arithmetic_with_variables() {
        assert_eq!(
            parse("a = 10; cube(a / 2 - 1);").unwrap(),
            Node::Cube {
                size: [4.0, 4.0, 4.0],
                center: false,
            }
        );
    }

    #[test]
    fn modulo_operator() {
        // 10 % 3 = 1
        assert_eq!(
            parse("cube(10 % 3);").unwrap(),
            Node::Cube {
                size: [1.0, 1.0, 1.0],
                center: false,
            }
        );
    }

    #[test]
    fn power_operator() {
        // 2 ^ 3 = 8
        assert_eq!(
            parse("cube(2 ^ 3);").unwrap(),
            Node::Cube {
                size: [8.0, 8.0, 8.0],
                center: false,
            }
        );
    }

    #[test]
    fn ternary_true() {
        assert_eq!(
            parse("cube(1 > 0 ? 5 : 10);").unwrap(),
            Node::Cube {
                size: [5.0, 5.0, 5.0],
                center: false,
            }
        );
    }

    #[test]
    fn ternary_false() {
        assert_eq!(
            parse("cube(0 > 1 ? 5 : 10);").unwrap(),
            Node::Cube {
                size: [10.0, 10.0, 10.0],
                center: false,
            }
        );
    }

    #[test]
    fn comparison_operators() {
        // All return 1.0 (true) or 0.0 (false), use in ternary to test
        assert_eq!(parse("cube(3 < 5 ? 1 : 0);").unwrap(), Node::Cube { size: [1.0; 3], center: false });
        assert_eq!(parse("cube(5 < 3 ? 1 : 0);").unwrap(), Node::Cube { size: [0.0; 3], center: false });
        assert_eq!(parse("cube(3 <= 3 ? 1 : 0);").unwrap(), Node::Cube { size: [1.0; 3], center: false });
        assert_eq!(parse("cube(3 >= 3 ? 1 : 0);").unwrap(), Node::Cube { size: [1.0; 3], center: false });
        assert_eq!(parse("cube(3 == 3 ? 1 : 0);").unwrap(), Node::Cube { size: [1.0; 3], center: false });
        assert_eq!(parse("cube(3 != 4 ? 1 : 0);").unwrap(), Node::Cube { size: [1.0; 3], center: false });
    }

    #[test]
    fn logical_operators() {
        assert_eq!(parse("cube(1 && 1 ? 2 : 0);").unwrap(), Node::Cube { size: [2.0; 3], center: false });
        assert_eq!(parse("cube(1 && 0 ? 2 : 0);").unwrap(), Node::Cube { size: [0.0; 3], center: false });
        assert_eq!(parse("cube(0 || 1 ? 2 : 0);").unwrap(), Node::Cube { size: [2.0; 3], center: false });
        assert_eq!(parse("cube(!0 ? 2 : 0);").unwrap(), Node::Cube { size: [2.0; 3], center: false });
        assert_eq!(parse("cube(!1 ? 2 : 0);").unwrap(), Node::Cube { size: [0.0; 3], center: false });
    }

    // ── Math functions ────────────────────────────────────────────

    #[test]
    fn math_trig() {
        // sin(90) = 1, cos(0) = 1
        let node = parse("cube(sin(90));").unwrap();
        match node {
            Node::Cube { size, .. } => assert!((size[0] - 1.0).abs() < 1e-5),
            _ => panic!("expected Cube"),
        }
        let node = parse("cube(cos(0));").unwrap();
        match node {
            Node::Cube { size, .. } => assert!((size[0] - 1.0).abs() < 1e-5),
            _ => panic!("expected Cube"),
        }
    }

    #[test]
    fn math_sqrt_pow() {
        let node = parse("cube(sqrt(9));").unwrap();
        match node {
            Node::Cube { size, .. } => assert!((size[0] - 3.0).abs() < 1e-5),
            _ => panic!("expected Cube"),
        }
        let node = parse("cube(pow(2, 3));").unwrap();
        match node {
            Node::Cube { size, .. } => assert!((size[0] - 8.0).abs() < 1e-5),
            _ => panic!("expected Cube"),
        }
    }

    #[test]
    fn math_abs_sign() {
        assert_eq!(
            parse("cube(abs(-5));").unwrap(),
            Node::Cube { size: [5.0; 3], center: false }
        );
    }

    #[test]
    fn math_floor_ceil_round() {
        assert_eq!(parse("cube(floor(3.7));").unwrap(), Node::Cube { size: [3.0; 3], center: false });
        assert_eq!(parse("cube(ceil(3.2));").unwrap(), Node::Cube { size: [4.0; 3], center: false });
        assert_eq!(parse("cube(round(3.5));").unwrap(), Node::Cube { size: [4.0; 3], center: false });
    }

    #[test]
    fn math_min_max() {
        assert_eq!(parse("cube(min(3, 5, 1));").unwrap(), Node::Cube { size: [1.0; 3], center: false });
        assert_eq!(parse("cube(max(3, 5, 1));").unwrap(), Node::Cube { size: [5.0; 3], center: false });
    }

    #[test]
    fn constant_pi() {
        let node = parse("cube(PI);").unwrap();
        match node {
            Node::Cube { size, .. } => assert!((size[0] - std::f32::consts::PI).abs() < 1e-5),
            _ => panic!("expected Cube"),
        }
    }

    // ── Control flow ──────────────────────────────────────────────

    #[test]
    fn if_true() {
        assert_eq!(
            parse("if (1) cube(1);").unwrap(),
            Node::Cube {
                size: [1.0; 3],
                center: false,
            }
        );
    }

    #[test]
    fn if_false_no_else() {
        assert_eq!(parse("if (0) cube(1);").unwrap(), Node::Union(Vec::new()));
    }

    #[test]
    fn if_else() {
        assert_eq!(
            parse("if (0) cube(1); else sphere(2);").unwrap(),
            Node::Sphere { radius: 2.0 }
        );
    }

    #[test]
    fn if_with_condition() {
        assert_eq!(
            parse("x = 5; if (x > 3) cube(x);").unwrap(),
            Node::Cube {
                size: [5.0; 3],
                center: false,
            }
        );
    }

    #[test]
    fn let_block() {
        assert_eq!(
            parse("let (x = 3, y = 4) translate([x, y, 0]) sphere(1);").unwrap(),
            Node::Translate {
                offset: [3.0, 4.0, 0.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn for_basic() {
        assert_eq!(
            parse("for (i = [0 : 2]) translate([i, 0, 0]) sphere(0.3);").unwrap(),
            Node::Union(vec![
                Node::Translate {
                    offset: [0.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
                Node::Translate {
                    offset: [1.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
                Node::Translate {
                    offset: [2.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
            ])
        );
    }

    #[test]
    fn for_with_step() {
        assert_eq!(
            parse("for (i = [0 : 2 : 4]) translate([i, 0, 0]) sphere(0.3);").unwrap(),
            Node::Union(vec![
                Node::Translate {
                    offset: [0.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
                Node::Translate {
                    offset: [2.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
                Node::Translate {
                    offset: [4.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
            ])
        );
    }

    #[test]
    fn for_with_list() {
        assert_eq!(
            parse("for (i = [1, 3, 7]) translate([i, 0, 0]) sphere(0.3);").unwrap(),
            Node::Union(vec![
                Node::Translate {
                    offset: [1.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
                Node::Translate {
                    offset: [3.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
                Node::Translate {
                    offset: [7.0, 0.0, 0.0],
                    child: Box::new(Node::Sphere { radius: 0.3 }),
                },
            ])
        );
    }

    #[test]
    fn for_variable_scoping() {
        let node =
            parse("i = 5; for (i = [0 : 0]) sphere(1); translate([i, 0, 0]) sphere(1);").unwrap();
        match node {
            Node::Union(ref items) => {
                assert_eq!(items.len(), 2);
                assert_eq!(
                    items[1],
                    Node::Translate {
                        offset: [5.0, 0.0, 0.0],
                        child: Box::new(Node::Sphere { radius: 1.0 }),
                    }
                );
            }
            _ => panic!("expected Union"),
        }
    }

    // ── Misc ──────────────────────────────────────────────────────

    #[test]
    fn comments() {
        assert_eq!(
            parse("// a comment\ncube(2); /* block */ sphere(1);").unwrap(),
            Node::Union(vec![
                Node::Cube {
                    size: [2.0, 2.0, 2.0],
                    center: false,
                },
                Node::Sphere { radius: 1.0 },
            ])
        );
    }

    #[test]
    fn variable_basic() {
        assert_eq!(
            parse("size = 3; cube(size);").unwrap(),
            Node::Cube {
                size: [3.0, 3.0, 3.0],
                center: false,
            }
        );
    }

    #[test]
    fn nested_transforms() {
        assert_eq!(
            parse("translate([1, 0, 0]) rotate([0, 0, 90]) scale(2) sphere(1);").unwrap(),
            Node::Translate {
                offset: [1.0, 0.0, 0.0],
                child: Box::new(Node::RotateEuler {
                    angles: [0.0, 0.0, 90.0],
                    child: Box::new(Node::Scale {
                        factor: [2.0, 2.0, 2.0],
                        child: Box::new(Node::Sphere { radius: 1.0 }),
                    }),
                }),
            }
        );
    }

    #[test]
    fn repeat_node() {
        assert_eq!(
            parse("repeat([2, 1, 1], [3, 1, 1]) sphere(0.5);").unwrap(),
            Node::Repeat {
                spacing: [2.0, 1.0, 1.0],
                copies: [3, 1, 1],
                child: Box::new(Node::Sphere { radius: 0.5 }),
            }
        );
    }

    #[test]
    fn echo_is_skipped() {
        // echo should be silently skipped
        assert_eq!(
            parse("echo(\"hello\"); sphere(1);").unwrap(),
            Node::Sphere { radius: 1.0 }
        );
    }

    #[test]
    fn modifier_hash_ignored() {
        assert_eq!(
            parse("# sphere(1);").unwrap(),
            Node::Sphere { radius: 1.0 }
        );
    }

    #[test]
    fn variable_undefined() {
        let err = parse("cube(undefined_var);").err().expect("should fail");
        assert!(err.contains("undefined variable"));
    }

    #[test]
    fn empty_input() {
        assert!(parse("").is_err());
    }

    #[test]
    fn unterminated_block_comment() {
        assert!(parse("/* oops").is_err());
    }

    #[test]
    fn string_literal() {
        let node = parse(r#"color("green") cube(1);"#).unwrap();
        match node {
            Node::Color { rgba, .. } => {
                assert_eq!(rgba[1], 0.5);
            }
            _ => panic!("expected Color"),
        }
    }

    // ── Module definitions ────────────────────────────────────────

    #[test]
    fn module_basic() {
        let node = parse(
            "module foo(s) { cube(s); } foo(5);",
        ).unwrap();
        assert_eq!(
            node,
            Node::Cube {
                size: [5.0, 5.0, 5.0],
                center: false,
            }
        );
    }

    #[test]
    fn module_default_params() {
        let node = parse(
            "module bar(x, y=10) { translate([x, y, 0]) sphere(1); } bar(3);",
        ).unwrap();
        assert_eq!(
            node,
            Node::Translate {
                offset: [3.0, 10.0, 0.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn module_named_args() {
        let node = parse(
            "module bar(x, y) { translate([x, y, 0]) sphere(1); } bar(y=4, x=2);",
        ).unwrap();
        assert_eq!(
            node,
            Node::Translate {
                offset: [2.0, 4.0, 0.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn module_with_children() {
        let node = parse(
            "module wrapper() { translate([1,0,0]) children(); } wrapper() sphere(1);",
        ).unwrap();
        assert_eq!(
            node,
            Node::Translate {
                offset: [1.0, 0.0, 0.0],
                child: Box::new(Node::Sphere { radius: 1.0 }),
            }
        );
    }

    #[test]
    fn module_children_block() {
        let node = parse(
            "module wrapper() { children(); } wrapper() { cube(1); sphere(1); }",
        ).unwrap();
        assert_eq!(
            node,
            Node::Union(vec![
                Node::Cube { size: [1.0; 3], center: false },
                Node::Sphere { radius: 1.0 },
            ])
        );
    }

    #[test]
    fn module_children_indexed() {
        let node = parse(
            "module pick() { children(1); } pick() { cube(1); sphere(2); }",
        ).unwrap();
        assert_eq!(node, Node::Sphere { radius: 2.0 });
    }

    #[test]
    fn module_no_children() {
        let node = parse(
            "module thing() { cube(1); } thing();",
        ).unwrap();
        assert_eq!(
            node,
            Node::Cube { size: [1.0; 3], center: false }
        );
    }

    #[test]
    fn module_forward_reference() {
        // Module used before it's defined
        let node = parse(
            "my_sphere(3); module my_sphere(r) { sphere(r); }",
        ).unwrap();
        assert_eq!(node, Node::Sphere { radius: 3.0 });
    }

    // ── Function definitions ──────────────────────────────────────

    #[test]
    fn function_basic() {
        let node = parse(
            "function double(x) = x * 2; cube(double(3));",
        ).unwrap();
        assert_eq!(
            node,
            Node::Cube { size: [6.0; 3], center: false }
        );
    }

    #[test]
    fn function_default_params() {
        let node = parse(
            "function add(a, b=10) = a + b; cube(add(5));",
        ).unwrap();
        assert_eq!(
            node,
            Node::Cube { size: [15.0; 3], center: false }
        );
    }

    #[test]
    fn function_forward_reference() {
        let node = parse(
            "cube(triple(2)); function triple(x) = x * 3;",
        ).unwrap();
        assert_eq!(
            node,
            Node::Cube { size: [6.0; 3], center: false }
        );
    }

    #[test]
    fn function_composition() {
        let node = parse(
            "function double(x) = x * 2; function quad(x) = double(double(x)); cube(quad(3));",
        ).unwrap();
        assert_eq!(
            node,
            Node::Cube { size: [12.0; 3], center: false }
        );
    }

    #[test]
    fn module_and_function_together() {
        let node = parse(
            "function radius(d) = d / 2; module ball(d) { sphere(radius(d)); } ball(10);",
        ).unwrap();
        assert_eq!(node, Node::Sphere { radius: 5.0 });
    }
}
