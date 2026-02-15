/// A node in the OpenSCAD geometry tree.
#[derive(Debug, Clone, PartialEq)]
pub enum Node {
    // 3D Primitives
    Cube {
        size: [f32; 3],
        center: bool,
    },
    Sphere {
        radius: f32,
    },
    Cylinder {
        h: f32,
        r1: f32,
        r2: f32,
        center: bool,
    },
    Polyhedron {
        points: Vec<[f32; 3]>,
        faces: Vec<Vec<usize>>,
    },

    // 2D Primitives
    Circle {
        radius: f32,
    },
    Square {
        size: [f32; 2],
        center: bool,
    },
    Polygon {
        points: Vec<[f32; 2]>,
        paths: Option<Vec<Vec<usize>>>,
    },

    // Transforms
    Translate {
        offset: [f32; 3],
        child: Box<Node>,
    },
    RotateEuler {
        angles: [f32; 3],
        child: Box<Node>,
    },
    RotateAxisAngle {
        axis: [f32; 3],
        angle: f32,
        child: Box<Node>,
    },
    Scale {
        factor: [f32; 3],
        child: Box<Node>,
    },
    Mirror {
        axes: [f32; 3],
        child: Box<Node>,
    },
    Color {
        rgba: [f32; 4],
        child: Box<Node>,
    },

    // Extrusions
    LinearExtrude {
        height: f32,
        center: bool,
        twist: f32,
        slices: Option<u32>,
        child: Box<Node>,
    },
    RotateExtrude {
        angle: f32,
        child: Box<Node>,
    },

    // CSG operations
    Union(Vec<Node>),
    Difference(Vec<Node>),
    Intersection(Vec<Node>),
    Hull(Vec<Node>),
    Minkowski(Vec<Node>),

    // Non-standard (spheres extension)
    Repeat {
        spacing: [f32; 3],
        copies: [u32; 3],
        child: Box<Node>,
    },
}
