pub mod debug_image;
pub mod math;
pub mod mesh;
pub mod scad;
pub mod scad_parser;
pub mod solid;
pub mod viewer;

pub use debug_image::{write_sdf_cross_section, write_sdf_image};
pub use math::Vec3;
pub use mesh::cache::SdfCache;
pub use mesh::octree::initial_step;
pub use mesh::{collapse_short_edges, decimate_flat, generate_bcc_mesh, BccMeshParams, MeshBuilder};
pub use solid::{
    Capsule, Cube, Difference, InfiniteCone, InfiniteCylinder, Intersection, Mirror, Plane, Repeat,
    Rotate, RoundedBox, Scale, SmoothDifference, SmoothIntersection, SmoothUnion, Solid, Sphere,
    Torus, Translate, Union, Vec3x8,
};
