pub mod adaptive;
pub mod bcc;
pub mod builder;
pub mod cache;
pub mod decimation;
pub mod marching;
pub mod octree;
pub mod tetrahedra;

pub use bcc::{generate_bcc_mesh, BccMeshParams};
pub use builder::MeshBuilder;
pub use decimation::{collapse_short_edges, decimate_flat};

#[cfg(test)]
#[path = "tests/fuzz_csg_test.rs"]
mod fuzz_csg_test;