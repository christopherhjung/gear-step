//! Parametric involute gear generator: geometry, STEP AP214 export, SVG dump
//! and a display mesh. No dependencies.

pub mod api;
pub mod brep;
pub mod earcut;
pub mod gear;
pub mod keyway;
pub mod mesh;
pub mod nurbs;
pub mod profile;
pub mod svg;

#[cfg(target_arch = "wasm32")]
pub mod wasm;

pub use api::{build, Built, Key, Row, Spec};
