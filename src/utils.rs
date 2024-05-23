use clap::ValueEnum;
use core::panic;
use kd_tree::KdPoint;
use nalgebra::SVector;
use std::f32::consts::PI;

pub type Vec3 = SVector<f32, 3>;

#[derive(Copy, Clone, Debug)]
pub enum Algorithm {
    MarchingCubes,
    SurfaceNets,
}

#[derive(Copy, Clone)]
pub struct TextCoords {
    pub x: u32,
    pub y: u32,
}

pub struct SphericalCoordinate {
    pub theta: f32,
    pub phi: f32,
    pub r: f32,
    pub from_text: TextCoords,
}

#[derive(Copy, Clone)]
pub struct CartesianCoordinate {
    pub vec_coord: Vec3,
    pub from_text: TextCoords,
}

impl KdPoint for CartesianCoordinate {
    type Scalar = f32;
    type Dim = typenum::U3;
    fn at(&self, k: usize) -> f32 {
        match k {
            0 => self.vec_coord.x,
            1 => self.vec_coord.y,
            2 => self.vec_coord.z,
            _ => panic!("Invalid dimension"),
        }
    }
}

#[derive(Default, Copy, Clone, Debug)]
pub struct Triangle {
    pub vertices: [[f32; 3]; 3],
    pub normal: [f32; 3],
}

impl ValueEnum for Algorithm {
    fn from_str(input: &str, _ignore_case: bool) -> Result<Self, String> {
        match input {
            "marching_cubes" => Ok(Algorithm::MarchingCubes),
            "surface_nets" => Ok(Algorithm::SurfaceNets),
            _ => Err(format!("Invalid output color: {}", input)),
        }
    }
    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        match self {
            Algorithm::MarchingCubes => Some(clap::builder::PossibleValue::new("marching_cubes")),
            Algorithm::SurfaceNets => Some(clap::builder::PossibleValue::new("surface_nets")),
        }
    }
    fn value_variants<'a>() -> &'a [Self] {
        &[Algorithm::MarchingCubes, Algorithm::SurfaceNets]
    }
}

pub fn text_coords_to_spherical(
    x: u32,
    y: u32,
    dimensions: (u32, u32),
    depth: f32,
) -> SphericalCoordinate {
    // Get percentages
    let perc_x = x as f32 / dimensions.0 as f32;
    let perc_y = y as f32 / dimensions.1 as f32;
    // Convert to spherical coordinates
    return SphericalCoordinate {
        theta: perc_x * 2. * PI,
        phi: perc_y * PI,
        r: depth,
        from_text: TextCoords { x, y },
    };
}

pub fn spherical_to_cartesian(sph: &SphericalCoordinate) -> CartesianCoordinate {
    let vec = Vec3::new(
        sph.r * sph.phi.sin() * sph.theta.cos(),
        sph.r * sph.phi.sin() * sph.theta.sin(),
        sph.r * sph.phi.cos(),
    );
    return CartesianCoordinate {
        vec_coord: vec,
        from_text: sph.from_text,
    };
}
