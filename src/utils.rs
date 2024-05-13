use clap::ValueEnum;
use core::panic;
use image::Rgb;
use kd_tree::KdPoint;
use nalgebra::SVector;
use std::f32::consts::PI;

pub type Vec3 = SVector<f32, 3>;

pub type ProcessedPixels = Vec<(u32, u32, Rgb<u8>)>;

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

#[derive(Copy, Clone, Debug)]
pub enum OutputType {
    Shaded,
    Normals,
    NormalsFull,
}
impl ValueEnum for OutputType {
    fn from_str(input: &str, _ignore_case: bool) -> Result<Self, String> {
        match input {
            "shaded" => Ok(OutputType::Shaded),
            "normals" => Ok(OutputType::Normals),
            _ => Err(format!("Invalid output color: {}", input)),
        }
    }
    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        match self {
            OutputType::Shaded => Some(clap::builder::PossibleValue::new("shaded")),
            OutputType::Normals => Some(clap::builder::PossibleValue::new("normals")),
            OutputType::NormalsFull => Some(clap::builder::PossibleValue::new("normals_full")),
        }
    }
    fn value_variants<'a>() -> &'a [Self] {
        &[
            OutputType::Shaded,
            OutputType::Normals,
            OutputType::NormalsFull,
        ]
    }
}
#[derive(Copy, Clone, Debug)]
pub enum NearPointAlgorithm {
    KNearest,
    WithinRadius,
}
impl ValueEnum for NearPointAlgorithm {
    fn from_str(input: &str, _ignore_case: bool) -> Result<Self, String> {
        match input {
            "k_nearest" => Ok(NearPointAlgorithm::KNearest),
            "within_radius" => Ok(NearPointAlgorithm::WithinRadius),
            _ => Err(format!("Invalid algorithm: {}", input)),
        }
    }
    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        match self {
            NearPointAlgorithm::KNearest => Some(clap::builder::PossibleValue::new("k_nearest")),
            NearPointAlgorithm::WithinRadius => {
                Some(clap::builder::PossibleValue::new("within_radius"))
            }
        }
    }
    fn value_variants<'a>() -> &'a [Self] {
        &[
            NearPointAlgorithm::KNearest,
            NearPointAlgorithm::WithinRadius,
        ]
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
