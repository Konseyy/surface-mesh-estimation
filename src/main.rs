use clap::Parser;
use image::{self};
use std::path::PathBuf;

mod draw_triangles;
mod marching_cubes;
mod process_points;
mod surface_nets;
mod utils;

use process_points::process_points;
use utils::{spherical_to_cartesian, text_coords_to_spherical, Algorithm, CartesianCoordinate};

// CLI Parameters
// -----------
#[derive(Parser, Debug)]
#[command( version, about="This is a CLI tool for generating normal maps, given an equirectangular format depth map", long_about = None )]
struct Args {
    // Input depth map file path
    #[arg(name = "Input file path (depth map)", short = 'i', long = "input")]
    input_path: PathBuf,
    // Algorithm used for surface reconstruction
    #[arg(
        name = "Surface construction algorithm",
        short = 'a',
        long = "algorithm"
    )]
    algorithm: Algorithm,
    // Input depth map file path
    #[arg(
        name = "Voxel size",
        short = 'v',
        long = "voxel_size",
        default_value = "25"
    )]
    voxel_size: usize,
    // Amount of smoothing steps used for surface nets algorithm
    #[arg(
        name = "Smoothing passes",
        short = 's',
        long = "smoothing_passes",
        default_value = "2"
    )]
    smoothing_steps: usize,
}

fn main() {
    let args = Args::parse();
    // check if output directory exists?
    let img = image::open(args.input_path)
        .expect("File not found!")
        .into_luma16();

    let (img_width, img_height) = img.dimensions();

    let num_points = img_width * img_height;

    let mut coordinates = Vec::<CartesianCoordinate>::with_capacity((num_points) as usize);

    for (x, y, pixel) in img.enumerate_pixels() {
        let depth = pixel.0[0] as f32;

        let spherical = text_coords_to_spherical(x, y, (img_width, img_height), depth);
        let cartesian = spherical_to_cartesian(&spherical);

        coordinates.push(cartesian);
    }

    process_points(
        &coordinates,
        args.algorithm,
        args.voxel_size,
        args.smoothing_steps,
    );
}
