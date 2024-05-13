use clap::Parser;
use image::codecs::jpeg::JpegEncoder;
use image::{self, ImageEncoder};
use std::path::PathBuf;

mod marching_cubes;
mod nearest_points;
mod utils;

use nearest_points::by_nearest_points;
use utils::{
    spherical_to_cartesian, text_coords_to_spherical, CartesianCoordinate, NearPointAlgorithm,
    OutputType,
};

// CLI Parameters
// -----------
#[derive(Parser, Debug)]
#[command( version, about="This is a CLI tool for generating normal maps, given an equirectangular format depth map", long_about = None )]
struct Args {
    // Input depth map file path
    #[arg(name = "Input file path (depth map)", short = 'i', long = "input")]
    input_path: PathBuf,
    // Output normal map file path
    #[arg(name = "Output file path (normal map)", short = 'o', long = "output")]
    output_path: PathBuf,
    // How many nearest neighbors to consider
    #[arg(
        name = "Num of nearest neighbors to consider",
        short = 'k',
        long = "k-nearest",
        default_value_t = 80usize
    )]
    k_nearest: usize,
    // Whether to output normals or absolute normals
    #[arg(
        name = "Output color",
        short = 'c',
        long = "color",
        default_value = "shaded"
    )]
    output_color: OutputType,
    #[arg(
        name = "Nearest point algorithm",
        short = 'a',
        long = "nearest-algorithm",
        default_value = "within_radius"
    )]
    algorithm: NearPointAlgorithm,
}

fn main() {
    let args = Args::parse();
    if args.k_nearest <= 0 {
        panic!("Number of nearest neighbors must be greater than 0");
    }
    // check if output directory exists?
    let img = image::open(args.input_path)
        .expect("File not found!")
        .into_luma16();

    let (img_width, img_height) = img.dimensions();

    let mut new_img = image::ImageBuffer::<image::Rgb<u8>, _>::new(img_width, img_height);

    let num_points = img_width * img_height;

    let mut coordinates = Vec::<CartesianCoordinate>::with_capacity((num_points) as usize);

    for (x, y, pixel) in img.enumerate_pixels() {
        let depth = pixel.0[0] as f32;

        let spherical = text_coords_to_spherical(x, y, (img_width, img_height), depth);
        let cartesian = spherical_to_cartesian(&spherical);

        coordinates.push(cartesian);
    }

    let res_nearest = by_nearest_points(
        &coordinates,
        args.k_nearest,
        args.algorithm,
        args.output_color,
    );

    for (x, y, rgb) in res_nearest {
        new_img.put_pixel(x, y, rgb);
    }

    let writer = std::fs::File::create(&args.output_path).expect("Failed to create file");
    let encoder = JpegEncoder::new_with_quality(writer, 100);

    encoder
        .write_image(&new_img, img_width, img_height, image::ColorType::Rgb8)
        .expect("Error writing image");

    println!(
        "File has been saved to {}",
        args.output_path.to_str().unwrap()
    );
}
