use std::time::{Duration, Instant};

use crate::{
    draw_triangles::draw_triangles,
    marching_cubes::by_marching_cubes,
    surface_nets::by_surface_nets,
    utils::{
        CartesianCoordinate, NearPointAlgorithm, OutputType, ProcessedPixels, TextCoords, Vec3,
    },
};
use image::Rgb;
use kd_tree::KdTree;
use nalgebra::{Const, Dyn, Matrix, VecStorage, SVD};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use std::ops::{Add, Mul};
pub struct ProcessedNearestPoint {
    pub text_coords: TextCoords,
    pub rgb: [u8; 3],
    pub tree_search_elapsed: Duration,
    pub plane_fit_elapsed: Duration,
}

pub enum OutputGenerator {
    MarchingCubes,
    SurfaceNets,
}

const GENERATOR: OutputGenerator = OutputGenerator::SurfaceNets;
const VOXEL_SIZE_MM: usize = 25;

pub fn by_nearest_points(
    coordinates: &Vec<CartesianCoordinate>,
    k_nearest: usize,
    algorithm: NearPointAlgorithm,
    color: OutputType,
) -> ProcessedPixels {
    let num_points = coordinates.len() as u32;
    let time_total_calc = Instant::now();
    let time_start = Instant::now();
    let tree = KdTree::par_build_by_ordered_float(coordinates.clone());

    let (triangles, dimensions_mm) = match GENERATOR {
        OutputGenerator::MarchingCubes => {
            by_marching_cubes(coordinates, &tree, VOXEL_SIZE_MM, true)
        }
        OutputGenerator::SurfaceNets => by_surface_nets(coordinates, &tree, VOXEL_SIZE_MM, 6),
    };

    let max_dist = usize::max(
        dimensions_mm.0,
        usize::max(dimensions_mm.1, dimensions_mm.2),
    ) as f32
        * 0.95;

    println!("Drawing triangles with max light dist {:?}", max_dist);
    draw_triangles(triangles, max_dist);

    return Vec::new();

    let elapsed_tree_constr = time_start.elapsed();
    println!("KD tree construction took {:.2?}", elapsed_tree_constr);

    println!(
        "Starting normal calculation using {:} nearest neighbors",
        k_nearest
    );

    let results = coordinates
        .par_iter()
        .map(|point| process_point(&point, k_nearest, &tree, algorithm, color))
        .collect::<Vec<ProcessedNearestPoint>>();

    let mut elapsed_tree_search = Duration::from_secs(0);
    let mut elapsed_plane_fit = Duration::from_secs(0);

    let result_pixels = results
        .iter()
        .map(|result_px| {
            elapsed_tree_search += result_px.tree_search_elapsed;
            elapsed_plane_fit += result_px.plane_fit_elapsed;
            (
                result_px.text_coords.x,
                result_px.text_coords.y,
                Rgb::<u8>(result_px.rgb),
            )
        })
        .collect::<Vec<_>>();

    elapsed_tree_search /= num_points;
    elapsed_plane_fit /= num_points;
    let elapsed_calc = time_total_calc.elapsed();

    println!(
        "Total calculation took {:.2?} with {:} nearest neighbors",
        elapsed_calc, k_nearest
    );
    println!(
        "Average time per pixel for tree search: {:.2?}",
        elapsed_tree_search
    );
    println!(
        "Average time per pixel for plane fit: {:.2?}",
        elapsed_plane_fit
    );

    result_pixels
}

fn process_point(
    point: &CartesianCoordinate,
    k: usize,
    tree: &KdTree<CartesianCoordinate>,
    algorithm: NearPointAlgorithm,
    desired_output: OutputType,
) -> ProcessedNearestPoint {
    let tree_search_start = Instant::now();

    let k_nearest = match algorithm {
        NearPointAlgorithm::KNearest => tree
            .nearests(point, k)
            .iter()
            .map(|x| x.item)
            .collect::<Vec<&CartesianCoordinate>>(),
        NearPointAlgorithm::WithinRadius => {
            let mut radius = k as f32;
            let mut points = tree.within_radius(point, radius);
            while points.len() < 10 {
                radius *= 2.;
                points = tree.within_radius(point, radius);
            }
            points
        }
    };

    let tree_search_elapsed = tree_search_start.elapsed();
    let num_points_sampled = k_nearest.len();

    let plane_fit_start = Instant::now();
    let mut sum = Vec3::new(0., 0., 0.);

    for near_point in &k_nearest {
        sum = sum + near_point.vec_coord;
    }

    // Calculate normal of the plane using SVD
    let centroid = sum / num_points_sampled as f32;

    // store the nearest points in a 3 x N matrix where N = number of points sampled
    // and each column is a point
    let matrix = Matrix::<f32, Const<3>, Dyn, VecStorage<f32, Const<3>, Dyn>>::from_fn(
        num_points_sampled,
        |row, col| {
            if col >= k_nearest.len() {
                panic!("Invalid column");
            }
            let point = k_nearest[col];
            match row {
                0 => point.vec_coord.x - centroid.x,
                1 => point.vec_coord.y - centroid.y,
                2 => point.vec_coord.z - centroid.z,
                _ => panic!("Invalid row"),
            }
        },
    );

    let svd = SVD::new(matrix, true, false);

    let plane_fit_elapsed = plane_fit_start.elapsed();

    let left_singular = svd.u.unwrap();

    let normal_col = left_singular.column(2);

    let mut normal_full_vec = normal_col.normalize();

    if normal_full_vec.dot(&point.vec_coord) < 0. {
        // If normal is oriented away from the camera, flip it so it's facing the camera
        normal_full_vec *= -1.;
    }

    const LIGHT_POS: Vec3 = Vec3::new(0., -2500., -100.);

    let light_dir = point.vec_coord - LIGHT_POS;

    let dist_to_light = (light_dir.magnitude() / (256. * 25.)).min(1.);

    let light_level = normal_full_vec.dot(&light_dir.normalize()).max(0.) * (1. - dist_to_light);

    let shaded_rgb = [
        (light_level * 255.) as u8,
        (light_level * 255.) as u8,
        (light_level * 255.) as u8,
    ];

    let normal_abs_vec = normal_col.abs().normalize();

    let normal_abs_rgb = [
        (normal_abs_vec.x * 255.) as u8,
        (normal_abs_vec.y * 255.) as u8,
        (normal_abs_vec.z * 255.) as u8,
    ];

    let normal_col = normal_full_vec.mul(-0.5).add(Vec3::new(0.5, 0.5, 0.5));

    let normal_full_rgb = [
        (normal_col.x * 255.) as u8,
        (normal_col.y * 255.) as u8,
        (normal_col.z * 255.) as u8,
    ];

    let text_coords = point.from_text;
    return ProcessedNearestPoint {
        text_coords,
        rgb: match desired_output {
            OutputType::Shaded => shaded_rgb,
            OutputType::Normals => normal_abs_rgb,
            OutputType::NormalsFull => normal_full_rgb,
        },
        tree_search_elapsed,
        plane_fit_elapsed,
    };
}
