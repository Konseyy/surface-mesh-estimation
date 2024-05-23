use std::time::Duration;

use crate::{
    draw_triangles::draw_triangles,
    marching_cubes::by_marching_cubes,
    surface_nets::by_surface_nets,
    utils::{Algorithm, CartesianCoordinate, TextCoords},
};
use kd_tree::KdTree;
pub struct ProcessedNearestPoint {
    pub text_coords: TextCoords,
    pub rgb: [u8; 3],
    pub tree_search_elapsed: Duration,
    pub plane_fit_elapsed: Duration,
}

pub fn process_points(
    coordinates: &Vec<CartesianCoordinate>,
    algorithm: Algorithm,
    voxel_size: usize,
    smoothing_passes: usize,
) {
    let tree = KdTree::par_build_by_ordered_float(coordinates.clone());

    let (triangles, dimensions_mm) = match algorithm {
        Algorithm::MarchingCubes => by_marching_cubes(coordinates, &tree, voxel_size, true),
        Algorithm::SurfaceNets => by_surface_nets(coordinates, &tree, voxel_size, smoothing_passes),
    };

    let max_dist = usize::max(
        dimensions_mm.0,
        usize::max(dimensions_mm.1, dimensions_mm.2),
    ) as f32
        * 1.2;

    println!("Drawing triangles with max light dist {:?}", max_dist);
    draw_triangles(triangles, max_dist);

    return;
}
