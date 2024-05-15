use kd_tree::KdTree;
use rayon::{
    iter::{IntoParallelRefIterator, ParallelIterator},
    prelude::*,
};

use crate::utils::{CartesianCoordinate, TextCoords, Vec3};

#[derive(Default, Copy, Clone, Debug)]
pub struct Triangle {
    pub vertices: [[f32; 3]; 3],
    pub normal: [f32; 3],
}

pub fn by_marching_cubes(
    coordinates: &Vec<CartesianCoordinate>,
    tree: &KdTree<CartesianCoordinate>,
    voxel_size: usize,
    use_interp: bool,
) -> Vec<Triangle> {
    // find the furthest point in each cartesian dimension
    let max_x = coordinates
        .iter()
        .map(|c| c.vec_coord.x)
        .fold(coordinates[0].vec_coord.x, |x1, x2| {
            f32::max(f32::abs(x1), f32::abs(x2))
        });

    let max_y = coordinates
        .iter()
        .map(|c| c.vec_coord.y)
        .fold(coordinates[0].vec_coord.y, |y1, y2| {
            f32::max(f32::abs(y1), f32::abs(y2))
        });

    let max_z = coordinates
        .iter()
        .map(|c| c.vec_coord.z)
        .fold(coordinates[0].vec_coord.z, |z1, z2| {
            f32::max(f32::abs(z1), f32::abs(z2))
        });

    // this will form the dimensions of the grid
    let max_mm_dimensions = (
        max_x.ceil() as usize,
        max_y.ceil() as usize,
        max_z.ceil() as usize,
    );

    let min_x = coordinates
        .iter()
        .map(|c| c.vec_coord.x)
        .fold(coordinates[0].vec_coord.x, |x1, x2| f32::min(x1, x2));

    let min_y = coordinates
        .iter()
        .map(|c| c.vec_coord.y)
        .fold(coordinates[0].vec_coord.y, |y1, y2| f32::min(y1, y2));

    let min_z = coordinates
        .iter()
        .map(|c| c.vec_coord.z)
        .fold(coordinates[0].vec_coord.z, |z1, z2| f32::min(z1, z2));

    let min_mm_dimensions = (min_x as i32, min_y as i32, min_z as i32);

    println!("Min dimensions: {:?}", min_mm_dimensions);

    println!("Max dimensions: {:?}", max_mm_dimensions);

    // mm

    // * 2 because we want to have the grid centered around the origin
    let grid_dimensions = (
        ((max_mm_dimensions.0 * 2) as f32 / voxel_size as f32).ceil() as usize,
        ((max_mm_dimensions.1 * 2) as f32 / voxel_size as f32).ceil() as usize,
        ((max_mm_dimensions.2 * 2) as f32 / voxel_size as f32).ceil() as usize,
    );

    println!("Grid dimensions: {:?}", grid_dimensions);

    // All x,y,z coordinate combinations in the grid
    let mut grid_coordinates: Vec<(usize, usize, usize)> =
        Vec::with_capacity((grid_dimensions.0 * grid_dimensions.1 * grid_dimensions.2) as usize);

    for x in 0..grid_dimensions.0 {
        for y in 0..grid_dimensions.1 {
            for z in 0..grid_dimensions.2 {
                grid_coordinates.push((x, y, z));
            }
        }
    }

    println!("Grid coordinates: {:?}", grid_coordinates.len());

    let mut vertex_densities = grid_coordinates
        .par_iter()
        .map(|(x_grid, y_grid, z_grid)| {
            // iterate through the whole cartesian space and create a density score for each vertex
            //  based on how many LiDAR data points are in the sphere around it
            let grid_point = Vec3::new(
                // Get the appropriate cartesian coordinate for the grid point
                ((x_grid * voxel_size) as i64 - (max_mm_dimensions.0) as i64) as f32,
                ((y_grid * voxel_size) as i64 - (max_mm_dimensions.1) as i64) as f32,
                ((z_grid * voxel_size) as i64 - (max_mm_dimensions.2) as i64) as f32,
            );
            if *x_grid == 0 && *y_grid == 0 && *z_grid == 0 {
                println!(
                    "Grid point start: {:?} {:?} {:?} ",
                    grid_point.x, grid_point.y, grid_point.z
                );
            } else if *x_grid == grid_dimensions.0 - 1
                && *y_grid == grid_dimensions.1 - 1
                && *z_grid == grid_dimensions.2 - 1
            {
                println!(
                    "Grid point end: {:?} {:?} {:?} ",
                    grid_point.x, grid_point.y, grid_point.z
                );
            }
            let density = tree.within_radius(
                &CartesianCoordinate {
                    vec_coord: grid_point,
                    from_text: TextCoords { x: 0, y: 0 },
                },
                (voxel_size) as f32,
            );

            return ((x_grid, y_grid, z_grid), density.len());
        })
        .collect::<Vec<((&usize, &usize, &usize), usize)>>();

    // Sort the results by the grid coordinates
    vertex_densities.par_sort_by(|a, b| {
        let idx_1 = get_grid_index(grid_dimensions, *a.0 .0, *a.0 .1, *a.0 .2);
        let idx_2 = get_grid_index(grid_dimensions, *b.0 .0, *b.0 .1, *b.0 .2);
        return idx_1.cmp(&idx_2);
    });
    // vertex_densities.sort_by(|a, b| {
    //     let idx_1 = get_grid_index(grid_dimensions, *a.0 .0, *a.0 .1, *a.0 .2);
    //     let idx_2 = get_grid_index(grid_dimensions, *b.0 .0, *b.0 .1, *b.0 .2);
    //     return idx_1.cmp(&idx_2);
    // });

    let max_dens = vertex_densities
        .iter()
        .map(|(_, d)| d)
        .fold(0, |d1, d2| i32::max(d1, *d2 as i32));

    println!("Max density: {:?}", max_dens);

    let avg_dens = vertex_densities
        .iter()
        .map(|(_, d)| d)
        .fold(0, |d1, d2| d1 + *d2 as i32) as f32
        / vertex_densities.len() as f32;

    println!("Average density: {:?}", avg_dens);

    // Densities below this are not considered solid points
    // let density_threshold = avg_dens / 2.;
    let density_threshold = 0.;

    let count_over_threshold = vertex_densities
        .par_iter()
        .filter(|(_, d)| *d as f32 > density_threshold)
        .count();

    println!("Count over threshold: {:?}", count_over_threshold);

    let mut triangles: Vec<Triangle> = Vec::with_capacity(coordinates.len());

    let mut printed = false;
    for x in 0..(grid_dimensions.0 - 1) {
        for y in 0..(grid_dimensions.1 - 1) {
            for z in 0..(grid_dimensions.2 - 1) {
                let idx_0 = get_grid_index(grid_dimensions, x, y, z);
                let idx_1 = get_grid_index(grid_dimensions, x + 1, y, z);
                let idx_2 = get_grid_index(grid_dimensions, x, y, z + 1);
                let idx_3 = get_grid_index(grid_dimensions, x + 1, y, z + 1);
                let idx_4 = get_grid_index(grid_dimensions, x, y + 1, z);
                let idx_5 = get_grid_index(grid_dimensions, x + 1, y + 1, z);
                let idx_6 = get_grid_index(grid_dimensions, x, y + 1, z + 1);
                let idx_7 = get_grid_index(grid_dimensions, x + 1, y + 1, z + 1);

                let vertices: [((&usize, &usize, &usize), usize); 8] = [
                    vertex_densities[idx_0],
                    vertex_densities[idx_1],
                    vertex_densities[idx_2],
                    vertex_densities[idx_3],
                    vertex_densities[idx_4],
                    vertex_densities[idx_5],
                    vertex_densities[idx_6],
                    vertex_densities[idx_7],
                ];

                let mask_idx = vertices.iter().enumerate().fold(0, |acc, (idx, d)| {
                    acc | ((d.1 as f32 > density_threshold) as usize) << idx
                });

                if EDGE_MASKS[mask_idx] == 0 {
                    continue;
                }

                let mut vert_list: [Vec3; 12] = Default::default();

                if EDGE_MASKS[mask_idx] & 0b00000001 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[0][0];
                    let edge_end = EDGE_VERTEX_IDX[0][1];
                    vert_list[0] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b00000010 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[1][0];
                    let edge_end = EDGE_VERTEX_IDX[1][1];
                    vert_list[1] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b000000000100 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[2][0];
                    let edge_end = EDGE_VERTEX_IDX[2][1];
                    vert_list[2] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b000000001000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[3][0];
                    let edge_end = EDGE_VERTEX_IDX[3][1];
                    vert_list[3] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b000000010000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[4][0];
                    let edge_end = EDGE_VERTEX_IDX[4][1];
                    vert_list[4] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b000000100000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[5][0];
                    let edge_end = EDGE_VERTEX_IDX[5][1];
                    vert_list[5] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b000001000000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[6][0];
                    let edge_end = EDGE_VERTEX_IDX[6][1];
                    vert_list[6] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b000010000000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[7][0];
                    let edge_end = EDGE_VERTEX_IDX[7][1];
                    vert_list[7] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b000100000000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[8][0];
                    let edge_end = EDGE_VERTEX_IDX[8][1];
                    vert_list[8] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b001000000000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[9][0];
                    let edge_end = EDGE_VERTEX_IDX[9][1];
                    vert_list[9] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b010000000000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[10][0];
                    let edge_end = EDGE_VERTEX_IDX[10][1];
                    vert_list[10] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }
                if EDGE_MASKS[mask_idx] & 0b100000000000 > 0 {
                    let edge_start = EDGE_VERTEX_IDX[11][0];
                    let edge_end = EDGE_VERTEX_IDX[11][1];
                    vert_list[11] = interp_vert_pos(
                        vertices[edge_start].0,
                        vertices[edge_end].0,
                        vertices[edge_start].1,
                        vertices[edge_end].1,
                        use_interp,
                    );
                }

                for i in 0..12 {
                    if TRI_TABLE[mask_idx][i] == -1
                        || TRI_TABLE[mask_idx][i + 1] == -1
                        || TRI_TABLE[mask_idx][i + 2] == -1
                    {
                        break;
                    }

                    let vert1_grid = vert_list[TRI_TABLE[mask_idx][i] as usize];
                    let vert2_grid = vert_list[TRI_TABLE[mask_idx][i + 1] as usize];
                    let vert3_grid = vert_list[TRI_TABLE[mask_idx][i + 2] as usize];

                    let dimensions_real = Vec3::new(
                        (max_mm_dimensions.0) as f32,
                        (max_mm_dimensions.1) as f32,
                        (max_mm_dimensions.2) as f32,
                    );

                    let vert1_real = vert1_grid * voxel_size as f32 - dimensions_real;
                    let vert2_real = vert2_grid * voxel_size as f32 - dimensions_real;
                    let vert3_real = vert3_grid * voxel_size as f32 - dimensions_real;

                    let a = vert2_real - vert1_real;
                    let b = vert3_real - vert1_real;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    let tri = Triangle {
                        vertices: [
                            [vert1_real.x, vert1_real.y, vert1_real.z],
                            [vert2_real.x, vert2_real.y, vert2_real.z],
                            [vert3_real.x, vert3_real.y, vert3_real.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    };
                    triangles.push(tri);

                    if !printed && (vertices[0].1 as f32 > avg_dens + density_threshold) {
                        println!("Mask index: {:?}", mask_idx);
                        println!(
                            "Vertices: {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
                            vertices[0],
                            vertices[1],
                            vertices[2],
                            vertices[3],
                            vertices[4],
                            vertices[5],
                            vertices[6],
                            vertices[7]
                        );
                        println!("Triangle: {:?}", tri);
                        printed = true;
                    }
                }
            }
        }
    }

    println!("Triangles: {:?}", triangles.len());

    return triangles;
}

fn interp_vert_pos(
    p1: (&usize, &usize, &usize),
    p2: (&usize, &usize, &usize),
    d1: usize,
    d2: usize,
    use_interp: bool,
) -> Vec3 {
    let p1 = Vec3::new(*p1.0 as f32, *p1.1 as f32, *p1.2 as f32);
    let p2 = Vec3::new(*p2.0 as f32, *p2.1 as f32, *p2.2 as f32);

    if !use_interp {
        let avg = (p1 + p2) / 2.0;
        return avg;
    }

    let t = (d1 as f32) / (d1 as f32 + d2 as f32);
    let interp = p1 + (p2 - p1) * t;
    return interp;
}

// Given the grid size and the elements coordinates, return the index of the element in the grid array
fn get_grid_index(grid_dimensions: (usize, usize, usize), x: usize, y: usize, z: usize) -> usize {
    return x + y * grid_dimensions.0 + z * grid_dimensions.0 * grid_dimensions.1;
}

// Pair of vertex indices for each edge on the cube
const EDGE_VERTEX_IDX: [[usize; 2]; 12] = [
    [0, 1],
    [1, 3],
    [3, 2],
    [2, 0],
    [4, 5],
    [5, 7],
    [7, 6],
    [6, 4],
    [0, 4],
    [1, 5],
    [3, 7],
    [2, 6],
];

// For each MC case, a mask of edge indices that need to be split
const EDGE_MASKS: [usize; 256] = [
    0x0, 0x109, 0x203, 0x30a, 0x80c, 0x905, 0xa0f, 0xb06, 0x406, 0x50f, 0x605, 0x70c, 0xc0a, 0xd03,
    0xe09, 0xf00, 0x190, 0x99, 0x393, 0x29a, 0x99c, 0x895, 0xb9f, 0xa96, 0x596, 0x49f, 0x795,
    0x69c, 0xd9a, 0xc93, 0xf99, 0xe90, 0x230, 0x339, 0x33, 0x13a, 0xa3c, 0xb35, 0x83f, 0x936,
    0x636, 0x73f, 0x435, 0x53c, 0xe3a, 0xf33, 0xc39, 0xd30, 0x3a0, 0x2a9, 0x1a3, 0xaa, 0xbac,
    0xaa5, 0x9af, 0x8a6, 0x7a6, 0x6af, 0x5a5, 0x4ac, 0xfaa, 0xea3, 0xda9, 0xca0, 0x8c0, 0x9c9,
    0xac3, 0xbca, 0xcc, 0x1c5, 0x2cf, 0x3c6, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0x4ca, 0x5c3, 0x6c9,
    0x7c0, 0x950, 0x859, 0xb53, 0xa5a, 0x15c, 0x55, 0x35f, 0x256, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x55a, 0x453, 0x759, 0x650, 0xaf0, 0xbf9, 0x8f3, 0x9fa, 0x2fc, 0x3f5, 0xff, 0x1f6, 0xef6,
    0xfff, 0xcf5, 0xdfc, 0x6fa, 0x7f3, 0x4f9, 0x5f0, 0xb60, 0xa69, 0x963, 0x86a, 0x36c, 0x265,
    0x16f, 0x66, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x76a, 0x663, 0x569, 0x460, 0x460, 0x569, 0x663,
    0x76a, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x66, 0x16f, 0x265, 0x36c, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0xdfc, 0xcf5, 0xfff, 0xef6, 0x1f6, 0xff, 0x3f5, 0x2fc, 0x9fa,
    0x8f3, 0xbf9, 0xaf0, 0x650, 0x759, 0x453, 0x55a, 0xe5c, 0xf55, 0xc5f, 0xd56, 0x256, 0x35f,
    0x55, 0x15c, 0xa5a, 0xb53, 0x859, 0x950, 0x7c0, 0x6c9, 0x5c3, 0x4ca, 0xfcc, 0xec5, 0xdcf,
    0xcc6, 0x3c6, 0x2cf, 0x1c5, 0xcc, 0xbca, 0xac3, 0x9c9, 0x8c0, 0xca0, 0xda9, 0xea3, 0xfaa,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0x8a6, 0x9af, 0xaa5, 0xbac, 0xaa, 0x1a3, 0x2a9, 0x3a0, 0xd30,
    0xc39, 0xf33, 0xe3a, 0x53c, 0x435, 0x73f, 0x636, 0x936, 0x83f, 0xb35, 0xa3c, 0x13a, 0x33,
    0x339, 0x230, 0xe90, 0xf99, 0xc93, 0xd9a, 0x69c, 0x795, 0x49f, 0x596, 0xa96, 0xb9f, 0x895,
    0x99c, 0x29a, 0x393, 0x99, 0x190, 0xf00, 0xe09, 0xd03, 0xc0a, 0x70c, 0x605, 0x50f, 0x406,
    0xb06, 0xa0f, 0x905, 0x80c, 0x30a, 0x203, 0x109, 0x0,
];

const TRI_TABLE: [[i8; 16]; 256] = [
    [
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    ],
    [0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 8, 1, 1, 8, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 0, 11, 11, 0, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 2, 11, 1, 0, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 1, 2, 11, 9, 1, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1],
    [1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 8, 2, 1, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 2, 9, 9, 2, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 2, 3, 8, 10, 2, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1],
    [11, 3, 10, 10, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 0, 1, 10, 8, 0, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1],
    [9, 3, 0, 9, 11, 3, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1],
    [8, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 4, 3, 3, 4, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 8, 7, 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 4, 9, 1, 7, 4, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1],
    [8, 7, 4, 11, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 11, 7, 4, 2, 11, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1],
    [0, 9, 1, 8, 7, 4, 11, 3, 2, -1, -1, -1, -1, -1, -1, -1],
    [7, 4, 11, 11, 4, 2, 2, 4, 9, 2, 9, 1, -1, -1, -1, -1],
    [4, 8, 7, 2, 1, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 4, 3, 3, 4, 0, 10, 2, 1, -1, -1, -1, -1, -1, -1, -1],
    [10, 2, 9, 9, 2, 0, 7, 4, 8, -1, -1, -1, -1, -1, -1, -1],
    [10, 2, 3, 10, 3, 4, 3, 7, 4, 9, 10, 4, -1, -1, -1, -1],
    [1, 10, 3, 3, 10, 11, 4, 8, 7, -1, -1, -1, -1, -1, -1, -1],
    [10, 11, 1, 11, 7, 4, 1, 11, 4, 1, 4, 0, -1, -1, -1, -1],
    [7, 4, 8, 9, 3, 0, 9, 11, 3, 9, 10, 11, -1, -1, -1, -1],
    [7, 4, 11, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1],
    [9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 4, 5, 8, 0, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 5, 0, 0, 5, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [5, 8, 4, 5, 3, 8, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1],
    [9, 4, 5, 11, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 11, 0, 0, 11, 8, 5, 9, 4, -1, -1, -1, -1, -1, -1, -1],
    [4, 5, 0, 0, 5, 1, 11, 3, 2, -1, -1, -1, -1, -1, -1, -1],
    [5, 1, 4, 1, 2, 11, 4, 1, 11, 4, 11, 8, -1, -1, -1, -1],
    [1, 10, 2, 5, 9, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 4, 5, 0, 3, 8, 2, 1, 10, -1, -1, -1, -1, -1, -1, -1],
    [2, 5, 10, 2, 4, 5, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1],
    [10, 2, 5, 5, 2, 4, 4, 2, 3, 4, 3, 8, -1, -1, -1, -1],
    [11, 3, 10, 10, 3, 1, 4, 5, 9, -1, -1, -1, -1, -1, -1, -1],
    [4, 5, 9, 10, 0, 1, 10, 8, 0, 10, 11, 8, -1, -1, -1, -1],
    [11, 3, 0, 11, 0, 5, 0, 4, 5, 10, 11, 5, -1, -1, -1, -1],
    [4, 5, 8, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1],
    [8, 7, 9, 9, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 9, 0, 3, 5, 9, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1],
    [7, 0, 8, 7, 1, 0, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1],
    [7, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [5, 9, 7, 7, 9, 8, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1],
    [2, 11, 7, 2, 7, 9, 7, 5, 9, 0, 2, 9, -1, -1, -1, -1],
    [2, 11, 3, 7, 0, 8, 7, 1, 0, 7, 5, 1, -1, -1, -1, -1],
    [2, 11, 1, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1],
    [8, 7, 9, 9, 7, 5, 2, 1, 10, -1, -1, -1, -1, -1, -1, -1],
    [10, 2, 1, 3, 9, 0, 3, 5, 9, 3, 7, 5, -1, -1, -1, -1],
    [7, 5, 8, 5, 10, 2, 8, 5, 2, 8, 2, 0, -1, -1, -1, -1],
    [10, 2, 5, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1],
    [8, 7, 5, 8, 5, 9, 11, 3, 10, 3, 1, 10, -1, -1, -1, -1],
    [5, 11, 7, 10, 11, 5, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1],
    [11, 5, 10, 7, 5, 11, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1],
    [5, 11, 7, 10, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 7, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 11, 6, 3, 8, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 7, 11, 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 1, 8, 8, 1, 3, 6, 7, 11, -1, -1, -1, -1, -1, -1, -1],
    [3, 2, 7, 7, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 7, 8, 0, 6, 7, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1],
    [6, 7, 2, 2, 7, 3, 9, 1, 0, -1, -1, -1, -1, -1, -1, -1],
    [6, 7, 8, 6, 8, 1, 8, 9, 1, 2, 6, 1, -1, -1, -1, -1],
    [11, 6, 7, 10, 2, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 8, 0, 11, 6, 7, 10, 2, 1, -1, -1, -1, -1, -1, -1, -1],
    [0, 9, 2, 2, 9, 10, 7, 11, 6, -1, -1, -1, -1, -1, -1, -1],
    [6, 7, 11, 8, 2, 3, 8, 10, 2, 8, 9, 10, -1, -1, -1, -1],
    [7, 10, 6, 7, 1, 10, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1],
    [8, 0, 7, 7, 0, 6, 6, 0, 1, 6, 1, 10, -1, -1, -1, -1],
    [7, 3, 6, 3, 0, 9, 6, 3, 9, 6, 9, 10, -1, -1, -1, -1],
    [6, 7, 10, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1],
    [11, 6, 8, 8, 6, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 3, 11, 6, 0, 3, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1],
    [11, 6, 8, 8, 6, 4, 1, 0, 9, -1, -1, -1, -1, -1, -1, -1],
    [1, 3, 9, 3, 11, 6, 9, 3, 6, 9, 6, 4, -1, -1, -1, -1],
    [2, 8, 3, 2, 4, 8, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1],
    [4, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 1, 0, 2, 8, 3, 2, 4, 8, 2, 6, 4, -1, -1, -1, -1],
    [9, 1, 4, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1],
    [4, 8, 6, 6, 8, 11, 1, 10, 2, -1, -1, -1, -1, -1, -1, -1],
    [1, 10, 2, 6, 3, 11, 6, 0, 3, 6, 4, 0, -1, -1, -1, -1],
    [11, 6, 4, 11, 4, 8, 10, 2, 9, 2, 0, 9, -1, -1, -1, -1],
    [10, 4, 9, 6, 4, 10, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1],
    [4, 8, 3, 4, 3, 10, 3, 1, 10, 6, 4, 10, -1, -1, -1, -1],
    [1, 10, 0, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1],
    [4, 10, 6, 9, 10, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1],
    [4, 10, 6, 9, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 7, 11, 4, 5, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 5, 9, 7, 11, 6, 3, 8, 0, -1, -1, -1, -1, -1, -1, -1],
    [1, 0, 5, 5, 0, 4, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1],
    [11, 6, 7, 5, 8, 4, 5, 3, 8, 5, 1, 3, -1, -1, -1, -1],
    [3, 2, 7, 7, 2, 6, 9, 4, 5, -1, -1, -1, -1, -1, -1, -1],
    [5, 9, 4, 0, 7, 8, 0, 6, 7, 0, 2, 6, -1, -1, -1, -1],
    [3, 2, 6, 3, 6, 7, 1, 0, 5, 0, 4, 5, -1, -1, -1, -1],
    [6, 1, 2, 5, 1, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1],
    [10, 2, 1, 6, 7, 11, 4, 5, 9, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 8, 4, 5, 9, 11, 6, 7, 10, 2, 1, -1, -1, -1, -1],
    [7, 11, 6, 2, 5, 10, 2, 4, 5, 2, 0, 4, -1, -1, -1, -1],
    [8, 4, 7, 5, 10, 6, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1],
    [9, 4, 5, 7, 10, 6, 7, 1, 10, 7, 3, 1, -1, -1, -1, -1],
    [10, 6, 5, 7, 8, 4, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1],
    [4, 3, 0, 7, 3, 4, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1],
    [10, 6, 5, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 6, 5, 9, 11, 6, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1],
    [11, 6, 3, 3, 6, 0, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1],
    [11, 6, 5, 11, 5, 0, 5, 1, 0, 8, 11, 0, -1, -1, -1, -1],
    [11, 6, 3, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1],
    [9, 8, 5, 8, 3, 2, 5, 8, 2, 5, 2, 6, -1, -1, -1, -1],
    [5, 9, 6, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1],
    [1, 6, 5, 2, 6, 1, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1],
    [1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 1, 10, 9, 6, 5, 9, 11, 6, 9, 8, 11, -1, -1, -1, -1],
    [9, 0, 1, 3, 11, 2, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1],
    [11, 0, 8, 2, 0, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1],
    [3, 11, 2, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 8, 3, 9, 8, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1],
    [6, 5, 10, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 3, 0, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 5, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 8, 6, 10, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 5, 6, 9, 1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 8, 1, 1, 8, 9, 6, 10, 5, -1, -1, -1, -1, -1, -1, -1],
    [2, 11, 3, 6, 10, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 0, 11, 11, 0, 2, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1],
    [1, 0, 9, 2, 11, 3, 6, 10, 5, -1, -1, -1, -1, -1, -1, -1],
    [5, 6, 10, 11, 1, 2, 11, 9, 1, 11, 8, 9, -1, -1, -1, -1],
    [5, 6, 1, 1, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [5, 6, 1, 1, 6, 2, 8, 0, 3, -1, -1, -1, -1, -1, -1, -1],
    [6, 9, 5, 6, 0, 9, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1],
    [6, 2, 5, 2, 3, 8, 5, 2, 8, 5, 8, 9, -1, -1, -1, -1],
    [3, 6, 11, 3, 5, 6, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1],
    [8, 0, 1, 8, 1, 6, 1, 5, 6, 11, 8, 6, -1, -1, -1, -1],
    [11, 3, 6, 6, 3, 5, 5, 3, 0, 5, 0, 9, -1, -1, -1, -1],
    [5, 6, 9, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1],
    [5, 6, 10, 7, 4, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 4, 4, 3, 7, 10, 5, 6, -1, -1, -1, -1, -1, -1, -1],
    [5, 6, 10, 4, 8, 7, 0, 9, 1, -1, -1, -1, -1, -1, -1, -1],
    [6, 10, 5, 1, 4, 9, 1, 7, 4, 1, 3, 7, -1, -1, -1, -1],
    [7, 4, 8, 6, 10, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1],
    [10, 5, 6, 4, 11, 7, 4, 2, 11, 4, 0, 2, -1, -1, -1, -1],
    [4, 8, 7, 6, 10, 5, 3, 2, 11, 1, 0, 9, -1, -1, -1, -1],
    [1, 2, 10, 11, 7, 6, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1],
    [2, 1, 6, 6, 1, 5, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 7, 0, 7, 4, 2, 1, 6, 1, 5, 6, -1, -1, -1, -1],
    [8, 7, 4, 6, 9, 5, 6, 0, 9, 6, 2, 0, -1, -1, -1, -1],
    [7, 2, 3, 6, 2, 7, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1],
    [4, 8, 7, 3, 6, 11, 3, 5, 6, 3, 1, 5, -1, -1, -1, -1],
    [5, 0, 1, 4, 0, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1],
    [9, 5, 4, 6, 11, 7, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1],
    [11, 7, 6, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 10, 4, 4, 10, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 10, 4, 4, 10, 9, 3, 8, 0, -1, -1, -1, -1, -1, -1, -1],
    [0, 10, 1, 0, 6, 10, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1],
    [6, 10, 1, 6, 1, 8, 1, 3, 8, 4, 6, 8, -1, -1, -1, -1],
    [9, 4, 10, 10, 4, 6, 3, 2, 11, -1, -1, -1, -1, -1, -1, -1],
    [2, 11, 8, 2, 8, 0, 6, 10, 4, 10, 9, 4, -1, -1, -1, -1],
    [11, 3, 2, 0, 10, 1, 0, 6, 10, 0, 4, 6, -1, -1, -1, -1],
    [6, 8, 4, 11, 8, 6, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1],
    [4, 1, 9, 4, 2, 1, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1],
    [3, 8, 0, 4, 1, 9, 4, 2, 1, 4, 6, 2, -1, -1, -1, -1],
    [6, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 8, 2, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1],
    [4, 6, 9, 6, 11, 3, 9, 6, 3, 9, 3, 1, -1, -1, -1, -1],
    [8, 6, 11, 4, 6, 8, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1],
    [11, 3, 6, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1],
    [8, 6, 11, 4, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 7, 6, 10, 8, 7, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1],
    [3, 7, 0, 7, 6, 10, 0, 7, 10, 0, 10, 9, -1, -1, -1, -1],
    [6, 10, 7, 7, 10, 8, 8, 10, 1, 8, 1, 0, -1, -1, -1, -1],
    [6, 10, 7, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1],
    [3, 2, 11, 10, 7, 6, 10, 8, 7, 10, 9, 8, -1, -1, -1, -1],
    [2, 9, 0, 10, 9, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1],
    [0, 8, 3, 7, 6, 11, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1],
    [7, 6, 11, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 1, 9, 2, 9, 7, 9, 8, 7, 6, 2, 7, -1, -1, -1, -1],
    [2, 7, 6, 3, 7, 2, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1],
    [8, 7, 0, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1],
    [7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 1, 9, 3, 1, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1],
    [11, 7, 6, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [6, 11, 7, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 11, 5, 5, 11, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [10, 5, 11, 11, 5, 7, 0, 3, 8, -1, -1, -1, -1, -1, -1, -1],
    [7, 11, 5, 5, 11, 10, 0, 9, 1, -1, -1, -1, -1, -1, -1, -1],
    [7, 11, 10, 7, 10, 5, 3, 8, 1, 8, 9, 1, -1, -1, -1, -1],
    [5, 2, 10, 5, 3, 2, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1],
    [5, 7, 10, 7, 8, 0, 10, 7, 0, 10, 0, 2, -1, -1, -1, -1],
    [0, 9, 1, 5, 2, 10, 5, 3, 2, 5, 7, 3, -1, -1, -1, -1],
    [9, 7, 8, 5, 7, 9, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1],
    [1, 11, 2, 1, 7, 11, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1],
    [8, 0, 3, 1, 11, 2, 1, 7, 11, 1, 5, 7, -1, -1, -1, -1],
    [7, 11, 2, 7, 2, 9, 2, 0, 9, 5, 7, 9, -1, -1, -1, -1],
    [7, 9, 5, 8, 9, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1],
    [3, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 0, 7, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1],
    [0, 9, 3, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1],
    [9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 5, 4, 8, 10, 5, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 11, 0, 11, 5, 11, 10, 5, 4, 0, 5, -1, -1, -1, -1],
    [1, 0, 9, 8, 5, 4, 8, 10, 5, 8, 11, 10, -1, -1, -1, -1],
    [10, 3, 11, 1, 3, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1],
    [3, 2, 8, 8, 2, 4, 4, 2, 10, 4, 10, 5, -1, -1, -1, -1],
    [10, 5, 2, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1],
    [5, 4, 9, 8, 3, 0, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1],
    [2, 10, 1, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 11, 4, 11, 2, 1, 4, 11, 1, 4, 1, 5, -1, -1, -1, -1],
    [0, 5, 4, 1, 5, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1],
    [0, 11, 2, 8, 11, 0, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1],
    [5, 4, 9, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [4, 8, 5, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1],
    [0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [5, 4, 9, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [5, 4, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 4, 7, 11, 9, 4, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 8, 11, 4, 7, 11, 9, 4, 11, 10, 9, -1, -1, -1, -1],
    [11, 10, 7, 10, 1, 0, 7, 10, 0, 7, 0, 4, -1, -1, -1, -1],
    [3, 10, 1, 11, 10, 3, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1],
    [3, 2, 10, 3, 10, 4, 10, 9, 4, 7, 3, 4, -1, -1, -1, -1],
    [9, 2, 10, 0, 2, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1],
    [3, 4, 7, 0, 4, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1],
    [7, 8, 4, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 11, 4, 4, 11, 9, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1],
    [1, 9, 0, 4, 7, 8, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1],
    [7, 11, 4, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1],
    [4, 7, 8, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [9, 4, 1, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1],
    [7, 8, 4, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 4, 7, 0, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [7, 8, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [0, 3, 9, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1],
    [1, 0, 10, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1],
    [10, 3, 11, 1, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 2, 8, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1],
    [9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 3, 0, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 10, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [2, 1, 11, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1],
    [11, 2, 3, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [11, 0, 8, 2, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [1, 9, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [8, 3, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
    [
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    ],
];
