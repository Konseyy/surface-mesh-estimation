use cgmath::num_traits::real;
use kd_tree::KdTree;
use rayon::{
    iter::{IntoParallelRefIterator, ParallelIterator},
    prelude::*,
};

use crate::utils::{CartesianCoordinate, TextCoords, Triangle, Vec3};

#[derive(Debug)]
struct NetVertex {
    idx: usize,
    is_surface: bool,
    grid_pos: Vec3,
    real_pos: Vec3,
}

pub fn by_surface_nets(
    coordinates: &Vec<CartesianCoordinate>,
    tree: &KdTree<CartesianCoordinate>,
    voxel_size: usize,
    smooth_steps: usize,
) -> (Vec<Triangle>, (usize, usize, usize)) {
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

            // Filter out only the points that are within the voxel
            let filtered_densities = density
                .par_iter()
                .filter(|c| {
                    (c.vec_coord.x - grid_point.x).abs() < voxel_size as f32 / 2.
                        && (c.vec_coord.y - grid_point.y).abs() < voxel_size as f32 / 2.
                        && (c.vec_coord.z - grid_point.z).abs() < voxel_size as f32 / 2.
                })
                .collect::<Vec<_>>();

            return ((x_grid, y_grid, z_grid), filtered_densities.len());
        })
        .collect::<Vec<((&usize, &usize, &usize), usize)>>();

    // Sort the results by the grid coordinates
    vertex_densities.par_sort_by(|a, b| {
        let idx_1 = get_grid_index(grid_dimensions, *a.0 .0, *a.0 .1, *a.0 .2);
        let idx_2 = get_grid_index(grid_dimensions, *b.0 .0, *b.0 .1, *b.0 .2);
        return idx_1.cmp(&idx_2);
    });

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

    let mut net_vertices =
        Vec::<NetVertex>::with_capacity(grid_dimensions.0 * grid_dimensions.1 * grid_dimensions.2);
    for x in 0..grid_dimensions.0 {
        for y in 0..grid_dimensions.1 {
            for z in 0..grid_dimensions.2 {
                let points = [
                    (x, y, z),
                    (x + 1, y, z),
                    (x, y + 1, z),
                    (x + 1, y + 1, z),
                    (x, y, z + 1),
                    (x + 1, y, z + 1),
                    (x, y + 1, z + 1),
                    (x + 1, y + 1, z + 1),
                ]
                .iter()
                .map(|p| get_grid_index(grid_dimensions, p.0, p.1, p.2))
                .filter(|&p| p.is_some())
                .map(|p| vertex_densities[p.unwrap()])
                .collect::<Vec<((&usize, &usize, &usize), usize)>>();

                let points_over_threshold = points
                    .iter()
                    .map(|p| p.1 as f32 > density_threshold)
                    .collect::<Vec<bool>>();

                let density_sum = points.iter().map(|p| p.1).sum::<usize>() as f32;

                let cube_center_pos: Vec3 = points
                    .iter()
                    .filter(|p| p.1 > 0)
                    .map(|p| {
                        let real_pos = net_space_to_real_space(
                            Vec3::new(*p.0 .0 as f32, *p.0 .1 as f32, *p.0 .2 as f32),
                            max_mm_dimensions,
                            voxel_size,
                        );

                        let weight = p.1 as f32 / density_sum;

                        real_pos * weight
                    })
                    .sum();

                let cube_center_idx = get_grid_index(grid_dimensions, x, y, z).unwrap();
                let grid_position = Vec3::new(x as f32, y as f32, z as f32);

                // let cube_middle_pos = Vec3::new(
                //     x as f32 * voxel_sf32 - max_mm_dimensions.0 as f32,
                //     y as f32 * voxel_sf32 - max_mm_dimensions.1 as f32,
                //     z as f32 * voxel_sf32 - max_mm_dimensions.2 as f32,
                // );

                if points_over_threshold.iter().all(|&p| p == true) {
                    net_vertices.push(NetVertex {
                        idx: cube_center_idx,
                        is_surface: false,
                        grid_pos: grid_position,
                        real_pos: cube_center_pos,
                    });
                } else if points_over_threshold.iter().all(|&p| p == false) {
                    net_vertices.push(NetVertex {
                        idx: cube_center_idx,
                        is_surface: false,
                        grid_pos: grid_position,
                        real_pos: cube_center_pos,
                    });
                } else {
                    net_vertices.push(NetVertex {
                        idx: cube_center_idx,
                        is_surface: true,
                        grid_pos: grid_position,
                        real_pos: cube_center_pos,
                    });
                }
            }
        }
    }

    net_vertices.par_sort_by(|a, b| a.idx.cmp(&b.idx));

    for i in 0..smooth_steps {
        println!("Smoothing iteration {:?}", i);
        net_vertices = net_vertices
            .par_iter()
            .map(|v| {
                let neighbor_vertices = [
                    get_grid_index(
                        grid_dimensions,
                        v.grid_pos.x as usize - 1,
                        v.grid_pos.y as usize,
                        v.grid_pos.z as usize,
                    ),
                    get_grid_index(
                        grid_dimensions,
                        v.grid_pos.x as usize + 1,
                        v.grid_pos.y as usize,
                        v.grid_pos.z as usize,
                    ),
                    get_grid_index(
                        grid_dimensions,
                        v.grid_pos.x as usize,
                        v.grid_pos.y as usize - 1,
                        v.grid_pos.z as usize,
                    ),
                    get_grid_index(
                        grid_dimensions,
                        v.grid_pos.x as usize,
                        v.grid_pos.y as usize + 1,
                        v.grid_pos.z as usize,
                    ),
                    get_grid_index(
                        grid_dimensions,
                        v.grid_pos.x as usize,
                        v.grid_pos.y as usize,
                        v.grid_pos.z as usize - 1,
                    ),
                    get_grid_index(
                        grid_dimensions,
                        v.grid_pos.x as usize,
                        v.grid_pos.y as usize,
                        v.grid_pos.z as usize + 1,
                    ),
                ]
                .iter()
                .map(|idx_o| match idx_o {
                    Some(idx) => Some(&net_vertices[*idx]),
                    None => None,
                })
                .collect::<Vec<Option<&NetVertex>>>();

                let existing_neighbors = neighbor_vertices
                    .iter()
                    .filter(|&v| {
                        if let Some(vert) = v {
                            return vert.is_surface;
                        } else {
                            return false;
                        }
                    })
                    .map(|v| v.unwrap())
                    .collect::<Vec<&NetVertex>>();

                let count_neighbors = existing_neighbors.len() as f32;

                NetVertex {
                    idx: v.idx,
                    is_surface: v.is_surface,
                    grid_pos: v.grid_pos,
                    real_pos: existing_neighbors
                        .iter()
                        .fold(Vec3::new(0., 0., 0.), |acc, v| {
                            acc + v.real_pos / (count_neighbors)
                        }),
                }
            })
            .collect::<Vec<NetVertex>>();
    }

    let mut printed = false;

    for x in 0..(grid_dimensions.0 - 1) {
        for y in 0..(grid_dimensions.1 - 1) {
            for z in 0..(grid_dimensions.2 - 1) {
                let net_idx_o = get_grid_index(grid_dimensions, x, y, z);
                let net_idx = match net_idx_o {
                    Some(idx) => idx,
                    None => continue,
                };

                let net_left_idx_o = get_grid_index(grid_dimensions, x - 1, y, z);
                let net_right_idx_o = get_grid_index(grid_dimensions, x + 1, y, z);
                let net_front_idx_o = get_grid_index(grid_dimensions, x, y + 1, z);
                let net_back_idx_o = get_grid_index(grid_dimensions, x, y - 1, z);
                let net_up_idx_o = get_grid_index(grid_dimensions, x, y, z + 1);
                let net_down_idx_o = get_grid_index(grid_dimensions, x, y, z - 1);

                let center = if net_vertices[net_idx].is_surface {
                    &net_vertices[net_idx]
                } else {
                    continue;
                };

                let surface_left = match net_left_idx_o {
                    Some(idx) => {
                        if net_vertices[idx].is_surface {
                            Some(&net_vertices[idx])
                        } else {
                            None
                        }
                    }
                    None => None,
                };
                let surface_right = match net_right_idx_o {
                    Some(idx) => {
                        if net_vertices[idx].is_surface {
                            Some(&net_vertices[idx])
                        } else {
                            None
                        }
                    }
                    None => None,
                };
                let surface_front = match net_front_idx_o {
                    Some(idx) => {
                        if net_vertices[idx].is_surface {
                            Some(&net_vertices[idx])
                        } else {
                            None
                        }
                    }
                    None => None,
                };
                let surface_back = match net_back_idx_o {
                    Some(idx) => {
                        if net_vertices[idx].is_surface {
                            Some(&net_vertices[idx])
                        } else {
                            None
                        }
                    }
                    None => None,
                };
                let surface_up = match net_up_idx_o {
                    Some(idx) => {
                        if net_vertices[idx].is_surface {
                            Some(&net_vertices[idx])
                        } else {
                            None
                        }
                    }
                    None => None,
                };
                let surface_down = match net_down_idx_o {
                    Some(idx) => {
                        if net_vertices[idx].is_surface {
                            Some(&net_vertices[idx])
                        } else {
                            None
                        }
                    }
                    None => None,
                };

                if let (Some(left), Some(up), Some(back)) = (surface_left, surface_up, surface_back)
                {
                    let a = up.real_pos - back.real_pos;
                    let b = left.real_pos - back.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });
                }

                if let (Some(left), Some(up), Some(front)) =
                    (surface_left, surface_up, surface_front)
                {
                    let a = up.real_pos - front.real_pos;
                    let b = left.real_pos - front.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });
                }

                if let (Some(left), Some(down), Some(back)) =
                    (surface_left, surface_down, surface_back)
                {
                    let a = down.real_pos - back.real_pos;
                    let b = left.real_pos - back.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });
                }

                if let (Some(left), Some(down), Some(front)) =
                    (surface_left, surface_down, surface_front)
                {
                    let a = down.real_pos - front.real_pos;
                    let b = left.real_pos - front.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });
                }

                if let (Some(right), Some(up), Some(back)) =
                    (surface_right, surface_up, surface_back)
                {
                    let a = up.real_pos - back.real_pos;
                    let b = right.real_pos - back.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });
                }

                if let (Some(right), Some(up), Some(front)) =
                    (surface_right, surface_up, surface_front)
                {
                    let a = up.real_pos - front.real_pos;
                    let b = right.real_pos - front.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });

                    if !printed {
                        println!(
                            "Triangle: {:?} {:?} {:?} {:?}",
                            Triangle {
                                vertices: [
                                    [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                                    [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                                    [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                                ],
                                normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                            },
                            front,
                            up,
                            right
                        );
                        printed = true;
                    }
                }

                if let (Some(right), Some(down), Some(back)) =
                    (surface_right, surface_down, surface_back)
                {
                    let a = down.real_pos - back.real_pos;
                    let b = right.real_pos - back.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });
                }

                if let (Some(right), Some(down), Some(front)) =
                    (surface_right, surface_down, surface_front)
                {
                    let a = down.real_pos - front.real_pos;
                    let b = right.real_pos - front.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    triangles.push(Triangle {
                        vertices: [
                            [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                        ],
                        normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                    });
                }

                // left and back
                if let (Some(left), Some(back)) = (surface_left, surface_back) {
                    let a = left.real_pos - center.real_pos;
                    let b = back.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_up.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_down.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // back and right
                if let (Some(back), Some(right)) = (surface_back, surface_right) {
                    let a = back.real_pos - center.real_pos;
                    let b = right.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_up.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_down.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // right and front
                if let (Some(right), Some(front)) = (surface_right, surface_front) {
                    let a = right.real_pos - center.real_pos;
                    let b = front.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_up.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_down.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // front and left
                if let (Some(front), Some(left)) = (surface_front, surface_left) {
                    let a = front.real_pos - center.real_pos;
                    let b = left.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_up.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_down.is_some() || (surface_up.is_none() && surface_down.is_none()) {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // top and back
                if let (Some(up), Some(back)) = (surface_up, surface_back) {
                    let a = up.real_pos - center.real_pos;
                    let b = back.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_left.is_some() || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_right.is_some()
                        || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // back and bottom
                if let (Some(back), Some(down)) = (surface_back, surface_down) {
                    let a = back.real_pos - center.real_pos;
                    let b = down.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_left.is_some() || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_right.is_some()
                        || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                                [back.real_pos.x, back.real_pos.y, back.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // bottom and front
                if let (Some(down), Some(front)) = (surface_down, surface_front) {
                    let a = front.real_pos - center.real_pos;
                    let b = down.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_left.is_some() || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_right.is_some()
                        || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // front and top
                if let (Some(front), Some(up)) = (surface_front, surface_up) {
                    let a = front.real_pos - center.real_pos;
                    let b = up.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_left.is_some() || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_right.is_some()
                        || (surface_left.is_none() && surface_right.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                                [front.real_pos.x, front.real_pos.y, front.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // top and right
                if let (Some(up), Some(right)) = (surface_up, surface_right) {
                    let a = up.real_pos - center.real_pos;
                    let b = right.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_front.is_some()
                        || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_back.is_some() || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // right and bottom
                if let (Some(right), Some(down)) = (surface_right, surface_down) {
                    let a = right.real_pos - center.real_pos;
                    let b = down.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_front.is_some()
                        || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_back.is_some() || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                                [right.real_pos.x, right.real_pos.y, right.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // bottom and left
                if let (Some(down), Some(left)) = (surface_down, surface_left) {
                    let a = down.real_pos - center.real_pos;
                    let b = left.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_front.is_some()
                        || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_back.is_some() || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                                [down.real_pos.x, down.real_pos.y, down.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }

                // left and top
                if let (Some(left), Some(up)) = (surface_left, surface_up) {
                    let a = left.real_pos - center.real_pos;
                    let b = up.real_pos - center.real_pos;

                    let norm_x = a.y * b.z - a.z * b.y;
                    let norm_y = a.z * b.x - a.x * b.z;
                    let norm_z = a.x * b.y - a.y * b.x;

                    let normal_vec = Vec3::new(norm_x, norm_y, norm_z).normalize();

                    if surface_front.is_some()
                        || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                            ],
                            normal: [normal_vec.x, normal_vec.y, normal_vec.z],
                        });
                    }
                    if surface_back.is_some() || (surface_front.is_none() && surface_back.is_none())
                    {
                        triangles.push(Triangle {
                            vertices: [
                                [center.real_pos.x, center.real_pos.y, center.real_pos.z],
                                [up.real_pos.x, up.real_pos.y, up.real_pos.z],
                                [left.real_pos.x, left.real_pos.y, left.real_pos.z],
                            ],
                            normal: [-normal_vec.x, -normal_vec.y, -normal_vec.z],
                        });
                    }
                }
            }
        }
    }

    println!("Triangles: {:?}", triangles.len());

    return (triangles, max_mm_dimensions);
}

// Given the grid size and the elements coordinates, return the index of the element in the grid array
fn get_grid_index(
    grid_dimensions: (usize, usize, usize),
    x: usize,
    y: usize,
    z: usize,
) -> Option<usize> {
    if x >= grid_dimensions.0 || y >= grid_dimensions.1 || z >= grid_dimensions.2 {
        return None;
    }
    return Some(x + y * grid_dimensions.0 + z * grid_dimensions.0 * grid_dimensions.1);
}

fn net_space_to_real_space(
    net_point: Vec3,
    max_mm_dimensions: (usize, usize, usize),
    voxel_size: usize,
) -> Vec3 {
    let dimensions_real = Vec3::new(
        (max_mm_dimensions.0) as f32,
        (max_mm_dimensions.1) as f32,
        (max_mm_dimensions.2) as f32,
    );

    return net_point * voxel_size as f32 - dimensions_real;
}
