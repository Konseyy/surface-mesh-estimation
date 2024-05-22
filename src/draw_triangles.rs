use cgmath::{vec3, vec4, Matrix4, Point3};
use glium::{implement_vertex, program, uniform, PolygonMode, Surface};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use winit::keyboard::{KeyCode, PhysicalKey};

use crate::utils::{Triangle, Vec3};

#[derive(Copy, Clone)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

implement_vertex!(Vertex, position, normal);

pub fn draw_triangles(triangles: Vec<Triangle>, max_light_distance: f32) {
    let event_loop = winit::event_loop::EventLoopBuilder::new()
        .build()
        .expect("event loop building");
    let (window, display) = glium::backend::glutin::SimpleWindowBuilder::new()
        .with_title("Render output")
        .with_inner_size(1200, 1200)
        .build(&event_loop);

    let tri_vertices = triangles
        .par_iter()
        .flat_map(|tri| {
            vec![
                Vertex {
                    position: tri.vertices[0],
                    normal: tri.normal,
                },
                Vertex {
                    position: tri.vertices[1],
                    normal: tri.normal,
                },
                Vertex {
                    position: tri.vertices[2],
                    normal: tri.normal,
                },
            ]
        })
        .collect::<Vec<Vertex>>();

    let vertex_buffer: glium::VertexBuffer<Vertex> =
        glium::VertexBuffer::new(&display, &tri_vertices).expect("failed to create vertex buffer");
    let index_buffer = glium::index::NoIndices(glium::index::PrimitiveType::TrianglesList);

    let shadow_program = program!(&display,
        330 => {
            vertex: include_str!("shaders/shadows.vs"),
            fragment: include_str!("shaders/shadows.fs"),
        },
    )
    .expect("failed to compile shaders");
    let render_program = program!(&display,
        330 => {
            vertex: include_str!("shaders/render.vs"),
            fragment: include_str!("shaders/render.fs"),
        },
    )
    .expect("failed to compile shaders");

    let shadow_texture = glium::texture::Cubemap::empty(&display, 1024).unwrap();
    let depth_tex = glium::texture::DepthCubemap::empty(&display, 1024).unwrap();

    let (view_w, view_h) = display.get_framebuffer_dimensions();
    let aspect = view_w as f32 / view_h as f32;
    const FAR_PLANE: f32 = 10000.0;
    let projection = cgmath::perspective(cgmath::Deg(90.0), aspect, 0.01, FAR_PLANE);
    let shadow_projection = cgmath::perspective(cgmath::Deg(90.0), aspect, 0.01, FAR_PLANE);

    const START_VIEW_HEIGHT: f32 = 55f32;

    let mut rotation = 0f32;
    let mut show_normals = false;
    let mut show_shadows = true;

    let mut camera = Vec3::new(0., 0., START_VIEW_HEIGHT);
    let mut look_at = Vec3::new(10., 10., START_VIEW_HEIGHT);
    let mut light_pos = vec3(0., 0., 0.);

    let mut rotation_paused = true;
    let mut ticks_paused = true;
    let mut key_rotate_left = false;
    let mut key_rotate_right = false;
    let mut key_go_forward = false;
    let mut key_go_backward = false;
    let mut key_go_up = false;
    let mut key_go_down = false;

    let mut start_time = std::time::Instant::now();
    let mut ms_since_start = start_time.elapsed().as_millis() as f32;

    event_loop
        .run(move |event, window_target| match event {
            winit::event::Event::WindowEvent { event, .. } => match event {
                // This event is sent by the OS when you close the Window, or request the program to quit via the taskbar.
                winit::event::WindowEvent::CloseRequested => window_target.exit(),
                winit::event::WindowEvent::KeyboardInput {
                    device_id: _,
                    event,
                    is_synthetic: _,
                } => {
                    if event.state == winit::event::ElementState::Pressed {
                        if let PhysicalKey::Code(code) = event.physical_key {
                            match code {
                                KeyCode::Escape => {
                                    window_target.exit();
                                }
                                KeyCode::KeyN => {
                                    show_normals = !show_normals;
                                }
                                KeyCode::Space => {
                                    rotation_paused = !rotation_paused;
                                }
                                KeyCode::KeyE => {
                                    if key_go_down {
                                        return;
                                    }
                                    key_go_up = true;
                                }
                                KeyCode::KeyQ => {
                                    if key_go_up {
                                        return;
                                    }
                                    key_go_down = true;
                                }
                                KeyCode::KeyW => {
                                    if !rotation_paused {
                                        return;
                                    }
                                    key_go_forward = true;
                                }
                                KeyCode::KeyS => {
                                    if !rotation_paused {
                                        return;
                                    }
                                    key_go_backward = true;
                                }
                                KeyCode::KeyA => {
                                    if !rotation_paused {
                                        return;
                                    }
                                    key_rotate_left = true;
                                }
                                KeyCode::KeyD => {
                                    if !rotation_paused {
                                        return;
                                    }
                                    key_rotate_right = true;
                                }
                                KeyCode::KeyT => {
                                    camera = Vec3::new(0., 0., START_VIEW_HEIGHT);
                                    look_at = Vec3::new(10., 10., START_VIEW_HEIGHT);
                                }
                                KeyCode::KeyY => {
                                    camera = Vec3::new(light_pos.x, light_pos.y, light_pos.z);
                                    look_at = Vec3::new(
                                        light_pos.x + 10.,
                                        light_pos.y + 10.,
                                        light_pos.z,
                                    );
                                }
                                KeyCode::KeyL => {
                                    ticks_paused = !ticks_paused;
                                }
                                KeyCode::KeyF => {
                                    show_shadows = !show_shadows;
                                }
                                _ => (),
                            }
                        }
                    } else if event.state == winit::event::ElementState::Released {
                        if let PhysicalKey::Code(code) = event.physical_key {
                            match code {
                                KeyCode::KeyE => {
                                    key_go_up = false;
                                }
                                KeyCode::KeyQ => {
                                    key_go_down = false;
                                }
                                KeyCode::KeyW => {
                                    key_go_forward = false;
                                }
                                KeyCode::KeyS => {
                                    key_go_backward = false;
                                }
                                KeyCode::KeyA => {
                                    key_rotate_left = false;
                                }
                                KeyCode::KeyD => {
                                    key_rotate_right = false;
                                }
                                _ => (),
                            }
                        }
                    }
                }
                winit::event::WindowEvent::RedrawRequested => {
                    if !ticks_paused {
                        ms_since_start += start_time.elapsed().as_millis() as f32;
                    }
                    start_time = std::time::Instant::now();

                    if !rotation_paused {
                        rotation += 0.6;
                    }
                    if key_rotate_left {
                        rotation += 1.0;
                    }
                    if key_rotate_right {
                        rotation -= 1.0;
                    }
                    if key_go_up {
                        camera.z += 10.0;
                        look_at.z += 10.0;
                    }
                    if key_go_down {
                        camera.z -= 10.0;
                        look_at.z -= 10.0;
                    }

                    let rotation = Matrix4::from_angle_x(cgmath::Deg(0.0))
                        * Matrix4::from_angle_y(cgmath::Deg(0.0))
                        * Matrix4::from_angle_z(cgmath::Deg(0.0 + rotation));

                    let translation = Matrix4::from_translation(vec3(camera.x, camera.y, camera.z));
                    let inv_translation =
                        Matrix4::from_translation(vec3(-camera.x, -camera.y, -camera.z));

                    let used_look_at = translation
                        * rotation
                        * inv_translation
                        * vec4(look_at.x, look_at.y, look_at.z, 1.0);

                    if key_go_forward {
                        let norm_forward =
                            (Vec3::new(used_look_at.x, used_look_at.y, used_look_at.z) - camera)
                                .normalize();
                        camera += norm_forward * 10.;
                        look_at += norm_forward * 10.;
                    }
                    if key_go_backward {
                        let norm_forward =
                            (Vec3::new(used_look_at.x, used_look_at.y, used_look_at.z) - camera)
                                .normalize();
                        camera -= norm_forward * 10.;
                        look_at -= norm_forward * 10.;
                    }

                    let view = Matrix4::look_at_rh(
                        Point3::new(camera.x, camera.y, camera.z),
                        Point3::new(used_look_at.x, used_look_at.y, used_look_at.z),
                        vec3(0.0, 0.0, 1.0),
                    );

                    let tick = ms_since_start / 550.;
                    light_pos = vec3(tick.cos() * 400., tick.sin() * 800., 0.0);

                    let shadow_views = &[
                        glium::texture::CubeLayer::PositiveX,
                        glium::texture::CubeLayer::NegativeX,
                        glium::texture::CubeLayer::PositiveY,
                        glium::texture::CubeLayer::NegativeY,
                        glium::texture::CubeLayer::PositiveZ,
                        glium::texture::CubeLayer::NegativeZ,
                    ]
                    .iter()
                    .map(|layer| {
                        let view_matrix = match layer {
                            glium::texture::CubeLayer::NegativeX => Matrix4::look_at_lh(
                                Point3::new(light_pos.x, light_pos.y, light_pos.z),
                                Point3::new(light_pos.x + 1.0, light_pos.y, light_pos.z),
                                vec3(0.0, -1.0, 0.0),
                            ),
                            glium::texture::CubeLayer::PositiveX => Matrix4::look_at_lh(
                                Point3::new(light_pos.x, light_pos.y, light_pos.z),
                                Point3::new(light_pos.x - 1.0, light_pos.y, light_pos.z),
                                vec3(0.0, -1.0, 0.0),
                            ),
                            // good
                            glium::texture::CubeLayer::NegativeY => Matrix4::look_at_lh(
                                Point3::new(light_pos.x, light_pos.y, light_pos.z),
                                Point3::new(light_pos.x, light_pos.y + 1.0, light_pos.z),
                                vec3(0.0, 0.0, -1.0),
                            ),
                            glium::texture::CubeLayer::PositiveY => Matrix4::look_at_lh(
                                Point3::new(light_pos.x, light_pos.y, light_pos.z),
                                Point3::new(light_pos.x, light_pos.y - 1.0, light_pos.z),
                                vec3(0.0, 0.0, 1.0),
                            ),
                            glium::texture::CubeLayer::NegativeZ => Matrix4::look_at_lh(
                                Point3::new(light_pos.x, light_pos.y, light_pos.z),
                                Point3::new(light_pos.x, light_pos.y, light_pos.z + 1.0),
                                vec3(0.0, -1.0, 0.0),
                            ),
                            glium::texture::CubeLayer::PositiveZ => Matrix4::look_at_lh(
                                Point3::new(light_pos.x, light_pos.y, light_pos.z),
                                Point3::new(light_pos.x, light_pos.y, light_pos.z - 1.0),
                                vec3(0.0, -1.0, 0.0),
                            ),
                        };
                        return (*layer, view_matrix);
                    })
                    .collect::<Vec<_>>();

                    // Render the scene from the light's point of view into depth buffer
                    // ===============================================================================
                    {
                        let mut draw_params: glium::draw_parameters::DrawParameters<'_> =
                            Default::default();
                        draw_params.depth = glium::Depth {
                            test: glium::draw_parameters::DepthTest::IfLessOrEqual,
                            write: true,
                            ..Default::default()
                        };
                        draw_params.backface_culling =
                            glium::BackfaceCullingMode::CullCounterClockwise;

                        for (layer, shadow_view) in shadow_views {
                            // Write depth to shadow map texture
                            let mut framebuffer =
                                glium::framebuffer::SimpleFrameBuffer::with_depth_buffer(
                                    &display,
                                    shadow_texture.main_level().image(*layer),
                                    depth_tex.main_level().image(*layer),
                                )
                                .unwrap();
                            framebuffer.clear_color_and_depth((0., 0., 0., 1.), 1.);

                            let uniforms = uniform! {
                            depth_mvp: Into::<[[f32; 4]; 4]>::into(shadow_projection*shadow_view),
                            far_plane: FAR_PLANE,
                            light_pos: [light_pos.x, light_pos.y, light_pos.z],
                            };

                            framebuffer
                                .draw(
                                    &vertex_buffer,
                                    &index_buffer,
                                    &shadow_program,
                                    &uniforms,
                                    &draw_params,
                                )
                                .unwrap();
                        }
                    }

                    let mut screen = display.draw();
                    screen.clear_color_and_depth((0.011, 0.0089, 0.1622, 0.0), 1.0);

                    // Render from camera POV
                    {
                        let shadow_sampler = glium::uniforms::Sampler::new(&shadow_texture)
                            .magnify_filter(glium::uniforms::MagnifySamplerFilter::Nearest)
                            .minify_filter(glium::uniforms::MinifySamplerFilter::Nearest);

                        let uniforms = uniform! {
                            mvp: Into::<[[f32; 4]; 4]>::into(projection  * view),
                            show_normals: if show_normals { 1f32 } else { 0f32 },
                            show_shadows: if show_shadows { 1f32 } else { 0f32 },
                            max_distance: max_light_distance,
                            light_pos: [light_pos.x, light_pos.y, light_pos.z],
                            shadow_map: shadow_sampler,
                            far_plane: FAR_PLANE,
                        };

                        let polygon_mode = PolygonMode::Fill;

                        let draw_parameters = glium::DrawParameters {
                            depth: glium::Depth {
                                test: glium::DepthTest::IfLess,
                                write: true,
                                ..Default::default()
                            },
                            point_size: None,
                            polygon_mode,
                            backface_culling:
                                glium::draw_parameters::BackfaceCullingMode::CullCounterClockwise,
                            ..Default::default()
                        };

                        screen
                            .draw(
                                &vertex_buffer,
                                &index_buffer,
                                &render_program,
                                &uniforms,
                                &draw_parameters,
                            )
                            .expect("failed to draw to surface");
                    }

                    screen.finish().expect("failed to finish rendering frame");
                }

                _ => (),
            },
            // By requesting a redraw in response to a RedrawEventsCleared event we get continuous rendering.
            // For applications that only change due to user input you could remove this handler.
            winit::event::Event::AboutToWait => {
                window.request_redraw();
            }
            _ => (),
        })
        .unwrap();
}
