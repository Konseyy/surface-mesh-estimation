use cgmath::{vec3, vec4, Matrix4, Point3};
use glium::{
    glutin::{
        self,
        dpi::LogicalSize,
        event::{ElementState, KeyboardInput, VirtualKeyCode, WindowEvent},
        Api, GlProfile, GlRequest,
    },
    implement_vertex, program, uniform, PolygonMode, Surface,
};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

use crate::utils::{Triangle, Vec3};

#[derive(Copy, Clone)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

implement_vertex!(Vertex, position, normal);

#[derive(Clone, Copy, Debug)]
struct DebugVertex {
    position: [f32; 2],
    tex_coords: [f32; 2],
}
implement_vertex!(DebugVertex, position, tex_coords);
impl DebugVertex {
    pub fn new(position: [f32; 2], tex_coords: [f32; 2]) -> Self {
        Self {
            position,
            tex_coords,
        }
    }
}

pub fn draw_triangles(triangles: Vec<Triangle>, max_light_distance: f32) {
    let events_loop = glutin::event_loop::EventLoop::new();
    let window = glutin::window::WindowBuilder::new()
        .with_title("Render output")
        .with_inner_size(LogicalSize {
            width: 1600.0,
            height: 900.0,
        });
    let context = glutin::ContextBuilder::new()
        .with_vsync(true)
        .with_gl_profile(GlProfile::Core)
        .with_gl(GlRequest::Specific(Api::OpenGl, (3, 3)))
        .with_depth_buffer(24);
    let display =
        glium::Display::new(window, context, &events_loop).expect("failed to create display");

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

    // Debug Resources (for displaying shadow map)
    let debug_vertex_buffer = glium::VertexBuffer::new(
        &display,
        &[
            DebugVertex::new([0.25, -1.0], [0.0, 0.0]),
            DebugVertex::new([0.25, -0.25], [0.0, 1.0]),
            DebugVertex::new([1.0, -0.25], [1.0, 1.0]),
            DebugVertex::new([1.0, -1.0], [1.0, 0.0]),
        ],
    )
    .unwrap();
    let debug_index_buffer = glium::IndexBuffer::new(
        &display,
        glium::index::PrimitiveType::TrianglesList,
        &[0u16, 1, 2, 0, 2, 3],
    )
    .unwrap();

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

    let shadow_debug_program = program!(&display,
        140 => {
            vertex: include_str!("shaders/shadows_debug.vs"),
            fragment: include_str!("shaders/shadows_debug.fs"),
        },
    )
    .expect("failed to compile shaders");

    let shadow_texture = glium::texture::DepthTexture2d::empty(&display, 1600, 900).unwrap();

    let (view_w, view_h) = display.get_framebuffer_dimensions();
    let aspect = view_w as f32 / view_h as f32;
    let projection = cgmath::perspective(cgmath::Deg(80.0), aspect, 0.01, 100000.0);

    const START_VIEW_HEIGHT: f32 = 55f32;

    let mut rotation = 0f32;
    let mut show_normals = false;

    let mut camera = Vec3::new(0., 0., START_VIEW_HEIGHT);
    let mut look_at = Vec3::new(10., 10., START_VIEW_HEIGHT);

    let mut rotation_paused = true;
    let mut key_rotate_left = false;
    let mut key_rotate_right = false;
    let mut key_go_forward = false;
    let mut key_go_backward = false;
    let mut key_go_up = false;
    let mut key_go_down = false;

    let start_time = std::time::Instant::now();

    events_loop.run(move |event, _, control_flow| match event {
        glutin::event::Event::WindowEvent { event, .. } => match event {
            WindowEvent::CloseRequested => *control_flow = glutin::event_loop::ControlFlow::Exit,
            WindowEvent::KeyboardInput {
                input:
                    KeyboardInput {
                        state: ElementState::Pressed,
                        virtual_keycode,
                        ..
                    },
                ..
            } => {
                println!("{:?}", virtual_keycode);
                match virtual_keycode {
                    Some(VirtualKeyCode::Escape) => {
                        *control_flow = glutin::event_loop::ControlFlow::Exit
                    }
                    Some(VirtualKeyCode::N) => {
                        show_normals = !show_normals;
                    }
                    Some(VirtualKeyCode::Space) => {
                        rotation_paused = !rotation_paused;
                    }
                    Some(VirtualKeyCode::E) => {
                        if key_go_down {
                            return;
                        }
                        key_go_up = true;
                    }
                    Some(VirtualKeyCode::Q) => {
                        if key_go_up {
                            return;
                        }
                        key_go_down = true;
                    }
                    Some(VirtualKeyCode::W) => {
                        if !rotation_paused {
                            return;
                        }
                        key_go_forward = true;
                    }
                    Some(VirtualKeyCode::S) => {
                        if !rotation_paused {
                            return;
                        }
                        key_go_backward = true;
                    }
                    Some(VirtualKeyCode::A) => {
                        if !rotation_paused {
                            return;
                        }
                        key_rotate_left = true;
                    }
                    Some(VirtualKeyCode::D) => {
                        if !rotation_paused {
                            return;
                        }
                        key_rotate_right = true;
                    }
                    Some(VirtualKeyCode::T) => {
                        camera = Vec3::new(0., 0., START_VIEW_HEIGHT);
                        look_at = Vec3::new(10., 10., START_VIEW_HEIGHT);
                    }
                    _ => (),
                }
            }
            WindowEvent::KeyboardInput {
                input:
                    KeyboardInput {
                        state: ElementState::Released,
                        virtual_keycode,
                        ..
                    },
                ..
            } => match virtual_keycode {
                Some(VirtualKeyCode::E) => {
                    key_go_up = false;
                }
                Some(VirtualKeyCode::Q) => {
                    key_go_down = false;
                }
                Some(VirtualKeyCode::W) => {
                    key_go_forward = false;
                }
                Some(VirtualKeyCode::S) => {
                    key_go_backward = false;
                }
                Some(VirtualKeyCode::A) => {
                    key_rotate_left = false;
                }
                Some(VirtualKeyCode::D) => {
                    key_rotate_right = false;
                }
                _ => (),
            },
            _ => (),
        },
        glutin::event::Event::MainEventsCleared => display.gl_window().window().request_redraw(),
        glutin::event::Event::RedrawRequested(_) => {
            let ms_since_start = start_time.elapsed().as_millis() as f32;

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
            let inv_translation = Matrix4::from_translation(vec3(-camera.x, -camera.y, -camera.z));

            let used_look_at = translation
                * rotation
                * inv_translation
                * vec4(look_at.x, look_at.y, look_at.z, 1.0);

            if key_go_forward {
                let norm_forward = (Vec3::new(used_look_at.x, used_look_at.y, used_look_at.z)
                    - camera)
                    .normalize();
                camera += norm_forward * 10.;
                look_at += norm_forward * 10.;
            }
            if key_go_backward {
                let norm_forward = (Vec3::new(used_look_at.x, used_look_at.y, used_look_at.z)
                    - camera)
                    .normalize();
                camera -= norm_forward * 10.;
                look_at -= norm_forward * 10.;
            }

            let view = Matrix4::look_at_rh(
                Point3::new(camera.x, camera.y, camera.z),
                Point3::new(used_look_at.x, used_look_at.y, used_look_at.z),
                vec3(0.0, 0.0, 1.0),
            );

            let tick = ms_since_start / 350.;
            let light_pos = vec3(tick.cos() * 400., tick.sin() * 800., 200.0);

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
                draw_params.backface_culling = glium::BackfaceCullingMode::CullClockwise;

                // Write depth to shadow map texture
                let mut framebuffer =
                    glium::framebuffer::SimpleFrameBuffer::depth_only(&display, &shadow_texture)
                        .unwrap();
                framebuffer.clear_color_and_depth((1.0, 1.0, 1.0, 1.0), 1.);

                let uniforms = uniform! {
                    depth_mvp: Into::<[[f32; 4]; 4]>::into(projection*view),
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

            let mut surface = display.draw();
            surface.clear_color_and_depth((0.011, 0.0089, 0.1622, 0.0), 1.0);

            // Render from camera POV
            {
                let uniforms = uniform! {
                    model_view_projection: Into::<[[f32; 4]; 4]>::into(projection  * view),
                    show_normals: if show_normals { 1f32 } else { 0f32 },
                    max_distance: max_light_distance,
                    light_pos: [light_pos.x, light_pos.y, light_pos.z],
                };

                let polygon_mode = PolygonMode::Fill;

                let draw_parameters = glium::DrawParameters {
                    depth: glium::Depth {
                        test: glium::DepthTest::IfLess,
                        write: true,
                        ..Default::default()
                    },
                    point_size: Some(8.0),
                    polygon_mode,
                    backface_culling:
                        glium::draw_parameters::BackfaceCullingMode::CullCounterClockwise,
                    ..Default::default()
                };

                surface
                    .draw(
                        &vertex_buffer,
                        &index_buffer,
                        &render_program,
                        &uniforms,
                        &draw_parameters,
                    )
                    .expect("failed to draw to surface");
            }

            // Debug for shadow map
            {
                let uniforms = uniform! {
                    tex: glium::uniforms::Sampler::new(&shadow_texture)
                        .magnify_filter(glium::uniforms::MagnifySamplerFilter::Nearest)
                        .minify_filter(glium::uniforms::MinifySamplerFilter::Nearest)
                };
                surface.clear_depth(1.0);
                surface
                    .draw(
                        &debug_vertex_buffer,
                        &debug_index_buffer,
                        &shadow_debug_program,
                        &uniforms,
                        &Default::default(),
                    )
                    .unwrap();
            }

            surface.finish().expect("failed to finish rendering frame");
        }
        _ => (),
    });
}
