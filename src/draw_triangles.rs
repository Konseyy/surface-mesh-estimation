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

use crate::{marching_cubes::Triangle, utils::Vec3};

#[derive(Copy, Clone)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

implement_vertex!(Vertex, position, normal);

pub fn draw_triangles(triangles: Vec<Triangle>) {
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

    let program = program!(&display,
        330 => {
            vertex: include_str!("shaders/shader.vs"),
            fragment: include_str!("shaders/shader.fs"),
        },
    )
    .expect("failed to compile shaders");

    let (view_w, view_h) = display.get_framebuffer_dimensions();
    let aspect = view_w as f32 / view_h as f32;
    let projection = cgmath::perspective(cgmath::Deg(80.0), aspect, 0.01, 100000.0);

    let mut rotation = 0f32;
    let mut view_height = 55f32;
    let mut show_normals = false;
    let mut camera = Vec3::new(100., 240., view_height);

    let mut rotation_paused = true;
    let mut key_rotate_left = false;
    let mut key_rotate_right = false;
    let mut key_go_forward = false;
    let mut key_go_backward = false;
    let mut key_go_up = false;
    let mut key_go_down = false;

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
            let mut surface = display.draw();
            surface.clear_color_and_depth((0.011, 0.0089, 0.1622, 0.0), 1.0);

            if !rotation_paused {
                rotation += 0.6;
            }
            if key_rotate_left {
                rotation -= 1.0;
            }
            if key_rotate_right {
                rotation += 1.0;
            }
            if key_go_up {
                view_height += 10.0;
            }
            if key_go_down {
                view_height -= 10.0;
            }

            let rotation = Matrix4::from_angle_x(cgmath::Deg(0.0))
                * Matrix4::from_angle_y(cgmath::Deg(0.0 + rotation))
                * Matrix4::from_angle_z(cgmath::Deg(0.0));

            if key_go_forward {
                let norm_forward = camera.normalize();
                let dir =
                    rotation * vec4(norm_forward.x, norm_forward.y, norm_forward.z, 1.) * 10.0;
                camera += Vec3::new(dir.x, dir.y, 0.0);
            }
            if key_go_backward {
                let norm_backward = camera.normalize();
                let dir =
                    rotation * vec4(norm_backward.x, norm_backward.y, norm_backward.z, 1.) * 10.0;
                camera -= Vec3::new(dir.x, dir.y, 0.0);
            }

            let view = Matrix4::look_at_rh(
                Point3::new(camera.x, camera.y, view_height),
                Point3::new(0.0, 0.0, view_height),
                vec3(0.0, 0.0, 1.0),
            );

            let uniforms = uniform! {
                model_view_projection: Into::<[[f32; 4]; 4]>::into(projection * rotation * view),
                show_normals: if show_normals { 1f32 } else { 0f32 },
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
                backface_culling: glium::draw_parameters::BackfaceCullingMode::CullCounterClockwise,
                ..Default::default()
            };

            surface
                .draw(
                    &vertex_buffer,
                    &index_buffer,
                    &program,
                    &uniforms,
                    &draw_parameters,
                )
                .expect("failed to draw to surface");

            surface.finish().expect("failed to finish rendering frame");
        }
        _ => (),
    });
}
