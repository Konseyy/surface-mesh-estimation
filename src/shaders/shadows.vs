#version 330 core
layout(location = 0) in vec3 position;
uniform mat4 depth_mvp;

void main() {
  vec4 pos = depth_mvp * vec4(position, 1.);

  gl_Position = pos;
}