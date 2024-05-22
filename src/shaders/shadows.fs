#version 330 core
layout(location = 0) out vec4 fragmentdepth;

in vec4 vert_pos;

uniform float far_plane;
uniform vec3 light_pos;

void main() {
  fragmentdepth = vec4(vec3(length(vert_pos.xyz - light_pos) / far_plane), 1.);

}