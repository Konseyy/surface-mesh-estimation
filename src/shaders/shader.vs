#version 330
uniform mat4 model_view_projection;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

out vec3 vNormal;
out vec3 vPosition;

void main() {
  gl_Position = model_view_projection * vec4(position, 1.0);
  vNormal = normal;
  vPosition = position;
}