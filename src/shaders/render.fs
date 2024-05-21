#version 330
uniform float show_normals;
uniform float max_distance;
uniform vec3 light_pos;

in vec3 vNormal;
in vec3 vPosition;

layout(location = 0) out vec4 color;

vec3 illuminate() {
  vec3 lightDir = vPosition - light_pos;

  float dist_to_light = min(1., length(lightDir) / max_distance);

  float light_level = max(0., dot(vNormal, normalize(lightDir))) * (1. - dist_to_light);
  return vec3(light_level);
}

void main() {
  if(show_normals > 0.5) {
    color = vec4(normalize(vNormal) * 0.5 + 0.5, 1.0);
  } else {
    color = vec4(illuminate(), 1.0);
  }
}