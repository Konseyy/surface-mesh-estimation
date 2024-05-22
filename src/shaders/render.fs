#version 330 core
uniform float show_normals;
uniform float max_distance;
uniform samplerCube shadow_map;
uniform vec3 light_pos;
uniform float far_plane;

in vec3 vNormal;
in vec3 vPosition;

layout(location = 0) out vec4 color;

vec3 illuminate() {
  vec3 lightDir = vPosition - light_pos;

  float dist_to_light = min(1., length(lightDir) / max_distance);

  float light_level = max(0., dot(vNormal, normalize(lightDir))) * (1. - dist_to_light);
  return vec3(light_level);
}

const float EPSILON = 0.01;

void main() {
  vec4 shadow_map_col = textureCube(shadow_map, vPosition - light_pos);
  color = vec4(shadow_map_col.rgb, 1.0);
  if(show_normals < .5)
    return;

  float shadow_depth = shadow_map_col.r;
  float curr_depth = length(vPosition - light_pos) / far_plane;

  vec3 c = illuminate();

  if(shadow_depth > curr_depth - EPSILON) {
    color = vec4(c, 1.0);
  } else {
    color = vec4(c * .2, 1.0);
  }
  return;

  color = vec4(vec3(shadow_depth), 1.);
  return;
  if(show_normals > 0.5) {
    color = vec4(normalize(vNormal) * 0.5 + 0.5, 1.0);
  } else {
    color = vec4(illuminate(), 1.0);
  }
}