 #version 330 core
layout(location = 0) out float fragmentdepth;
in float depth;
void main() {
  fragmentdepth = gl_FragCoord.z;

}