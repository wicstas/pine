#version 450 core

out vec4 out_color;

layout(binding = 0) uniform sampler2D Tp;
layout(binding = 1) uniform sampler2D Tn;
layout(binding = 2) uniform sampler2D Tcd;
layout(binding = 3) uniform sampler2D Tdirect;
uniform ivec2 film_size;

uniform int n_discs;
layout(std430, binding = 0) buffer SSBO {
    float discs[];
};

const int mr_x = 8;
const int mr_y = 8;

const float Pi = 3.1415927;

// Helpers
float sqr(float x) {
    return x * x;
}
mat3 to_local(vec3 n) {
  vec3 t;
  if (abs(n.x) > abs(n.y))
    t = normalize(cross(n, vec3(0, 1, 0)));
  else
    t = normalize(cross(n, vec3(1, 0, 0)));
  return inverse(mat3(t, cross(n, t), n));
}
float radical_inverse(int a) {
  const int base = 2;
  float invBase = 1.0 / base, invBaseN = 1.0;
  uint reversedDigits = 0;
  while (a != 0) {
    int next = a >> 1;
    int digits = a - (next << 1);
    reversedDigits = (reversedDigits << 1) + digits;
    invBaseN *= invBase;
    a = next;
  }

  return min(reversedDigits * invBaseN, 1 - 1e-9);
}

void main() {
    vec2 coord = gl_FragCoord.xy / film_size;
    vec3 position = texture(Tp, coord).xyz;
    vec3 normal = texture(Tn, coord).xyz;
    vec3 cd = texture(Tcd, coord).xyz;
    vec3 direct = texture(Tdirect, coord).xyz;

    mat3 w2n = to_local(normal);

    vec4 color[mr_x][mr_y];
    for(int x = 0; x < mr_x; x++)
        for(int y = 0; y < mr_y; y++)
            color[x][y] = vec4(0, 0, 0, 1e+10);

    for(int i = 0; i < n_discs; i++) {
    vec3 Dp = vec3(discs[i*10+0],discs[i*10+1],discs[i*10+2]);
    vec3 Dn = vec3(discs[i*10+3],discs[i*10+4],discs[i*10+5]);
    float Dr = discs[i*10+6];
    vec3 Dc = vec3(discs[i*10+7],discs[i*10+8],discs[i*10+9]);
    vec3 p = normalize(w2n * (Dp - position));
    if(p.z <= 0)
        continue;
    float phi = (atan(p.y, p.x) / Pi / 2) + 0.49f;
    float theta = sqrt(p.x*p.x+p.y*p.y);
    int px = int(phi * mr_x);
    int py = int(theta * mr_y);
     
    float depth = length(Dp - position);
    if (depth < color[px][py].w)
        color[px][py] = vec4(Dc, depth);
    }

    vec3 integral = vec3(0);
    for(int x = 0; x < mr_x; x++)
        for(int y = 0; y < mr_y; y++)
            integral += color[x][y].xyz;
    integral /= mr_x * mr_y;
    
    out_color = sqrt(vec4(direct + cd * integral, 1));
}