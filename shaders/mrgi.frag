#version 450 core

out vec4 out_color;

layout(binding = 0) uniform sampler2D Tp;
layout(binding = 1) uniform sampler2D Tn;
layout(binding = 2) uniform sampler2D Tcd;
layout(binding = 3) uniform sampler2D Tdirect;
layout(binding = 4) uniform sampler2D Tdebug;
layout(binding = 5) uniform sampler2D Tdebug1;
layout(binding = 6) uniform sampler2D Tdebug2;
uniform ivec2 film_size;

uniform int n_discs;
layout(std430, binding = 0) buffer SSBO {
    float discs[];
};

const int mr_x = 16;
const int mr_y = 16;
const float Pi = 3.1415927;

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
    if(coord.x <= 1) {
      out_color = sqrt(vec4(texture(Tdirect, coord).xyz, 1));
      return;
    }
    else if (coord.x <= 2){
      coord.x -= 1;
      out_color = sqrt(vec4(texture(Tdebug, coord).xyz, 1));
      return;
    }
    else if (coord.x <= 3){
      coord.x -= 2;
      out_color = sqrt(vec4(texture(Tdebug1, coord).xyz, 1));
      return;
    }
    else if (coord.x <= 4){
      coord.x -= 3;
      out_color = sqrt(vec4(texture(Tdebug2, coord).xyz, 1));
      return;
    }
    // vec3 position = texture(Tp, coord).xyz;
    // vec3 normal = texture(Tn, coord).xyz;
    // vec3 cd = texture(Tcd, coord).xyz;
    // vec3 direct = texture(Tdirect, coord).xyz;

    // mat3 w2n = to_local(normal);

    // uint MR[mr_x][mr_y];
    // for(int x = 0; x < mr_x; x++)
    //     for(int y = 0; y < mr_y; y++)
    //       MR[x][y] = 0xffffffffu;
    

    // for(int i = 0; i < n_discs; i++) {
    // vec3 Dp = vec3(discs[i*3+0],discs[i*3+1],discs[i*3+2]);
    // vec3 p = normalize(w2n * (Dp - position));
    // if(p.z <= 0)
    //     continue;
    // float phi = (atan(p.y, p.x) / Pi / 2) + 0.5f;
    // float theta = (p.x*p.x + p.y*p.y);
    // if(phi < 0 || phi >= 1 || theta < 0 || theta >= 1) continue;
    // int px = int(phi * mr_x);
    // int py = int(theta * mr_y);
     
    // int depth = int(length(Dp - position) / 100);
    // if (depth < (MR[px][py] >> 16)) {
    //     MR[px][py] = (depth << 16) + i;
    //   }
    // }

    // for(int x = 0; x < mr_x / 2; x++)
    //     for(int y = 0; y < mr_y / 2; y++) {
    //       uint selected = min(min(MR[x*2+0][y*2+0], MR[x*2+1][y*2+0]), min(MR[x*2+0][y*2+1], MR[x*2+1][y*2+1]));
    //       if(selected == 0xffffffffu)
    //         continue;
    //       if(MR[x*2+0][y*2+0] == 0xffffffffu)
    //         MR[x*2+0][y*2+0] = selected;
    //       if(MR[x*2+1][y*2+0] == 0xffffffffu)
    //       MR[x*2+1][y*2+0] = selected;
    //       if(MR[x*2+0][y*2+1] == 0xffffffffu)
    //       MR[x*2+0][y*2+1] = selected;
    //       if(MR[x*2+1][y*2+1] == 0xffffffffu)
    //       MR[x*2+1][y*2+1] = selected;
    // }

    // vec3 integral = vec3(0);
    // for(int x = 0; x < mr_x; x++)
    //     for(int y = 0; y < mr_y; y++) {
    //       uint i = MR[x][y] & 0xffffu;
    //       uint I = n_discs*6+i*3;
    //       if(i != 0xffffu)
    //         integral += vec3(discs[I+0],discs[I+1],discs[I+2]);
    //     }
    // integral /= mr_x * mr_y;
    
    // out_color = sqrt(vec4(cd * integral, 1));
}