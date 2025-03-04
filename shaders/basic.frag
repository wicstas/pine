#version 450 core

out vec4 out_color;

struct Disc {
    vec3 p;
    vec3 n;
    float r;
    vec3 color;
};

layout(std430, binding = 0) buffer SSBO {
    Disc discs[];
};

uniform int n_discs;

layout(rgba32f, binding = 0) uniform image2D pixels;

void main() {
    out_color = imageLoad(pixels, ivec2(gl_FragCoord.xy));
}