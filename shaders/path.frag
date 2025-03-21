#version 450 core

out vec4 out_color;
in vec2 coord;

uniform ivec2 film_size;
uniform int alpha;
layout(location = 0, rgba32f) uniform image2D image;

#include constants.frag
#include sampler.frag
#include sampling.frag
#include ray.frag
#include camera.frag
#include geometry.frag
#include integrator.frag

void main() {
  vec3 L = radiance(gen_ray(coord));
  vec3 color = (imageLoad(image, ivec2(gl_FragCoord.xy)).xyz * alpha + L) / (alpha + 1);
  out_color = vec4(color, 1.0);
}