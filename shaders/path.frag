#version 450 core

out vec4 out_color;
in vec2 coord;

uniform ivec2 film_size;
uniform int alpha;
layout(location = 0, rgba32f) uniform image2D image;

ivec2 frag_coord = ivec2(gl_FragCoord.xy);

#include constants.frag
#include sampler.frag
#include sampling.frag
#include ray.frag
#include camera.frag
#include geometry.frag
#include integrator.frag

void main() {
  vec3 L = radiance(gen_ray(coord));
  vec3 color = (imageLoad(image, frag_coord).xyz * alpha + L) / (alpha + 1);
  imageStore(image, frag_coord, vec4(color, 1.0));
  out_color = vec4(sqrt(color), 1.0);
}