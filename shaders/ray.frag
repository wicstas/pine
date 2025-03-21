
struct Ray {
    vec3 o;
    float tmin;
    vec3 d;
    float tmax;
};
Ray Ray_(vec3 o, vec3 d) {
    return Ray(o, 0, d, Infinity);
}
vec3 offset_ray_origin(vec3 p, vec3 n) {
  const float origin = 1.0f / 32.0f;
  const float float_scale = 1.0f / 65536.0f;
  const float int_scale = 256.0f;

  ivec3 of_i = ivec3(int_scale * n);
  return vec3(
          abs(p.x) < origin ? p.x + n.x * float_scale : intBitsToFloat(floatBitsToInt(p.x) + (p.x < 0 ? -of_i.x : of_i.x)),
          abs(p.y) < origin ? p.y + n.y * float_scale : intBitsToFloat(floatBitsToInt(p.y) + (p.y < 0 ? -of_i.y : of_i.y)),
          abs(p.z) < origin ? p.z + n.z * float_scale : intBitsToFloat(floatBitsToInt(p.z) + (p.z < 0 ? -of_i.z : of_i.z))
          );
}

Ray spawn_ray(vec3 p, vec3 n, vec3 wo) {
  Ray ray;
  ray.o = offset_ray_origin(p, face_same_hemisphere(n, wo));
  ray.d = wo;
  ray.tmin = 0.0f;
  ray.tmax = Infinity;
  return ray;
}