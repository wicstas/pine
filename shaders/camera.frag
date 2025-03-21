struct Camera {
  vec3 position;
  mat3 c2w;
  vec2 fov2d;
};
uniform Camera cam;

Ray gen_ray(vec2 p_film) {
    vec2 pc = (p_film - vec2(0.5f)) * 2 * cam.fov2d;
    return Ray_(cam.position, normalize(cam.c2w * vec3(pc, 1.0f)));
}