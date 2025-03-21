vec3 face_same_hemisphere(vec3 v, vec3 ref) { 
    return dot(v, ref) < 0 ? -v : v; 
}

mat3 coordinate_system(vec3 n) {
  mat3 m;
  m[2] = n;
  if (abs(n.x) > abs(n.y))
    m[0] = normalize(cross(n, vec3(0, 1, 0)));
  else
    m[0] = normalize(cross(n, vec3(1, 0, 0)));
  m[1] = cross(n, m[0]);
  return m;
}

vec2 sample_disk_concentric(vec2 u) {
  u = vec2(u.x * 2 - 1.0f, u.y * 2 - 1.0f);
  float theta, r;
  if (abs(u.x) > abs(u.y)) {
    r = u.x;
    theta = Pi / 4.0f * u.y / u.x;
  } else {
    r = u.y;
    theta = Pi / 2.0f - Pi / 4.0f * (u.x / u.y);
  }
  return r * vec2(cos(theta), sin(theta));
}

vec3 cosine_weighted_hemisphere(vec2 u) {
  vec2 d = sample_disk_concentric(u);
  float z = sqrt(max(1.0f - d.x * d.x - d.y * d.y, 0.0f));
  return vec3(d.x, d.y, z);
}