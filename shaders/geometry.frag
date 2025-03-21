struct Material {
    vec4 color;
};

struct SurfaceInteraction {
  vec3 p;
  vec3 n;
  vec2 uv;
  Material material;
};

struct Mesh {
    int n_faces;
    int face_index_offset;
    Material material;
};

uniform int n_meshes;

layout(std430, binding = 0) buffer SSBO0 {
    Mesh meshes[];
};
layout(std430, binding = 1) buffer SSBO1 {
    ivec4 faces[];
};
layout(std430, binding = 2) buffer SSBO2 {
    vec4 vertices[];
};
layout(std430, binding = 3) buffer SSBO3 {
    vec4 normals[];
};
layout(std430, binding = 4) buffer SSBO4 {
    vec2 texcoords[];
};

bool intersect(inout Ray ray, vec3 v0, vec3 v1, vec3 v2) {
  vec3 E1 = v1 - v0;
  vec3 E2 = v2 - v0;
  vec3 T = ray.o - v0;
  vec3 P = cross(ray.d, E2);
  vec3 Q = cross(T, E1);
  float D = dot(P, E1);
  if (D == 0.0f) return false;
  float t = dot(Q, E2) / D;
  if (t <= ray.tmin || t >= ray.tmax) return false;
  float u = dot(P, T) / D;
  if (u < 0.0f || u > 1.0f) return false;
  float v = dot(Q, ray.d) / D;
  if (v < 0.0f || u + v > 1.0f) return false;
  ray.tmax = t;
  return true;
}

ivec3 load_face(int i) { return ivec3(faces[i]); }
vec3 load_vertex(int i) { return vec3(vertices[i]); }
vec3 load_normal(int i) { return vec3(normals[i]); }
vec2 load_texcoord(int i) { return texcoords[i]; }

vec2 lerp(vec2 uv, vec2 a, vec2 b, vec2 c) { return (1 - uv[0] - uv[1]) * a + uv[0] * b + uv[1] * c; }
vec3 lerp(vec2 uv, vec3 a, vec3 b, vec3 c) { return (1 - uv[0] - uv[1]) * a + uv[0] * b + uv[1] * c; }

bool intersect(inout Ray ray, out SurfaceInteraction it, Mesh mesh) {
    int hit_index = -1;
    for(int i = 0; i < min(mesh.n_faces, 10); i++) {
        ivec3 face = load_face(mesh.face_index_offset + i);
        if(intersect(ray, load_vertex(face[0]), load_vertex(face[1]), load_vertex(face[2])))
            hit_index = i;
    }

    if(hit_index != -1) {
        ivec3 face = load_face(mesh.face_index_offset + hit_index);
        vec3 v0 = load_vertex(face[0]), v1 = load_vertex(face[1]), v2 = load_vertex(face[2]);
        vec3 e1 = v1 - v0;
        vec3 e2 = v2 - v0;
        vec3 p = ray.o + ray.tmax * ray.d;
        it.n = cross(e1, e2);
        mat3 tbn = inverse(mat3(e1, e2, it.n));
        it.uv = vec2(tbn * (p - v0));
        it.p = lerp(it.uv, v0, v1, v2);
        it.n = lerp(it.uv, load_normal(face[0]), load_normal(face[1]), load_normal(face[2]));
        it.uv = lerp(it.uv, load_texcoord(face[0]), load_texcoord(face[1]), load_texcoord(face[2]));
        it.material = mesh.material;

        return true;
    } else {
        return false;
    }
}

bool intersect(inout Ray ray, out SurfaceInteraction it) {
    bool hit = false;
    for(int i = 0; i < n_meshes; i++)
        if(intersect(ray, it, meshes[i]))
            hit = true;
    return hit;
}