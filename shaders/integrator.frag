vec3 radiance(Ray ray) {
    vec3 L = vec3(0.0f);
    vec3 beta = vec3(1.0f);

    for(int depth = 0; depth < 3; depth++) {
        SurfaceInteraction it;
        if(!intersect(ray, it)) {
            L += beta * vec3(1, 1, 1);
            break;
        }
        it.n = face_same_hemisphere(it.n, -ray.d);
        

        // SurfaceInteraction itt;
        // if(!intersect(spawn_ray(it.p, it.n, normalize(vec3(1, 1, -1))), itt))
        //     L += beta * vec3(it.material.color) * vec3(1, 1, 1) / 2;

        ray = spawn_ray(it.p, it.n, coordinate_system(it.n) * cosine_weighted_hemisphere(next2()));
        beta *= vec3(it.material.color);
    }

    return L;
}