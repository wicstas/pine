vec3 radiance(Ray ray) {
    vec3 L = vec3(1.0f);

    for(int depth = 0; depth < 5; depth++) {
        SurfaceInteraction it;
        if(!intersect(ray, it))
            return L;
        
        L *= 0.9;
        it.n = face_same_hemisphere(it.n, -ray.d);
        ray = spawn_ray(it.p, it.n, coordinate_system(it.n) * cosine_weighted_hemisphere(next2()));
    }

    return vec3(0);
}