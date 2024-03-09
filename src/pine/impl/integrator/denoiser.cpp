#include <pine/impl/integrator/denoiser.h>
#include <pine/core/parallel.h>
#include <pine/core/denoise.h>
#include <pine/core/scene.h>
#include <pine/core/color.h>

namespace pine {

void DenoiseIntegrator::render(Scene& scene) {
  RTIntegrator::render(scene);
  auto& film = scene.camera.film();
  auto color = Array2d3f::from(film.pixels);
  auto albedo = Array2d3f(film.size());
  auto normal = Array2d3f(film.size());

  parallel_for(film.size(), [&](vec2i p) {
    auto p_film = (p + vec2(0.5f)) / film.size();
    auto ray = scene.camera.gen_ray(p_film, vec2(0.5f));
    auto it = Interaction();
    if (intersect(ray, it)) {
      albedo[p] = it.material()->albedo({it.p, it.n, it.uv});
      normal[p] = it.n;
    }
  });

  denoise(DenoiseQuality::High, color, color, albedo, normal);
  film.pixels = Array2d4f::from(color);
}

}  // namespace pine