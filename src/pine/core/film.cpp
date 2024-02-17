#include <pine/core/context.h>
#include <pine/core/fileio.h>
#include <pine/core/color.h>
#include <pine/core/film.h>

namespace pine {

void save_film_as_image(psl::string_view filename, Film film) {
  film.finalize();
  save_image(psl::string(filename), film.pixels);
}

void Film::clear() {
  for (auto& pixel : pixels)
    pixel = {};
}
void Film::finalize(float scale) {
  for (auto& pixel : pixels) {
    pixel *= scale;
    pixel.w = 1;
  }
  apply_tone_mapping();
  apply_gamma_correction();
}
void Film::scale(float factor) {
  for (auto& pixel : pixels) {
    pixel *= factor;
    pixel.w = 1;
  }
}
void Film::offset(vec3 factor) {
  for (auto& pixel : pixels) {
    pixel += vec4{factor, 0.0f};
  }
}
void Film::add_sample(vec2i p_film, vec3 color) {
  DCHECK_RANGE(p_film.x, 0, size().x - 1);
  DCHECK_RANGE(p_film.y, 0, size().y - 1);
  p_film.y = size().y - 1 - p_film.y;
  auto& pixel = pixels[p_film];
  const auto alpha = pixel.w + 1;
  pixel = vec4((vec3(pixel) * pixel.w + color) / alpha, alpha);
}
void Film::add_sample_thread_safe(vec2i p_film, vec3 color) {
  p_film = clamp(p_film, vec2i{0}, size() - vec2i{1});
  p_film.y = size().y - 1 - p_film.y;
  auto& pixel = pixels[p_film];
  spin_lock.lock();
  const auto alpha = pixel.w + 1;
  pixel = vec4((vec3(pixel) * pixel.w + color) / alpha, alpha);
  spin_lock.unlock();
}
void Film::apply_tone_mapping() {
  for (auto& pixel : pixels)
    pixel = vec4{uncharted2_filmic(vec3{pixel}), pixel.w};
}
void Film::apply_gamma_correction() {
  for (auto& pixel : pixels)
    pixel = vec4{pow(vec3{pixel}, 1.0f / 2.2f), pixel.w};
}

Film combine(Film a, const Film& b, float weight_a, float weight_b) {
  auto index = size_t{0};
  CHECK_EQ(a.size(), b.size());
  CHECK(weight_a + weight_b != 0.0f);
  for (auto& pixel : a.pixels) {
    pixel = (weight_a * pixel + weight_b * b.pixels.data()[index]) / (weight_a + weight_b);
    index++;
  }
  return a;
}

void film_context(Context& ctx) {
  ctx.type<Film>("Film")
      .ctor<vec2i>()
      .method("scale", &Film::scale)
      .method("offset", &Film::offset);
  ctx("save") = tag<void, psl::string, Film>(save_film_as_image);
}

}  // namespace pine