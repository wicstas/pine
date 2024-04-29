#include <pine/core/context.h>
#include <pine/core/fileio.h>
#include <pine/core/color.h>
#include <pine/core/film.h>

namespace pine {

void save_film_as_image(psl::string_view filename, Film film) {
  film.finalize();
  save_image(psl::string(filename), film.pixels, true);
}

Film& Film::clear() {
  for (auto& pixel : pixels)
    pixel = {};
  return *this;
}
void Film::finalize(float scale) {
  for (auto& pixel : pixels) {
    pixel *= scale;
    pixel.w = 1;
  }
  apply_tone_mapping();
}
void Film::add_sample(vec2i p_film, vec3 color, float weight) {
  if (weight == 0.0f)
    return;
  DCHECK_RANGE(p_film.x, 0, size().x - 1);
  DCHECK_RANGE(p_film.y, 0, size().y - 1);
  auto& pixel = pixels[p_film];
  const auto alpha = pixel.w + weight;
  pixel = vec4((vec3(pixel) * pixel.w + color * weight) / alpha, alpha);
}
void Film::add_sample_thread_safe(vec2i p_film, vec3 color) {
  DCHECK_RANGE(p_film.x, 0, size().x - 1);
  DCHECK_RANGE(p_film.y, 0, size().y - 1);
  auto& pixel = pixels[p_film];
  spin_lock.lock();
  const auto alpha = pixel.w + 1;
  pixel = vec4((vec3(pixel) * pixel.w + color) / alpha, alpha);
  spin_lock.unlock();
}
void Film::apply_tone_mapping() {
  for (auto& pixel : pixels)
    pixel = vec4{ACES(vec3(pixel)), pixel.w};
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

void visualize(Film& film) {
  auto max_value = 0.0f;
  auto min_value = float_max;
  for_2d(film.size(), [&](vec2i p) {
    auto x = film.pixels[p].x;
    if (x == 0.0f)
      return;
    max_value = psl::max(max_value, x);
    min_value = psl::min(min_value, x);
  });
  for_2d(film.size(), [&](vec2i p) {
    auto x = film.pixels[p].x;
    if (x == 0.0f)
      return;
    film.pixels[p] = vec4(color_map((x - min_value) / (max_value - min_value)));
  });
}

void film_context(Context& ctx) {
  ctx.type<Film>("Film").ctor<vec2i>().member("pixels", &Film::pixels);
  ctx("=") = +[](Film& film, const Array2d3f& color) {
    CHECK_EQ(film.size(), color.size());
    for_2d(film.size(), [&](vec2i p) { film[p] = vec4(color[p]); });
  };
  ctx("=") = +[](Film& film, const Array2d4f& color) {
    CHECK_EQ(film.size(), color.size());
    for_2d(film.size(), [&](vec2i p) { film[p] = color[p]; });
  };
  ctx("=") = +[](Film& film, const Image& color) {
    CHECK_EQ(film.size(), color.size());
    for_2d(film.size(), [&](vec2i p) { film[p] = vec4(color[p]); });
  };
  ctx("=") = +[](Film& film, const psl::shared_ptr<Image>& color) {
    CHECK_EQ(film.size(), color->size());
    for_2d(film.size(), [&](vec2i p) { film[p] = vec4((*color)[p]); });
  };
  ctx("save") = tag<void, Film&, psl::string_view>(
      [](Film& film, psl::string_view filename) { save_film_as_image(filename, film); });
}

}  // namespace pine