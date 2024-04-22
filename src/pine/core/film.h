#pragma once
#include <pine/core/atomic.h>
#include <pine/core/array.h>

namespace pine {

struct Film {
  Film() = default;
  Film(vec2i size) : pixels(size){};

  void add_sample(vec2i p_film, vec3 color, float weight = 1.0f);
  void add_sample_thread_safe(vec2i p_film, vec3 color);
  float aspect() const {
    return float(size().x) / size().y;
  }
  Film& clear();
  void finalize(float scale = 1.0f);

  vec4& operator[](vec2i p) {
    return pixels[p];
  }
  const vec4& operator[](vec2i p) const {
    return pixels[p];
  }

  vec2i size() const {
    return pixels.size();
  }
  int width() const {
    return size().x;
  }
  int height() const {
    return size().y;
  }
  vec4* data() {
    return pixels.data();
  }
  const vec4* data() const {
    return pixels.data();
  }

  Array2d<vec4> pixels;

private:
  void apply_tone_mapping();
  SpinLock spin_lock;
};

Film combine(Film a, const Film& b, float weight_a, float weight_b);

void save_film_as_image(psl::string_view filename, Film film);

void film_context(Context& ctx);

}  // namespace pine
