#include <pine/core/context.h>
#include <pine/core/fileio.h>
#include <pine/core/image.h>

namespace pine {

vec4 Image::operator[](vec2i p) const {
  return dispatch([p](auto&& x) {
    auto value = x[p];
    if constexpr (psl::SameAs<decltype(value), vec3u8>)
      return vec4(inverse_gamma_correction(value / 255.0f));
    else if constexpr (psl::SameAs<decltype(value), vec4u8>)
      return inverse_gamma_correction(value / 255.0f);
    else
      return vec4(value);
  });
}
vec4 Image::filtered(vec2 p) const {
  auto& img = *this;
  using psl::ceil;
  using psl::floor;
  using psl::fract;
  auto c0 = img[{floor(p.x), floor(p.y)}];
  auto c1 = img[{ceil(p.x), floor(p.y)}];
  auto c2 = img[{floor(p.x), ceil(p.y)}];
  auto c3 = img[{ceil(p.x), ceil(p.y)}];
  auto cy0 = lerp(fract(p.x), c0, c1);
  auto cy1 = lerp(fract(p.x), c2, c3);
  return lerp(fract(p.y), cy0, cy1);
}

float mse(const Image& a, const Image& b) {
  CHECK_EQ(a.size(), b.size());
  auto error = 0.0;
  for_2d(a.size(), [&](vec2i p) { error += average(psl::sqr(vec3(a[p] - b[p]))); });
  return error / area(a.size());
}
float rmse(const Image& ref, const Image& b) {
  CHECK_EQ(ref.size(), b.size());
  auto error = 0.0;
  for_2d(ref.size(), [&](vec2i p) {
    error += average(psl::sqr(vec3(ref[p] - b[p]) / max(vec3(ref[p]), vec3(1e-3f))));
  });
  return error / area(ref.size());
}

void image_context(Context& ctx) {
  ctx.type<Image>("Image").ctor_variant<Array2d3u8, Array2d3f, Array2d4f>();
  ctx.type<psl::shared_ptr<Image>>("ImagePtr")
      .ctor<psl::string>([](psl::string_view filename) {
        auto data = read_binary_file(filename);
        auto image = image_from(data.data(), data.size());
        if (!image)
          Fatal("Unable to load `", filename, '`');
        return psl::make_shared<Image>(*image);
      })
      .ctor<vec3>([](vec3 color) {
        auto pixels = Array2d<vec3>({1, 1});
        pixels[{0, 0}] = color;
        return psl::make_shared<Image>(pixels);
      });
  ctx("size") = [](const psl::shared_ptr<Image>& image) { return image->size(); };
  ctx("[]") = [](const psl::shared_ptr<Image>& image, vec2i p) { return (*image)[p]; };
  ctx("mse") = led<mse>;
  ctx("mse") = [](psl::shared_ptr<Image> a, psl::shared_ptr<Image> b) { return mse(*a, *b); };
  ctx("rmse") = led<rmse>;
  ctx("rmse") = [](psl::shared_ptr<Image> ref, psl::shared_ptr<Image> b) { return rmse(*ref, *b); };
}

}  // namespace pine
