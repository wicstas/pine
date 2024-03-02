#include <pine/core/context.h>
#include <pine/core/image.h>

namespace pine {

float mse(const Image& a, const Image& b) {
  CHECK_EQ(a.size(), b.size());
  auto error = 0.0;
  for_2d(a.size(), [&](vec2i p) { error += average(psl::sqr(a[p] - b[p])); });
  return error / area(a.size());
}
float rmse(const Image& ref, const Image& b) {
  CHECK_EQ(ref.size(), b.size());
  auto error = 0.0;
  for_2d(ref.size(),
         [&](vec2i p) { error += average(psl::sqr((ref[p] - b[p]) / max(ref[p], vec3(1e-3f)))); });
  return error / area(ref.size());
}

void image_context(Context& ctx) {
  ctx.type<Image>("Image").ctor_variant<Array2d<vec3u8>, Array2d<vec3>, Array2d<vec4>>();
  ctx.type<psl::shared_ptr<Image>>("ImagePtr")
      .converter<psl::string>([&ctx](psl::string_view filename) {
        return ctx.call<psl::shared_ptr<Image>>("load_image", filename);
      })
      .converter<vec3, vec3i>(+[](vec3 color) {
        auto pixels = Array2d<vec3>({1, 1});
        pixels[{0, 0}] = color;
        return psl::make_shared<Image>(pixels);
      })
      .method(
          "size", +[](const psl::shared_ptr<Image>& image) { return image->size(); })
      .method(
          "[]", +[](const psl::shared_ptr<Image>& image, vec2i p) { return (*image)[p]; });
  ctx("mse") = mse;
  ctx("mse") = +[](psl::shared_ptr<Image> a, psl::shared_ptr<Image> b) { return mse(*a, *b); };
  ctx("rmse") = rmse;
  ctx("rmse") =
      +[](psl::shared_ptr<Image> ref, psl::shared_ptr<Image> b) { return rmse(*ref, *b); };
}

}  // namespace pine
