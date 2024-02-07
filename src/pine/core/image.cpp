#include <pine/core/context.h>
#include <pine/core/image.h>

namespace pine {

void image_context(Context& ctx) {
  ctx.type<psl::shared_ptr<Image>>("Image")
      .converter<psl::string>([&ctx](psl::string_view filename) {
        return ctx.call<psl::shared_ptr<Image>>("load_image", filename);
      })
      .converter<vec3, vec3i>(+[](vec3 color) {
        auto pixels = Array2d<vec3>({1, 1});
        pixels[{0, 0}] = color;
        return psl::make_shared<Image>(pixels);
      });
}

}  // namespace pine
