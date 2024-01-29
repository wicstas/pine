#include <pine/core/context.h>
#include <pine/core/image.h>

namespace pine {

void image_context(Context& ctx) {
  ctx.type<psl::shared_ptr<Image>>("Image");
}

}  // namespace pine
