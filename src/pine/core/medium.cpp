#include <pine/core/sampling.h>
#include <pine/core/medium.h>
#include <pine/core/context.h>

namespace pine {

void medium_context(Context& context) {
  context.type<HomogeneousMedium>("HomogeneousMedium")
      .ctor<float>()
      .member("density", &HomogeneousMedium::density);
  context.type<Medium>("Medium").ctor_variant<HomogeneousMedium>();
}

}  // namespace pine
