#include <pine/core/interaction.h>
#include <pine/core/geometry.h>

namespace pine {

const Material* SurfaceInteraction::material() const {
  DCHECK(geometry);
  DCHECK(geometry->material.get());
  return geometry->material.get();
}

}  // namespace pine
