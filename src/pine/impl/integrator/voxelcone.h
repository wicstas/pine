#include <pine/core/integrator.h>

namespace pine {

class VoxelConeIntegrator : public RTIntegrator {
public:
  using RTIntegrator::RTIntegrator;
  void render(Scene& scene) override;
};

}  // namespace pine
