#include <pine/core/profiler.h>
#include <pine/core/denoise.h>

// #include <OpenImageDenoise/oidn.hpp>

namespace pine {

void denoise(DenoiseQuality quality, Array2d<vec3>& output, const Array2d<vec3>& color,
             const Array2d<vec3>* albedo, const Array2d<vec3>* normal) {
  (void)quality;
  (void)output;
  (void)color;
  (void)albedo;
  (void)normal;
  //   Profiler _("Denoise");
  //   CHECK_EQ(color.size(), output.size());
  //   if (albedo)
  //     CHECK_EQ(color.size(), albedo->size());
  //   if (normal)
  //     CHECK_EQ(color.size(), normal->size());
  //   auto width = color.size().x;
  //   auto height = color.size().y;
  //   auto device = oidn::newDevice();
  //   device.commit();

  //   auto filter = device.newFilter("RT");
  //   filter.setImage("output", (void*)output.data(), oidn::Format::Float3, width, height);
  //   filter.setImage("color", (void*)color.data(), oidn::Format::Float3, width, height);
  //   if (albedo)
  //     filter.setImage("albedo", (void*)albedo->data(), oidn::Format::Float3, width, height);
  //   if (normal)
  //     filter.setImage("normal", (void*)normal->data(), oidn::Format::Float3, width, height);
  //   if (quality == DenoiseQuality::Medium)
  //     filter.set("quality", oidn::Quality::Balanced);
  //   else
  //     filter.set("quality", oidn::Quality::High);
  //   filter.set("hdr", true);
  //   filter.commit();
  //   filter.execute();
  //   const char* error_message;
  //   if (device.getError(error_message) != oidn::Error::None)
  //     Fatal(error_message);
}

}  // namespace pine
