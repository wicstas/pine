#include <pine/core/parallel.h>
#include <pine/core/context.h>
#include <pine/core/array.h>

#include <psl/function.h>
#include <psl/vector.h>

namespace pine {

void array2d_context(Context& ctx) {
  ctx.type<Array2df>("Array2df")
      .layout<vec2i, psl::vector<char>>()
      .ctor<vec2i>()
      .method<&Array2df::size>("size")
      .method<overloaded(&Array2df::operator[])>("[]");
  ctx.type<Array2d2f>("Array2d2f")
      .layout<vec2i, psl::vector<char>>()
      .ctor<vec2i>()
      .method<&Array2d2f::size>("size")
      .method<overloaded(&Array2d2f::operator[])>("[]");
  ctx.type<Array2d3f>("Array2d3f")
      .layout<vec2i, psl::vector<char>>()
      .ctor<vec2i>()
      .method<&Array2d3f::size>("size")
      .method<overloaded(&Array2d3f::operator[])>("[]");
  ctx.type<Array2d4f>("Array2d4f")
      .layout<vec2i, psl::vector<char>>()
      .ctor<vec2i>()
      .method<&Array2d4f::size>("size")
      .method<overloaded(&Array2d4f::operator[])>("[]");
  ctx.type<Array2d3u8>("Array2d3u8").layout<vec2i, psl::vector<char>>().ctor<vec2i>();
  ctx.type<Array2d4u8>("Array2d4u8").layout<vec2i, psl::vector<char>>().ctor<vec2i>();

  ctx("draw") = [](vec2i size, psl::function<vec3(vec2)> fragment) {
    auto image = Array2d3f(size);
    parallel_for(size, [&](vec2i p) {
      image[p] = fragment((p + vec2(0.5f)) / size);
    });
    return image;
  };
}

}  // namespace pine
