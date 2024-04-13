#include <pine/core/parallel.h>
#include <pine/core/context.h>
#include <pine/core/array.h>

#include <psl/function.h>
#include <psl/vector.h>

namespace pine {

void array2d_context(Context& context) {
  context.type<psl::vector<int>>("list_i").ctor<int>().method(
      "[]", +[](psl::vector<int>& list, int i) -> int& { return list[i]; });
  context.type<psl::vector<float>>("list_f").ctor<int>().method(
      "[]", +[](psl::vector<float>& list, int i) -> float& { return list[i]; });
  context.type<Array2df>("Array2df")
      .ctor<vec2i>()
      .method("size", &Array2df::size)
      .method("[]", overloaded(&Array2df::operator[]));
  context.type<Array2d2f>("Array2d2f")
      .ctor<vec2i>()
      .method("size", &Array2d2f::size)
      .method("[]", overloaded(&Array2d2f::operator[]));
  context.type<Array2d3f>("Array2d3f")
      .ctor<vec2i>()
      .method("size", &Array2d3f::size)
      .method("[]", overloaded(&Array2d3f::operator[]));
  context.type<Array2d4f>("Array2d4f")
      .ctor<vec2i>()
      .method("size", &Array2d4f::size)
      .method("[]", overloaded(&Array2d4f::operator[]));
  context.type<Array2d3u8>("Array2d3u8").ctor<vec2i>();
  context.type<Array2d4u8>("Array2d4u8").ctor<vec2i>();

  context("draw") = +[](vec2i size, psl::function<vec3(vec2)> fragment) {
    auto image = Array2d3f(size);
    parallel_for(size, [&](vec2i p) { image[p] = fragment((p + vec2(0.5f)) / size); });
    return image;
  };
}

}  // namespace pine
