#include <pine/core/context.h>
#include <pine/core/array.h>

namespace pine {

void array2d_context(Context &context) {
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
}

}  // namespace pine
