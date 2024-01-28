#include <pine/core/context.h>
#include <pine/core/array.h>

namespace pine {

void array2d_context(Context &context) {
  context.type<Array2D2f>("Array2D2f")
      .ctor<vec2i>()
      .method("[]", overloaded(&Array2D2f::operator[]));
  context.type<Array2D3f>("Array2D3f")
      .ctor<vec2i>()
      .method("[]", overloaded(&Array2D3f::operator[]));
}

}  // namespace pine
