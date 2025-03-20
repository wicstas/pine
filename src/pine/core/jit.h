#pragma once
#include <pine/core/context.h>

namespace pine {

void jit_interpret(Context& ctx, psl::string source);

}  // namespace pine