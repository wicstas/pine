#pragma once
#include <pine/core/context.h>
#include <pine/core/scene.h>

namespace pine {

Context get_default_context();
void interpret(Context& context, psl::string source);

}  // namespace pine