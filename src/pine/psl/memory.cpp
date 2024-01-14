#include <pine/psl/memory.h>

#include <stdlib.h>

namespace psl {
void free(void* ptr) {
  ::free(ptr);
}

}  // namespace psl
