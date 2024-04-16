#include <psl/memory.h>
#include <psl/vector.h>

#include <stdlib.h>

namespace psl {
void free(void* ptr) {
  ::free(ptr);
}

auto allocators = psl::vector_of<pm_allocator>(global_allocator());

void push_context_allocator(pm_allocator allocator) {
  allocators.push_back(MOVE(allocator));
}
void pop_context_allocator() {
  allocators.pop_back();
  psl_check(allocators.size() != 0);
}

void* context_alloc(size_t size) {
  psl_check(allocators.size() != 0);
  return allocators.back().alloc(size);
}
void context_free(const void* ptr) {
  psl_check(allocators.size() != 0);
  allocators.back().free((void*)ptr);
}

}  // namespace psl
