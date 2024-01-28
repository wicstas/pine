#include <psl/iostream.h>

#include <stdio.h>

namespace psl {

void ostream::write(const char* str, size_t size) {
    fwrite(str, size, 1, stdout);
    fflush(stdout);
}

ostream cout{};

}  // namespace psl