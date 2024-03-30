#include <pine/core/fileio.h>

using namespace pine;

int main(int argc, char* argv[]) {
  auto image = load_image(argv[1]);
  save_image(argv[2], *image);
}