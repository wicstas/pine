#include <pine/core/log.h>
#include <pine/core/fileio.h>

#include <contrib/tiny_gltf.h>

#include <unistd.h>

using namespace pine;

int main(int argc, char* argv[]) {
  try {
    auto op = psl::string(argv[1]);
    if (op == "pack") {
      CHECK_EQ(argc, 3);
      auto path = psl::string(argv[2]);
      auto dir = psl::until_last_of(path, '/');
      auto output_path = psl::until_last_of(path, '.') + ".pineb";
      auto old_basedir = get_current_dir_name();
      Log("-----------------------------------------------------------");
      Log("Packing...\n", path, "\nInto:\n", output_path, "\n");
      chdir(dir.c_str());
      auto fs = read_folder("./");
      chdir(old_basedir);
      free(old_basedir);
      auto bytes = serialize(fs);
      write_binary_file(output_path, bytes.data(), bytes.byte_size());
    }

  } catch (const pine::Exception& e) {
    Log(e.what());
  }
}