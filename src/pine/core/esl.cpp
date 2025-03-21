#include <pine/core/fileio.h>
#include <pine/core/esl.h>
#include <pine/core/log.h>

#include <psl/variant.h>

namespace pine {

psl::string load_esl(psl::string filename) {
  auto foldername = filename.substr(filename.begin(), psl::find_last_of(filename, '/'));
  auto source = read_string_file(filename);
  while (true) {
    auto it = psl::find_subrange(source, psl::string("#include"));
    if (it == source.end()) break;
    auto eit = psl::find_if(psl::range(it, source.end()),
                            psl::isnewline);  // Ok if `eit` == source.end()
    auto include_filename = psl::string(it + 9, eit);
    it = source.erase_range(it, eit);
    source.insert_range(it, read_string_file(foldername + "/" + include_filename));
  }

  write_string_file(filename + ".esl", source);
  return source;
}

}  // namespace pine