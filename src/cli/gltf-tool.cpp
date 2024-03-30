#include <pine/core/log.h>

#include <contrib/tiny_gltf.h>

using namespace pine;

int main(int argc, char* argv[]) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  auto filename = argv[1];
  auto ext = psl::from_last_of(filename, '.');
  auto ret = ext == "glb" ? gltf_ctx.LoadBinaryFromFile(&model, &err, &warn, filename)
                          : gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, filename);

  if (!warn.empty())
    Warning("[FileIO]", filename, ": ", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", filename, ": ", err.c_str());
  if (!ret)
    Fatal("[FIleIO]", filename, ": Unable to load");

  gltf_ctx.WriteGltfSceneToFile(&model, (psl::until_last_of(filename, '.') + "glb").c_str(), true,
                                true, true, true);
}