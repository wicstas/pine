#include <pine/core/profiler.h>
#include <pine/core/parser.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <ext/stb_image.h>
#include <ext/stb_image_write.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace pine {

psl::string read_string_file(psl::string_view filename) {
  auto file = psl::ScopedFile(filename, psl::ios::in);
  if (!file.is_open())
    Warning("Unable to open file `", filename, '`');
  size_t size = file.size();
  psl::string str;
  str.resize(size);
  file.read(&str[0], size);
  return str;
}
void write_binary_file(psl::string_view filename, const void *ptr, size_t size) {
  auto file = psl::ScopedFile(filename, psl::ios::binary | psl::ios::out);
  if (!file.is_open())
    Warning("Unable to open file `", filename, '`');
  file.write((const char *)ptr, size);
}
psl::vector<char> read_binary_file(psl::string_view filename) {
  auto file = psl::ScopedFile(filename, psl::ios::binary | psl::ios::in);
  if (!file.is_open())
    Warning("Unable to open file `", filename, '`');
  psl::vector<char> data(file.size());
  file.read(&data[0], file.size());
  return data;
}

psl::vector<uint8_t> to_uint8_array(vec2i size, int nchannel, const float *data) {
  psl::vector<uint8_t> pixels(area(size) * nchannel);
  for (int x = 0; x < size.x; x++)
    for (int y = 0; y < size.y; y++)
      for (int c = 0; c < nchannel; c++)
        pixels[y * size.x * nchannel + x * nchannel + c] =
            psl::clamp(data[y * size.x * nchannel + x * nchannel + c] * 256.0f, 0.0f, 255.0f);
  return pixels;
}
void save_image(psl::string filename, vec2i size, int nchannel, const float *data) {
  Profiler _("[FileIO]Save image");
  auto pixels = to_uint8_array(size, nchannel, data);
  save_image(filename, size, nchannel, pixels.data());
}
void save_image(psl::string filename, vec2i size, int nchannel, const uint8_t *data) {
  auto ext = from_last_of(filename, '.');
  if (ext == "bmp")
    stbi_write_bmp(filename.c_str(), size.x, size.y, nchannel, data);
  else if (ext == "png")
    stbi_write_png(filename.c_str(), size.x, size.y, nchannel, data,
                   size.x * nchannel * sizeof(uint8_t));
  else if (ext == "jpg")
    stbi_write_jpg(filename.c_str(), size.x, size.y, nchannel, data, 90);
  else if (ext == "tga")
    stbi_write_tga(filename.c_str(), size.x, size.y, nchannel, data);
  else {
    Warning("Unknown format `", ext, "` during saving `", filename, "`; assuming png");
    stbi_write_png((filename + ".png").c_str(), size.x, size.y, nchannel, data,
                   size.x * nchannel * sizeof(uint8_t));
  }
}
Image read_image(psl::string_view filename) {
  Profiler _("[FileIO]Load image");
  auto data = read_binary_file(filename);
  return read_image(data.data(), data.size());
}
Image read_image(void *buffer, size_t size) {
  int width, height, channels;
  if (stbi_is_hdr_from_memory(reinterpret_cast<const stbi_uc *>(buffer), size)) {
    auto data = stbi_loadf_from_memory(reinterpret_cast<const stbi_uc *>(buffer), size, &width,
                                       &height, &channels, 3);
    if (!data)
      Fatal("Unable to load image");
    CHECK_EQ(channels, 3);
    auto image = Array2D<vec3>{vec2i{width, height}, reinterpret_cast<const vec3 *>(data)};
    STBI_FREE(data);
    return image;
  } else {
    auto data = stbi_load_from_memory(reinterpret_cast<const stbi_uc *>(buffer), size, &width,
                                      &height, &channels, 3);
    if (!data)
      Fatal("Unable to load image");
    CHECK_EQ(channels, 3);
    auto image = Array2D<vec3u8>{vec2i{width, height}, reinterpret_cast<const vec3u8 *>(data)};
    STBI_FREE(data);
    return image;
  }
}

TriangleMesh load_mesh(void *data, size_t size) {
  Assimp::Importer importer;
  const aiScene *scene =
      importer.ReadFileFromMemory(data, size,
                                  aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_FlipUVs |
                                      aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    Fatal("Unable to load mesh");
    return TriangleMesh{};
  }

  auto indices = psl::vector<uint32_t>{};
  auto vertices = psl::vector<vec3>{};
  auto normals = psl::vector<vec3>{};
  auto texcoords = psl::vector<vec2>{};
  auto index_offset = 0u;

  for (size_t i_mesh = 0; i_mesh < scene->mNumMeshes; i_mesh++) {
    aiMesh *mesh = scene->mMeshes[i_mesh];

    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      auto v = mesh->mVertices[i];
      vertices.push_back(vec3{v.x, v.y, v.z});
    }

    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
      auto face = mesh->mFaces[i];
      CHECK_EQ(face.mNumIndices, 3);
      for (unsigned int j = 0; j < face.mNumIndices; ++j)
        indices.push_back(index_offset + face.mIndices[j]);
    }

    index_offset = vertices.size();

    if (mesh->HasTextureCoords(0)) {
      for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
        aiVector3D aiTexCoords = mesh->mTextureCoords[0][i];
        texcoords.push_back(vec2(aiTexCoords.x, aiTexCoords.y));
      }
    }

    if (mesh->HasNormals()) {
      for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
        aiVector3D aiNormal = mesh->mNormals[i];
        normals.push_back(vec3(aiNormal.x, aiNormal.y, aiNormal.z));
      }
    }
  }

  return TriangleMesh{psl::move(vertices), psl::move(indices), psl::move(texcoords),
                      psl::move(normals)};
}
TriangleMesh load_mesh(psl::string_view filename) {
  Profiler _("Loading mesh");
  auto data = read_binary_file(filename);
  return load_mesh(data.data(), data.size());
}

void interpret_file(Context &context, psl::string_view filename) {
  Debug("[FileIO]Loading ", filename);
  auto source = read_string_file(filename);
  interpret(context, source);
}

void fileio_context(Context &ctx) {
  ctx("load_mesh") = +[](psl::string filename) { return load_mesh(filename); };
  ctx("save_image") = overloaded<psl::string, const Array2D2f &>(save_image);
  ctx("save_image") = overloaded<psl::string, const Array2D3f &>(save_image);
  ctx("save_image") = overloaded<psl::string, const Array2D4f &>(save_image);
}

}  // namespace pine