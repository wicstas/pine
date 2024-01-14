#include <pine/core/profiler.h>
#include <pine/core/parser.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

#define STB_IMAGE_IMPLEMENTATION
#include <ext/stb_image.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace pine {

ScopedFile::ScopedFile(psl::string_view filename_view, psl::ios::OpenMode mode) {
  auto filename = (psl::string)filename_view;
  psl::replace(filename, '\\', '/');
  CHECK(filename != "");
  file.open(filename.c_str(), mode);
  if (file.is_open() == false)
    Warning("[ScopedFile]Can not open file `", filename.c_str(), '`');
}
void ScopedFile::Write(const void *data, size_t size) {
  if (file.is_open())
    file.write((char *)data, size);
}
void ScopedFile::Read(void *data, size_t size) {
  if (file.is_open())
    file.read((char *)data, size);
}

psl::string ReadStringFile(psl::string_view filename) {
  ScopedFile file(filename, psl::ios::in);
  size_t size = file.Size();
  psl::string str;
  str.resize(size);
  file.Read(&str[0], size);
  return str;
}
void WriteBinaryData(psl::string_view filename, const void *ptr, size_t size) {
  ScopedFile file(filename, psl::ios::binary | psl::ios::out);
  file.Write((const char *)ptr, size);
}
psl::vector<char> ReadBinaryData(psl::string_view filename) {
  ScopedFile file(filename, psl::ios::binary | psl::ios::in);
  psl::vector<char> data(file.Size());
  file.Read(&data[0], file.Size());
  return data;
}

void WriteImageBMP(psl::string_view filename, vec2i size, int nchannel, const uint8_t *data) {
  ScopedFile file(filename, psl::ios::out);

  psl::vector<vec3u8> colors(size.x * size.y);
  for (int x = 0; x < size.x; x++)
    for (int y = 0; y < size.y; y++) {
      vec3u8 c;
      c.z = data[(x + y * size.x) * nchannel + 0];
      c.y = data[(x + y * size.x) * nchannel + 1];
      c.x = data[(x + y * size.x) * nchannel + 2];
      colors[x + (size.y - 1 - y) * size.x] = c;
    }

  int filesize = 54 + 3 * size.x * size.y;
  // clang-format off
  uint8_t header[] = {'B','M',(uint8_t)(filesize),(uint8_t)(filesize >> 8),(uint8_t)(filesize >> 16),(uint8_t)(filesize >> 24),0,0,0,0,54,0,0,0,40,0,0,0,(uint8_t)(size.x),(uint8_t)(size.x >> 8),(uint8_t)(size.x >> 16),(uint8_t)(size.x >> 24),(uint8_t)(size.y),(uint8_t)(size.y >> 8),(uint8_t)(size.y >> 16),(uint8_t)(size.y >> 24),1,0,24,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  // clang-format on
  file.Write(header, sizeof(header));

  uint8_t padding[3] = {0, 0, 0};
  size_t paddingSize = (4 - (size.x * 3) % 4) % 4;
  if (paddingSize == 0) {
    file.Write(colors.data(), sizeof(colors[0]) * colors.size());
  } else {
    for (int y = 0; y < size.y; y++) {
      file.Write(colors.data() + y * size.x, size.x * 3);
      file.Write(padding, paddingSize);
    }
  }
}

psl::vector<uint8_t> ToUint8Image(vec2i size, int nchannel, const float *data) {
  psl::vector<uint8_t> pixels(area(size) * nchannel);
  for (int x = 0; x < size.x; x++)
    for (int y = 0; y < size.y; y++)
      for (int c = 0; c < nchannel; c++)
        pixels[y * size.x * nchannel + x * nchannel + c] =
            psl::clamp(data[y * size.x * nchannel + x * nchannel + c] * 256.0f, 0.0f, 255.0f);
  return pixels;
}
void SaveImage(psl::string_view filename, vec2i size, int nchannel, const float *data) {
  Profiler _("[FileIO]Save image");
  auto pixels = ToUint8Image(size, nchannel, data);
  SaveImage(filename, size, nchannel, pixels.data());
}
void SaveImage(psl::string_view filename, vec2i size, int nchannel, const uint8_t *data) {
  WriteImageBMP(filename, size, nchannel, data);
}
Image read_image(psl::string_view filename) {
  Profiler _("[FileIO]Load image");
  auto data = ReadBinaryData(filename);
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
  auto data = ReadBinaryData(filename);
  return load_mesh(data.data(), data.size());
}

void interpretFile(Context &context, psl::string_view filename) {
  Log("[FileIO]Loading ", filename);
  auto source = ReadStringFile(filename);
  interpret(context, source);
}

}  // namespace pine