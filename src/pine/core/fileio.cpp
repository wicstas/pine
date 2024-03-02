#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/parser.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <contrib/stb_image.h>
#include <contrib/stb_image_write.h>

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
    stbi_write_png(filename.c_str(), size.x, size.y, nchannel, data, 0);
  else if (ext == "jpg")
    stbi_write_jpg(filename.c_str(), size.x, size.y, nchannel, data, 90);
  else if (ext == "tga")
    stbi_write_tga(filename.c_str(), size.x, size.y, nchannel, data);
  else {
    Warning("Unknown format `", ext, "` during saving `", filename, "`; assuming png");
    stbi_write_png((filename + ".png").c_str(), size.x, size.y, nchannel, data, 0);
  }
}
psl::shared_ptr<Image> load_image(psl::string_view filename) {
  static psl::map<psl::string, psl::shared_ptr<Image>> caches;
  if (auto it = caches.find(filename); it != caches.end())
    return it->second;

  auto data = read_binary_file(filename);
  if (auto image = load_image(data.data(), data.size())) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock{mutex};
    return caches[psl::string(filename)] = psl::make_shared<Image>(psl::move(*image));
  } else {
    Fatal("Unable to load `", filename, "`");
  }
}

psl::vector<uint8_t> reshape(vec2i size, int source_comp, int target_comp, const uint8_t *data) {
  auto result = psl::vector<uint8_t>(size_t(area(size)) * target_comp);
  for (int y = 0; y < size.y; y++)
    for (int x = 0; x < size.x; x++)
      for (int i = 0; i < psl::min(source_comp, target_comp); i++)
        result[size_t(y) * size.x * target_comp + x * target_comp + i] =
            data[size_t(y) * size.x * source_comp + x * source_comp + i];
  return result;
}
psl::optional<Image> load_image(void *buffer, size_t size) {
  int width, height, channels;
  if (stbi_is_hdr_from_memory(reinterpret_cast<const stbi_uc *>(buffer), size)) {
    auto data = stbi_loadf_from_memory(reinterpret_cast<const stbi_uc *>(buffer), size, &width,
                                       &height, &channels, 3);
    if (!data)
      return psl::nullopt;
    auto image = Array2d<vec3>{vec2i(width, height), reinterpret_cast<const vec3 *>(data)};
    STBI_FREE(data);
    return image;
  } else {
    auto data = stbi_load_from_memory(reinterpret_cast<const stbi_uc *>(buffer), size, &width,
                                      &height, &channels, 3);
    if (!data)
      return psl::nullopt;
    auto image = Array2d<vec3u8>{vec2i(width, height), reinterpret_cast<const vec3u8 *>(data)};
    STBI_FREE(data);
    return image;
  }
}

psl::optional<TriangleMesh> load_mesh(void *data, size_t size) {
  Assimp::Importer importer;
  const aiScene *scene =
      importer.ReadFileFromMemory(data, size,
                                  aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                                      aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    return psl::nullopt;

  auto indices = psl::vector<vec3u32>{};
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
      indices.push_back(vec3i(index_offset + face.mIndices[0], index_offset + face.mIndices[1],
                              index_offset + face.mIndices[2]));
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
  if (auto mesh = load_mesh(data.data(), data.size()))
    return *mesh;
  else
    Fatal("Unable to load `", filename, "`");
}

void load_scene(Scene &scene_, psl::string_view filename) {
  auto p0 = psl::find_last_of(filename, '/');
  if (auto p1 = psl::find_last_of(filename, '\\'); p1 != filename.end()) {
    if (p0 != filename.end())
      p0 = psl::max(p0, p1);
    else
      p0 = p1;
  }
  if (p0 == filename.end())
    p0 = filename.begin();
  else
    p0 = psl::next(p0);
  auto working_directory = psl::string_view(filename.begin(), p0);
  ;  // Debug("Working directory ", working_directory);

  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(
      psl::string(filename).c_str(),
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    Fatal("Unable to load `", filename, "`");
  }

  parallel_for(scene->mNumMeshes, [&](size_t i_mesh) {
    auto indices = psl::vector<vec3u32>{};
    auto vertices = psl::vector<vec3>{};
    auto normals = psl::vector<vec3>{};
    auto texcoords = psl::vector<vec2>{};
    aiMesh *mesh = scene->mMeshes[i_mesh];

    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      auto v = mesh->mVertices[i];
      vertices.push_back(vec3{v.x, v.y, v.z});
    }

    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
      auto face = mesh->mFaces[i];
      CHECK_EQ(face.mNumIndices, 3);
      indices.push_back(vec3i(face.mIndices[0], face.mIndices[1], face.mIndices[2]));
    }

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

    auto material = scene->mMaterials[mesh->mMaterialIndex];
    auto dc = aiColor3D(1.0, 1.0, 1.0);
    if (material->Get(AI_MATKEY_COLOR_DIFFUSE, dc) == aiReturn_SUCCESS)
      ;  // Debug(mesh->mName.C_Str(), " has diffuse ", dc.r, ' ', dc.g, ' ', dc.b);
    auto diffuse_texture = aiString();
    if (material->Get(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0), diffuse_texture) ==
        aiReturn_SUCCESS)
      ;  // Debug(mesh->mName.C_Str(), " has diffuse texture ", diffuse_texture.C_Str());
    auto base_texture = aiString();
    if (material->Get(AI_MATKEY_TEXTURE(aiTextureType_BASE_COLOR, 0), base_texture) ==
        aiReturn_SUCCESS)
      ;  // Debug(mesh->mName.C_Str(), " has base texture ", base_texture.C_Str());
    auto alpha_texture = aiString();
    if (material->Get(AI_MATKEY_TEXTURE(aiTextureType_OPACITY, 0), alpha_texture) ==
        aiReturn_SUCCESS)
      ;  // Debug(mesh->mName.C_Str(), " has alpha texture ", alpha_texture.C_Str());

    auto material_ = Material();
    if (diffuse_texture.length != 0) {
      auto basecolor = vec3(dc.r, dc.g, dc.b);
      if (basecolor == vec3(1.0f))
        material_ = DiffuseMaterial(
            NodeImage(NodeUV(), load_image(working_directory + diffuse_texture.C_Str())));
      else
        material_ = DiffuseMaterial(NodeBinary<vec3, '*'>(
            Node3f(basecolor),
            NodeImage(NodeUV(), load_image(working_directory + diffuse_texture.C_Str()))));
    } else {
      material_ = DiffuseMaterial(vec3(dc.r, dc.g, dc.b));
    }

    scene_.add_geometry(TriangleMesh(psl::move(vertices), psl::move(indices), psl::move(texcoords),
                                     psl::move(normals)),
                        psl::move(material_));
  });
}

void interpret_file(Context &context, psl::string_view filename) {
  Debug("[FileIO]Loading `", filename, "`");
  auto source = read_string_file(filename);
  interpret(context, source);
}

void fileio_context(Context &ctx) {
  ctx("load_mesh") = overloaded<psl::string_view>(load_mesh);
  ctx("load") = load_scene;
  ctx("load_image") = overloaded<psl::string_view>(load_image);
  ctx("save_image") = overloaded<psl::string, Array2d2f>(save_image);
  ctx("save_image") = overloaded<psl::string, Array2d3f>(save_image);
  ctx("save_image") = overloaded<psl::string, Array2d4f>(save_image);
}

}  // namespace pine