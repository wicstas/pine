#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/parser.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

#include <contrib/stb_image.h>
#include <contrib/stb_image_write.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <contrib/tiny_gltf.h>

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

psl::vector<uint8_t> to_uint8_array(vec2i size, int nchannel, const float *data, bool flip_y) {
  psl::vector<uint8_t> pixels(area(size) * nchannel);
  for (int x = 0; x < size.x; x++)
    for (int y = 0; y < size.y; y++)
      for (int c = 0; c < nchannel; c++) {
        auto y_ = flip_y ? size.y - 1 - y : y;
        pixels[y * size.x * nchannel + x * nchannel + c] =
            psl::clamp(data[y_ * size.x * nchannel + x * nchannel + c] * 256.0f, 0.0f, 255.0f);
      }
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
    free(data);
    return image;
  } else {
    auto data = stbi_load_from_memory(reinterpret_cast<const stbi_uc *>(buffer), size, &width,
                                      &height, &channels, 3);
    if (!data)
      return psl::nullopt;
    auto image = Array2d<vec3u8>{vec2i(width, height), reinterpret_cast<const vec3u8 *>(data)};
    free(data);
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

static psl::string get_directory(psl::string_view filename) {
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
  return psl::string(filename.begin(), p0);
}

void load_gltf(Scene &scene, psl::string_view filename) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  auto ext = from_last_of(psl::string(filename), '.');
  auto ret = ext == "glb"
                 ? gltf_ctx.LoadBinaryFromFile(&model, &err, &warn, psl::string(filename).c_str())
                 : gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, psl::string(filename).c_str());
  auto working_directory = get_directory(filename);

  if (!warn.empty())
    Warning("[FileIO]", filename, ": ", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", filename, ": ", err.c_str());
  if (!ret)
    Fatal("[FIleIO]", filename, ": Unable to load");

  auto images = psl::vector<ImagePtr>();
  for (const auto &image : model.images) {
    CHECK(image.component == 3 || image.component == 4);
    CHECK(image.pixel_type == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE);
    if (image.component == 3)
      images.push_back(psl::make_shared<Image>(
          Array2d<vec3u8>({image.width, image.height}, (vec3u8 *)image.image.data())));
    else if (image.component == 4)
      images.push_back(psl::make_shared<Image>(
          Array2d<vec4u8>({image.width, image.height}, (vec4u8 *)image.image.data())));
  }

  for (const auto &node : model.nodes) {
    Logs(node.name.c_str());
    auto transform = mat4::identity();
    for (size_t i = 0; i < node.matrix.size(); i++)
      transform[i / 4][i % 4] = node.matrix[i];
    if (auto &S = node.scale; S.size() == 3)
      transform = scale(S[0], S[1], S[2]) * transform;
    if (auto &T = node.translation; T.size() == 3)
      transform = translate(T[0], T[1], T[2]) * transform;

    for (const auto &primitive : model.meshes[node.mesh].primitives) {
      auto mesh_ = TriangleMesh();
      const auto &indicesAccessor = model.accessors[primitive.indices];
      const auto &indicesBbufferView = model.bufferViews[indicesAccessor.bufferView];
      const auto &indicesBuffer = model.buffers[indicesBbufferView.buffer];
      const auto indicesPtr =
          indicesBuffer.data.data() + indicesBbufferView.byteOffset + indicesAccessor.byteOffset;
      const auto indexSize = indicesAccessor.ByteStride(indicesBbufferView);
      CHECK_EQ(indexSize, 2);
      switch (primitive.mode) {
        case TINYGLTF_MODE_TRIANGLE_FAN: {
          Log("Fan");
        }
        case TINYGLTF_MODE_TRIANGLE_STRIP: {
          Log("Strip");
        }
        case TINYGLTF_MODE_TRIANGLES: {
          auto ptr = (uint16_t *)indicesPtr;
          for (size_t i = 0; i < indicesAccessor.count; i += 3)
            mesh_.indices.push_back({ptr[i], ptr[i + 1], ptr[i + 2]});

          for (const auto &attribute : primitive.attributes) {
            const auto accessor = model.accessors[attribute.second];
            const auto &bufferView = model.bufferViews[accessor.bufferView];
            const auto &buffer = model.buffers[bufferView.buffer];
            const auto data = buffer.data.data() + bufferView.byteOffset + accessor.byteOffset;
            if (attribute.first == "POSITION") {
              if (accessor.type != TINYGLTF_TYPE_VEC3)
                Fatal("[FIleIO]", filename, ": Expect position data to be vec3");
              switch (accessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                  auto ptr = (vec3 *)data;
                  for (size_t i = 0; i < accessor.count; i++)
                    mesh_.vertices.push_back(ptr[i]);
                } break;
                case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                  auto ptr = (vec3d *)data;
                  for (size_t i = 0; i < accessor.count; i++)
                    mesh_.vertices.push_back(ptr[i]);
                } break;
                default: PINE_UNREACHABLE; break;
              }
            } else if (attribute.first == "NORMAL") {
              if (accessor.type != TINYGLTF_TYPE_VEC3)
                Fatal("[FIleIO]", filename, ": Expect normal data to be vec3");
              switch (accessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                  auto ptr = (vec3 *)data;
                  for (size_t i = 0; i < accessor.count; i++)
                    mesh_.normals.push_back(ptr[i]);
                } break;
                case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                  auto ptr = (vec3d *)data;
                  for (size_t i = 0; i < accessor.count; i++)
                    mesh_.normals.push_back(ptr[i]);
                } break;
                default: PINE_UNREACHABLE; break;
              }
            } else if (attribute.first == "TEXCOORD_0") {
              if (accessor.type != TINYGLTF_TYPE_VEC2)
                Fatal("[FIleIO]", filename, ": Expect texcoord data to be vec2");
              switch (accessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                  auto ptr = (vec2 *)data;
                  for (size_t i = 0; i < accessor.count; i++)
                    mesh_.texcoords.push_back(ptr[i]);
                } break;
                case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                  auto ptr = (vec2d *)data;
                  for (size_t i = 0; i < accessor.count; i++)
                    mesh_.texcoords.push_back(ptr[i]);
                } break;
                default: PINE_UNREACHABLE; break;
              }
            }
          }
        }
        case TINYGLTF_MODE_POINTS:
        case TINYGLTF_MODE_LINE:
        case TINYGLTF_MODE_LINE_LOOP: break;
      }

      auto basecolor = Node3f(vec3(1.0f));
      auto roughness = Nodef(1.0f);
      auto metallic = Nodef(0.0f);
      auto transmission = Nodef(0.0f);
      auto ior = Nodef(1.4f);
      if (primitive.material != -1) {
        const auto &mat = model.materials[primitive.material];
        if (auto it = mat.extensions.find("KHR_materials_transmission"); it != mat.extensions.end())
          transmission = it->second.Get("transmissionFactor").GetNumberAsDouble();
        if (auto it = mat.extensions.find("KHR_materials_ior"); it != mat.extensions.end())
          ior = it->second.Get("ior").GetNumberAsDouble();

        for (auto &[name, param] : mat.values) {
          Logs("\t", mat.name.c_str(), name.c_str());
          if (name == "baseColorFactor")
            basecolor =
                vec3(param.ColorFactor()[0], param.ColorFactor()[1], param.ColorFactor()[2]);
          else if (name == "roughnessFactor")
            roughness = param.Factor();
          else if (name == "baseColorTexture")
            basecolor = NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]);
          else if (name == "metallicRoughnessTexture") {
            roughness = NodeBinary<float, '^'>(
                NodeComponent(
                    NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]), 1),
                1.0f / 2.2f);
            metallic = NodeBinary<float, '^'>(
                NodeComponent(
                    NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]), 2),
                1.0f / 2.2f);
          }
        }
      }

      mesh_.apply(transform);
      scene.add_geometry(psl::move(mesh_),
                         UberMaterial(basecolor, roughness, metallic, transmission, ior));
    }
  }
}

void load_scene(Scene &scene_, psl::string_view filename) {
  auto ext = from_last_of(psl::string(filename), '.');
  if (ext == "gltf" || ext == "glb")
    return load_gltf(scene_, filename);
  auto working_directory = get_directory(filename);

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