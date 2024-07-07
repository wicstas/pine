#include <pine/core/program_context.h>
#include <pine/core/jit_compiler.h>
#include <pine/core/video_writer.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/fileio.h>
#include <pine/core/scene.h>
#include <pine/core/log.h>

#include <contrib/stb_image_write.h>
#include <contrib/stb_image.h>
#include <contrib/tiny_gltf.h>

#include <filesystem>

namespace pine {

psl::string read_string_file(psl::string_view filename) {
  auto file = psl::ScopedFile(filename, psl::ios::in);
  if (!file.is_open())
    Fatal("Unable to open file `", filename, '`');
  size_t size = file.size();
  auto str = psl::string(file.size());
  file.read(&str[0], size);
  return str;
}
void write_binary_file(psl::string_view filename, const void *ptr, size_t size) {
  auto file = psl::ScopedFile(filename, psl::ios::binary | psl::ios::out);
  if (!file.is_open())
    Warning("Unable to create file `", filename, '`');
  file.write((const char *)ptr, size);
}
Bytes read_binary_file(psl::string_view filename) {
  auto file = psl::ScopedFile(filename, psl::ios::binary | psl::ios::in);
  if (!file.is_open())
    Fatal("Unable to open file `", filename, '`');
  Bytes data(file.size());
  file.read(&data[0], file.size());
  return data;
}
psl::map<psl::string, Bytes> read_folder(psl::string path) {
  auto fs = psl::map<psl::string, Bytes>();
  for (const auto &entry : std::filesystem::recursive_directory_iterator(path.c_str())) {
    auto entry_path = psl::string(entry.path().c_str());
    if (entry_path[0] == '.') {
      entry_path.pop_front();
      if (entry_path[0] == '/')
        entry_path.pop_front();
    }
    // Debug(entry_path);
    if (entry.is_regular_file())
      fs[entry_path] = read_binary_file(entry_path);
  }
  return fs;
}
Bytes serialize(const psl::map<psl::string, Bytes> &fs) {
  auto bytes = Bytes();
  auto write = [&](const auto &first_elem, size_t size) {
    bytes.insert_range(bytes.end(), psl::range((uint8_t *)&first_elem, size));
  };
  write(fs.size(), sizeof(size_t));
  for (auto &[path, data] : fs) {
    write(path.byte_size(), sizeof(size_t));
    write(path[0], path.byte_size());
    write(data.byte_size(), sizeof(size_t));
    write(data[0], data.byte_size());
  }

  return bytes;
}
template <>
psl::map<psl::string, Bytes> deserialize(BytesView bytes) {
  auto fs = psl::map<psl::string, Bytes>();
  auto read = [&](auto &first_elem, size_t size) {
    psl::copy((uint8_t *)&first_elem, bytes.subspan(0, size));
    bytes = bytes.subspan(size);
  };
  auto read_pod = [&]<typename T>() {
    T value;
    read(value, sizeof(T));
    return value;
  };

  auto size = read_pod.operator()<size_t>();
  for (size_t i = 0; i < size; i++) {
    auto str_size = read_pod.operator()<size_t>();
    auto str = psl::string(str_size);
    read(str[0], str_size);

    auto data_size = read_pod.operator()<size_t>();
    auto data = Bytes(data_size);
    read(data[0], data_size);
    fs[MOVE(str)] = MOVE(data);
  }

  return fs;
}

psl::vector<uint8_t> to_uint8_array(vec2i size, int nchannel, const float *data, bool flip_y,
                                    bool apply_gamma) {
  psl::vector<uint8_t> pixels(area(size) * nchannel);
  for (int x = 0; x < size.x; x++)
    for (int y = 0; y < size.y; y++)
      for (int c = 0; c < nchannel; c++) {
        auto y_ = flip_y ? size.y - 1 - y : y;
        auto value = data[y_ * size.x * nchannel + x * nchannel + c];
        if (apply_gamma)
          value = psl::pow(value, 1 / 2.2f);
        pixels[y * size.x * nchannel + x * nchannel + c] = psl::clamp(value * 256.0f, 0.0f, 255.0f);
      }
  return pixels;
}
void save_image(psl::string filename, vec2i size, int nchannel, const float *data) {
  Profiler _("[FileIO]Save image");
  auto pixels = to_uint8_array(size, nchannel, data, false, true);
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

psl::optional<Image> image_from(void *buffer, size_t size) {
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
psl::shared_ptr<Image> load_image(psl::string_view filename,
                                  psl::function<Bytes(psl::string_view)> reader) {
  static psl::map<psl::string, psl::shared_ptr<Image>> caches;
  if (auto it = caches.find(filename); it != caches.end())
    return it->second;

  auto data = reader(filename);
  if (auto image = image_from(data.data(), data.size())) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock{mutex};
    return caches[psl::string(filename)] = psl::make_shared<Image>(MOVE(*image));
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

int count(tinygltf::Model &model, const tinygltf::Node &node) {
  auto cnt = 1;
  for (auto &&child : node.children)
    cnt += count(model, model.nodes[child]);
  return cnt;
}

inline mat4 q2m(float a, float b, float c, float d) {
  return {a * a + b * b - c * c - d * d,
          2 * b * c - 2 * a * d,
          2 * b * d + 2 * a * c,
          0,
          2 * b * c + 2 * a * d,
          a * a - b * b + c * c - d * d,
          2 * c * d - 2 * a * b,
          0,
          2 * b * d - 2 * a * c,
          2 * c * d + 2 * a * b,
          a * a - b * b - c * c + d * d,
          0,
          0,
          0,
          0,
          1};
}

void scene_from_gltf(Scene &scene, void *tiny_gltf_model, mat4 global_transform) {
  auto &model = *(tinygltf::Model *)tiny_gltf_model;
  tinygltf::TinyGLTF gltf_ctx;

  auto images = psl::vector<ImagePtr>();
  for (const auto &image : model.images) {
    CHECK(image.component == 3 || image.component == 4);
    CHECK(image.pixel_type == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE);
    if (image.component == 3)
      images.push_back(psl::make_shared<Image>(
          Array2d<vec3u8>({image.width, image.height}, (vec3u8 *)image.image.data())));
    else
      images.push_back(psl::make_shared<Image>(
          Array2d<vec4u8>({image.width, image.height}, (vec4u8 *)image.image.data())));
  }

  auto process_node = [&](auto &me, tinygltf::Node &node, mat4 transform) -> void {
    if (auto &m = node.matrix; m.size() == 16)
      transform = transform * transpose(mat4(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8],
                                             m[9], m[10], m[11], m[12], m[13], m[14], m[15]));
    if (auto &T = node.translation; T.size() == 3)
      transform = transform * translate(T[0], T[1], T[2]);
    if (auto &R = node.rotation; R.size() == 4)
      transform = transform * q2m(R[3], R[0], R[1], R[2]);
    if (auto &S = node.scale; S.size() == 3)
      transform = transform * scale(S[0], S[1], S[2]);

    // Debug(node.name.c_str());
    if (node.mesh >= 0)
      for (const auto &primitive : model.meshes[node.mesh].primitives) {
        auto mesh_ = Mesh();
        const auto &indicesAccessor = model.accessors[primitive.indices];
        const auto &indicesBbufferView = model.bufferViews[indicesAccessor.bufferView];
        const auto &indicesBuffer = model.buffers[indicesBbufferView.buffer];
        const auto indicesPtr =
            indicesBuffer.data.data() + indicesBbufferView.byteOffset + indicesAccessor.byteOffset;
        const auto indexSize = indicesAccessor.ByteStride(indicesBbufferView);
        CHECK(indexSize == 2 || indexSize == 4);
        switch (primitive.mode) {
          case TINYGLTF_MODE_TRIANGLE_FAN: {
            Fatal("Fan");
          }
          case TINYGLTF_MODE_TRIANGLE_STRIP: {
            Fatal("Strip");
          }
          case TINYGLTF_MODE_TRIANGLES: {
            CHECK(indicesAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE ||
                  indicesAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT ||
                  indicesAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT);
            if (indexSize == 1) {
              auto ptr = (uint8_t *)indicesPtr;
              for (size_t i = 0; i < indicesAccessor.count; i += 3)
                mesh_.indices.push_back({ptr[i], ptr[i + 1], ptr[i + 2]});
            } else if (indexSize == 2) {
              auto ptr = (uint16_t *)indicesPtr;
              for (size_t i = 0; i < indicesAccessor.count; i += 3)
                mesh_.indices.push_back({ptr[i], ptr[i + 1], ptr[i + 2]});
            } else {
              auto ptr = (uint32_t *)indicesPtr;
              for (size_t i = 0; i < indicesAccessor.count; i += 3)
                mesh_.indices.push_back({ptr[i], ptr[i + 1], ptr[i + 2]});
            }

            for (const auto &attribute : primitive.attributes) {
              const auto accessor = model.accessors[attribute.second];
              const auto &bufferView = model.bufferViews[accessor.bufferView];
              const auto &buffer = model.buffers[bufferView.buffer];
              const auto data = buffer.data.data() + bufferView.byteOffset + accessor.byteOffset;
              if (attribute.first == "POSITION") {
                if (accessor.type != TINYGLTF_TYPE_VEC3)
                  Fatal("[FIleIO][TinyGLTF]Expect position data to be vec3");
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
                  Fatal("[FIleIO][TinyGLFT]Expect normal data to be vec3");
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
                  Fatal("[FIleIO][TinyGLFT]Expect texcoord data to be vec2");
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
                  default: PINE_UNREACHABLE;
                }
              }
            }
          }
          case TINYGLTF_MODE_POINTS:
          case TINYGLTF_MODE_LINE:
          case TINYGLTF_MODE_LINE_LOOP: break;
          default: PINE_UNREACHABLE;
        }

        auto basecolor = Node3f(vec3(1.0f));
        auto roughness = Nodef(1.0f);
        auto metallic = Nodef(0.0f);
        auto transmission = Nodef(0.0f);
        auto emission_color = vec3(1.0f);
        auto emission_strength = 0.0f;
        auto ior = 1.45f;
        if (primitive.material != -1) {
          const auto &mat = model.materials[primitive.material];
          if (auto it = psl::find_or_nullopt(mat.extensions, "KHR_materials_transmission"))
            transmission = it->Get("transmissionFactor").GetNumberAsDouble();
          if (auto it = psl::find_or_nullopt(mat.extensions, "KHR_materials_ior"))
            ior = it->Get("ior").GetNumberAsDouble();
          if (auto it = psl::find_or_nullopt(mat.extensions, "KHR_materials_emissive_strength"))
            emission_strength = it->Get("emissiveStrength").GetNumberAsDouble();

          auto to_vec3 = [](auto v) { return vec3(v[0], v[1], v[2]); };

          basecolor = to_vec3(mat.pbrMetallicRoughness.baseColorFactor);
          metallic = mat.pbrMetallicRoughness.metallicFactor;
          roughness = mat.pbrMetallicRoughness.roughnessFactor;
          emission_color = to_vec3(mat.emissiveFactor);

          for (auto &[name, param] : mat.values) {
            if (name == "baseColorFactor")
              basecolor = to_vec3(param.ColorFactor());
            else if (name == "roughnessFactor")
              roughness = param.Factor();
            else if (name == "metallicFactor")
              metallic = param.Factor();
            else if (name == "baseColorTexture")
              basecolor = NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]);
            else if (name == "metallicRoughnessTexture") {
              roughness = NodeComponent(
                  NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]), 1);
              metallic = NodeComponent(
                  NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]), 2);
            }
          }
        }

        mesh_.apply(transform);

        auto emission = emission_color * emission_strength;
        if (emission.is_zero())
          scene.add_geometry(MOVE(mesh_),
                             UberMaterial(basecolor, roughness, metallic, transmission, ior));
        else
          scene.add_geometry(MOVE(mesh_), EmissiveMaterial(emission));
      }

    for (auto node_index : node.children)
      me(me, model.nodes[node_index], transform);
  };

  for (auto &&scene : model.scenes)
    for (auto node_index : scene.nodes)
      process_node(process_node, model.nodes[node_index], global_transform);

  for (const auto &node : model.nodes) {
    if (node.camera >= 0) {
      auto &&cam = model.cameras[node.camera];
      auto &&P = node.translation;
      auto &&R = node.rotation;
      CHECK(P.size() == 3);
      CHECK(R.size() == 4);
      auto pos = vec3(P[0], P[1], P[2]);
      auto rot = mat3(q2m(R[3], R[0], R[1], R[2]));
      auto at = pos + rot * vec3(0, 0, -1);
      scene.set_camera(ThinLenCamera(Film(vec2i(640 * cam.perspective.aspectRatio, 640)), pos, at,
                                     cam.perspective.yfov / 2));
    }
  }
}
Mesh mesh_from_gltf(void *tiny_gltf_model) {
  auto mesh_ = Mesh();
  auto &model = *(tinygltf::Model *)tiny_gltf_model;
  tinygltf::TinyGLTF gltf_ctx;

  auto indices_offset = size_t(0);
  for (const auto &node : model.nodes) {
    if (node.mesh < 0)
      continue;
    indices_offset = mesh_.vertices.size();
    Debug(node.name.c_str());
    auto transform = mat4::identity();
    for (size_t i = 0; i < node.matrix.size(); i++)
      transform[i / 4][i % 4] = node.matrix[i];
    if (auto &S = node.scale; S.size() == 3)
      transform = scale(S[0], S[1], S[2]) * transform;
    if (auto &T = node.translation; T.size() == 3)
      transform = translate(T[0], T[1], T[2]) * transform;

    for (const auto &primitive : model.meshes[node.mesh].primitives) {
      const auto &indicesAccessor = model.accessors[primitive.indices];
      const auto &indicesBbufferView = model.bufferViews[indicesAccessor.bufferView];
      const auto &indicesBuffer = model.buffers[indicesBbufferView.buffer];
      const auto indicesPtr =
          indicesBuffer.data.data() + indicesBbufferView.byteOffset + indicesAccessor.byteOffset;
      const auto indexSize = indicesAccessor.ByteStride(indicesBbufferView);
      CHECK(indexSize == 2 || indexSize == 4);
      switch (primitive.mode) {
        case TINYGLTF_MODE_TRIANGLE_FAN: {
          Fatal("Fan");
        }
        case TINYGLTF_MODE_TRIANGLE_STRIP: {
          Fatal("Strip");
        }
        case TINYGLTF_MODE_TRIANGLES: {
          if (indexSize == 2) {
            auto ptr = (uint16_t *)indicesPtr;
            for (size_t i = 0; i < indicesAccessor.count; i += 3)
              mesh_.indices.push_back(vec3u32(indices_offset) +
                                      vec3u32(ptr[i], ptr[i + 1], ptr[i + 2]));
          } else {
            auto ptr = (uint32_t *)indicesPtr;
            for (size_t i = 0; i < indicesAccessor.count; i += 3)
              mesh_.indices.push_back(vec3u32(indices_offset) +
                                      vec3u32(ptr[i], ptr[i + 1], ptr[i + 2]));
          }

          for (const auto &attribute : primitive.attributes) {
            const auto accessor = model.accessors[attribute.second];
            const auto &bufferView = model.bufferViews[accessor.bufferView];
            const auto &buffer = model.buffers[bufferView.buffer];
            const auto data = buffer.data.data() + bufferView.byteOffset + accessor.byteOffset;
            if (attribute.first == "POSITION") {
              if (accessor.type != TINYGLTF_TYPE_VEC3)
                Fatal("[FIleIO][TinyGLFT]Expect position data to be vec3");
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
                Fatal("[FIleIO][TinyGLFT]Expect normal data to be vec3");
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
                Fatal("[FIleIO][TinyGLFT]Expect texcoord data to be vec2");
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
    }
  }
  return mesh_;
}
UberMaterial material_from_gltf(void *tiny_gltf_model) {
  auto &model = *(tinygltf::Model *)tiny_gltf_model;
  tinygltf::TinyGLTF gltf_ctx;

  auto images = psl::vector<ImagePtr>();
  for (const auto &image : model.images) {
    CHECK(image.component == 3 || image.component == 4);
    CHECK(image.pixel_type == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE);
    if (image.component == 3)
      images.push_back(psl::make_shared<Image>(
          Array2d<vec3u8>({image.width, image.height}, (vec3u8 *)image.image.data())));
    else
      images.push_back(psl::make_shared<Image>(
          Array2d<vec4u8>({image.width, image.height}, (vec4u8 *)image.image.data())));
  }

  if (model.materials.size() != 1)
    Fatal("[FileIO][GLTF]Expect a unique material, get ", model.materials.size());
  const auto &mat = model.materials[0];

  auto basecolor = Node3f(vec3(1.0f));
  auto roughness = Nodef(1.0f);
  auto metallic = Nodef(0.0f);
  auto transmission = Nodef(0.0f);
  auto ior = 1.45f;
  if (auto it = mat.extensions.find("KHR_materials_transmission"); it != mat.extensions.end())
    transmission = it->second.Get("transmissionFactor").GetNumberAsDouble();
  if (auto it = mat.extensions.find("KHR_materials_ior"); it != mat.extensions.end())
    ior = it->second.Get("ior").GetNumberAsDouble();

  for (auto &[name, param] : mat.values) {
    Debug("\t", mat.name.c_str(), ' ', name.c_str());
    if (name == "baseColorFactor")
      basecolor = vec3(param.ColorFactor()[0], param.ColorFactor()[1], param.ColorFactor()[2]);
    else if (name == "roughnessFactor")
      roughness = param.Factor();
    else if (name == "baseColorTexture")
      basecolor = NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]);
    else if (name == "metallicRoughnessTexture") {
      roughness = NodeBinary<float, '^'>(
          NodeComponent(NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]),
                        1),
          1.0f / 2.2f);
      metallic = NodeBinary<float, '^'>(
          NodeComponent(NodeImage(NodeUV(), images[model.textures[param.TextureIndex()].source]),
                        2),
          1.0f / 2.2f);
    }
  }

  return UberMaterial(basecolor, roughness, metallic, transmission, ior);
}

tinygltf::FsCallbacks create_tinygltf_fs(const psl::map<psl::string, Bytes> *files) {
  auto fs = tinygltf::FsCallbacks();
  fs.FileExists = +[](const std::string &path, void *user_data) {
    auto &files = *(const psl::map<psl::string, Bytes> *)user_data;
    return files.find(path.c_str()) != files.end();
  };
  fs.ExpandFilePath = +[](const std::string &abs_filename, void *) { return abs_filename; };
  fs.ReadWholeFile = +[](std::vector<unsigned char> *out_data, std::string *err,
                         const std::string &path, void *user_data) {
    auto &files = *(const psl::map<psl::string, Bytes> *)user_data;
    if (auto it = files.find(path.c_str()); it != files.end()) {
      out_data->insert(out_data->end(), it->second.begin(), it->second.end());
      return true;
    } else {
      *err = "Unable to find `" + path + '`';
      return false;
    }
  };
  fs.GetFileSizeInBytes =
      +[](size_t *size, std::string *err, const std::string &path, void *user_data) {
        auto &files = *(const psl::map<psl::string, Bytes> *)user_data;
        if (auto it = files.find(path.c_str()); it != files.end()) {
          *size = it->second.byte_size();
          return true;
        } else {
          *err = "Unable to find `" + path + '`';
          return false;
        }
      };
  fs.user_data = (void *)files;
  return fs;
}
void scene_from_gltf(Scene &scene, psl::string file_name, const psl::map<psl::string, Bytes> &files,
                     mat4 m) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  gltf_ctx.SetStoreOriginalJSONForExtrasAndExtensions(true);
  gltf_ctx.SetFsCallbacks(create_tinygltf_fs(&files));
  auto ret = gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, file_name.c_str());
  if (!warn.empty())
    Warning("[FileIO]", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", err.c_str());
  if (!ret)
    Fatal("[FIleIO]Unable to create scene from GLTF file");

  scene_from_gltf(scene, &model, m);
}
void scene_from_gltf_binary(Scene &scene, const Bytes &data, mat4 m) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  gltf_ctx.SetStoreOriginalJSONForExtrasAndExtensions(true);
  auto ret = gltf_ctx.LoadBinaryFromMemory(&model, &err, &warn, data.data(), data.size());
  if (!warn.empty())
    Warning("[FileIO]", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", err.c_str());
  if (!ret)
    Fatal("[FIleIO]Unable to create scene from GLTF file");

  scene_from_gltf(scene, &model, m);
}
Mesh mesh_from_gltf(psl::string file_name, const psl::map<psl::string, Bytes> &files) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  gltf_ctx.SetFsCallbacks(create_tinygltf_fs(&files));
  auto ret = gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, file_name.c_str());
  if (!warn.empty())
    Warning("[FileIO]", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", err.c_str());
  if (!ret)
    Fatal("[FIleIO]Unable to create scene from GLTF file");

  return mesh_from_gltf(&model);
}
UberMaterial material_from_gltf(psl::string file_name, const psl::map<psl::string, Bytes> &files) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  gltf_ctx.SetFsCallbacks(create_tinygltf_fs(&files));
  auto ret = gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, file_name.c_str());
  if (!warn.empty())
    Warning("[FileIO]", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", err.c_str());
  if (!ret)
    Fatal("[FIleIO]Unable to create scene from GLTF file");

  return material_from_gltf(&model);
}

void interpret_file(Context &context, psl::string_view filename) {
  Debug("[FileIO]Loading `", filename, "`");
  auto source = read_string_file(filename);
  jit_interpret(context, source);
}

void fileio_context(Context &ctx) {
  ctx.type<Bytes>("Bytes");
  ctx("read_binary") = [](psl::string_view filename) { return read_binary_file(filename); };
  ctx("load") = [](Scene &scene, psl::string file_path) {
    auto ext = from_last_of(file_path, '.');
    if (ext == "pineb") {
      auto scene_name = psl::until_last_of(psl::from_last_of(file_path, '/'), '.') + ".gltf";
      auto bytes = Context::context.call<Bytes>("read_binary", psl::string_view(file_path));
      auto fs = deserialize<psl::map<psl::string, Bytes>>(bytes);
      scene_from_gltf(scene, scene_name, fs, mat4::identity());
    } else {
      auto bytes = Context::context.call<Bytes>("read_binary", psl::string_view(file_path));
      scene_from_gltf_binary(scene, bytes, mat4::identity());
    }
  };
  ctx("load") = [](Scene &scene, psl::string file_path, mat4 m) {
    auto ext = from_last_of(file_path, '.');
    if (ext == "pineb") {
      auto scene_name = psl::until_last_of(psl::from_last_of(file_path, '/'), '.') + ".gltf";
      auto bytes = Context::context.call<Bytes>("read_binary", psl::string_view(file_path));
      auto fs = deserialize<psl::map<psl::string, Bytes>>(bytes);
      scene_from_gltf(scene, scene_name, fs, m);
    } else {
      auto bytes = Context::context.call<Bytes>("read_binary", psl::string_view(file_path));
      scene_from_gltf_binary(scene, bytes, m);
    }
  };
  ctx("Mesh") = [](psl::string file_path) {
    auto scene_name = psl::until_last_of(psl::from_last_of(file_path, '/'), '.') + ".gltf";
    auto bytes = Context::context.call<Bytes>("read_binary", psl::string_view(file_path));
    auto fs = deserialize<psl::map<psl::string, Bytes>>(bytes);
    return mesh_from_gltf(scene_name, fs);
  };
  ctx("load_material") = [](psl::string file_path) {
    auto scene_name = psl::until_last_of(psl::from_last_of(file_path, '/'), '.') + ".gltf";
    auto bytes = Context::context.call<Bytes>("read_binary", psl::string_view(file_path));
    auto fs = deserialize<psl::map<psl::string, Bytes>>(bytes);
    return material_from_gltf(scene_name, fs);
  };
  ctx("load_image") = [](psl::string_view filename) {
    auto reader = [](psl::string_view filename) {
      return Context::context.call<Bytes>("read_binary", filename);
    };
    return load_image(filename, reader);
  };
  ctx("heightmap") = [](psl::string_view path) {
    auto image = Context::context.call<psl::shared_ptr<Image>>("load_image", path);
    return heightmap(image->size(),
                     psl::function<float(vec2i)>([&](vec2i p) { return (*image)[p].x; }));
  };
  ctx("save") = [](const Image &array, psl::string filename) { save_image(filename, array); };
  ctx("save") = [](const Array2d3f &array, psl::string filename) { save_image(filename, array); };
  ctx("save") = [](const Array2d4f &array, psl::string filename) { save_image(filename, array); };
  ctx("save") = [](const Image &array, psl::string filename, bool should_invert_y) {
    save_image(filename, array, should_invert_y);
  };
  ctx("save") = [](const Array2d3f &array, psl::string filename, bool should_invert_y) {
    save_image(filename, array, should_invert_y);
  };
  ctx("save") = [](const Array2d4f &array, psl::string filename, bool should_invert_y) {
    save_image(filename, array, should_invert_y);
  };

  ctx.type<VideoWriter>("VideoWriter")
      .ctor<psl::string, vec2i, int>()
      .method<overloaded<const Array2d3f &>(&VideoWriter::add_frame)>("add_frame")
      .method<overloaded<const Array2d4f &>(&VideoWriter::add_frame)>("add_frame")
      .method<&VideoWriter::done>("done");
}

}  // namespace pine
