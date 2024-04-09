#include <pine/core/video_writer.h>
#include <pine/core/profiler.h>
#include <pine/core/parallel.h>
#include <pine/core/parser.h>
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
    fs[psl::move(str)] = psl::move(data);
  }

  return fs;
}

// static psl::string get_directory(psl::string_view filename) {
//   auto p0 = psl::find_last_of(filename, '/');
//   if (auto p1 = psl::find_last_of(filename, '\\'); p1 != filename.end()) {
//     if (p0 != filename.end())
//       p0 = psl::max(p0, p1);
//     else
//       p0 = p1;
//   }
//   if (p0 == filename.end())
//     p0 = filename.begin();
//   else
//     p0 = psl::next(p0);
//   return psl::string(filename.begin(), p0);
// }

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

void scene_from(Scene &scene, void *tiny_gltf_model) {
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

  for (const auto &node : model.nodes) {
    Debug(node.name.c_str());
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
          Fatal("Fan");
        }
        case TINYGLTF_MODE_TRIANGLE_STRIP: {
          Fatal("Strip");
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
          Debug("\t", mat.name.c_str(), ' ', name.c_str());
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
UberMaterial material_from(void *tiny_gltf_model) {
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
  auto ior = Nodef(1.4f);
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

void scene_from(Scene &scene, psl::string file_name, const psl::map<psl::string, Bytes> &files) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  auto fs = tinygltf::FsCallbacks();
  fs.FileExists = +[](const std::string &path, void *user_data) {
    Log(path.c_str());
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
  fs.user_data = (void *)&files;

  gltf_ctx.SetFsCallbacks(fs);
  auto ret = gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, file_name.c_str());
  if (!warn.empty())
    Warning("[FileIO]", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", err.c_str());
  if (!ret)
    Fatal("[FIleIO]Unable to create scene from GLTF file");

  scene_from(scene, &model);
}
UberMaterial material_from(psl::string file_name, const psl::map<psl::string, Bytes> &files) {
  tinygltf::Model model;
  tinygltf::TinyGLTF gltf_ctx;
  std::string err, warn;

  auto fs = tinygltf::FsCallbacks();
  fs.FileExists = +[](const std::string &path, void *user_data) {
    Log(path.c_str());
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
  fs.user_data = (void *)&files;

  gltf_ctx.SetFsCallbacks(fs);
  auto ret = gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, file_name.c_str());
  if (!warn.empty())
    Warning("[FileIO]", warn.c_str());
  if (!err.empty())
    Fatal("[FileIO]", err.c_str());
  if (!ret)
    Fatal("[FIleIO]Unable to create scene from GLTF file");

  return material_from(&model);
}

void interpret_file(Context &context, psl::string_view filename) {
  Debug("[FileIO]Loading `", filename, "`");
  auto source = read_string_file(filename);
  interpret(context, source);
}

void fileio_context(Context &ctx) {
  ctx.type<Bytes>("Bytes");
  ctx("read_binary") = +[](psl::string_view filename) { return read_binary_file(filename); };
  ctx("load") = tag<void, Scene &, psl::string>([&](Scene &scene, psl::string file_path) {
    auto scene_name = psl::until_last_of(psl::from_last_of(file_path, '/'), '.') + ".gltf";
    auto bytes = ctx.call<Bytes>("read_binary", psl::string_view(file_path));
    auto fs = deserialize<psl::map<psl::string, Bytes>>(bytes);
    scene_from(scene, scene_name, fs);
  });
  ctx("load_material") = tag<UberMaterial, psl::string>([&](psl::string file_path) {
    auto scene_name = psl::until_last_of(psl::from_last_of(file_path, '/'), '.') + ".gltf";
    auto bytes = ctx.call<Bytes>("read_binary", psl::string_view(file_path));
    auto fs = deserialize<psl::map<psl::string, Bytes>>(bytes);
    return material_from(scene_name, fs);
  });
  ctx("load_image") = tag<Image, psl::string_view>([&](psl::string_view filename) {
    auto reader = [&ctx](psl::string_view filename) {
      return ctx.call<Bytes>("read_binary", filename);
    };
    return load_image(filename, reader);
  });
  ctx("save_image") =
      +[](psl::string filename, const Array2d3f &array) { save_image(filename, array); };
  ctx("save_image") = +[](psl::string filename, const Array2d3f &array, bool should_invert_y) {
    save_image(filename, array, should_invert_y);
  };
  ctx("save_image") =
      +[](psl::string filename, const Array2d4f &array) { save_image(filename, array); };
  ctx("save_image") = +[](psl::string filename, const Array2d4f &array, bool should_invert_y) {
    save_image(filename, array, should_invert_y);
  };
  ctx("save_image") =
      +[](psl::string filename, const Image &array) { save_image(filename, array); };
  ctx("save_image") = +[](psl::string filename, const Image &array, bool should_invert_y) {
    save_image(filename, array, should_invert_y);
  };

  ctx.type<VideoWriter>("VideoWriter")
      .ctor<psl::string, vec2i, int>()
      .method("add_frame", overloaded<const Array2d3f &>(&VideoWriter::add_frame))
      .method("add_frame", overloaded<const Array2d4f &>(&VideoWriter::add_frame))
      .method("done", &VideoWriter::done);
}

}  // namespace pine