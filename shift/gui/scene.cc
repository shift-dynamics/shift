#include "shift/gui/scene.h"

#include <QtDebug>

#include <algorithm>

void shift::gui::SceneBounds(const Scene& scene, const Poses& poses,
                             QVector3D& box_min, QVector3D& box_max) {
  box_min = QVector3D(1e30f, 1e30f, 1e30f);
  box_max = -box_min;
  for(const auto& instance : scene.instances) {
    const SceneMesh& mesh = scene.meshes[instance.mesh_index];
    if(mesh.terrain >= 0) {
      continue; // frame the model, not the ground it stands on
    }
    const QMatrix4x4& pose = poses.bodies.at(instance.body_index);
    for(int corner = 0; corner < 8; corner++) {
      const QVector3D local((corner & 1) ? mesh.bounds_max.x() : mesh.bounds_min.x(),
                            (corner & 2) ? mesh.bounds_max.y() : mesh.bounds_min.y(),
                            (corner & 4) ? mesh.bounds_max.z() : mesh.bounds_min.z());
      const QVector3D world = pose.map(local);
      for(int k = 0; k < 3; k++) {
        box_min[k] = std::min(box_min[k], world[k]);
        box_max[k] = std::max(box_max[k], world[k]);
      }
    }
  }
  for(const auto& cad : scene.cads) {
    if(cad.patches.empty()) {
      continue;
    }
    for(int body : cad.bodies) {
      const QMatrix4x4& pose = poses.bodies.at(body);
      for(int corner = 0; corner < 8; corner++) {
        const QVector3D local(
          (corner & 1) ? cad.bounds_max.x() : cad.bounds_min.x(),
          (corner & 2) ? cad.bounds_max.y() : cad.bounds_min.y(),
          (corner & 4) ? cad.bounds_max.z() : cad.bounds_min.z());
        const QVector3D world = pose.map(local);
        for(int k = 0; k < 3; k++) {
          box_min[k] = std::min(box_min[k], world[k]);
          box_max[k] = std::max(box_max[k], world[k]);
        }
      }
    }
  }
  if(box_min.x() > box_max.x()) {
    box_min = QVector3D(-1.f, -1.f, -1.f);
    box_max = QVector3D(1.f, 1.f, 1.f);
  }
}

void shift::gui::TextureLoader::AddEmbedded(
  const std::map<std::string, EmbeddedTexture>& images) {
  for(const auto& [key, image] : images) {
    embedded_[key] = &image;
  }
}

QImage shift::gui::TextureLoader::Read(const std::string& path) {
  QImage image;
  auto embedded = embedded_.find(path);
  if(embedded == embedded_.end()) {
    // qrc:/textures/... is a texture built into the viewer, which Qt names
    // :/textures/...
    const bool resource = path.rfind("qrc:/", 0) == 0;
    image.load(QString::fromStdString(resource ? path.substr(3) : path));
  } else if(embedded->second->height == 0) {
    // an encoded image file
    const auto& data = embedded->second->data;
    image.loadFromData(data.data(), static_cast<int>(data.size()));
  } else {
    // B G R A texels, as Qt's ARGB32 on little endian machines
    const EmbeddedTexture& t = *embedded->second;
    image = QImage(t.data.data(), int(t.width), int(t.height),
                   int(4 * t.width), QImage::Format_ARGB32)
              .copy();
  }
  if(image.isNull()) {
    qWarning("Cannot read texture %s", path.c_str());
    return image;
  }
  const int size = Scene::kTextureSize;
  if(image.width() != size || image.height() != size) {
    image = image.scaled(size, size, Qt::IgnoreAspectRatio,
                         Qt::SmoothTransformation);
  }
  return image.convertToFormat(QImage::Format_RGBA8888);
}

int shift::gui::TextureLoader::Load(const std::string& path) {
  if(path.empty()) {
    return -1;
  }
  auto it = layers_.find(path);
  if(it != layers_.end()) {
    return it->second;
  }
  QImage image = Read(path);
  int layer = -1;
  if(!image.isNull()) {
    layer = static_cast<int>(scene_.texture_layers.size());
    scene_.texture_layers.push_back(std::move(image));
  }
  layers_[path] = layer;
  return layer;
}

int shift::gui::TextureLoader::LoadMetallicRoughness(
  const std::string& packed, const std::string& metallic,
  const std::string& roughness) {
  if(!packed.empty()) {
    return Load(packed);
  }
  if(metallic.empty() && roughness.empty()) {
    return -1;
  }
  const std::string key = "metallic:" + metallic + "|roughness:" + roughness;
  auto it = layers_.find(key);
  if(it != layers_.end()) {
    return it->second;
  }
  // channels without an image are 1, so the factor alone applies
  const int size = Scene::kTextureSize;
  QImage combined(size, size, QImage::Format_RGBA8888);
  combined.fill(Qt::white);
  auto copy_channel = [&](const std::string& path, int channel) {
    if(path.empty()) {
      return;
    }
    QImage image = Read(path);
    if(image.isNull()) {
      return;
    }
    // single channel images: red equals the gray value
    for(int y = 0; y < size; y++) {
      const uchar* in = image.constScanLine(y);
      uchar* out = combined.scanLine(y);
      for(int x = 0; x < size; x++) {
        out[4 * x + channel] = in[4 * x];
      }
    }
  };
  copy_channel(roughness, 1);
  copy_channel(metallic, 2);
  const int layer = static_cast<int>(scene_.texture_layers.size());
  scene_.texture_layers.push_back(std::move(combined));
  layers_[key] = layer;
  return layer;
}
