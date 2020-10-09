#ifndef SHIFTGUI_MESH_H_
#define SHIFTGUI_MESH_H_

#include <QFuture>
#include <QString>

struct MeshData {
  bool isValid() const {
    return vertexCount > 0;
  }
  int vertexCount = 0;
  float aabb[6];
  QByteArray geom; // x, y, z, u, v, nx, ny, nz
};

class Mesh {
public:
  void load(const QString& fn);
  MeshData* data();
  bool isValid() {
    return data()->isValid();
  }
  void reset();

private:
  bool maybeRunning_ = false;
  QFuture<MeshData> future_;
  MeshData data_;
};

#endif
