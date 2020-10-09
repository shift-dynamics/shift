#include "shift/gui/mesh.h"

#include <QFile>
#include <QtConcurrent/QtConcurrent>

void Mesh::load(const QString& fn) {
  qDebug() << "loading string" << fn;

  reset();
  maybeRunning_ = true;

  future_ = QtConcurrent::run([fn]() {
    MeshData md;
    QFile f(fn);
    if(!f.open(QIODevice::ReadOnly)) {
      qWarning("Failed to open %s", qPrintable(fn));
      return md;
    }
    QByteArray buf = f.readAll();
    const char* p = buf.constData();
    quint32 format;
    memcpy(&format, p, 4);
    if(format != 1) {
      qWarning("Invalid format in %s", qPrintable(fn));
      return md;
    }
    int ofs = 4;
    memcpy(&md.vertexCount, p + ofs, 4);
    ofs += 4;
    memcpy(md.aabb, p + ofs, 6 * 4);
    ofs += 6 * 4;
    const int byteCount = md.vertexCount * 8 * 4;
    md.geom.resize(byteCount);
    memcpy(md.geom.data(), p + ofs, byteCount);
    return md;
  });
}

MeshData* Mesh::data() {
  if(maybeRunning_ && !data_.isValid())
    data_ = future_.result();

  return &data_;
}

void Mesh::reset() {
  *data() = MeshData();
  maybeRunning_ = false;
}
