#ifndef _SHIFT_POINTCLOUD_HPP_
#define _SHIFT_POINTCLOUD_HPP_

#include "../shift.h"
#include "geometry.hpp"
#include "lights.hpp"
#include "openglwindow.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QResizeEvent>
#include <QString>
#include <QTimer>
#include <QWheelEvent>
#include <QWidget>
#include <QtOpenGL/QGLWidget>

#include "pointcloudshaderprogram.hpp"

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

struct s2_lights_t;
struct s2_pointcloud_t;

namespace Shift {

class RenderWindow;

/*!
@brief vertex array and vertex buffer objects for each material
*/
struct pointcloud_buffer_t {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* colorBuffer;
  s2_pointcloud_t* pointcloud;
  s2_displacement_t* transform;
};

/*!
@brief used to set up visualization assets associated with s2_body_t data
structure
*/
class PointCloud : public Geometry {

  Q_OBJECT

public:
  PointCloud(RenderWindow* renderWindow)
    : Geometry(renderWindow)
    , m_shadingType(S2_POINT_SHADING_SIMPLE)
    , m_drawEnabled(true){};

  ~PointCloud(){};

  void initialize();

  void initializePointCloud(s2_pointcloud_t*);

  void setDrawStyle(s2_point_shading_enum_t shadingType);

  void loadContext();

  void clearContext();

  void setLights(Lights*);

  void setShader(PointCloudShaderProgram* program) {
    m_program = program;
  };

  void setAttributeLocations(QOpenGLShaderProgram*, GLint, GLint,
                             pointcloud_buffer_t*);

  QVector<pointcloud_buffer_t>* getBufferObjects() {
    return &m_bufferObjects;
  }

  bool isEnabled() {
    return m_drawEnabled;
  }

  void draw();
  void drawShadows();

public slots:

  void setPhongShaded() {
    setDrawStyle(S2_POINT_SHADING_PHONG);
  }

  void setSpriteShaded() {
    setDrawStyle(S2_POINT_SHADING_SPRITE);
  }

  void setSimpleShaded() {
    setDrawStyle(S2_POINT_SHADING_SIMPLE);
  }

private:
  QVector<pointcloud_buffer_t> m_bufferObjects;

  s2_point_shading_enum_t m_shadingType;

  /* OpenGL shader programs */
  PointCloudShaderProgram* m_program;

  ShadowShaderProgram* m_programShadow;

  Lights* m_lights;

  bool m_drawEnabled;
};

} // namespace Shift

#endif
