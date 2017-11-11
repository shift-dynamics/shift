#ifndef _STAR2_POINTCLOUD_HPP_
#define _STAR2_POINTCLOUD_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "geometry.hpp"
#include "lights.hpp"

#include <QString>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QtOpenGL/QGLWidget>
#include <QHBoxLayout>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QWidget>
#include <QString>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QResizeEvent>
#include <QTimer>
#include <QOpenGLTexture>

#include "pointcloudshaderprogram.hpp"

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class RenderWindow;
class PointCloudShaderProgram;
struct s2_lights_t;
struct s2_pointcloud_t;

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
class s2_PointCloud : public s2_Geometry {

  Q_OBJECT

public:
  s2_PointCloud(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_shadingType(S2_POINT_SHADING_SIMPLE) {

  };

  ~s2_PointCloud() {

  };

  void initialize();

  void initializeCloud(s2_body_t* body);

  void initializeMaterial(s2_material_t*, s2_displacement_t*);

  void setDrawStyle(s2_point_shading_enum_t shadingType);

  void loadContext();

  void clearContext();

  void setLights(s2_Lights*);

  void setShader(PointCloudShaderProgram*);

  void setAttributeLocations(
    QOpenGLShaderProgram*,
    GLint,
    GLint,
    pointcloud_buffer_t*);

  QVector<pointcloud_buffer_t>* getBufferObjects() {
    return &m_bufferObjects;
  }

  void draw();
  void drawShadows();

public slots:

  void setFlatShaded() {
    setDrawStyle(S2_SHADING_FLAT);
  }

  void setPhongShaded() {
    setDrawStyle(S2_SHADING_PHONG);
  }

  void setWireframe() {
    setDrawStyle(S2_SHADING_WIREFRAME);
  }


private:

  QVector<pointcloud_buffer_t> m_bufferObjects;

  s2_point_shading_enum_t m_shadingType;

  /* OpenGL shader programs */
  PointCloudShaderProgram* m_program;

  ShadowShaderProgram* m_programShadow;

  s2_Lights* m_lights;

};


#endif
