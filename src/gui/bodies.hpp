#ifndef _STAR2_BODIES_HPP_
#define _STAR2_BODIES_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "geometry.hpp"

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

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class RenderWindow;
/*!
@brief vertex array and vertex buffer objects for each material
*/
typedef struct BufferObjects {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  s2_material_t* material;
  s2_displacement_t* transform;
} BufferObjects;

struct UniformLocations {
  GLuint m;
  GLuint mv;
  GLuint mvp;
  GLuint nm;
  GLuint v;

  GLuint KaMaterial;
  GLuint KdMaterial;
  GLuint KsMaterial;
  GLuint KeMaterial;
  GLuint Ns;

  GLuint posLight;
  GLuint directional;
  GLuint Ka;
  GLuint Kd;
  GLuint Ks;
  GLuint lightCount;

  GLint position;
  GLint normal;
};

/*!
@brief used to set up visualization assets associated with s2_body_t data
structure
*/
class s2_Bodies : public s2_Geometry {

  Q_OBJECT

public:
  s2_Bodies(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow) {
    m_shadingType = PHONG;
  };

  virtual void initialize();

  void initializeBody(
    s2_body_t* body);

  void initializeMaterial(
    s2_material_t* material,
    s2_displacement_t* origin);

  void setDrawStyle(s2_shading_enum_t shadingType) {
    switch(shadingType) {
      case WIREFRAME: {
        m_shadingType = WIREFRAME;
      } break;
      default: {
        m_shadingType = PHONG;
      } break;
    }
  }

  void compileShader();
  void loadContext();

  virtual void draw();

private:

  QVector<BufferObjects> m_bufferObjects;

  /* uniform locations */
  UniformLocations m_uniformLocationsFlat;
  UniformLocations m_uniformLocationsPhong;

  s2_shading_enum_t m_shadingType;

  QOpenGLShaderProgram* m_programFlat;
  QOpenGLShaderProgram* m_programPhong;

};


#endif
