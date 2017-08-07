#ifndef _STAR2_BODIES_HPP_
#define _STAR2_BODIES_HPP_

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

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;
class RenderWindow;

/*!
@brief vertex array and vertex buffer objects for each material
*/
struct BufferObjects {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  s2_material_t* material;
  s2_displacement_t* transform;
};

/*!
@brief uniform locations for each shader program
*/
struct UniformLocations {
  GLuint m_u;
  GLuint mv_u;
  GLuint mvp_u;
  GLuint nm_u;
  GLuint v_u;

  GLuint KaMaterial_u;
  GLuint KdMaterial_u;
  GLuint KsMaterial_u;
  GLuint KeMaterial_u;
  GLuint Ns_u;

  GLuint posLight_u;
  GLuint directional_u;
  GLuint Ka_u;
  GLuint Kd_u;
  GLuint Ks_u;
  GLuint lightCount_u;

  GLint position_a;
  GLint normal_a;
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

  ~s2_Bodies() {

  };

  void initialize();

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
  void setUniformLocations(
    QOpenGLShaderProgram* program,
    UniformLocations* uniforms);
  void setAttributeLocations(
    QOpenGLShaderProgram* program,
    UniformLocations* uniforms,
    BufferObjects* bufferObjects);
  void loadContext();
  void setLights(s2_Lights* lights) {
    m_lights = lights;
  }
  void setPhongShader(QOpenGLShaderProgram* program) {
    m_programPhong = program;
  }
  void setFlatShader(QOpenGLShaderProgram* program) {
    m_programFlat = program;
  }

  void draw();

private:

  QVector<BufferObjects> m_bufferObjects;

  /* uniform locations */
  UniformLocations m_uniformsFlat;
  UniformLocations m_uniformsPhong;

  s2_shading_enum_t m_shadingType;

  /* OpenGL shader programs */
  QOpenGLShaderProgram* m_programFlat;
  QOpenGLShaderProgram* m_programPhong;

  s2_Lights* m_lights;

};


#endif
