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
class PhongShaderProgram;
class FlatShaderProgram;
struct s2_lights_t;

/*!
@brief vertex array and vertex buffer objects for each material
*/
struct buffer_objects_t {
  QOpenGLVertexArrayObject* vao;
  QOpenGLBuffer* vertexBuffer;
  QOpenGLBuffer* vertexNormalBuffer;
  QOpenGLBuffer* ibo;
  s2_material_t* material;
  s2_displacement_t* transform;
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

  void initializeBody(s2_body_t* body);

  void initializeMaterial(s2_material_t*, s2_displacement_t*);

  void setDrawStyle(s2_shading_enum_t shadingType);
  void loadContext();
  void setLights(s2_Lights*);
  void setFlatShader(FlatShaderProgram*);
  void setPhongShader(PhongShaderProgram*);

  void setAttributeLocations(
    QOpenGLShaderProgram*,
    GLint,
    GLint,
    buffer_objects_t*);

  void draw();

private:

  QVector<buffer_objects_t> m_bufferObjects;

  s2_shading_enum_t m_shadingType;

  /* OpenGL shader programs */
  FlatShaderProgram* m_programFlat;
  PhongShaderProgram* m_programPhong;

  s2_Lights* m_lights;

};


#endif
