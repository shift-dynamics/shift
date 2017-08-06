#ifndef _STAR2_GRID_HPP_
#define _STAR2_GRID_HPP_

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
class s2_Geometry;


class s2_Grid : public s2_Geometry {

  Q_OBJECT

public:
  s2_Grid(RenderWindow* renderWindow, int gridSize=100, float scaleFactor=1.f)
  : s2_Geometry(renderWindow)
  , m_material(NULL)
  , m_transform(NULL) {

    m_gridSize = gridSize;
    m_scaleFactor = scaleFactor;
    m_model.setToIdentity();

    m_color[0] = 36.f / 255.f;
    m_color[1] = 36.f / 255.f;
    m_color[2] = 36.f / 255.f;
    m_color[3] = 0.9f;
  };

  ~s2_Grid() {
    s2_materials_free(m_material, 1);
    free(m_transform);
  }

  void initialize();
  void draw();

public slots:

  void setGridSize(int gridSize, float scaleFactor) {
    m_gridSize = gridSize;
    m_scaleFactor = scaleFactor;
  }

signals:


protected:


private:

  int m_gridSize;
  float m_scaleFactor;
  QMatrix4x4 m_model;

  s2_shading_enum_t m_shadingType;
  GLfloat m_color[4];

  s2_material_t* m_material;
  s2_displacement_t* m_transform;

  QOpenGLVertexArrayObject* m_vao;
  QOpenGLBuffer* m_vbo;
  QOpenGLBuffer* m_ibo;

  /* attributes */
  GLuint m_position_a;

  /* uniforms */
  GLuint m_color_u;
  GLuint m_mvp_u;

  //
  // BufferObjects* faceBufferObject;
  // BufferObjects* lineBufferObject;

};



#endif
