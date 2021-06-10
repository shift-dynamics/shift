#ifndef _SHIFT_GRID_HPP_
#define _SHIFT_GRID_HPP_

#include "geometry.h"
#include "openglwindow.h"
#include "shift.h"

#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>

namespace Shift {

class RenderWindow;
class SimpleShaderProgram;

class Grid : public Geometry {

  Q_OBJECT

public:
  Grid(RenderWindow* renderWindow, int gridSize = 100, float scaleFactor = 1.f)
    : Geometry(renderWindow)
    , m_material(NULL)
    , m_transform(NULL)
    , m_program(NULL) {

    m_gridSize = gridSize;
    m_scaleFactor = scaleFactor;
    m_model.setToIdentity();

    m_color[0] = 36.f / 255.f;
    m_color[1] = 36.f / 255.f;
    m_color[2] = 36.f / 255.f;
    m_color[3] = 0.9f;
  };

  ~Grid() {
    s2_material_free(m_material, 1);
    free(m_transform);
  };

  void initialize();
  void draw();
  void setShader(SimpleShaderProgram* program) {
    m_program = program;
    initialize();
  }
  void bindVao() {
    m_vao->bind();
  }

public slots:

  void setGridSize(int gridSize, float scaleFactor) {
    m_gridSize = gridSize;
    m_scaleFactor = scaleFactor;
  }

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

  SimpleShaderProgram* m_program;
};
} // namespace Shift

#endif
