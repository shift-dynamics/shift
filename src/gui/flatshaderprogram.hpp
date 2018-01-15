#ifndef _SHIFT_FLATSHADERPROGRAM_HPP_
#define _SHIFT_FLATSHADERPROGRAM_HPP_

#include "../shift.h"
#include "openglwindow.hpp"
#include "shaderprogram.hpp"

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

class QOpenGLShaderProgram;
class QOpenGLBuffer;
class QOpenGLVertexArrayObject;

namespace Shift {

class RenderWindow;
class Lights;

class FlatShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  FlatShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~FlatShaderProgram(){};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setMaterial(s2_material_t*);
  void setLights(Lights*);
  GLint getPositionAttribute() {
    return m_position_a;
  };
  GLint getNormalAttribute() {
    return m_normal_a;
  };

signals:

protected:
private:
  GLint m_matrices_u;
  GLint m_matrices_mvp_u;
  GLint m_matrices_mv_u;
  GLint m_matrices_v_u;
  GLint m_matrices_nm_u;

  GLint m_material_u;
  GLint m_material_Ka_u;
  GLint m_material_Kd_u;
  GLint m_material_Ks_u;
  GLint m_material_Ke_u;
  GLint m_material_Ns_u;

  GLint m_lights_u;
  GLint m_lights_position_u;
  GLint m_lights_Ka_u;
  GLint m_lights_Kd_u;
  GLint m_lights_Ks_u;
  GLint m_lights_type_u;
  GLint m_lights_count_u;

  GLint m_position_a;
  GLint m_normal_a;
};

} // namespace Shift

#endif
