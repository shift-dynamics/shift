#ifndef _STAR2_FLATSHADERPROGRAM_HPP_
#define _STAR2_FLATSHADERPROGRAM_HPP_

#include "../star2.h"
#include "openglwindow.hpp"
#include "shaderprogram.hpp"

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
class s2_Lights;


class FlatShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  FlatShaderProgram(RenderWindow* renderWindow)
  : ShaderProgram(renderWindow) {

  };

  ~FlatShaderProgram() {};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setMaterial(s2_material_t*);
  void setLights(s2_Lights*);
  GLint getPositionAttribute() {
    return m_position_a;
  };
  GLint getNormalAttribute() {
    return m_normal_a;
  };

signals:


protected:


private:

  GLuint m_matrices_u;
  GLuint m_matrices_mvp_u;
  GLuint m_matrices_mv_u;
  GLuint m_matrices_v_u;
  GLuint m_matrices_nm_u;

  GLuint m_material_u;
  GLuint m_material_Ka_u;
  GLuint m_material_Kd_u;
  GLuint m_material_Ks_u;
  GLuint m_material_Ke_u;
  GLuint m_material_Ns_u;

  GLuint m_lights_u;
  GLuint m_lights_position_u;
  GLuint m_lights_Ka_u;
  GLuint m_lights_Kd_u;
  GLuint m_lights_Ks_u;
  GLuint m_lights_directional_u;
  GLuint m_lights_count_u;

  GLint m_position_a;
  GLint m_normal_a;

};

#endif
