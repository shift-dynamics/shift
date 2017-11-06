#ifndef _STAR2_PHONGSHADERPROGRAM_HPP_
#define _STAR2_PHONGSHADERPROGRAM_HPP_

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


class PhongShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  PhongShaderProgram(RenderWindow* renderWindow)
  : ShaderProgram(renderWindow) {
    initialize();
  };

  ~PhongShaderProgram() {};

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
  void bindDepthBuffer();

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
  GLint m_lights_directional_u;
  GLint m_lights_count_u;
  GLint m_lights_drawShadows_u;
  GLint m_lights_shadowMatrix_u;

  GLint m_texture_shadow_u;

  GLint m_position_a;
  GLint m_normal_a;

};



#endif
