#ifndef _SHIFT_FLOORSHADERPROGRAM_HPP_
#define _SHIFT_FLOORSHADERPROGRAM_HPP_

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

class FloorShaderProgram : public ShaderProgram {

  Q_OBJECT

public:
  FloorShaderProgram(RenderWindow* renderWindow)
    : ShaderProgram(renderWindow) {
    initialize();
  };

  ~FloorShaderProgram(){};

  void initialize();
  void draw();
  void setTransform(s2_displacement_t*);
  void setTransform(QMatrix4x4&);
  void setScale(GLfloat);
  void bindVao();
  void releaseVao();
  void bindDiffuseTexture(QOpenGLTexture*);
  void bindNormalTexture(QOpenGLTexture*);
  void bindSpecularTexture(QOpenGLTexture*);
  void bindBumpTexture(QOpenGLTexture*);

signals:

protected:
private:
  /* texture uniform locations */
  GLint m_texture_shadow_u;
  GLint m_texture_shadowCube_u;
  GLint m_texture_diffuse_u;
  GLint m_texture_normal_u;
  GLint m_texture_specular_u;
  GLint m_texture_bump_u;

  /* matrix uniform locations */
  GLint m_matrices_mvp_u;
  GLint m_matrices_mv_u;
  GLint m_matrices_v_u;
  GLint m_matrices_p_u;
  GLint m_matrices_nm_u;

  /* material uniform locations */
  GLint m_material_Kd_u;
  GLint m_material_KaOverKd_u;
  GLint m_material_KsOverKd_u;
  GLint m_material_Ns_u;

  /* light uniform locations */
  GLint m_lights_position_u;
  GLint m_lights_Ka_u;
  GLint m_lights_Kd_u;
  GLint m_lights_Ks_u;
  GLint m_lights_type_u;
  GLint m_lights_count_u;
  GLint m_lights_drawShadows_u;
  GLint m_lights_shadowMatrix_u;

  /* terrain uniform locations */
  GLint m_floor_tcScale_u;
  GLint m_floor_enableNormalMap_u;
  GLint m_floor_enableSpecularMap_u;
  GLint m_floor_enableBumpMap_u;

  QOpenGLVertexArrayObject* m_vao;
};
} // namespace Shift
#endif
