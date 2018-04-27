#include "floorshadowshaderprogram.h"
#include "lights.h"
#include "renderwindow.h"

/*!
@brief initialize frame shader and check for errors
*/
void Shift::FloorShadowShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/floor_shadow.glsl");
    addFragmentShader(":/shaders/floor_shadow.glsl");
    link();
    bind();

    /* set uniform locations */
    m_matrices_mvp_u = m_program->uniformLocation("matrices_u.mvp");
    qDebug() << "m_matrices_mvp_u:" << m_matrices_mvp_u;

    m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
    m_vao->create();

    release();
    checkError(
      "FloorShadowShaderProgram::initialize(): after initialize shadow shader");
    m_initialized = true;
  }
}

/*!
@brief set mvp matrix
*/
void Shift::FloorShadowShaderProgram::setTransform(QMatrix4x4& m) {
  QMatrix4x4 v = m_renderWindow->getLights()->getActiveViewMatrix();
  QMatrix4x4 p = m_renderWindow->getLights()->getActiveProjectionMatrix();
  QMatrix4x4 mvp = p * v * m;
  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  checkError("terrain-shadow: after set mvp");
}

/*!
@brief set transform
*/
void Shift::FloorShadowShaderProgram::setTransform(
  s2_displacement_t* transform) {
  QMatrix4x4 m = displacementToQMatrix4x4(transform);
  setTransform(m);
}
