#include "normalshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"

void NormalShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/vertex_normal.vert");
    addGeometryShader(":/shaders/vertex_normal.geom");
    addFragmentShader(":/shaders/vertex_normal.frag");
    link();
    bind();

    /* set uniform locations */
    m_matrices_mvp_u = m_program->uniformLocation("matrices_u.mvp");
    m_matrices_mv_u = m_program->uniformLocation("matrices_u.mv");
    m_matrices_v_u = m_program->uniformLocation("matrices_u.v");
    m_matrices_nm_u = m_program->uniformLocation("matrices_u.nm");
    m_scale_u = m_program->uniformLocation("scale_u");
    m_color_u = m_program->uniformLocation("color_u");
    checkError("after set normal uniforms");

    qDebug() << "  m_matrices_mvp_u:" << m_matrices_mvp_u;
    qDebug() << "  m_matrices_mv_u:" << m_matrices_mv_u;
    qDebug() << "  m_matrices_v_u:" << m_matrices_v_u;
    qDebug() << "  m_matrices_nm_u:" <<  m_matrices_nm_u;
    qDebug() << "  m_scale_u:" <<  m_scale_u;
    qDebug() << "  m_color_u:" << m_color_u;

    /* set attribute locations */
    m_position_a = m_program->attributeLocation("position_a");
    m_normal_a = m_program->attributeLocation("normal_a");

    release();
    checkError("after initialize normal shader");
    m_initialized = true;
  }
}


void NormalShaderProgram::setTransform(s2_displacement_t* transform) {

  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  // QMatrix4x4 mv = view * model;
  QMatrix4x4 mvp = projection * view * model;
  // QMatrix3x3 nm = mv.normalMatrix();

  m_program->setUniformValue(m_matrices_mvp_u, mvp);
  checkError("normal: after set mvp");
}


void NormalShaderProgram::setColor(float& r, float& g, float& b) {

  m_program->setUniformValue(m_color_u, r, g, b);
  checkError("normal: after set color");
}


void NormalShaderProgram::setScale(float& scale) {
  m_program->setUniformValue(m_scale_u, scale);
  checkError("normal: after set scale");
}
