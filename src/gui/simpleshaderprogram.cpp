#include "simpleshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"

void SimpleShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/simple.vert");
    addFragmentShader(":/shaders/simple.frag");
    link();
    bind();

    /* set uniform locations */
    m_mvp_u = m_program->uniformLocation("mvp_u");
    m_color_u = m_program->uniformLocation("color_u");
    m_scale_u = m_program->uniformLocation("scale_u");

    qDebug() << "line: mvp_u: " << m_mvp_u;
    qDebug() << "line: color_u: " << m_color_u;
    qDebug() << "line: scale_u" << m_scale_u;

    /* set attribute locations */
    m_position_a = m_program->attributeLocation("position_a");
    m_program->enableAttributeArray(m_position_a);

    release();
    checkError("line: after initialize line shader");
    m_initialized = true;
  }
}


void SimpleShaderProgram::setTransform(QMatrix4x4& model) {
  bind();
  qDebug() << model;
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  qDebug() << view;
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  qDebug() << projection;
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_mvp_u, mvp);
}


void SimpleShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_mvp_u, mvp);
}


void SimpleShaderProgram::setColor(
  GLfloat red,
  GLfloat green,
  GLfloat blue,
  GLfloat alpha) {

  m_program->setUniformValue(
    m_color_u,
    red,
    green,
    blue,
    alpha);
}

void SimpleShaderProgram::setScale(GLfloat scale) {
  m_program->setUniformValue(m_scale_u, scale);
}
