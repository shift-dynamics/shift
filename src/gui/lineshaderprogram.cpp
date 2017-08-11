#include "lineshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"

void LineShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/line.vert");
    addFragmentShader(":/shaders/line.frag");
    link();
    bind();

    /* set uniform locations */
    m_mvp_u = m_program->uniformLocation("mvp_u");
    m_color_u = m_program->uniformLocation("color_u");
    m_scale_u = m_program->uniformLocation("scale_u");

    /* set attribute locations */
    m_position_a = m_program->attributeLocation("position_a");
    m_program->enableAttributeArray(m_position_a);
    m_program->setAttributeBuffer(m_position_a, GL_FLOAT, 0, 3);

    release();
    checkError("after initialize line shader");
    m_initialized = true;
  }
}


void LineShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_mvp_u, mvp);
  checkError("after set transform");
}


void LineShaderProgram::setColor(
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

  checkError("after set color");
}

void LineShaderProgram::setScale(GLfloat scale) {
  m_program->setUniformValue(m_scale_u, scale);
}
