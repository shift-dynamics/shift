#include "shadowshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"


/*!
@brief initialize frame shader and check for errors
*/
void ShadowShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/frame.vert");
    addFragmentShader(":/shaders/frame.frag");
    link();
    bind();

    /* set uniform locations */
    m_mvp_u = m_program->uniformLocation("mvp_u");
    m_scale_u = m_program->uniformLocation("scale_u");

    release();
    checkError("after initialize line shader");
    m_initialized = true;
  }
}


/*!
@brief set mvp using only a transform data structure. The model and view matrices are obtained from the current render window.
*/
void ShadowShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_mvp_u, mvp);
  checkError("after set transform");
}


/*!
@brief set mvp matrix
*/
void ShadowShaderProgram::setMVP(QMatrix4x4& mvp) {
  m_program->setUniformValue(m_mvp_u, mvp);
  checkError("after set transform");
}


/*!
@brief set scale factor
*/
void ShadowShaderProgram::setScale(GLfloat scale) {
  m_program->setUniformValue(m_scale_u, scale);
}
