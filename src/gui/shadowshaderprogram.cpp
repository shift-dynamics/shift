#include "shadowshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"


/*!
@brief initialize frame shader and check for errors
*/
void ShadowShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/shadow_light.vert");
    addFragmentShader(":/shaders/shadow_light.frag");
    link();
    bind();

    /* set uniform locations */
    m_mvp_u = m_program->uniformLocation("mvp_u");

    release();
    checkError("after initialize shadow shader");
    m_initialized = true;
  }
}


/*!
@brief set mvp using only a transform data structure. The model and view matrices are obtained from the current activated light
*/
void ShadowShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view =
    m_renderWindow->getLights()->getCurrentViewMatrix();
  QMatrix4x4 projection =
    m_renderWindow->getLights()->getCurrentProjectionMatrix();
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
