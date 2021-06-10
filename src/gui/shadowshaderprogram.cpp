#include "shadowshaderprogram.h"
#include "lights.h"
#include "renderwindow.h"

/*!
@brief initialize frame shader and check for errors
*/
void Shift::ShadowShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/shadow_light.glsl");
    addFragmentShader(":/shaders/shadow_light.glsl");
    link();
    bind();

    /* set uniform locations */
    m_mvp_u = m_program->uniformLocation("mvp_u");

    /* set attribute locations */
    m_position_a = m_program->attributeLocation("position_a");
    m_program->enableAttributeArray(m_position_a);
    release();
    checkError("after initialize shadow shader");
    m_initialized = true;
  }
}

/*!
@brief set mvp using only a transform data structure. The model and view
matrices are obtained from the current activated light
*/
void Shift::ShadowShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getLights()->getActiveViewMatrix();
  QMatrix4x4 projection =
    m_renderWindow->getLights()->getActiveProjectionMatrix();
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_mvp_u, mvp);
  checkError("after set transform");
}
