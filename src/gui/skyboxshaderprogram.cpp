#include "skyboxshaderprogram.hpp"
#include "lights.hpp"
#include "renderwindow.hpp"

void Shift::SkyboxShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/skybox.glsl");
    addFragmentShader(":/shaders/skybox.glsl");
    link();
    bind();

    /* set uniform locations */
    m_matrices_v_u = m_program->uniformLocation("view_u");
    m_texture_cube_u = m_program->uniformLocation("texture_cube_u");
    checkError("after set skybox uniforms");
    release();
    checkError("after initialize skybox shader");
    m_initialized = true;
  }
}

void Shift::SkyboxShaderProgram::setTransform() {
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  m_program->setUniformValue(m_matrices_v_u, view);
}
