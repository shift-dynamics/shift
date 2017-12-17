#include "shaderprogram.hpp"
#include "renderwindow.hpp"

void ShaderProgram::addProgram() {
  if(m_program == NULL) {
    m_program = new QOpenGLShaderProgram(m_renderWindow);
  }
}

void ShaderProgram::addVertexShader(const char* shaderSource) {
  m_program->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    shaderSource);
}

void ShaderProgram::addGeometryShader(const char* shaderSource) {
  m_program->addShaderFromSourceFile(
    QOpenGLShader::Geometry,
    shaderSource);
}

void ShaderProgram::addTessellationControlShader(const char* shaderSource) {
  m_program->addShaderFromSourceFile(
    QOpenGLShader::TessellationControl,
    shaderSource);
}

void ShaderProgram::addTessellationEvaluationShader(const char* shaderSource) {
  m_program->addShaderFromSourceFile(
    QOpenGLShader::TessellationEvaluation,
    shaderSource);
}

void ShaderProgram::addFragmentShader(const char* shaderSource) {
  m_program->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    shaderSource);
}

void ShaderProgram::addComputeShader(const char* shaderSource) {
  m_program->addShaderFromSourceFile(
    QOpenGLShader::Compute,
    shaderSource);
}

void ShaderProgram::link() {
  m_program->link();
  m_linked = true;
}

void ShaderProgram::bind() {
  if(!m_bound) {
    m_program->bind();
    m_bound = true;
  }
}

void ShaderProgram::release() {
  if(m_bound) {
    m_program->release();
    m_bound = false;
  }
}

void ShaderProgram::checkError(const QString& prefix) {
  m_renderWindow->checkError(prefix);
}
