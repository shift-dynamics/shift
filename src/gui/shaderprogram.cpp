#include "shaderprogram.hpp"
#include "renderwindow.hpp"

#include <QTextStream>

void ShaderProgram::addProgram() {
  if(m_program == NULL) {
    m_program = new QOpenGLShaderProgram(m_renderWindow);
  }
}

QString ShaderProgram::readSource(
  const char* shaderSource, const char* define) {
  QFile file(shaderSource);
  file.open(QIODevice::ReadOnly);
  QString sourceCode;
  QTextStream s(&file);
  sourceCode.append(s.readAll());
  QString defineStr(define);
  sourceCode.insert(sourceCode.indexOf("\n"), "\n#define " + defineStr + "\n");
  return sourceCode;
}

void ShaderProgram::addVertexShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "VERTEX_SHADER");
  m_program->addShaderFromSourceCode(
    QOpenGLShader::Vertex,
    shaderSourceCode);
}

void ShaderProgram::addGeometryShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "GEOMETRY_SHADER");
  m_program->addShaderFromSourceCode(
    QOpenGLShader::Geometry,
    shaderSourceCode);
}

void ShaderProgram::addTessellationControlShader(const char* shaderSource) {
  QString shaderSourceCode =
    readSource(shaderSource, "TESSELATION_CONTROL_SHADER");
  m_program->addShaderFromSourceCode(
    QOpenGLShader::TessellationControl,
    shaderSourceCode);
}

void ShaderProgram::addTessellationEvaluationShader(const char* shaderSource) {
  QString shaderSourceCode =
    readSource(shaderSource, "TESSELATION_EVALUATION_SHADER");
  m_program->addShaderFromSourceCode(
    QOpenGLShader::TessellationEvaluation,
    shaderSourceCode);
}

void ShaderProgram::addFragmentShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "FRAGMENT_SHADER");
  m_program->addShaderFromSourceCode(
    QOpenGLShader::Fragment,
    shaderSourceCode);
}

void ShaderProgram::addComputeShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "COMPUTE_SHADER");
  m_program->addShaderFromSourceCode(
    QOpenGLShader::Compute,
    shaderSourceCode);
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
