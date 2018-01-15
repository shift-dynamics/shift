#include "shaderprogram.hpp"
#include "renderwindow.hpp"

#include <QTextStream>

void Shift::ShaderProgram::addProgram() {
  if(m_program == NULL) {
    m_program = new QOpenGLShaderProgram(m_renderWindow);
  }
}

QString Shift::ShaderProgram::readSource(const char* shaderSource,
                                         const char* define) {
  QFile file(shaderSource);
  file.open(QIODevice::ReadOnly);
  QString sourceCode;
  QTextStream s(&file);
  sourceCode.append(s.readAll());
  QString defineStr(define);
  sourceCode.insert(sourceCode.indexOf("\n"), "\n#define " + defineStr + "\n");
  return sourceCode;
}

void Shift::ShaderProgram::addVertexShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "VERTEX_SHADER");
  m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, shaderSourceCode);
}

void Shift::ShaderProgram::addGeometryShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "GEOMETRY_SHADER");
  m_program->addShaderFromSourceCode(QOpenGLShader::Geometry, shaderSourceCode);
}

void Shift::ShaderProgram::addTessellationControlShader(
  const char* shaderSource) {
  QString shaderSourceCode =
    readSource(shaderSource, "TESSELATION_CONTROL_SHADER");
  m_program->addShaderFromSourceCode(QOpenGLShader::TessellationControl,
                                     shaderSourceCode);
}

void Shift::ShaderProgram::addTessellationEvaluationShader(
  const char* shaderSource) {
  QString shaderSourceCode =
    readSource(shaderSource, "TESSELATION_EVALUATION_SHADER");
  m_program->addShaderFromSourceCode(QOpenGLShader::TessellationEvaluation,
                                     shaderSourceCode);
}

void Shift::ShaderProgram::addFragmentShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "FRAGMENT_SHADER");
  m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, shaderSourceCode);
}

void Shift::ShaderProgram::addComputeShader(const char* shaderSource) {
  QString shaderSourceCode = readSource(shaderSource, "COMPUTE_SHADER");
  m_program->addShaderFromSourceCode(QOpenGLShader::Compute, shaderSourceCode);
}

void Shift::ShaderProgram::link() {
  m_program->link();
  m_linked = true;
}

void Shift::ShaderProgram::bind() {
  if(!m_bound) {
    m_program->bind();
    m_bound = true;
  }
}

void Shift::ShaderProgram::release() {
  if(m_bound) {
    m_program->release();
    m_bound = false;
  }
}

void Shift::ShaderProgram::checkError(const QString& prefix) {
  m_renderWindow->checkError(prefix);
}
