#include "floor.hpp"

void Shift::Floor::initialize() {
  // if(!m_initialized) {
  //   addProgram();
  //   addVertexShader(":/shaders/floor.glsl");
  //   addFragmentShader(":/shaders/floor.glsl");
  //   link();
  //   bind();
  //   checkError("Floor::initialize(): after link program");
  //
  //   m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
  //   m_vao->create();
  //   m_vao->bind();
  //
  //   /* set uniform locations */
  //   m_mvp_u = m_program->uniformLocation("mvp_u");
  //   m_scale_u = m_program->uniformLocation("scale_u");
  //
  //   // qDebug() << "m_mvp_u: " << m_mvp_u;
  //   // qDebug() << "m_scale_u: " << m_scale_u;
  //
  //   m_vao->release();
  //   release();
  //   checkError("frame: after initialize");
  //   m_initialized = true;
  // }
}
