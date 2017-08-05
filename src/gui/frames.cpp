#include "frames.hpp"

/*!
@brief initialize frames
*/
void s2_Frames::initialize() {

  /* set vertices */
  m_vertices[0] = 0.f;
  m_vertices[1] = 0.f;
  m_vertices[2] = 0.f;

  m_vertices[3] = 1.0f;
  m_vertices[4] = 0.f;
  m_vertices[5] = 0.f;

  m_vertices[6] = 0.f;
  m_vertices[7] = 0.f;
  m_vertices[8] = 0.f;

  m_vertices[9] = 0.f;
  m_vertices[10] = 1.0f;
  m_vertices[11] = 0.f;

  m_vertices[12] = 0.f;
  m_vertices[13] = 0.f;
  m_vertices[14] = 0.f;

  m_vertices[15] = 0.f;
  m_vertices[16] = 0.f;
  m_vertices[17] = 1.0f;

  /* set colors */
  m_colors[0] = 1.f;
  m_colors[1] = 0.f;
  m_colors[2] = 0.f;

  m_colors[3] = 1.f;
  m_colors[4] = 0.f;
  m_colors[5] = 0.f;

  m_colors[6] = 0.f;
  m_colors[7] = 1.f;
  m_colors[8] = 0.f;

  m_colors[9] = 0.f;
  m_colors[10] = 1.f;
  m_colors[11] = 0.f;

  m_colors[12] = 0.f;
  m_colors[13] = 0.f;
  m_colors[14] = 1.f;

  m_colors[15] = 0.f;
  m_colors[16] = 0.f;
  m_colors[17] = 1.f;

  /* set indices */
  m_indices[0] = 0;
  m_indices[1] = 1;
  m_indices[2] = 2;
  m_indices[3] = 3;
  m_indices[4] = 4;
  m_indices[5] = 5;

  m_program->bind();
  m_renderWindow->checkError("after frame program bind");

  m_vao = new QOpenGLVertexArrayObject(static_cast <QWindow*>(m_renderWindow));
  m_vao->create();
  m_vao->bind();

  m_uniformsFrame.vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_uniformsFrame.vbo->create();

  m_uniformsFrame.vbo->bind();
  m_uniformsFrame.vbo->allocate(2 * 18 * sizeof(GLfloat));
  m_uniformsFrame.vbo->write(
    0, m_uniformsFrame.vertices, 18 * sizeof(GLfloat));
  m_uniformsFrame.vbo->write(
    18 * sizeof(GLfloat), m_uniformsFrame.colors, 18 * sizeof(GLfloat));

  m_uniformsFrame.ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  m_uniformsFrame.ibo->create();
  m_uniformsFrame.ibo->bind();
  m_uniformsFrame.ibo->allocate(m_uniformsFrame.indices, 6 * sizeof(GLuint));

  m_uniformsFrame.position = m_programFrame->attributeLocation("posAttr");
  m_programFrame->enableAttributeArray(m_uniformsFrame.position);
  m_programFrame->setAttributeBuffer(m_uniformsFrame.position, GL_FLOAT, 0, 3);

  m_uniformsFrame.color = m_programFrame->attributeLocation("colorAttr");
  m_programFrame->enableAttributeArray(m_uniformsFrame.color);
  m_programFrame->setAttributeBuffer(
    m_uniformsFrame.color, GL_FLOAT, 18 * sizeof(GLfloat), 3);

  m_programFrame->release();

  checkError("after set up frame shader");
}
