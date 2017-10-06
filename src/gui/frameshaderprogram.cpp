#include "frameshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"


/*!
@brief initialize frame shader and check for errors
*/
void FrameShaderProgram::initialize() {
  if(!m_initialized) {
    addProgram();
    addVertexShader(":/shaders/frame.vert");
    addFragmentShader(":/shaders/frame.frag");
    link();
    bind();
    checkError("frame: after link frame program");

    m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
    m_vao->create();
    m_vao->bind();

    m_vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    m_vbo->create();
    m_vbo->bind();
    m_vbo->allocate(m_vertices, 12 * sizeof(GLfloat));

    m_ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    m_ibo->create();
    m_ibo->bind();
    m_ibo->allocate(m_indices, 6 * sizeof(GLuint));

    m_position_a = m_program->attributeLocation("position_a");
    m_program->enableAttributeArray(m_position_a);
    m_program->setAttributeBuffer(m_position_a, GL_FLOAT, 0, 3);
    // qDebug() << "m_position_a: " << m_position_a;

    /* set uniform locations */
    m_mvp_u = m_program->uniformLocation("mvp_u");
    m_scale_u = m_program->uniformLocation("scale_u");

    // qDebug() << "m_mvp_u: " << m_mvp_u;
    // qDebug() << "m_scale_u: " << m_scale_u;

    m_vao->release();
    release();
    checkError("frame: after initialize");
    m_initialized = true;
  }
}


/*!
@brief set mvp using only a transform data structure. The model and view matrices are obtained from the current render window.
*/
void FrameShaderProgram::setTransform(s2_displacement_t* transform) {
  QMatrix4x4 model = s2_displacementToQMatrix4x4(transform);
  QMatrix4x4 view = m_renderWindow->getViewMatrix();
  QMatrix4x4 projection = m_renderWindow->getProjectionMatrix();
  QMatrix4x4 mvp = projection * view * model;
  m_program->setUniformValue(m_mvp_u, mvp);
}


/*!
@brief set mvp matrix
*/
void FrameShaderProgram::setMVP(QMatrix4x4& mvp) {
  // m_mvp_u = m_program->uniformLocation("mvp_u");
  // checkError("after set uniform location");
  // qDebug() << "m_mvp_u:" << m_mvp_u;
  m_program->setUniformValue(m_mvp_u, mvp);
}


/*!
@brief set scale factor
*/
void FrameShaderProgram::setScale(GLfloat scale) {
  m_program->setUniformValue(m_scale_u, scale);
}


/*!

*/
void FrameShaderProgram::bindVao() {
  m_program->bind();
  m_vao->bind();
}


/*!
@brief release vao object
*/
void FrameShaderProgram::releaseVao() {
  m_vao->release();
  m_program->release();
}
