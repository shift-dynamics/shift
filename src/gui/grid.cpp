#include "grid.hpp"
#include "renderwindow.hpp"

/*!
@brief initialize grid
*/
void s2_Grid::initialize() {

  /* grid size must be an even number */
  if(m_gridSize % 2 != 0) {
    m_gridSize++;
  }

  /*
    set up line segments
  */
  m_material = s2_materials(1);
  m_material->shadingType = LINE;
  m_material->Ns = 10;

  m_transform = s2_displacement();


  /* set up vertices and line segments for grid */
  m_material->vertexCount = 4 * m_gridSize;
  m_material->faceCount = 2 * (m_gridSize + 1);

  m_material->vertices = (GLfloat*) malloc(3 * m_material->vertexCount * sizeof(GLfloat));
  m_material->vertexNormals = (GLfloat*) malloc(3 * m_material->vertexCount * sizeof(GLfloat));
  m_material->faces = (GLuint*) malloc(2 * m_material->faceCount);

  /* set vertices for vertical line segments */
  int gridIndex = 0;
  for(int i = 0; i <= m_gridSize; i++) {
    m_material->vertices[gridIndex * 3 + 0] = (float)(i - m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 1] = (float)(m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_material->faces[gridIndex] = gridIndex;
    gridIndex++;

    m_material->vertices[gridIndex * 3 + 0] = (float)(i - m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 1] = -(float)(m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_material->faces[gridIndex] = gridIndex;
    gridIndex++;
  }

  /* set vertices for remaining horizontal line segments */
  for(int i = 1; i < m_gridSize; i++) {
    m_material->vertices[gridIndex * 3 + 0] = -(float)(m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 1] = (float)(i - m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_material->faces[gridIndex] = gridIndex;
    gridIndex++;

    m_material->vertices[gridIndex * 3 + 0] = (float)(m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 1] = (float)(i - m_gridSize / 2);
    m_material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_material->faces[gridIndex] = gridIndex;
    gridIndex++;
  }

  /* set top horizontal line segment */
  m_material->faces[gridIndex++] = 0;
  m_material->faces[gridIndex++] = m_gridSize * 2;

  /* set bottom horizontal line segment */
  m_material->faces[gridIndex++] = 1;
  m_material->faces[gridIndex++] = m_gridSize * 2 + 1;

  // qDebug() << "gridIndex final " << gridIndex;
  // qDebug() << "face count" << m_grid->lineBufferObject->material->faceCount;


  m_program->bind();
  // checkError("after line program bind");

  /* initialize vao and vbo */
  m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
  m_vao->create();
  m_vao->bind();
  m_renderWindow->checkError("after initialize vao");

  m_vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_vbo->create();
  m_vbo->bind();
  m_vbo->allocate(
    m_material->vertices,
    3 * m_material->vertexCount * sizeof(GLfloat));
  checkError("after initialize grid vbo");

  m_ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  m_ibo->create();
  m_ibo->bind();
  m_renderWindow->checkError("after create grid ibo");

  m_ibo->allocate(
    m_material->faces,
    2 * m_material->faceCount * sizeof(GLuint));
  m_renderWindow->checkError("after set ibo in line program");

  m_position_a = m_program->attributeLocation("posAttr");
  m_program->enableAttributeArray(m_position_a);
  m_program->setAttributeBuffer(
    m_position_a,
    GL_FLOAT,
    0,
    3);

  m_program->release();
}


/*!
@brief draw grid in OpenGL window
*/
void s2_Grid::draw() {
  m_program->bind();
  m_vao->bind();

  QMatrix4x4 mv = m_renderWindow->getViewMatrix() * m_model;

  /* set model * view * projection matrix */
  m_program->setUniformValue(
    m_mvp_u,
    m_renderWindow->getProjectionMatrix() * mv);
  checkError("line: after set mvp");

  m_program->setUniformValue(
    m_color_u,
    m_color[0],
    m_color[1],
    m_color[2],
    m_color[3]);
  checkError("after set uniform color");

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  checkError("after set polygon mode");

  glDrawElements(
    GL_LINES,
    2 * m_material->faceCount,
    GL_UNSIGNED_INT,
    0);
  checkError("after draw elements");

  m_vao->release();
  checkError("after vao release");

  m_program->release();
  checkError("after flat release");

}
