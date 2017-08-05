#include "grid.hpp"

/*!
@brief initialize grid
*/
void s2_Grid::initialize(
  int gridSize,
  float scaleFactor) {

  /* grid size must be an even number */
  if(gridSize % 2 != 0) {
    gridSize++;
  }
  m_gridSize = gridSize;

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

  m_material->vertices = (GLfloat*)malloc(3 * m_material->vertexCount * sizeof(GLfloat));
  m_material->vertexNormals = (GLfloat*)malloc(3 * m_material->vertexCount * sizeof(GLfloat));
  material->faces =
    new GLuint[2 * m_grid->lineBufferObject->material->faceCount];

  /* set vertices for vertical line segments */
  int gridIndex = 0;
  for(int i = 0; i <= m_grid->gridSize; i++) {
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      (float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;

    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      -(float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;
  }

  // for(int i = 0; i < gridIndex; i++) {
  //   printf("%f, %f, %f\n",
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 0],
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 1],
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 2]);
  // }

  // qDebug() << "gridIndex: " << gridIndex;

  /* set vertices for remaining horizontal line segments */
  for(int i = 1; i < m_grid->gridSize; i++) {
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      -(float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;

    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      (float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;
  }

  /* set top horizontal line segment */
  m_grid->lineBufferObject->material->faces[gridIndex++] = 0;
  m_grid->lineBufferObject->material->faces[gridIndex++] =
    m_grid->gridSize * 2;

  /* set bottom horizontal line segment */
  m_grid->lineBufferObject->material->faces[gridIndex++] = 1;
  m_grid->lineBufferObject->material->faces[gridIndex++] =
    m_grid->gridSize * 2 + 1;

  // qDebug() << "gridIndex final " << gridIndex;
  // qDebug() << "face count" << m_grid->lineBufferObject->material->faceCount;


  m_programLine->bind();
  // checkError("after line program bind");

  /* initialize vao and vbo */
  m_grid->lineBufferObject->vao = new QOpenGLVertexArrayObject(this);
  m_grid->lineBufferObject->vao->create();
  m_grid->lineBufferObject->vao->bind();

  // checkError("after initialize vao");

  // checkError("after set attribute posAttr in line program");

  m_grid->lineBufferObject->vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_grid->lineBufferObject->vertexBuffer->create();

  m_grid->lineBufferObject->vertexBuffer->bind();
  m_grid->lineBufferObject->vertexBuffer->allocate(
    m_grid->lineBufferObject->material->vertices,
    3 * m_grid->lineBufferObject->material->vertexCount * sizeof(GLfloat));
  // checkError("after initialize vbo");
  // m_grid->lineBufferObject->vertexNormalBuffer =
  //   new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  // m_grid->lineBufferObject->vertexNormalBuffer->create();
  //
  // m_grid->lineBufferObject->vertexNormalBuffer->allocate(
  //   m_grid->lineBufferObject->material->vertexNormals,
  //   m_grid->lineBufferObject->material->vertexCount * sizeof(GLfloat));

  m_grid->lineBufferObject->ibo =
    new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  m_grid->lineBufferObject->ibo->create();
  m_grid->lineBufferObject->ibo->bind();

  // checkError("after create ibo in line program");

  // for(int i = 0; i < m_grid->lineBufferObject->material->faceCount * 2; i++) {
  //   printf("[%d]\n", m_grid->lineBufferObject->material->faces[i]);
  // }

  m_grid->lineBufferObject->ibo->allocate(
    m_grid->lineBufferObject->material->faces,
    2 * m_grid->lineBufferObject->material->faceCount * sizeof(GLuint));
  checkError("after set ibo in line program");

  m_uniformsLine.position =
    m_programLine->attributeLocation("posAttr");
  m_programLine->enableAttributeArray(m_uniformsLine.position);
  m_programLine->setAttributeBuffer(
    m_uniformsLine.position,
    GL_FLOAT,
    0,
    3);

  m_programLine->release();

  m_bufferObjects.append(m_grid->lineBufferObject);
}


/*!
@brief draw grid in OpenGL window
*/
void s2_Grid::draw() {


}
