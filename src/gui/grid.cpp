#include "grid.hpp"
#include "renderwindow.hpp"
#include "simpleshaderprogram.hpp"

/*!
@brief initialize grid
*/
void s2_Grid::initialize() {

  if(!m_initialized) {
    /* grid size must be an even number */
    if(m_gridSize % 2 != 0) {
      m_gridSize++;
    }

    /*
      set up line segments
    */
    m_material = s2_material_alloc(1);
    m_material->shadingType = S2_SHADING_LINE;
    m_material->GLuniforms.Ns = 10;

    m_transform = s2_displacement();

    /* set up vertices and line segments for grid */
    m_material->vertexCount = 4 * m_gridSize;
    m_material->faceCount = 2 * (m_gridSize + 1);

    m_material->vertices = (GLfloat*) malloc(3 * m_material->vertexCount * sizeof(GLfloat));
    m_material->vertexNormals = (GLfloat*) malloc(3 * m_material->vertexCount * sizeof(GLfloat));
    m_material->faces = (GLuint*) malloc(2 * m_material->faceCount * sizeof(GLuint));

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
    checkError("grid: after bind simple shader program in grid initialization");

    /* initialize vao and vbo */
    m_vao = new QOpenGLVertexArrayObject(m_renderWindow);
    m_vao->create();
    m_vao->bind();
    m_renderWindow->checkError("grid: after initialize vao");

    m_vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    m_vbo->create();
    m_vbo->bind();
    m_vbo->allocate(
      m_material->vertices,
      3 * m_material->vertexCount * sizeof(GLfloat));
    checkError("grid: after initialize vbo");

    m_ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    m_ibo->create();
    m_ibo->bind();
    m_renderWindow->checkError("grid: after create ibo");

    m_ibo->allocate(
      m_material->faces,
      2 * m_material->faceCount * sizeof(GLuint));
    m_renderWindow->checkError("grid: after set ibo");

    QOpenGLShaderProgram* program = m_program->getProgram();
    GLuint position_a = m_program->getPositionAttribute();
    program->enableAttributeArray(position_a);
    program->setAttributeBuffer(
      position_a,
      GL_FLOAT,
      0,
      3);

    m_vao->release();
    m_program->release();
    m_initialized = true;
  }
}


/*!
@brief draw grid in OpenGL window
*/
void s2_Grid::draw() {
  if(m_drawEnabled && m_renderWindow->isInitialized() && m_initialized) {

    m_program->bind();
    checkError("grid: after bind line program");

    m_vao->bind();
    checkError("grid: after bind vao line program");

    m_program->setTransform(m_transform);
    checkError("grid: after set transform in line program");

    m_program->setScale(1.f);
    checkError("grid: after set scale");

    m_program->setColor(
      m_color[0],
      m_color[1],
      m_color[2],
      m_color[3]);
    checkError("grid: after set color in line program");

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    checkError("grid: after set polygon mode");

    glDrawElements(
      GL_LINES,
      2 * m_material->faceCount,
      GL_UNSIGNED_INT,
      0);
    checkError("grid: after draw elements");

    m_vao->release();
    checkError("grid: after vao release");

    m_program->release();
    checkError("grid: after release");
  }
}
