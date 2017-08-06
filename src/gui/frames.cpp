#include "frames.hpp"

/*!
@brief initialize frames
*/
void s2_Frames::initialize() {

  m_frames.append(s2_displacement());

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

  m_vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_vbo->create();

  m_vbo->bind();
  m_vbo->allocate(2 * 18 * sizeof(GLfloat));
  m_vbo->write(0, m_vertices, 18 * sizeof(GLfloat));
  m_vbo->write(18 * sizeof(GLfloat), m_colors, 18 * sizeof(GLfloat));

  m_ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  m_ibo->create();
  m_ibo->bind();
  m_ibo->allocate(m_indices, 6 * sizeof(GLuint));

  m_posAttr = m_program->attributeLocation("posAttr");
  m_program->enableAttributeArray(m_posAttr);
  m_program->setAttributeBuffer(m_posAttr, GL_FLOAT, 0, 3);

  m_colorAttr = m_program->attributeLocation("colorAttr");
  m_program->enableAttributeArray(m_colorAttr);
  m_program->setAttributeBuffer(m_colorAttr, GL_FLOAT, 18 * sizeof(GLfloat), 3);

  m_program->release();

  checkError("after set up frame shader");
}

/*!
@brief load context datastructure
*/
s2_Frames::loadContext() {
  m_context = renderWindow->getContext();
  if(m_context) {

  }

}


/*!
@brief draw function
*/
s2_Frames::draw() {
  if(drawEnabled) {
    m_program->bind();
    m_vao->bind();
    checkError("after bind frames vao");
    QMatrix4x4 pv =
      m_renderWindow->getProjectionMatrix() *
      m_renderWindow->getViewMatrix();
    for(int i = 0; i < m_frames.size(); i++) {
      double* d = (double*)(m_frames.at(i));
      QMatrix4x4 model =
        QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                   (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                   (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                   0.f,  0.f,  0.f,  1.f);
      QMatrix4x4 mvp = pv * model;

      /* set model * view * projection matrix */
      m_program->setUniformValue(m_mvp, mvp);
      // checkError("after set mvp coordinate systems");

      // glEnable(GL_LINE_SMOOTH);
      // checkError("after gl gl_line_smooth");

      glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
      checkError("after draw elements coordinate systems");
    }
  }
}


/*!
@brief draw frames, m_programFrame and m_uniformsFrame.vao must be bound
prior to calling this recursive function
*/
void s2_Frames::drawFrames(s2_assembly_t* assembly) {
  s2_body_t* bodyHead = assembly->bodyHead;
  while(bodyHead != NULL) {
    s2_frame_t* frameHead = bodyHead->frameHead;
    while(frameHead != NULL) {

      double* d = (double*)frameHead->dAbsolute;
      QMatrix4x4 model = s2_displacementToQMatrix4x4(frameHead->dAbsolute);
      QMatrix4x4 scale =
        QMatrix4x4(m_frameScaleFactor, 0.0f, 0.0f, 0.0f,
                   0.0f, m_frameScaleFactor, 0.0f, 0.0f,
                   0.0f, 0.0f, m_frameScaleFactor, 0.0f,
                   0.0f, 0.0f, 0.0f, 1.0f);

      QMatrix4x4 mvp = m_projection * m_view * model * scale;

      /* set model * view * projection matrix */
      m_programFrame->setUniformValue(
        m_uniformsFrame.mvp,
        mvp);
      // checkError("after set mvp coordinate systems");
      // glEnable(GL_LINE_SMOOTH);
      glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
      // checkError("after draw elements coordinate systems");

      frameHead = frameHead->next;
    }

    bodyHead = bodyHead->next;
  }
  s2_assembly_t* assemblyHead = assembly->subAssemblyHead;
  while(assemblyHead != NULL) {
    drawFrames(assemblyHead);
    assemblyHead = assemblyHead->next;
  }
}


void RenderWindow::drawWFrames() {
  for(int i = 0; i < m_context->forceModuleCount; i++) {
    s2_tire_pac2002_t* tire =
      (s2_tire_pac2002_t*)m_context->forceModulePointers[i]->parameters[0];

    QMatrix4x4 model = s2_displacementToQMatrix4x4(&tire->wFrame);
    QMatrix4x4 scale =
      QMatrix4x4(m_frameScaleFactor, 0.0f, 0.0f, 0.0f,
                 0.0f, m_frameScaleFactor, 0.0f, 0.0f,
                 0.0f, 0.0f, m_frameScaleFactor, 0.0f,
                 0.0f, 0.0f, 0.0f, 1.0f);

    QMatrix4x4 mvp = m_projection * m_view * model * scale;

    /* set model * view * projection matrix */
    m_programFrame->setUniformValue(
      m_uniformsFrame.mvp,
      mvp);
    // checkError("after set mvp coordinate systems");
    // glEnable(GL_LINE_SMOOTH);
    glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
  }
}
