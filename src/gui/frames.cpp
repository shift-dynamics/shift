#include "frames.hpp"
#include "frameshaderprogram.hpp"
#include "renderwindow.hpp"

#include <QHBoxLayout>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector>
#include <QtOpenGL/QGLWidget>

/*!
@brief load context datastructure
*/
void Shift::Frames::loadContext() {
  if(m_renderWindow->getContext()) {
    addFrames(m_renderWindow->getContext()->assembly);
  }
  m_context = m_renderWindow->getContext();
}

/*!
@brief clears pointers to frames, leaving origin at first position
*/
void Shift::Frames::clearContext() {
  m_context = NULL;
  m_frames.erase(m_frames.begin() + 1, m_frames.end());
}

/*!
@brief draw function
*/
void Shift::Frames::draw() {
  if(m_drawEnabled && m_context) {

    checkError("frame: before set frame");

    m_program->bindVao();
    checkError("frame: after bind vao");

    m_renderWindow->m_funcs->glProvokingVertex(0x8E4E);
    checkError("frame: after set provoking index");

    QMatrix4x4 pv =
      m_renderWindow->getProjectionMatrix() * m_renderWindow->getViewMatrix();

    m_program->setScale(m_scaleFactor);
    checkError("frame: after set scale factor");

    glEnable(GL_LINE_SMOOTH);
    checkError("frame: after set gl_line_smooth");

    for(int i = 0; i < m_frames.size(); i++) {
      if(m_frames.at(i)->isVisible) {
        QMatrix4x4 model = displacementToQMatrix4x4(m_frames.at(i)->dAbsolute);
        QMatrix4x4 mvp = pv * model;
        m_program->setMVP(mvp);
        checkError("frame: after set mvp");

        glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
        checkError("frame: after draw elements");
      }
    }

    m_program->releaseVao();
  }
}

/*!
@brief draw frames, m_programFrame and m_uniformsFrame.vao must be bound
prior to calling this recursive function
*/
void Shift::Frames::addFrames(s2_assembly_t* assembly) {
  s2_body_t* bodyHead = assembly->bodyHead;
  while(bodyHead != NULL) {
    s2_frame_t* frameHead = bodyHead->frameHead;
    while(frameHead != NULL) {
      m_frames.append(frameHead);
      frameHead = frameHead->next;
    }
    bodyHead = bodyHead->next;
  }
  s2_assembly_t* assemblyHead = assembly->subAssemblyHead;
  while(assemblyHead != NULL) {
    addFrames(assemblyHead);
    assemblyHead = assemblyHead->next;
  }
}

/*!
@brief draw "W" frames associated with tire contact patches
*/
void Shift::Frames::drawWFrames() {
  if(m_drawEnabled && m_context) {
    m_program->bindVao();

    m_program->setScale(m_scaleFactor);
    checkError("frame: after set scale factor");

    m_renderWindow->m_funcs->glProvokingVertex(0x8E4E);
    checkError("frame: after set provoking index");

    QMatrix4x4 pv =
      m_renderWindow->getProjectionMatrix() * m_renderWindow->getViewMatrix();

    for(int i = 0; i < m_context->forceModuleCount; i++) {
      s2_tire_pac2002_t* tire =
        (s2_tire_pac2002_t*)m_context->forceModulePointers[i]->parameters[0];
      m_program->setTransform(&tire->wFrame);
      m_program->setScale(1.f);
      QMatrix4x4 model = displacementToQMatrix4x4(&tire->wFrame);
      QMatrix4x4 mvp = pv * model;

      /* set model * view * projection matrix */
      m_program->setMVP(mvp);
      checkError("frame: after set mvp wframe");

      // checkError("after set mvp coordinate systems");
      // glEnable(GL_LINE_SMOOTH);
      glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
      checkError("frame: after draw elements wframe");
    }

    m_program->releaseVao();
  }
}
