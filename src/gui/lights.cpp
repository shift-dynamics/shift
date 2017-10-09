#include "geometry.hpp"
#include "lights.hpp"
#include "renderwindow.hpp"

s2_Lights::s2_Lights(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL)
  , m_depthBufferSize(1024) {

  m_lights.count = 0;
  memset(m_lights.position, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ka, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Kd, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ks, 0, 30 * sizeof(GLfloat));
  memset(m_lights.directional, 0, 10 * sizeof(GLfloat));


}


s2_Lights::s2_Lights(RenderWindow* renderWindow,
GLint depthBufferSize)
  : s2_Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL)
  , m_depthBufferSize(1024) {

  m_lights.count = 0;
  memset(m_lights.position, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ka, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Kd, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ks, 0, 30 * sizeof(GLfloat));
  memset(m_lights.directional, 0, 10 * sizeof(GLfloat));

  m_depthBufferSize = depthBufferSize;
}


/*!
@brief initialize lights
*/
void s2_Lights::initialize() {

  // /* @TODO move lights to uniform buffer blocks
  //   m_funcs->glGetActiveUniformBlockName();
  // */
  // // qDebug() << "initializing lights";
  // // qDebug() << "lightCount: " << m_context->activeLightCount;
  // if(m_context->activeLightCount > 10) {
  //   m_lightCount = 10;
  // } else {
  //   m_lightCount = m_context->activeLightCount;
  // }
  //
  // for(unsigned int i = 0; i < m_lightCount; i++) {
  //   QVector4D position;
  //   s2_light_t* light = m_context->lightPointers[i];
  //   s2_displacement_t* d = light->parentFramePointer->dAbsolute;
  //   switch(light->lightType) {
  //     case POINT: {
  //       m_directional[i] = 0;
  //       m_lightPosition[i * 3 + 0] = d->T.x;
  //       m_lightPosition[i * 3 + 1] = d->T.y;
  //       m_lightPosition[i * 3 + 2] = d->T.z;
  //
  //     } break;
  //     case DIRECTIONAL: {
  //       m_directional[i] = 1;
  //       m_lightPosition[i * 3 + 0] = d->T.x;
  //       m_lightPosition[i * 3 + 1] = d->T.x;
  //       m_lightPosition[i * 3 + 2] = d->T.x;
  //     }
  //     break;
  //   }
  //
  //   for(int j = 0; j < 3; j++) {
  //     m_KaLight[3 * i + j] = m_context->lightPointers[i]->glLight.Ka[j];
  //     m_KdLight[3 * i + j] = m_context->lightPointers[i]->glLight.Kd[j];
  //     m_KsLight[3 * i + j] = m_context->lightPointers[i]->glLight.Ks[j];
  //   }
  // }
}

/*!
@brief load context data structure
*/
void s2_Lights::loadContext() {

  m_context = m_renderWindow->getContext();
  if(m_context) {
    if(m_context->activeLightCount > 10) {
      m_lights.count = 10;
    } else {
      m_lights.count = m_context->activeLightCount;
    }
    for(int i = 0; i < m_lights.count; i++) {
      QVector4D position;
      s2_light_t* light = m_context->lightPointers[i];
      s2_displacement_t* d = light->parentFramePointer->dAbsolute;
      switch(light->lightType) {
        case POINT: {
          m_lights.directional[i] = 0;
          m_lights.position[i * 3 + 0] = d->T.x;
          m_lights.position[i * 3 + 1] = d->T.y;
          m_lights.position[i * 3 + 2] = d->T.z;

        } break;
        case DIRECTIONAL: {
          m_lights.directional[i] = 1;
          m_lights.position[i * 3 + 0] = d->T.x;
          m_lights.position[i * 3 + 1] = d->T.x;
          m_lights.position[i * 3 + 2] = d->T.x;
        }
        break;
      }

      for(int j = 0; j < 3; j++) {
        m_lights.Ka[3 * i + j] =
          m_context->lightPointers[i]->glLight.Ka[j];
        m_lights.Kd[3 * i + j] =
          m_context->lightPointers[i]->glLight.Kd[j];
        m_lights.Ks[3 * i + j] =
          m_context->lightPointers[i]->glLight.Ks[j];
      }
    }

    if(m_depthBufferArray == NULL) {
      /* generate array of depth buffers, one for each light. @TODO add logic to separate out point lights into separate array of cubemap depth buffers later */
      m_depthBufferArray =
        new QOpenGLTexture(QOpenGLTexture::Target2DArray);
      checkError("lights: after new depth texture array");

      m_depthBufferArray->setFormat(QOpenGLTexture::D32);
      checkError("lights: after set format D32");

      m_depthBufferArray->setWrapMode(
        QOpenGLTexture::ClampToEdge);
      checkError("lights: after set wrap mode");

      m_depthBufferArray->setMinMagFilters(
        QOpenGLTexture::Linear,
        QOpenGLTexture::Linear);
      checkError("lights: after set min mag filters");

      m_depthBufferArray->setComparisonFunction(
        QOpenGLTexture::CompareLessEqual);
      checkError("lights: after set comparison function");

      m_depthBufferArray->setSize(
        m_depthBufferSize,
        m_depthBufferSize,
        10);
      checkError("lights: after set size");

      m_depthBufferArray->allocateStorage(
        QOpenGLTexture::Depth,
        QOpenGLTexture::Float32);
      checkError("lights: after allocate storage");

      m_depthBufferArray->bind();

      m_renderWindow->m_funcs->glGenFramebuffers(1, &m_fbo);
      m_renderWindow->m_funcs->glBindFramebuffer(
        GL_FRAMEBUFFER,
        m_fbo);

      m_renderWindow->m_funcs->glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depthBufferArray->textureId(), 0);
      glDrawBuffer(GL_NONE);
    }
  }
}


/*!
@brief draw function
*/
void s2_Lights::draw() {

  m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
  glViewport(0, 0, m_depthBufferSize, m_depthBufferSize);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(2.f, 4.f);
  glEnable(GL_DEPTH_TEST);

  m_depthBufferArray->bind();
  for(int light = 0; light < m_lights.count; light++) {

    m_renderWindow->m_funcs->glFramebufferTexture(
      GL_FRAMEBUFFER,
      GL_DEPTH_STENCIL_ATTACHMENT,
      m_depthBufferArray->textureId(),
      light);

    m_renderWindow->m_funcs->glDrawBuffer(GL_NONE);

    glClearDepth(1.f);
    glClear(GL_DEPTH_BUFFER_BIT);

    m_renderWindow->drawShadows();

  }

  glDisable(GL_POLYGON_OFFSET_FILL);
  m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, 0);
  m_renderWindow->m_funcs->glDrawBuffer(GL_FRONT_AND_BACK);

}
