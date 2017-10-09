#include "geometry.hpp"
#include "lights.hpp"
#include "renderwindow.hpp"
#include "shadowshaderprogram.hpp"

/*!
@brief constructor
*/
s2_Lights::s2_Lights(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL)
  , m_depthBufferSize(1024)
  , m_maxCount(10) {

  m_lights.count = 0;
  memset(m_lights.position, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ka, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Kd, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ks, 0, 30 * sizeof(GLfloat));
  memset(m_lights.directional, 0, 10 * sizeof(GLfloat));


}


/*!
@brief alternate constructor, allows overriding default depth buffer size
*/
s2_Lights::s2_Lights(
  RenderWindow* renderWindow,
  GLint depthBufferSize)
  : s2_Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL)
  , m_maxCount(10) {

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
      qDebug() << "initializing m_depthBufferArray";

      m_renderWindow->m_funcs->glGenTextures(1, &m_depthTexture);
      glBindTexture(GL_TEXTURE_2D, m_depthTexture);
      m_renderWindow->m_funcs->glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH_COMPONENT32F, m_depthBufferSize, m_depthBufferSize);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glBindTexture(GL_TEXTURE_2D, 0);

      m_renderWindow->m_funcs->glGenFramebuffers(1, &m_fbo);
      m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
      m_renderWindow->m_funcs->glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_depthTexture, 0);
      glDrawBuffer(GL_NONE);
      glReadBuffer(GL_NONE);
      glEnable(GL_DEPTH_TEST);

      // glGenTextures(1, &depth_debug_tex);
      // glBindTexture(GL_TEXTURE_2D, depth_debug_tex);
      // glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, DEPTH_TEXTURE_SIZE, DEPTH_TEXTURE_SIZE);
      // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      //
      // glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, depth_debug_tex, 0);
      //
      // glBindTexture(GL_TEXTURE_2D, 0);
      m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, 0);

      glEnable(GL_DEPTH_TEST);


      // /* generate array of depth buffers, one for each light. @TODO add logic to separate out point lights into separate array of cubemap depth buffers later */
      // m_depthBufferArray = new QOpenGLTexture(QOpenGLTexture::Target2DArray);
      // checkError("lights: after new depth texture array");
      //
      // m_depthBufferArray->setFormat(QOpenGLTexture::D32);
      // checkError("lights: after set format D32");
      //
      // m_depthBufferArray->setWrapMode(QOpenGLTexture::ClampToEdge);
      // checkError("lights: after set wrap mode");
      //
      // m_depthBufferArray->setMinMagFilters(
      //   QOpenGLTexture::Linear,
      //   QOpenGLTexture::Linear);
      // checkError("lights: after set min mag filters");
      //
      // m_depthBufferArray->setMipLevels(0);
      // checkError("lights: after set mip levels");
      //
      // m_depthBufferArray->setComparisonFunction(
      //   QOpenGLTexture::CompareLessEqual);
      // checkError("lights: after set comparison function");
      //
      // m_depthBufferArray->setComparisonMode(
      //   QOpenGLTexture::CompareRefToTexture);
      // checkError("lights: after set comparison mode");
      //
      // m_depthBufferArray->setLayers(m_maxCount);
      // checkError("lights: after set layers");
      //
      // m_depthBufferArray->setSize(
      //   m_depthBufferSize,
      //   m_depthBufferSize,
      //   1);
      // checkError("lights: after set size");
      //
      // m_depthBufferArray->setDepthStencilMode(QOpenGLTexture::DepthMode);
      // checkError("lights: after set depth stencil mode");
      //
      // m_depthBufferArray->allocateStorage();
      // checkError("lights: after texture 2d array allocate storage");
      //
      // m_depthBufferArray->bind(0);
      //
      // m_renderWindow->m_funcs->glFramebufferTextureLayer(
      //   GL_FRAMEBUFFER,
      //   GL_DEPTH_ATTACHMENT,
      //   m_depthBufferArray->textureId(),
      //   0,
      //   0);
      //
      // m_depthBufferArray->release();
      m_renderWindow->m_funcs->glBindFramebuffer(
        GL_FRAMEBUFFER, 0);
    }
  }
}


/*!
@brief draw function
*/
void s2_Lights::draw() {

  // m_program->bind();
  // checkError("lights: after bind shadow shader program");

  // m_depthBufferArray->bind(0);
  // checkError("lights: after bind 2d texture array");

  m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
  checkError("lights: after glBindFramebuffer(GL_FRAMEBUFFER, m_fbo)");

  glClear(GL_DEPTH_BUFFER_BIT);
  checkError("lights: after glClear()");

  glViewport(0, 0, m_depthBufferSize, m_depthBufferSize);
  checkError("lights: after glViewport()");

  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  checkError("lights: after glColorMask()");

  m_renderWindow->m_funcs->glEnable(GL_POLYGON_OFFSET_FILL);
  checkError("lights: after glEnable(GL_POLYGON_OFFSET_FILL)");

  glPolygonOffset(2.f, 4.f);
  checkError("lights: after glPolygonOffset(2.f, 4.f)");

  glEnable(GL_DEPTH_TEST);
  checkError("lights: after glEnable(GL_DEPTH_TEST)");

  for(int light = 0; light < m_lights.count; light++) {

    qDebug() << "[" << light << "]";

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_depthTexture);
    checkError("lights: after glFramebufferTextureLayer");

    glDrawBuffer(GL_NONE);
    checkError("lights: after glDrawBuffer(GL_NONE)");

    glClearDepth(1.f);
    checkError("lights: after glClearDepth(1.f)");

    getFramebufferStatus();

    checkError("lights: after glCheckFramebufferStatus");
    m_renderWindow->drawShadows();

  }

  // m_depthBufferArray->release();
  m_renderWindow->m_funcs->glDisable(GL_POLYGON_OFFSET_FILL);
  m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, 0);
  m_renderWindow->m_funcs->glDrawBuffer(GL_FRONT_AND_BACK);

}


/*!
@brief utility function to get frame buffer status, prints errors to qDebug()
*/
void s2_Lights::getFramebufferStatus() {
  GLenum status = m_renderWindow->m_funcs->glCheckFramebufferStatus(GL_FRAMEBUFFER);
  switch(status) {
    case GL_INVALID_ENUM: {
      qDebug() << "framebuffer status: GL_INVALID_ENUM";
    } break;
    case GL_FRAMEBUFFER_UNDEFINED: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_UNDEFINED";
    } break;
    case GL_FRAMEBUFFER_COMPLETE: {
    } break;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
    } break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
    } break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
    } break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
    } break;
    case GL_FRAMEBUFFER_UNSUPPORTED: {
     qDebug() << "framebuffer status: GL_FRAMEBUFFER_UNSUPPORTED";
    } break;
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
    } break;
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
    } break;
    default: {
      qDebug() << status;
    } break;
  }
}
