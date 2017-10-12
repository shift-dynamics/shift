#include "geometry.hpp"
#include "lights.hpp"
#include "renderwindow.hpp"
#include "shadowshaderprogram.hpp"
#include "bodies.hpp"
#include "terrain.hpp"

/*!
@brief constructor
*/
s2_Lights::s2_Lights(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL)
  , m_depthBufferSize(1024)
  , m_maxCount(10) {

  initialize();

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

  initialize();
  m_depthBufferSize = depthBufferSize;
}


/*!
@brief initialize lights
*/
void s2_Lights::initialize() {

  memset(&m_lights, 0, sizeof(m_lights));
  m_drawEnabled = true;
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
      glBindTexture(GL_TEXTURE_2D_ARRAY, m_depthTexture);
      m_renderWindow->m_funcs->glTexStorage3D(
        GL_TEXTURE_2D_ARRAY,
        1,
        GL_DEPTH_COMPONENT32F, // format
        m_depthBufferSize,     // width
        m_depthBufferSize,     // height
        m_maxCount);           // layer count
      checkError("lights: after glTexStorage3D()");
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      checkError("lights: after set glTexParameteri");

      m_renderWindow->m_funcs->glGenFramebuffers(m_maxCount, m_fbo);
      checkError("lights: after glGenFramebuffers");

      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D_ARRAY, m_depthTexture);

      for(int light = 0; light < m_maxCount; light++) {

        m_renderWindow->m_funcs->glBindFramebuffer(
          GL_FRAMEBUFFER,
          m_fbo[light]);
        m_renderWindow->m_funcs->glFramebufferTextureLayer(
          GL_FRAMEBUFFER,
          GL_DEPTH_ATTACHMENT,
          m_depthTexture,
          0, // mipmap level
          light); // layer of texture to attach
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glEnable(GL_DEPTH_TEST);
      }

      m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, 0);
      glEnable(GL_DEPTH_TEST);

    }
  }
}


/*!
@brief draw function
*/
void s2_Lights::draw() {

  if(m_drawEnabled) {

    for(int light = 0; light < m_lights.count; light++) {

      m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo[light]);
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

      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D_ARRAY, m_depthTexture);
      checkError("lights: after glFramebufferTextureLayer");

      glDrawBuffer(GL_NONE);
      checkError("lights: after glDrawBuffer(GL_NONE)");

      glClearDepth(1.f);
      checkError("lights: after glClearDepth(1.f)");

      getFramebufferStatus();

      checkError("lights: after glCheckFramebufferStatus");

      m_renderWindow->getTerrain()->drawShadows();
      m_renderWindow->getBodies()->drawShadows();

    }

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    m_renderWindow->m_funcs->glDisable(GL_POLYGON_OFFSET_FILL);
    m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    m_renderWindow->m_funcs->glDrawBuffer(GL_FRONT_AND_BACK);
  }
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
      /* no error */
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
