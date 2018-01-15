#include "lights.hpp"
#include "bodies.hpp"
#include "geometry.hpp"
#include "renderwindow.hpp"
#include "shadowshaderprogram.hpp"
#include "terrain.hpp"

/*!
@brief constructor
*/
Shift::Lights::Lights(RenderWindow* renderWindow)
  : Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL)
  , m_depthBufferSize(4096)
  , m_maxCount(10)
  , m_activeLightIndex(0) {

  initialize();
}

/*!
@brief alternate constructor, allows overriding default depth buffer size
*/
Shift::Lights::Lights(RenderWindow* renderWindow, GLint depthBufferSize)
  : Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL)
  , m_maxCount(10)
  , m_activeLightIndex(0) {

  initialize();
  m_depthBufferSize = depthBufferSize;
}

/*!
@brief initialize lights
*/
void Shift::Lights::initialize() {

  memset(&m_lights.position, 0, 30 * sizeof(GLfloat));
  memset(&m_lights.Ka, 0, 30 * sizeof(GLfloat));
  memset(&m_lights.Kd, 0, 30 * sizeof(GLfloat));
  memset(&m_lights.Ks, 0, 30 * sizeof(GLfloat));
  memset(&m_lights.type, 0,
         10 * sizeof(GLfloat)); /*!< 0 = point, 1 = directional, 2 = spot */
  m_lights.count = 0;
  m_lights.drawShadows = 1; /*!< 0 = false, 1 = true */
  for(int i = 0; i < m_maxCount; i++) {
    m_lights.projectionMatrix[i].perspective(90.f, 1.f, 0.001f, 1000.f);
    m_lights.viewMatrix[i].setToIdentity();
  }
  m_drawEnabled = true;
}

/*!
@brief load context data structure
*/
void Shift::Lights::loadContext() {

  m_context = m_renderWindow->getContext();
  if(m_context) {
    if(m_context->activeLightCount > 10) {
      m_lights.count = 10;
    } else {
      m_lights.count = m_context->activeLightCount;
    }
    for(int i = 0; i < m_lights.count; i++) {
      s2_light_t* light = m_context->lightPointers[i];
      s2_displacement_t* d = light->parentFramePointer->dAbsolute;
      switch(light->lightType) {
        case S2_LIGHT_POINT: {
          m_lights.type[i] = 0;
          m_lights.position[i * 3 + 0] = d->T.x;
          m_lights.position[i * 3 + 1] = d->T.y;
          m_lights.position[i * 3 + 2] = d->T.z;
          break;
        }
        case S2_LIGHT_SPOT: {
          m_lights.type[i] = 1;
          m_lights.position[i * 3 + 0] = d->T.x;
          m_lights.position[i * 3 + 1] = d->T.y;
          m_lights.position[i * 3 + 2] = d->T.z;
          break;
        }
        case S2_LIGHT_DIRECTIONAL: {
          m_lights.type[i] = 2;
          m_lights.position[i * 3 + 0] = d->R.m20;
          m_lights.position[i * 3 + 1] = d->R.m21;
          m_lights.position[i * 3 + 2] = d->R.m22;
          break;
        }
      }

      for(int j = 0; j < 3; j++) {
        m_lights.Ka[3 * i + j] = m_context->lightPointers[i]->glLight.Ka[j];
        m_lights.Kd[3 * i + j] = m_context->lightPointers[i]->glLight.Kd[j];
        m_lights.Ks[3 * i + j] = m_context->lightPointers[i]->glLight.Ks[j];
      }

      /* initialize view matrix */
      m_lights.viewMatrix[i] = displacementToQMatrix4x4(d).inverted();
      qDebug() << "initialized view matrix to:" << m_lights.viewMatrix[i];

      /* initialize projection matrix */

      QMatrix4x4 projection;
      switch(light->lightType) {
        case S2_LIGHT_POINT: {
          projection.perspective(90.f, 1.f, 0.01f, 100.f);
          qDebug() << "initialized point (perspective) projection matrix";
        } break;
        case S2_LIGHT_SPOT: {
          projection.perspective(light->glLight.fov, 1.0, 0.01f, 100.f);
          qDebug() << "initialized spot (perspective) projection matrix";
        } break;
        case S2_LIGHT_DIRECTIONAL: {
          projection.ortho(-1.f, 1.f, -1.f, 1.f, 0.01f, 100.f);
          qDebug() << "initialized directional (ortho) projection matrix";
        } break;
      }
      m_lights.projectionMatrix[i] = projection;
      qDebug() << m_lights.projectionMatrix[i];
    }

    if(m_lights.count == 0) {
      m_lights.count = 1;
      m_lights.type[0] = 2;
      s2_displacement_t d;
      s2_displacement_setIdentity(&d);
      d.T = *m_context->modelParameters.gravity;
      s2_vector_normalize(&d.T);
      s2_vector_invertInPlace(&d.T);
      s2_vector_scale(50, &d.T);
      m_lights.position[0 * 3 + 0] = d.T.x;
      m_lights.position[0 * 3 + 1] = d.T.y;
      m_lights.position[0 * 3 + 2] = d.T.z;
      m_lights.viewMatrix[0] = displacementToQMatrix4x4(&d).inverted();
      QMatrix4x4 projection;
      projection.ortho(-1.f, 1.f, -1.f, 1.f, 0.01f, 100.f);
      m_lights.projectionMatrix[0] = projection;
      for(int i = 0; i < 3; i++) {
        m_lights.Ka[i] = 1.0;
        m_lights.Kd[i] = 1.0;
        m_lights.Ks[i] = 1.0;
      }
    }

    if(m_depthBufferArray == NULL) {
      // qDebug() << "initializing m_depthBufferArray";

      glActiveTexture(GL_TEXTURE0);
      m_renderWindow->m_funcs->glGenTextures(1, &m_depthTexture);
      glBindTexture(GL_TEXTURE_2D_ARRAY, m_depthTexture);
      checkError("lights: after glBindTexture");
      m_renderWindow->m_funcs->glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1,
                                              GL_DEPTH_COMPONENT32F, // format
                                              m_depthBufferSize,     // width
                                              m_depthBufferSize,     // height
                                              m_maxCount); // layer count
      checkError("lights: after glTexStorage3D()");
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_MODE,
                      GL_COMPARE_REF_TO_TEXTURE);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      checkError("lights: after set glTexParameteri");

      m_renderWindow->m_funcs->glGenFramebuffers(m_maxCount, m_fbo);
      checkError("lights: after glGenFramebuffers");

      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D_ARRAY, m_depthTexture);

      for(int light = 0; light < m_maxCount; light++) {

        m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER,
                                                   m_fbo[light]);
        m_renderWindow->m_funcs->glFramebufferTextureLayer(
          GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_depthTexture,
          0,      // mipmap level
          light); // layer of texture to attach
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glEnable(GL_DEPTH_TEST);
      }

      /* restore default frame buffer */
      m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, 0);
      glEnable(GL_DEPTH_TEST);
    }
  }
}

/*!
@brief draw function
*/
void Shift::Lights::draw() {

  if(m_drawEnabled) {

    glActiveTexture(GL_TEXTURE0);
    checkError("lights: after glActiveTexture");
    glBindTexture(GL_TEXTURE_2D_ARRAY, m_depthTexture);
    checkError("lights: after glBindTexture");

    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    for(int i = 0; i < m_lights.count; i++) {
      if(m_context->lightPointers) {
        s2_light_t* light = m_context->lightPointers[i];

        /* update view matrix only if not attached to fixed body */
        if(light->parentBodyPointer != m_context->assembly->bodyHead) {
          s2_displacement_t* d = light->parentFramePointer->dAbsolute;
          m_lights.viewMatrix[i] = displacementToQMatrix4x4(d).inverted();

          switch(light->lightType) {
            case S2_LIGHT_POINT: {
              m_lights.type[i] = 0;
              m_lights.position[i * 3 + 0] = d->T.x;
              m_lights.position[i * 3 + 1] = d->T.y;
              m_lights.position[i * 3 + 2] = d->T.z;
              break;
            }
            case S2_LIGHT_SPOT: {
              m_lights.type[i] = 1;
              m_lights.position[i * 3 + 0] = d->T.x;
              m_lights.position[i * 3 + 1] = d->T.y;
              m_lights.position[i * 3 + 2] = d->T.z;
              break;
            }
            case S2_LIGHT_DIRECTIONAL: {
              m_lights.type[i] = 2;
              m_lights.position[i * 3 + 0] = d->R.m20;
              m_lights.position[i * 3 + 1] = d->R.m21;
              m_lights.position[i * 3 + 2] = d->R.m22;
              break;
            }
          }
        }
      }

      /* set current active view and projection matrix */
      m_activeLightIndex = i;

      m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo[i]);
      checkError("lights: after glBindFramebuffer(GL_FRAMEBUFFER, m_fbo)");

      m_renderWindow->m_funcs->glEnable(GL_POLYGON_OFFSET_FILL);
      checkError("lights: after glEnable(GL_POLYGON_OFFSET_FILL)");

      glPolygonOffset(2.f, 4.f);
      checkError("lights: after glPolygonOffset(2.f, 4.f)");

      glClear(GL_DEPTH_BUFFER_BIT);
      checkError("lights: after glClear()");

      glViewport(0, 0, m_depthBufferSize, m_depthBufferSize);
      checkError("lights: after glViewport()");

      // glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
      // checkError("lights: after glColorMask()");

      glEnable(GL_DEPTH_TEST);
      checkError("lights: after glEnable(GL_DEPTH_TEST)");

      glDrawBuffer(GL_NONE);
      checkError("lights: after glDrawBuffer(GL_NONE)");

      glClearDepth(1.f);
      checkError("lights: after glClearDepth(1.f)");

      getFramebufferStatus();
      checkError("lights: after glCheckFramebufferStatus");

      m_renderWindow->getBodies()->drawShadows();
      m_renderWindow->getTerrain()->drawShadows();
    }

    glCullFace(GL_BACK);

    m_renderWindow->m_funcs->glDisable(GL_POLYGON_OFFSET_FILL);

    /* restore default frame buffer */
    m_renderWindow->m_funcs->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    m_renderWindow->m_funcs->glDrawBuffer(GL_FRONT_AND_BACK);
  }
}

/*!
@brief set array of shadow matrices using current transform
@return pointer to beginning of shadowMatrix array
*/
QMatrix4x4* Shift::Lights::setShadowMatrix(s2_displacement_t* transform) {
  QMatrix4x4 model = displacementToQMatrix4x4(transform);
  QMatrix4x4 bias = QMatrix4x4(0.5f, 0.f, 0.f, 0.5f, 0.f, 0.5f, 0.f, 0.5f, 0.f,
                               0.f, 0.5f, 0.5f, 0.f, 0.f, 0.f, 1.f);
  for(int i = 0; i < m_lights.count; i++) {
    m_lights.shadowMatrix[i] =
      bias * getProjectionMatrix(i) * getViewMatrix(i) * model;
  }
  return m_lights.shadowMatrix;
}

/*!
@brief set array of shadow matrices using current transform
@return pointer to beginning of shadowMatrix array
*/
QMatrix4x4* Shift::Lights::setShadowMatrix(QMatrix4x4& model) {
  QMatrix4x4 bias = QMatrix4x4(0.5f, 0.f, 0.f, 0.5f, 0.f, 0.5f, 0.f, 0.5f, 0.f,
                               0.f, 0.5f, 0.5f, 0.f, 0.f, 0.f, 1.f);
  for(int i = 0; i < m_lights.count; i++) {
    m_lights.shadowMatrix[i] =
      bias * getProjectionMatrix(i) * getViewMatrix(i) * model;
  }
  return m_lights.shadowMatrix;
}

/*!
@brief utility function to get frame buffer status, prints errors to qDebug()
*/
void Shift::Lights::getFramebufferStatus() {
  GLenum status =
    m_renderWindow->m_funcs->glCheckFramebufferStatus(GL_FRAMEBUFFER);
  switch(status) {
    case GL_INVALID_ENUM: {
      qDebug() << "framebuffer status: GL_INVALID_ENUM";
      break;
    }
    case GL_FRAMEBUFFER_UNDEFINED: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_UNDEFINED";
      break;
    }
    case GL_FRAMEBUFFER_COMPLETE: {
      /* no error */
      break;
    }
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
      break;
    }
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: {
      qDebug()
        << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
      break;
    }
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
      break;
    }
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
      break;
    }
    case GL_FRAMEBUFFER_UNSUPPORTED: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_UNSUPPORTED";
      break;
    }
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
      break;
    }
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS: {
      qDebug() << "framebuffer status: GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
      break;
    }
    default: {
      qDebug() << status;
      break;
    };
  }
}
