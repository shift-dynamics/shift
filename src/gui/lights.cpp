#include "geometry.hpp"
#include "lights.hpp"
#include "renderwindow.hpp"

s2_Lights::s2_Lights(RenderWindow* renderWindow)
  : s2_Geometry(renderWindow)
  , m_bodies(NULL)
  , m_depthBufferArray(NULL) {

  m_lights.count = 0;
  memset(m_lights.position, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ka, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Kd, 0, 30 * sizeof(GLfloat));
  memset(m_lights.Ks, 0, 30 * sizeof(GLfloat));
  memset(m_lights.directional, 0, 10 * sizeof(GLfloat));
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

    /* generate array of depth buffers, one for each light. @TODO add logic to separate out point lights into separate array of cubemap depth buffers later */
    m_depthBufferArray =
      new QOpenGLTexture(QOpenGLTexture::Target2D);
    checkError("terrain: after new depth texture");

    m_depthBufferArray->setFormat(
      QOpenGLTexture::R32F);
    checkError("terrain: after set format R32F");

    m_depthBufferArray->setWrapMode(
      QOpenGLTexture::ClampToEdge);
    checkError("terrain: after set wrap mode");

    m_depthBufferArray->setMinMagFilters(
      QOpenGLTexture::Linear,
      QOpenGLTexture::Linear);
    checkError("terrain: after set min mag filters");

    m_depthBufferArray->setSize(
      512,
      512,
      1);
    checkError("terrain: after set size");
    m_depthBufferArray->allocateStorage();
    checkError("terrain after allocate storage for displacement texture");

  }
}


/*!
@brief draw function
*/
void s2_Lights::draw() {

}
