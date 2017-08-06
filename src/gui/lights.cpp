#include "lights.hpp"

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


void s2_Lights::loadContext() {

  m_context = m_renderWindow->getContext();
  if(m_context) {
    if(m_context->activeLightCount > 10) {
      m_count = 10;
    } else {
      m_count = m_context->activeLightCount;
    }
    for(unsigned int i = 0; i < m_count; i++) {
      QVector4D position;
      s2_light_t* light = m_context->lightPointers[i];
      s2_displacement_t* d = light->parentFramePointer->dAbsolute;
      switch(light->lightType) {
        case POINT: {
          m_directional[i] = 0;
          m_position[i * 3 + 0] = d->T.x;
          m_position[i * 3 + 1] = d->T.y;
          m_position[i * 3 + 2] = d->T.z;

        } break;
        case DIRECTIONAL: {
          m_directional[i] = 1;
          m_position[i * 3 + 0] = d->T.x;
          m_position[i * 3 + 1] = d->T.x;
          m_position[i * 3 + 2] = d->T.x;
        }
        break;
      }

      for(int j = 0; j < 3; j++) {
        m_Ka[3 * i + j] =
          m_context->lightPointers[i]->glLight.Ka[j];
        m_Kd[3 * i + j] =
          m_context->lightPointers[i]->glLight.Kd[j];
        m_Ks[3 * i + j] =
          m_context->lightPointers[i]->glLight.Ks[j];
      }
    }
  }
}
