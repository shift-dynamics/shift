#include "terrain.hpp"
#include "terrainshaderprogram.hpp"
#include "renderwindow.hpp"
#include "lights.hpp"


void s2_Terrain::loadContext() {

  m_context = m_renderWindow->getContext();
  /* initializes terrain maps, checks m_terrainInitialized to verify
  that terrain maps have not already been loaded for this context */
  if(m_context && m_context->terrainCount != 0 && m_program->isLinked()) {

    if(m_terrainObjects) {
      delete [] m_terrainObjects;
    }
    m_terrainObjects = new TerrainObjects[m_context->terrainCount];

    m_program->bind();
    for(int i = 0; i < m_context->terrainCount; i++) {
      qDebug() << "generating terrain uniforms [" << i << "]";

      s2_terrain_t* terrain = m_context->terrains[i];

      m_terrainObjects[i].textureDisplacement = NULL;
      m_terrainObjects[i].textureColor = NULL;
      m_terrainObjects[i].textureBump = NULL;
      m_terrainObjects[i].textureNormal = NULL;
      m_terrainObjects[i].textureSpecular = NULL;

      m_terrainObjects[i].textureDisplacement =
        new QOpenGLTexture(QOpenGLTexture::Target2D);
      checkError("terrain: after new depth texture");

      m_terrainObjects[i].textureDisplacement->setFormat(
        QOpenGLTexture::R32F);
      checkError("terrain: after set format R32F");

      m_terrainObjects[i].textureDisplacement->setWrapMode(
        QOpenGLTexture::ClampToEdge);
      checkError("terrain: after set wrap mode");

      m_terrainObjects[i].textureDisplacement->setMinMagFilters(
        QOpenGLTexture::Linear,
        QOpenGLTexture::Linear);
      checkError("terrain: after set min mag filters");

      m_terrainObjects[i].textureDisplacement->setSize(
        terrain->columns,
        terrain->rows,
        1);
      checkError("terrain: after set size");
      m_terrainObjects[i].textureDisplacement->allocateStorage();
      checkError("terrain after allocate storage for displacement texture");

      /* normalize elevation map control points from 0 to 1 */
      size_t n = terrain->columns * terrain->rows;
      float* depthFloat = (float*)malloc(n * sizeof(float));
      double range = terrain->zMax - terrain->zMin;
      for(size_t j = 0; j < n; j++) {
        depthFloat[j] = (float)((terrain->controlPoints[j] -
          terrain->zMin) / range);
        // printf(
        //   "%f -> %f\n",
        //   terrain->controlPoints[j],
        //   depthFloat[j]);
      }

      m_terrainObjects[i].textureDisplacement->setData(
        QOpenGLTexture::Red,
        QOpenGLTexture::Float32,
        depthFloat);
      checkError("after setData");

      free(depthFloat);


      m_terrainObjects[i].textureColor =
        new QOpenGLTexture(QOpenGLTexture::Target2D);
      checkError("terrain: after new color texture");

      m_terrainObjects[i].textureColor->setFormat(
        QOpenGLTexture::RGBA32F);
      checkError("after set format RF");
      m_terrainObjects[i].textureColor->setWrapMode(
        QOpenGLTexture::ClampToEdge);
      checkError("after set wrap mode");
      m_terrainObjects[i].textureColor->setMinMagFilters(
        QOpenGLTexture::Linear,
        QOpenGLTexture::Linear);
      checkError("after set min mag filters");

      // qDebug() << "allocating color texture with size: " <<
      //   elevationMap->textureColumns << "," << elevationMap->textureRows;

      m_terrainObjects[i].textureColor->setSize(
        terrain->textureColumns,
        terrain->textureRows,
        1);
      checkError("after set size");

      m_terrainObjects[i].textureColor->allocateStorage();
      checkError("after allocate storage");

      // for(size_t j = 0; j < n * 4; j++) {
      //   printf("[%lu]: %d\n", j, terrain->texture[j]);
      // }

      m_terrainObjects[i].textureColor->setData(
        QOpenGLTexture::RGBA,
        QOpenGLTexture::UInt8,
        terrain->texture);
      checkError("after setData color texture");

    }
    m_program->release();
    checkError("after generate terrain texture uniforms");

    m_initialized = true;
    enable(true);
    emit terrainLoaded();
  }
}


void s2_Terrain::draw() {
  if(m_context && m_drawEnabled && m_program->isInitialized()) {
    checkError("terrain: before program bind");

    m_program->bind();
    checkError("terrain: after program bind");

    m_program->bindVao();

    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    m_program->setViewPort((float)vp[2], (float)vp[3]);
    checkError("terrain: after set viewport");

    m_program->setLights(m_lights);
    checkError("terrain: after set lights");

    for(int i = 0; i < m_context->terrainCount; i++) {
      s2_terrain_t* terrain = m_context->terrains[i];

      QMatrix4x4 model =
        QMatrix4x4(1.f, 0.f, 0.f, 0.f,
                   0.f, 1.f, 0.f, 0.f,
                   0.f, 0.f, 1.f, 0.f,
                   0.f, 0.f, 0.f, 1.f);
      m_program->setTransform(model);
      checkError("terrain: after set transform");

      m_program->setUniforms(terrain);
      checkError("terrain: after set uniforms");

      m_terrainObjects[i].textureDisplacement->bind(0);
      m_program->setTextureDisplacement(
        m_terrainObjects[i].textureDisplacement);
      checkError("terrain: after set texture displacement");

      m_terrainObjects[i].textureColor->bind(1);
      m_program->setTextureColor(
        m_terrainObjects[i].textureColor);
      checkError("terrain: after set texture color");

      // m_program->setTextureBump(
      //   m_terrainObjects[i].textureBump);
      // checkError("after set texture bump");
      //
      // m_program->setTextureNormal(
      //   m_terrainObjects[i].textureNormal);
      // checkError("after set texture normal");
      //
      // m_program->setTextureSpecular(
      //   m_terrainObjects[i].textureSpecular);
      // checkError("after set texture specular");

      glEnable(GL_DEPTH_TEST);
      glEnable(GL_CULL_FACE);
      glDepthFunc(GL_LEQUAL);

      if(m_shadingType == WIREFRAME) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }
      checkError("terrain: after glPolygonMode");

      m_renderWindow->m_funcs->glPatchParameteri(GL_PATCH_VERTICES, 4);
      checkError("terrain: after set patch parameteri");

      m_renderWindow->m_funcs->glDrawArraysInstanced(GL_PATCHES, 0, 4, 64 * 64);
      checkError("terrain: after draw arrays instanced");

      m_terrainObjects[i].textureDisplacement->release();
      checkError("terrain: after release depth texture");

      m_terrainObjects[i].textureColor->release();
      checkError("terrain: after release color texture");

    }
    m_program->releaseVao();
    m_program->release();
  }
}


/*!
@brief free memory for context datastructure
*/
void s2_Terrain::clearContext() {
  enable(false);
  m_context = NULL;
  delete [] m_terrainObjects;
  m_terrainObjects = NULL;
}


/*!
@brief sets pointer to lights datastructure
*/
void s2_Terrain::setLights(s2_Lights* lights) {
  m_lights = lights;
}
