#include "terrain.h"
#include "lights.h"
#include "renderwindow.h"
#include "terrainshaderprogram.h"
#include "terrainshadowshaderprogram.h"

void Shift::Terrain::loadContext() {

  m_context = m_renderWindow->getContext();
  /* initializes terrain maps, checks m_terrainInitialized to verify
  that terrain maps have not already been loaded for this context */
  if(m_context && m_context->terrainCount != 0 && m_program->isLinked()) {

    if(m_terrainObjects) {
      delete[] m_terrainObjects;
    }
    m_terrainObjects = new TerrainObjects[m_context->terrainCount];

    m_program->bind();
    for(int i = 0; i < m_context->terrainCount; i++) {
      qDebug() << "generating terrain uniforms [" << i << "]";

      s2_terrain_t* terrain = m_context->terrains[i];

      m_terrainObjects[i].textureElevation = NULL;
      m_terrainObjects[i].textureDiffuse = NULL;
      m_terrainObjects[i].textureNormal = NULL;
      m_terrainObjects[i].textureSpecular = NULL;
      m_terrainObjects[i].textureBump = NULL;

      m_terrainObjects[i].textureElevation =
        new QOpenGLTexture(QOpenGLTexture::Target2D);
      checkError("terrain: after new depth texture");

      m_terrainObjects[i].textureElevation->setFormat(QOpenGLTexture::R32F);
      checkError("terrain: after set format R32F");

      m_terrainObjects[i].textureElevation->setWrapMode(
        QOpenGLTexture::ClampToEdge);
      checkError("terrain: after set wrap mode");

      m_terrainObjects[i].textureElevation->setMinMagFilters(
        QOpenGLTexture::Linear, QOpenGLTexture::Linear);
      checkError("terrain: after set min mag filters");

      m_terrainObjects[i].textureElevation->setSize(terrain->columns,
                                                    terrain->rows, 1);
      m_terrainObjects[i].textureElevationSize =
        terrain->columns * terrain->rows;
      checkError("terrain: after set size");
      m_terrainObjects[i].textureElevation->allocateStorage();
      checkError("terrain after allocate storage for displacement texture");

      /* normalize elevation map control points from 0 to 1 */
      size_t n = terrain->columns * terrain->rows;
      float* depthFloat = (float*)malloc(n * sizeof(float));
      double range = terrain->zMax - terrain->zMin;
      for(size_t j = 0; j < n; j++) {
        depthFloat[j] =
          (float)((terrain->controlPoints[j] - terrain->zMin) / range);
        // printf(
        //   "%f -> %f\n",
        //   terrain->controlPoints[j],
        //   depthFloat[j]);
      }

      m_terrainObjects[i].textureElevation->setData(
        QOpenGLTexture::Red, QOpenGLTexture::Float32, depthFloat);
      checkError("after setData");
      free(depthFloat);

      if(terrain->diffuseMapFileName) {
        qDebug() << "creating terrain texture:" << terrain->diffuseMapFileName;
        m_terrainObjects[i].textureDiffuse =
          new QOpenGLTexture(QImage(terrain->diffuseMapFileName));
        checkError("terrain: after new color texture");

        // m_terrainObjects[i].textureColor->setFormat(
        //   QOpenGLTexture::RGBA32F);
        // checkError("color: after set format RF");
        m_terrainObjects[i].textureDiffuse->setWrapMode(QOpenGLTexture::Repeat);
        checkError("color: after set wrap mode");
        m_terrainObjects[i].textureDiffuse->setMinMagFilters(
          QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        checkError("color: after set min mag filters");

        // m_terrainObjects[i].textureColor->allocateStorage();
        // checkError("color: after allocate storage");
      }

      qDebug() << terrain->specularMapFileName;
      if(terrain->specularMapFileName) {
        qDebug() << "creating terrain specular map:"
                 << terrain->specularMapFileName;
        m_terrainObjects[i].textureSpecular =
          new QOpenGLTexture(QImage(terrain->specularMapFileName));
        checkError("terrain: after new specular texture");

        // m_terrainObjects[i].textureSpecular->setFormat(
        //   QOpenGLTexture::RGBA32F);
        // checkError("specular: after set format RF");
        m_terrainObjects[i].textureSpecular->setWrapMode(
          QOpenGLTexture::Repeat);
        checkError("specular: after set wrap mode");
        m_terrainObjects[i].textureSpecular->setMinMagFilters(
          QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        checkError("specular: after set min mag filters");
        // m_terrainObjects[i].textureSpecular->allocateStorage();
        // checkError("specular: after allocate storage");
      }

      if(terrain->normalMapFileName) {
        qDebug() << "creating terrain normal map:"
                 << terrain->normalMapFileName;
        m_terrainObjects[i].textureNormal =
          new QOpenGLTexture(QImage(terrain->normalMapFileName));
        checkError("terrain: after new normal texture");

        // m_terrainObjects[i].textureNormal->setFormat(
        //   QOpenGLTexture::RGBA32F);
        // checkError("normal after set format RF");
        m_terrainObjects[i].textureNormal->setWrapMode(QOpenGLTexture::Repeat);
        checkError("normal: after set wrap mode");
        m_terrainObjects[i].textureNormal->setMinMagFilters(
          QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        checkError("normal: after set min mag filters");
        // m_terrainObjects[i].textureNormal->allocateStorage();
        // checkError("normal: after allocate storage");
      }

      if(terrain->bumpMapFileName) {
        qDebug() << "creating terrain bump map:" << terrain->bumpMapFileName;
        m_terrainObjects[i].textureBump =
          new QOpenGLTexture(QImage(terrain->bumpMapFileName));
        checkError("terrain: after new bump texture");

        // m_terrainObjects[i].textureNormal->setFormat(
        //   QOpenGLTexture::RGBA32F);
        // checkError("normal after set format RF");
        m_terrainObjects[i].textureBump->setWrapMode(QOpenGLTexture::Repeat);
        checkError("normal: after set wrap mode");
        m_terrainObjects[i].textureBump->setMinMagFilters(
          QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        checkError("normal: after set min mag filters");
        // m_terrainObjects[i].textureNormal->allocateStorage();
        // checkError("normal: after allocate storage");
      }
    }
    m_program->release();
    checkError("after generate terrain texture uniforms");

    m_initialized = true;
    enable(true);
    emit terrainLoaded();
  }
}

/*!
@brief draw functions for rendering terrain geometry
*/
void Shift::Terrain::draw() {
  if(m_context && m_drawEnabled && m_program->isInitialized()) {

    m_program->bind();
    checkError("terrain: after program bind");

    m_program->bindVao();

    glActiveTexture(GL_TEXTURE0);

    glBindTexture(GL_TEXTURE_2D_ARRAY,
                  m_renderWindow->getLights()->getDepthTexture());
    checkError("terrain: after bind texture");

    m_program->bindDepthBuffer();
    checkError("terrain: after bind depth buffer uniform");

    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    m_program->setViewPort((float)vp[2], (float)vp[3]);
    checkError("terrain: after set viewport");

    m_program->setLights(m_lights);
    checkError("terrain: after set lights");

    for(int i = 0; i < m_context->terrainCount; i++) {

      s2_terrain_t* terrain = m_context->terrains[i];
      if(terrain->isVisible) {

        m_program->setTransform(terrain->parentFramePointer->dAbsolute);
        checkError("terrain: after set transform");

        m_program->setUniforms(terrain);
        checkError("terrain: after set uniforms");

        m_program->setTextureElevation(m_terrainObjects[i].textureElevation);
        checkError("terrain: after set texture elevation");

        m_program->setTextureDiffuse(m_terrainObjects[i].textureDiffuse);
        checkError("terrain: after set texture diffuse");

        m_program->setTextureNormal(m_terrainObjects[i].textureNormal);
        checkError("after set texture normal");

        m_program->setTextureSpecular(m_terrainObjects[i].textureSpecular);
        checkError("after set texture specular");

        m_program->setTextureBump(m_terrainObjects[i].textureBump);
        checkError("after set texture bump");

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glDepthFunc(GL_LEQUAL);

        if(m_shadingType == S2_SHADING_WIREFRAME) {
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        } else {
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
        checkError("terrain: after glPolygonMode");

        m_renderWindow->m_funcs->glPatchParameteri(GL_PATCH_VERTICES, 4);
        checkError("terrain: after set patch parameteri");

        m_renderWindow->m_funcs->glDrawArraysInstanced(
          GL_PATCHES, 0, 4, m_terrainObjects[i].textureElevationSize);
        checkError("terrain: after draw arrays instanced");

        m_terrainObjects[i].textureElevation->release();
      }
    }
    m_program->releaseVao();
    m_program->release();
  }
}

/*!
@brief draw functions for rendering terrain depth map from light's perspective
*/
void Shift::Terrain::drawShadows() {
  if(m_context && m_drawEnabled && m_renderWindow->isInitialized()) {

    m_programShadow->bind();
    checkError("terrain: after program bind");

    m_programShadow->bindVao();

    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    m_programShadow->setViewPort((float)vp[2], (float)vp[3]);

    for(int i = 0; i < m_context->terrainCount; i++) {
      s2_terrain_t* terrain = m_context->terrains[i];

      m_programShadow->setTransform(terrain->parentFramePointer->dAbsolute);
      checkError("terrain: after set transform");

      m_programShadow->setUniforms(terrain);
      checkError("terrain: after set uniforms");

      m_programShadow->setTextureElevation(
        m_terrainObjects[i].textureElevation);
      checkError("terrain: after set texture displacement");

      glEnable(GL_DEPTH_TEST);

      m_renderWindow->m_funcs->glPatchParameteri(GL_PATCH_VERTICES, 4);
      m_renderWindow->m_funcs->glDrawArraysInstanced(
        GL_PATCHES, 0, 4, m_terrainObjects[i].textureElevationSize);
      checkError("terrain: after draw arrays instanced");

      m_terrainObjects[i].textureElevation->release();
      checkError("terrain: after release depth texture");
    }
    m_programShadow->releaseVao();
    m_programShadow->release();
  }
}

/*!
@brief free memory for context datastructure
*/
void Shift::Terrain::clearContext() {
  enable(false);
  m_context = NULL;
  delete[] m_terrainObjects;
  m_terrainObjects = NULL;
}

/*!
@brief sets pointer to lights datastructure
*/
void Shift::Terrain::setLights(Lights* lights) {
  m_lights = lights;
}
