#include "terrain.hpp"
#include "renderwindow.hpp"

/*!
@brief initialize terrain textures

*/
void s2_Terrain::initialize() {

  /* compiles terrain shader only if terrain maps used in
  star2 input file */
  if(m_context->elevationMapCount != 0 && !m_initialized) {

    // qDebug() << "compiling terrain shader";
    // m_program = new QOpenGLShaderProgram(m_renderWindow);
    // m_program->addShaderFromSourceFile(
    //   QOpenGLShader::Vertex,
    //   ":/shaders/terrain.vert");
    // m_program->addShaderFromSourceFile(
    //   QOpenGLShader::TessellationControl,
    //   ":/shaders/terrain.tesc");
    // m_program->addShaderFromSourceFile(
    //   QOpenGLShader::TessellationEvaluation,
    //   ":/shaders/terrain.tese");
    // m_program->addShaderFromSourceFile(
    //   QOpenGLShader::Fragment,
    //   ":/shaders/terrain.frag");
    // m_program->link();
    // checkError("after link program terrain");

    /* most program uniforms are initialized here */
    // initializeProgramUniforms(
    //   &m_uniformsTerrain,
    //   m_program);

    /* initialize additional elevation map specific uniforms */
    m_program->bind();
    m_projectionMatrix_u =
      m_program->uniformLocation("proj_matrix");
    m_textureDisplacementMap_u =
      m_program->uniformLocation("tex_displacement");
    m_textureColor_u = m_program->uniformLocation("tex_color");
    m_zMax_u = m_program->uniformLocation("zMax");
    m_zMin_u = m_program->uniformLocation("zMin");
    m_viewport_u = m_program->uniformLocation("viewport");

    m_KaOverKd_u = m_program->uniformLocation("KaOverKd");
    checkError("after set uniform location KaOverKd");
    // qDebug() << "Kd: " << programUniforms->Kd;

    m_KsOverKd_u = m_program->uniformLocation("KsOverKd");
    checkError("after set uniform location Ks");

    m_program->release();

    checkError("after initialize terrain uniforms");
    m_initialized = true;
  }

}

void s2_Terrain::loadContext() {

  m_context = m_renderWindow->getContext();
  /* initializes terrain maps, checks m_terrainInitialized to verify
  that terrain maps have not already been loaded for this context */
  if(m_context && m_context->elevationMapCount != 0 && !m_initialized) {
    if(m_terrainObjects) {
      delete [] m_terrainObjects;
    }
    m_terrainObjects = new TerrainObjects[m_context->elevationMapCount];

    m_program->bind();
    for(int i = 0; i < m_context->elevationMapCount; i++) {

      s2_elevationmap_t* elevationMap = m_context->elevationmaps[i];

      m_terrainObjects[i].textureDisplacementMap =
        new QOpenGLTexture(QOpenGLTexture::Target2D);
      checkError("after new depth texture");
      m_terrainObjects[i].textureDisplacementMap->setFormat(
        QOpenGLTexture::R32F);
      checkError("after set format R32F");
      m_terrainObjects[i].textureDisplacementMap->setWrapMode(
        QOpenGLTexture::ClampToEdge);
      checkError("after set wrap mode");
      m_terrainObjects[i].textureDisplacementMap->setMinMagFilters(
        QOpenGLTexture::Linear,
        QOpenGLTexture::Linear);
      checkError("after set min mag filters");
      m_terrainObjects[i].textureDisplacementMap->setSize(
        elevationMap->columns,
        elevationMap->rows,
        1);
      checkError("after set size");
      m_terrainObjects[i].textureDisplacementMap->allocateStorage();
      checkError("after allocate storage");

      /* normalize elevation map control points from 0 to 1 */
      size_t n = elevationMap->columns * elevationMap->rows;
      float* depthFloat = (float*)malloc(n * sizeof(float));
      double range = elevationMap->zMax - elevationMap->zMin;
      for(size_t j = 0; j < n; j++) {
        depthFloat[j] = (float)((elevationMap->controlPoints[j] -
          elevationMap->zMin) / range);
        // printf(
        //   "%f -> %f\n",
        //   elevationMap->controlPoints[j],
        //   depthFloat[j]);
      }

      m_terrainObjects[i].textureDisplacementMap->setData(
        QOpenGLTexture::Red,
        QOpenGLTexture::Float32,
        depthFloat);
      checkError("after setData");
      free(depthFloat);


      m_terrainObjects[i].textureColor =
        new QOpenGLTexture(QOpenGLTexture::Target2D);
      checkError("after new color texture");

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
        elevationMap->textureColumns,
        elevationMap->textureRows,
        1);
      checkError("after set size");

      m_terrainObjects[i].textureColor->allocateStorage();
      checkError("after allocate storage");

      // size_t size = elevationMap->textureRows * elevationMap->textureColumns * 4;
      // for(size_t i = 0; i < size; i++) {
      //   printf("[%lu]: %d\n", i, elevationMap->texture[i]);
      // }

      m_terrainObjects[i].textureColor->setData(
        QOpenGLTexture::RGBA,
        QOpenGLTexture::UInt8,
        elevationMap->texture);
      checkError("after setData color texture");

    }
    m_program->release();
    checkError("after generate terrain texture uniforms");

    m_drawEnabled = true;
    m_initialized = true;
  }
}


void s2_Terrain::draw() {

  
}
