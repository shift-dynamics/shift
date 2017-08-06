#include "terrain.hpp"

/*!
@brief initialize terrain textures

*/
void RenderWindow::initialize() {

  /* compiles terrain shader only if terrain maps used in
  star2 input file */
  if(m_context->elevationMapCount != 0 && !m_initialized) {

    qDebug() << "compiling terrain shader";
    m_program = new QOpenGLShaderProgram(m_renderWindow);
    m_program->addShaderFromSourceFile(
      QOpenGLShader::Vertex,
      ":/shaders/terrain.vert");
    m_program->addShaderFromSourceFile(
      QOpenGLShader::TessellationControl,
      ":/shaders/terrain.tesc");
    m_program->addShaderFromSourceFile(
      QOpenGLShader::TessellationEvaluation,
      ":/shaders/terrain.tese");
    m_program->addShaderFromSourceFile(
      QOpenGLShader::Fragment,
      ":/shaders/terrain.frag");
    m_program->link();
    checkError("after link program terrain");

    /* most program uniforms are initialized here */
    initializeProgramUniforms(
      &m_uniformsTerrain,
      m_programTerrain);

    /* initialize additional elevation map specific uniforms */
    m_programTerrain->bind();
    m_uniformsTerrain.projectionMatrix =
      m_programTerrain->uniformLocation("proj_matrix");
    m_uniformsTerrain.textureDisplacement =
      m_programTerrain->uniformLocation("tex_displacement");
    m_uniformsTerrain.textureColor =
      m_programTerrain->uniformLocation("tex_color");
    m_uniformsTerrain.zMax =
      m_programTerrain->uniformLocation("zMax");
    m_uniformsTerrain.zMin =
      m_programTerrain->uniformLocation("zMin");
    m_uniformsTerrain.viewport =
      m_programTerrain->uniformLocation("viewport");

    m_uniformsTerrain.KaOverKd =
      m_programTerrain->uniformLocation("KaOverKd");
    checkError("after set uniform location KaOverKd");
    // qDebug() << "Kd: " << programUniforms->Kd;

    m_uniformsTerrain.KsOverKd =
      m_programTerrain->uniformLocation("KsOverKd");
    checkError("after set uniform location Ks");

    m_programTerrain->release();

    checkError("after initialize terrain uniforms");
    m_terrainShaderInitialized = true;
  }

  /* initializes terrain maps, checks m_terrainInitialized to verify
  that terrain maps have not already been loaded for this context */
  if(m_context->elevationMapCount != 0 && !m_terrainInitialized) {
    if(m_terrainObjects) {
      delete [] m_terrainObjects;
    }
    m_terrainObjects = new TerrainObjects[m_context->elevationMapCount];

    m_programTerrain->bind();
    for(int i = 0; i < m_context->elevationMapCount; i++) {

      s2_elevationmap_t* elevationMap = m_context->elevationmaps[i];

      m_terrainObjects[i].depthTexture =
        new QOpenGLTexture(QOpenGLTexture::Target2D);
      checkError("after new depth texture");
      m_terrainObjects[i].depthTexture->setFormat(
        QOpenGLTexture::R32F);
      checkError("after set format R32F");
      m_terrainObjects[i].depthTexture->setWrapMode(
        QOpenGLTexture::ClampToEdge);
      checkError("after set wrap mode");
      m_terrainObjects[i].depthTexture->setMinMagFilters(
        QOpenGLTexture::Linear,
        QOpenGLTexture::Linear);
      checkError("after set min mag filters");
      m_terrainObjects[i].depthTexture->setSize(
        elevationMap->columns,
        elevationMap->rows,
        1);
      checkError("after set size");
      m_terrainObjects[i].depthTexture->allocateStorage();
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

      m_terrainObjects[i].depthTexture->setData(
        QOpenGLTexture::Red,
        QOpenGLTexture::Float32,
        depthFloat);
      checkError("after setData");
      free(depthFloat);


      m_terrainObjects[i].colorTexture =
        new QOpenGLTexture(QOpenGLTexture::Target2D);
      checkError("after new color texture");

      m_terrainObjects[i].colorTexture->setFormat(
        QOpenGLTexture::RGBA32F);
      checkError("after set format RF");
      m_terrainObjects[i].colorTexture->setWrapMode(
        QOpenGLTexture::ClampToEdge);
      checkError("after set wrap mode");
      m_terrainObjects[i].colorTexture->setMinMagFilters(
        QOpenGLTexture::Linear,
        QOpenGLTexture::Linear);
      checkError("after set min mag filters");

      // qDebug() << "allocating color texture with size: " <<
      //   elevationMap->textureColumns << "," << elevationMap->textureRows;

      m_terrainObjects[i].colorTexture->setSize(
        elevationMap->textureColumns,
        elevationMap->textureRows,
        1);
      checkError("after set size");

      m_terrainObjects[i].colorTexture->allocateStorage();
      checkError("after allocate storage");

      // size_t size = elevationMap->textureRows * elevationMap->textureColumns * 4;
      // for(size_t i = 0; i < size; i++) {
      //   printf("[%lu]: %d\n", i, elevationMap->texture[i]);
      // }

      m_terrainObjects[i].colorTexture->setData(
        QOpenGLTexture::RGBA,
        QOpenGLTexture::UInt8,
        elevationMap->texture);
      checkError("after setData color texture");

    }
    m_programTerrain->release();
    checkError("after generate terrain texture uniforms");

    m_drawTerrain = true;
    m_terrainInitialized = true;
  }
}
