
#include "renderwindow.hpp"
#include "../star2.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QScreen>
#include <QMouseEvent>
#include <cmath>
#include <QFileDialog>
#include <QResizeEvent>
#include <iostream>
#include <QVector3D>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <QGenericMatrix>
#include <QTimer>

RenderWindow::RenderWindow()
  : OpenGLWindow()
  , m_context(NULL)
  , m_x(0.0)
  , m_y(0.0)
  , m_xRot(0.0)
  , m_yRot(0.0)
  , m_zRot(0.0)
  , m_xPan(0.0)
  , m_yPan(0.0)
  , m_zPan(0.0)
  , m_camDistance(3.0)
  , m_perspective(true)
  , m_initialized(false)
  , m_simulationRunning(false) {

  setAnimating(true);

  /*
    initialize background color
    @TODO move default background color to a config or theme file
  */
  m_backgroundColor[0] = 0.f / 255.f;
  m_backgroundColor[1] = 0.f / 255.f;
  m_backgroundColor[2] = 0.f / 255.f;
  m_backgroundColor[3] = 1.f;

  /* set initial normal display color */
  m_normalColor[0] = 246.f / 255.f;
  m_normalColor[1] = 116.f / 255.f;
  m_normalColor[2] = 255.f / 255.f;
  m_normalColor[3] = 1.0f;

  m_grid = new s2_Grid(this);
  m_bodies = new s2_Bodies(this);
  m_frames = new s2_Frames(this);
  m_joints = new s2_Joints(this);
  m_lights = new s2_Lights(this);
  m_terrain = new s2_Terrain(this);


  /* set initial plane display colors */
  // m_planeFaceColor[0] = 36.f / 255.f;
  // m_planeFaceColor[1] = 36.f / 255.f;
  // m_planeFaceColor[2] = 36.f / 255.f;
  // m_planeFaceColor[3] = 0.5f;

  // m_planeLineColor[0] = 36.f / 255.f;
  // m_planeLineColor[1] = 36.f / 255.f;
  // m_planeLineColor[2] = 36.f / 255.f;
  // m_planeLineColor[3] = 1.0f;

  m_timer = new QTimer();
  m_timer->setTimerType(Qt::PreciseTimer);
  m_timer->setSingleShot(true);
  connect(m_timer, SIGNAL(timeout()), this, SLOT(stopSimulationPlayback()));
  // m_drawVertexNormals = false;
  // m_drawJoints = false;
  // m_shadingType = PHONG;
}


/*!
@brief resize GL window signal
*/
void RenderWindow::resizeGL(
  int width,
  int height) {
}


/*!
@brief report whether render window is initialized
*/
bool RenderWindow::isInitialized() {
  return m_initialized;
}


/*!
@brief handler for mouse move events

@param event a QMouseEvent
*/
void RenderWindow::mouseMoveEvent(
  QMouseEvent* event) {

  int dx = event->x() - m_lastPos.x();
  int dy = event->y() - m_lastPos.y();

  if(event->buttons() & Qt::LeftButton) {
    setXRotation(m_xRot + 2 * dy);
    setZRotation(m_zRot + 2 * dx);
  } else if (event->buttons() & Qt::RightButton) {
    m_xPan += dx / 120.0;
    m_zPan += dy / 120.0;
    updateGL();
  }

  m_lastPos = event->pos();
  m_x = event->pos().x();
  m_y = event->pos().y();
  emit mouseMove();
}


/*!
@brief handler for mouse press events

@param event a QMouseEvent
*/
void RenderWindow::mousePressEvent(
  QMouseEvent* event) {

  m_lastPos = event->pos();
  if(event->button() == Qt::RightButton) {
    emit mouseRightPressEvent();
  }

  if(event->button() == Qt::LeftButton) {
    emit mouseLeftPressEvent();
  }
}


/*!
@brief handler for mouse release events

@param event a QMouseEvent
*/
void RenderWindow::mouseReleaseEvent(
  QMouseEvent* event) {

  emit mouseReleaseEvent();
}


/*!
@brief handler for mouse scroll wheel events

@param event a QMouseEvent
*/
void RenderWindow::wheelEvent(
  QWheelEvent* event) {

  m_camDistance += (event->delta() / 360.0);
  if(m_camDistance < 0) m_camDistance = 0;
  updateGL();
}


/*!
@brief toggles display of vertex normals, currently not implemented

*/
void RenderWindow::toggleVertexNormalDisplay() {
  m_drawVertexNormals = !m_drawVertexNormals;
}


/*!
@brief toggles display of coordinate systems

*/
void RenderWindow::toggleDrawFrames() {
  m_frames->toggleEnabled();
}


/*!
@brief toggles display of coordinate systems

*/
void RenderWindow::toggleDrawJoints() {
  m_joints->toggleEnabled();
}

/*!
@brief toggle display of terrain map
*/
void RenderWindow::toggleDrawTerrain() {
  m_terrain->toggleEnabled();
}


/*!
@brief toggle display of dynamic shadow mapping
*/
void RenderWindow::toggleDrawShadows() {
  m_lights->toggleEnabled();
}


/*!
@brief gets current position of cursor

@return cursor position as a string "x,y=0.00,0.00"
*/
QString RenderWindow::getCursorPositionText() {
  QString str;
  QTextStream(&str) << m_x << "," << m_y;
  return str;
}


QMatrix4x4 s2_displacementToQMatrix4x4(
  s2_displacement_t* displacement) {

  double* d = (double*)displacement;
  return
    QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
               (float)d[1], (float)d[4], (float)d[7], (float)d[10],
               (float)d[2], (float)d[5], (float)d[8], (float)d[11],
               0.f,  0.f,  0.f,  1.f);
}

// /*!
// @brief initialize lights
// */
// void RenderWindow::initializeLights() {
//
//   /* @TODO move lights to uniform buffer blocks
//     m_funcs->glGetActiveUniformBlockName();
//   */
//   // qDebug() << "initializing lights";
//   // qDebug() << "lightCount: " << m_context->activeLightCount;
//   if(m_context->activeLightCount > 10)
//     m_lightParameters.lightCount = 10;
//   else m_lightParameters.lightCount = m_context->activeLightCount;
//
//   for(unsigned int i = 0; i < m_lightParameters.lightCount; i++) {
//     QVector4D position;
//     s2_light_t* light = m_context->lightPointers[i];
//     s2_displacement_t* d = light->parentFramePointer->dAbsolute;
//     switch(light->lightType) {
//       case POINT: {
//         m_lightParameters.directional[i] = 0;
//         m_lightParameters.posLight[i * 3 + 0] = d->T.x;
//         m_lightParameters.posLight[i * 3 + 1] = d->T.y;
//         m_lightParameters.posLight[i * 3 + 2] = d->T.z;
//
//       } break;
//       case DIRECTIONAL: {
//         m_lightParameters.directional[i] = 1;
//         m_lightParameters.posLight[i * 3 + 0] = d->T.x;
//         m_lightParameters.posLight[i * 3 + 1] = d->T.x;
//         m_lightParameters.posLight[i * 3 + 2] = d->T.x;
//       }
//       break;
//     }
//
//     for(int j = 0; j < 3; j++) {
//       m_lightParameters.KaLight[3 * i + j] =
//         m_context->lightPointers[i]->glLight.Ka[j];
//       m_lightParameters.KdLight[3 * i + j] =
//         m_context->lightPointers[i]->glLight.Kd[j];
//       m_lightParameters.KsLight[3 * i + j] =
//         m_context->lightPointers[i]->glLight.Ks[j];
//     }
//   }
// }


/*!
@brief set camera x rotation
*/
void RenderWindow::setXRotation(
  int angle) {

  angle = s2_normalizeDegrees(angle);
  if(angle != m_xRot) {
    m_xRot = angle;
    emit xRotationChanged(angle);
    updateGL();
  }
}


/*!
@brief set camera y rotation
*/
void RenderWindow::setYRotation(int angle) {
  angle = s2_normalizeDegrees(angle);
  if(angle != m_yRot) {
    m_yRot = angle;
    emit yRotationChanged(angle);
    updateGL();
  }
}


/*!
@brief set camera z rotation
*/
void RenderWindow::setZRotation(int angle) {
  angle = s2_normalizeDegrees(angle);
  if(angle != m_zRot) {
    m_zRot = angle;
    emit zRotationChanged(angle);
    updateGL();
  }
}


/*!
@brief set camera projection to perspective

*/
void RenderWindow::setPerspective() {
  m_perspective = true;
  updateGL();
}


/*!
@brief set camera projection to orthogonal

*/
void RenderWindow::setOrthogonal() {
  m_perspective = false;
  updateGL();
}

/*!
@brief set wire frame rendering
*/
void RenderWindow::setBodyDrawStyleWireframe() {

  m_bodies->setDrawStyle(WIREFRAME);
  updateGL();
}

/*!
@brief set phong shading
*/
void RenderWindow::setBodyDrawStylePhong() {
  m_bodies->setDrawStyle(PHONG);
  updateGL();
}


/*!
@brief set wire frame rendering for terrain
*/
void RenderWindow::setTerrainDrawStyleWireframe() {

  m_terrain->setDrawStyle(WIREFRAME);
  updateGL();
}


/*!
@brief set phong shading for terrain
*/
void RenderWindow::setTerrainDrawStylePhong() {
  m_terrain->setDrawStyle(PHONG);
  updateGL();
}

/* @brief load a Context data structure for rendering

*/
void RenderWindow::loadContext(
  s2_context_t* context) {
  m_context = context;
  initialize();
}


/*!
@brief check error in OpenGL

*/
void RenderWindow::checkError(
  const QString &prefix) {

  GLenum glErr = glGetError();

  while( glErr != GL_NO_ERROR ) {
    QString error;
    switch (glErr) {
    case GL_NO_ERROR:
      break;
    case GL_INVALID_OPERATION:
      qDebug() << prefix << ": INVALID_OPERATION";
      break;
    case GL_INVALID_ENUM:
      qDebug() << prefix << ": INVALID_ENUM";
      break;
    case GL_INVALID_VALUE:
      qDebug() << prefix << ": INVALID_VALUE";
      break;
    case GL_OUT_OF_MEMORY:
      qDebug() << prefix << ": OUT_OF_MEMORY";
      break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      qDebug() << prefix << ": INVALID_FRAMEBUFFER_OPERATION";
      break;
    }
    glErr = glGetError();
  }
}


/*!
@brief get shader source code
*/
const GLchar* RenderWindow::getShaderSource(
  const char* shaderSource) {

  GLchar* out;
  QString fileContents;
  QFile file(shaderSource);
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();
  std::string result = fileContents.toLocal8Bit().constData();
  int len = strlen(result.c_str());
  out = (GLchar*) malloc((len + 1) * sizeof(char));
  strcpy(out, result.c_str());
  out[len] = '\0';
  return out;
}


/*!
@brief update lights, currently not implemented

*/
// void RenderWindow::updateLights() {

  // GLint activeLightCount = 10;
  // if(m_context->activeLightCount <= 10) {
  //   activeLightCount = (GLint) m_context->activeLightCount;
  // }
  //
  // m_lightsBuffer = new UniformBufferObject();
  // m_lightsBuffer->create();
  // m_lightsBuffer->bind(m_programPhong->programId(), "light");
  //
  // m_lightCountBuffer = new UniformBufferObject();
  // m_lightCountBuffer->create();
  // m_lightCountBuffer->bind(m_programPhong->programId(), "count");

  //Update the uniforms using ARB_uniform_buffer_object
  // glGenBuffers = (PFNGLGENBUFFERSPROC)QGLContext->getProcAddress("glGenBuffers");
  //
  // GLuint buffer_id, uniformBlockIndex, index, vshad_id, fshad_id, prog_id;
  //   GLsizei uniformBlockSize;
  //   GLint singleSize, offset;
  // *PFNGLGENBUFFERSPROC(&buffer_id);
  //
  // //There's only one uniform block here, the 'colors0' uniform block.
  // //It contains the color info for the gooch shader.
  // uniformBlockIndex = glGetUniformBlockIndex(prog_id, "colors0");
  //
  // //We need to get the uniform block's size in order to back it with the
  // //appropriate buffer
  // glGetActiveUniformBlockiv(prog_id, uniformBlockIndex,
  //                              GL_UNIFORM_BLOCK_DATA_SIZE,
  //                              &uniformBlockSize);
  // glError();
  //
  // //Create UBO.
  // glBindBuffer(GL_UNIFORM_BUFFER, buffer_id);
  // glBufferData(GL_UNIFORM_BUFFER, uniformBlockSize,
  //              NULL, GL_DYNAMIC_DRAW);
  //
  // //Now we attach the buffer to UBO binding point 0...
  // glBindBufferBase(GL_UNIFORM_BUFFER, 0, buffer_id);
  // //And associate the uniform block to this binding point.
  // glUniformBlockBinding(prog_id, uniformBlockIndex, 0);
  // glError();
  //
  // //To update a single uniform in a uniform block, we need to get its
  // //offset into the buffer.
  // glGetUniformIndices(prog_id, 1, &names[2], &index);
  // glGetActiveUniformsiv(prog_id, 1, &index,
  //                          GL_UNIFORM_OFFSET, &offset);
  // lightUniforms.vertexBuffer =
  //   new m_renderwindow(QOpenGLBuffer::VertexBuffer);
  // bufferObjects.vertexBuffer->create();
  //
  // glGenBuffers(1, &(m_context->lightCountUniform));
  // checkError("after gen buffer");
  //
  // glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_context->lightCountUniform);
  //
  // glBufferData(
  //   GL_ELEMENT_ARRAY_BUFFER,
  //   sizeof(GLint),
  //   &activeLightCount,
  //   GL_STATIC_DRAW);
  // s2_glCheckError("after bind uniform light count");
  //
  // for(int i = 0; i < context->activeLightCount; i++) {
  //   // glBindBufferBase(GL_UNIFORM_BUFFER, 0, )
  //   Light* light = context->lightPointers[i];
  //   glGenBuffers(1, &(light->lightUniform));
  //
  //   printf("%d\n", i);
  //   printf(
  //     "Ka: %f, %f, %f\n",
  //     light->glLight.Ka[0],
  //     light->glLight.Ka[1],
  //     light->glLight.Ka[2]);
  //
  //   glBindBufferBase(GL_UNIFORM_BUFFER, light->index + 1, light->lightUniform);
  //
  //   glBufferData(
  //     GL_ELEMENT_ARRAY_BUFFER,
  //     sizeof(light->glLight),
  //     &(light->glLight),
  //     GL_STATIC_DRAW);

  //   s2_glCheckError("after bind uniform light buffer");
  //   if(i >= 10) break;
  // }
// }


/*!
@brief initialize program uniforms
*/
// void RenderWindow::initializeProgramUniforms(
//   ProgramUniforms* programUniforms,
//   QOpenGLShaderProgram* shaderProgram) {
//
//   shaderProgram->bind();
//
//   programUniforms->directional = shaderProgram->uniformLocation("directional");
//   checkError("after set uniform location directional");
//   // qDebug() << "directional: " << programUniforms->directional;
//
//   programUniforms->posLight = shaderProgram->uniformLocation("posLight");
//   checkError("after set uniform location posLight");
//   // qDebug() << "posLight: " << programUniforms->posLight;
//
//   programUniforms->lightCount = shaderProgram->uniformLocation("lightCount");
//   checkError("after set uniform location lightCount");
//   // qDebug() << "lightCount: " << programUniforms->lightCount;
//
//   programUniforms->Ka = shaderProgram->uniformLocation("Ka");
//   checkError("after set uniform location Ka");
//   // qDebug() << "Ka: " << programUniforms->Ka;
//
//   programUniforms->Kd = shaderProgram->uniformLocation("Kd");
//   checkError("after set uniform location Kd");
//   // qDebug() << "Kd: " << programUniforms->Kd;
//
//   programUniforms->Ks = shaderProgram->uniformLocation("Ks");
//   checkError("after set uniform location Ks");
//   // qDebug() << "Ks: " << programUniforms->Ks;
//
//   // programUniforms->m = shaderProgram->uniformLocation("m");
//   // // qDebug() << "m: " << programUniforms->m;
//   // checkError("after set uniform location m");
//
//   programUniforms->v = shaderProgram->uniformLocation("v");
//   // qDebug() << "v: " << programUniforms->v;
//   checkError("after set uniform location v");
//
//   programUniforms->mv = shaderProgram->uniformLocation("mv");
//   // qDebug() << "mv: " << programUniforms->mv;
//   checkError("after set uniform location mv");
//
//   programUniforms->mvp = shaderProgram->uniformLocation("mvp");
//   // qDebug() << "mvp: " << programUniforms->mvp;
//   checkError("after set uniform location mvp");
//
//   programUniforms->nm = shaderProgram->uniformLocation("nm");
//   // qDebug() << "nm: " << programUniforms->nm;
//   checkError("after set uniform location nm");
//
//   programUniforms->KaMaterial = shaderProgram->uniformLocation("KaMaterial");
//   // qDebug() << "KaMaterial: " << programUniforms->KaMaterial;
//   checkError("after set uniform location KaMaterial");
//
//   programUniforms->KdMaterial = shaderProgram->uniformLocation("KdMaterial");
//   // qDebug() << "KdMaterial: " << programUniforms->KdMaterial;
//   checkError("after set uniform location KdMaterial");
//
//   programUniforms->KsMaterial = shaderProgram->uniformLocation("KsMaterial");
//   // qDebug() << "KsMaterial: " << programUniforms->KsMaterial;
//   checkError("after set uniform location KsMaterial");
//
//   programUniforms->Ns = shaderProgram->uniformLocation("Ns");
//   // qDebug() << "Ns: " << programUniforms->Ns;
//   checkError("after set uniform location Ns");
//   shaderProgram->release();
// }







/*!
@brief initialize attributes for each shader program

*/
// void RenderWindow::setAttributeLocation(
//   BufferObjects* bufferObjects,
//   ProgramUniforms* uniforms,
//   QOpenGLShaderProgram* shaderProgram) {
//
//   shaderProgram->bind();
//
//   bufferObjects->vao->bind();
//
//   uniforms->position =
//     shaderProgram->attributeLocation("posAttr");
//   uniforms->normal =
//     shaderProgram->attributeLocation("normAttr");
//
//   // printf("  uniform position: %d\n", uniforms->position);
//   // printf("  normal position: %d\n", uniforms->normal);
//
//   shaderProgram->bind();
//   bufferObjects->vertexBuffer->bind();
//
//   shaderProgram->enableAttributeArray(
//     uniforms->position);
//   shaderProgram->setAttributeBuffer(
//     uniforms->position,
//     GL_FLOAT,
//     0,
//     3);
//
//   if(uniforms->normal != -1) {
//     bufferObjects->vertexNormalBuffer->bind();
//     shaderProgram->enableAttributeArray(
//       uniforms->normal);
//     shaderProgram->setAttributeBuffer(
//       uniforms->normal,
//       GL_FLOAT,
//       0,
//       3);
//   }
//
//   bufferObjects->vao->release();
//   shaderProgram->release();
// }


/*!
@brief initialize all shader handles on a given material

*/




/*!
@brief compile shaders
*/
void RenderWindow::compileShaders() {

  /* initialize line shader */
  qDebug() << "compiling line shader program";
  m_programLine = new QOpenGLShaderProgram(this);
  m_programLine->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/simple.vert");
  m_programLine->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/simple.frag");
  m_programLine->link();
  checkError("after link program line");
  // m_programLine->bind();
  // m_uniformsLine.mvp = m_programLine->uniformLocation("mvp");
  // m_uniformsLine.colorUniform = m_programLine->uniformLocation("colorUniform");
  // m_programLine->release();
  // checkError("after release program line");

  /* initialize line shader */
  qDebug() << "compiling line shader program";
  m_programFrame = new QOpenGLShaderProgram(this);
  m_programFrame->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/frames.vert");
  m_programFrame->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/frame.frag");
  m_programFrame->link();
  checkError("after link program frame");

  /* compile frame shader */
  // m_programFrame->bind();
  // m_uniformsFrame.mvp = m_programFrame->uniformLocation("mvp");
  // m_programFrame->release();
  // checkError("after release program frame");

  qDebug() << "compiling flat shader program";
  m_programFlat = new QOpenGLShaderProgram(this);
  m_programFlat->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/flat.vert");
  m_programFlat->addShaderFromSourceFile(
    QOpenGLShader::Geometry,
    ":/shaders/flat.geom");
  m_programFlat->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/flat.frag");
  m_programFlat->link();
  checkError("after link program flat");
  //
  // initializeProgramUniforms(
  //   &m_uniformsFlat,
  //   m_programFlat);
  //

  qDebug() << "compiling phong";
  m_programPhong = new QOpenGLShaderProgram(this);
  m_programPhong->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/phong.vert");
  m_programPhong->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/phong.frag");
  m_programPhong->link();
  //
  // initializeProgramUniforms(
  //   &m_uniformsPhong,
  //   m_programPhong);
  //

  qDebug() << "compiling shadow shader program";
  m_programShadow = new QOpenGLShaderProgram(this);
  m_programShadow->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/shadow.vert");
  m_programShadow->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/shadow.frag");
  m_programShadow->link();
  checkError("after compiling shadow shaders");


  qDebug() << "compiling terrain shader program";
  m_programTerrain = new QOpenGLShaderProgram(this);
  m_programTerrain->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/terrain.vert");
  m_programTerrain->addShaderFromSourceFile(
    QOpenGLShader::TessellationControl,
    ":/shaders/terrain.tesc");
  m_programTerrain->addShaderFromSourceFile(
    QOpenGLShader::TessellationEvaluation,
    ":/shaders/terrain.tese");
  m_programTerrain->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/terrain.frag");
  m_programTerrain->link();
  checkError("after compiling terrain shaders");


  qDebug() << "shaders compiled";
}


// /*!
// @brief initialize frames
// */
// void RenderWindow::initializeFrames() {
//
//   /* set vertices */
//   m_uniformsFrame.vertices = new GLfloat[18];
//   m_uniformsFrame.vertices[0] = 0.f;
//   m_uniformsFrame.vertices[1] = 0.f;
//   m_uniformsFrame.vertices[2] = 0.f;
//
//   m_uniformsFrame.vertices[3] = 1.0f;
//   m_uniformsFrame.vertices[4] = 0.f;
//   m_uniformsFrame.vertices[5] = 0.f;
//
//   m_uniformsFrame.vertices[6] = 0.f;
//   m_uniformsFrame.vertices[7] = 0.f;
//   m_uniformsFrame.vertices[8] = 0.f;
//
//   m_uniformsFrame.vertices[9] = 0.f;
//   m_uniformsFrame.vertices[10] = 1.0f;
//   m_uniformsFrame.vertices[11] = 0.f;
//
//   m_uniformsFrame.vertices[12] = 0.f;
//   m_uniformsFrame.vertices[13] = 0.f;
//   m_uniformsFrame.vertices[14] = 0.f;
//
//   m_uniformsFrame.vertices[15] = 0.f;
//   m_uniformsFrame.vertices[16] = 0.f;
//   m_uniformsFrame.vertices[17] = 1.0f;
//
//   /* set colors */
//   m_uniformsFrame.colors = new GLfloat[18];
//   m_uniformsFrame.colors[0] = 1.f;
//   m_uniformsFrame.colors[1] = 0.f;
//   m_uniformsFrame.colors[2] = 0.f;
//
//   m_uniformsFrame.colors[3] = 1.f;
//   m_uniformsFrame.colors[4] = 0.f;
//   m_uniformsFrame.colors[5] = 0.f;
//
//   m_uniformsFrame.colors[6] = 0.f;
//   m_uniformsFrame.colors[7] = 1.f;
//   m_uniformsFrame.colors[8] = 0.f;
//
//   m_uniformsFrame.colors[9] = 0.f;
//   m_uniformsFrame.colors[10] = 1.f;
//   m_uniformsFrame.colors[11] = 0.f;
//
//   m_uniformsFrame.colors[12] = 0.f;
//   m_uniformsFrame.colors[13] = 0.f;
//   m_uniformsFrame.colors[14] = 1.f;
//
//   m_uniformsFrame.colors[15] = 0.f;
//   m_uniformsFrame.colors[16] = 0.f;
//   m_uniformsFrame.colors[17] = 1.f;
//
//   /* set indices */
//   m_uniformsFrame.indices = new GLuint[6];
//   m_uniformsFrame.indices[0] = 0;
//   m_uniformsFrame.indices[1] = 1;
//   m_uniformsFrame.indices[2] = 2;
//   m_uniformsFrame.indices[3] = 3;
//   m_uniformsFrame.indices[4] = 4;
//   m_uniformsFrame.indices[5] = 5;
//
//   m_programFrame->bind();
//   checkError("after frame program bind");
//
//   m_uniformsFrame.vao = new QOpenGLVertexArrayObject(this);
//   m_uniformsFrame.vao->create();
//   m_uniformsFrame.vao->bind();
//
//   m_uniformsFrame.vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
//   m_uniformsFrame.vbo->create();
//
//   m_uniformsFrame.vbo->bind();
//   m_uniformsFrame.vbo->allocate(2 * 18 * sizeof(GLfloat));
//   m_uniformsFrame.vbo->write(
//     0, m_uniformsFrame.vertices, 18 * sizeof(GLfloat));
//   m_uniformsFrame.vbo->write(
//     18 * sizeof(GLfloat), m_uniformsFrame.colors, 18 * sizeof(GLfloat));
//
//   m_uniformsFrame.ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
//   m_uniformsFrame.ibo->create();
//   m_uniformsFrame.ibo->bind();
//   m_uniformsFrame.ibo->allocate(m_uniformsFrame.indices, 6 * sizeof(GLuint));
//
//   m_uniformsFrame.position = m_programFrame->attributeLocation("posAttr");
//   m_programFrame->enableAttributeArray(m_uniformsFrame.position);
//   m_programFrame->setAttributeBuffer(m_uniformsFrame.position, GL_FLOAT, 0, 3);
//
//   m_uniformsFrame.color = m_programFrame->attributeLocation("colorAttr");
//   m_programFrame->enableAttributeArray(m_uniformsFrame.color);
//   m_programFrame->setAttributeBuffer(
//     m_uniformsFrame.color, GL_FLOAT, 18 * sizeof(GLfloat), 3);
//
//   m_programFrame->release();
//
//   checkError("after set up frame shader");
// }


// /*!
// @brief initialize grid
// */
// void RenderWindow::initializeGrid(
//   int gridSize,
//   float scaleFactor) {
//
//   m_grid = new Grid();
//
//   /* grid size must be an even number */
//   if(gridSize % 2 != 0) {
//     gridSize++;
//   }
//   m_grid->gridSize = gridSize;

  //
  // /* set up faces */
  // m_grid->faceBufferObject = new BufferObjects;
  // m_grid->faceBufferObject->transform = s2_displacement();
  // m_grid->faceBufferObject->material = s2_materials(1);
  // m_grid->faceBufferObject->material->shadingType = FLAT;
  // m_grid->faceBufferObject->material->Ns = 10;
  // memcpy(
  //   m_grid->faceBufferObject->material->Ka,
  //   m_planeFaceColor,
  //   4 * sizeof(GLfloat));
  // memcpy(
  //   m_grid->faceBufferObject->material->Kd,
  //   m_planeFaceColor,
  //   4 * sizeof(GLfloat));
  // memcpy(
  //   m_grid->faceBufferObject->material->Ks,
  //   m_planeFaceColor,
  //   4 * sizeof(GLfloat));
  // memcpy(
  //   m_grid->faceBufferObject->material->Ke,
  //   m_planeFaceColor,
  //   4 * sizeof(GLfloat));
  //
  // m_grid->faceBufferObject->material->vertexCount = 4;
  // m_grid->faceBufferObject->material->faceCount = 2;
  //
  // m_grid->faceBufferObject->material->vertices =
  //   new GLfloat[3 * m_grid->faceBufferObject->material->vertexCount];
  // m_grid->faceBufferObject->material->faces =
  //   new GLuint[3 * m_grid->faceBufferObject->material->faceCount];
  //
  // /* set up vertices and indices to patch for faces, two triangles */
  // m_grid->faceBufferObject->material->vertices[0] = -m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[1] = m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[2] = 1e-6;
  // m_grid->faceBufferObject->material->vertices[3] = m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[4] = m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[5] = 1e-6;
  // m_grid->faceBufferObject->material->vertices[6] = -m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[7] = -m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[8] = 1e-6;
  // m_grid->faceBufferObject->material->vertices[9] = m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[10] = m_grid->gridSize / 2;
  // m_grid->faceBufferObject->material->vertices[11] = 1e-6;
  //
  // m_grid->faceBufferObject->material->faces[0] = 0;
  // m_grid->faceBufferObject->material->faces[1] = 2;
  // m_grid->faceBufferObject->material->faces[2] = 1;
  // m_grid->faceBufferObject->material->faces[3] = 2;
  // m_grid->faceBufferObject->material->faces[4] = 3;
  // m_grid->faceBufferObject->material->faces[5] = 1;
  //
  // m_programFlat->bind();
  //
  // m_grid->faceBufferObject->vao = new QOpenGLVertexArrayObject(this);
  // m_grid->faceBufferObject->vao->create();
  // m_grid->faceBufferObject->vao->bind();
  //
  // m_grid->faceBufferObject->vertexBuffer =
  //   new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  // m_grid->faceBufferObject->vertexBuffer->create();
  // m_grid->faceBufferObject->vertexBuffer->bind();
  // m_grid->faceBufferObject->vertexBuffer->allocate(
  //   m_grid->faceBufferObject->material->vertices,
  //   3 * m_grid->faceBufferObject->material->vertexCount * sizeof(GLfloat));
  //
  // m_grid->faceBufferObject->ibo =
  //   new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  // m_grid->faceBufferObject->ibo->create();
  // m_grid->faceBufferObject->ibo->bind();
  // m_grid->faceBufferObject->ibo->allocate(
  //   m_grid->faceBufferObject->material->faces,
  //   3 * m_grid->faceBufferObject->material->faceCount * sizeof(GLuint));
  //
  // m_uniformsFlat.position =
  //   m_programFlat->attributeLocation("posAttr");
  // m_programFlat->enableAttributeArray(m_uniformsFlat.position);
  // m_programFlat->setAttributeBuffer(
  //   m_uniformsFlat.position,
  //   GL_FLOAT,
  //   0,
  //   3);
  //
  // m_programFlat->release();
  //
  // m_bufferObjects.append(m_grid->faceBufferObject);
  //
  // /*
  //   set up line segments
  // */
  // m_grid->lineBufferObject = new BufferObjects;
  //
  // m_grid->lineBufferObject->material = s2_materials(1);
  // m_grid->lineBufferObject->transform = s2_displacement();
  //
  // m_grid->lineBufferObject->material->shadingType = LINE;
  // m_grid->lineBufferObject->material->Ns = 10;
  //
  // /* set up vertices and line segments for grid */
  // m_grid->lineBufferObject->material->vertexCount = 4 * m_grid->gridSize;
  // m_grid->lineBufferObject->material->faceCount = 2 * (m_grid->gridSize + 1);
  //
  // m_grid->lineBufferObject->material->vertices =
  //   new GLfloat[3 * m_grid->lineBufferObject->material->vertexCount];
  // m_grid->lineBufferObject->material->vertexNormals =
  //   new GLfloat[3 * m_grid->lineBufferObject->material->vertexCount];
  // m_grid->lineBufferObject->material->faces =
  //   new GLuint[2 * m_grid->lineBufferObject->material->faceCount];
  //
  // /* set vertices for vertical line segments */
  // int gridIndex = 0;
  // for(int i = 0; i <= m_grid->gridSize; i++) {
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
  //     (float)(i - m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
  //     (float)(m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
  //   m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
  //   gridIndex++;
  //
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
  //     (float)(i - m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
  //     -(float)(m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
  //   m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
  //   gridIndex++;
  // }

  // for(int i = 0; i < gridIndex; i++) {
  //   printf("%f, %f, %f\n",
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 0],
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 1],
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 2]);
  // }

  // qDebug() << "gridIndex: " << gridIndex;

  // /* set vertices for remaining horizontal line segments */
  // for(int i = 1; i < m_grid->gridSize; i++) {
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
  //     -(float)(m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
  //     (float)(i - m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
  //   m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
  //   gridIndex++;
  //
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
  //     (float)(m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
  //     (float)(i - m_grid->gridSize / 2);
  //   m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
  //   m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
  //   gridIndex++;
  // }
  //
  // /* set top horizontal line segment */
  // m_grid->lineBufferObject->material->faces[gridIndex++] = 0;
  // m_grid->lineBufferObject->material->faces[gridIndex++] =
  //   m_grid->gridSize * 2;
  //
  // /* set bottom horizontal line segment */
  // m_grid->lineBufferObject->material->faces[gridIndex++] = 1;
  // m_grid->lineBufferObject->material->faces[gridIndex++] =
  //   m_grid->gridSize * 2 + 1;
  //
  // // qDebug() << "gridIndex final " << gridIndex;
  // // qDebug() << "face count" << m_grid->lineBufferObject->material->faceCount;
  //
  //
  // m_programLine->bind();
  // // checkError("after line program bind");
  //
  // /* initialize vao and vbo */
  // m_grid->lineBufferObject->vao = new QOpenGLVertexArrayObject(this);
  // m_grid->lineBufferObject->vao->create();
  // m_grid->lineBufferObject->vao->bind();
  //
  // // checkError("after initialize vao");
  //
  // // checkError("after set attribute posAttr in line program");
  //
  // m_grid->lineBufferObject->vertexBuffer =
  //   new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  // m_grid->lineBufferObject->vertexBuffer->create();
  //
  // m_grid->lineBufferObject->vertexBuffer->bind();
  // m_grid->lineBufferObject->vertexBuffer->allocate(
  //   m_grid->lineBufferObject->material->vertices,
  //   3 * m_grid->lineBufferObject->material->vertexCount * sizeof(GLfloat));
  // checkError("after initialize vbo");
  // m_grid->lineBufferObject->vertexNormalBuffer =
  //   new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  // m_grid->lineBufferObject->vertexNormalBuffer->create();
  //
  // m_grid->lineBufferObject->vertexNormalBuffer->allocate(
  //   m_grid->lineBufferObject->material->vertexNormals,
  //   m_grid->lineBufferObject->material->vertexCount * sizeof(GLfloat));
//
//   m_grid->lineBufferObject->ibo =
//     new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
//   m_grid->lineBufferObject->ibo->create();
//   m_grid->lineBufferObject->ibo->bind();
//
//   // checkError("after create ibo in line program");
//
//   // for(int i = 0; i < m_grid->lineBufferObject->material->faceCount * 2; i++) {
//   //   printf("[%d]\n", m_grid->lineBufferObject->material->faces[i]);
//   // }
//
//   m_grid->lineBufferObject->ibo->allocate(
//     m_grid->lineBufferObject->material->faces,
//     2 * m_grid->lineBufferObject->material->faceCount * sizeof(GLuint));
//   checkError("after set ibo in line program");
//
//   m_uniformsLine.position =
//     m_programLine->attributeLocation("posAttr");
//   m_programLine->enableAttributeArray(m_uniformsLine.position);
//   m_programLine->setAttributeBuffer(
//     m_uniformsLine.position,
//     GL_FLOAT,
//     0,
//     3);
//
//   m_programLine->release();
//
//   m_bufferObjects.append(m_grid->lineBufferObject);
// }


/*! @brief Initialize render window

*/
void RenderWindow::initialize() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(
    m_backgroundColor[0],
    m_backgroundColor[1],
    m_backgroundColor[2],
    1.0f);
  glEnable(GL_DEPTH_TEST);
  // glEnable(GL_BLEND);
  // glEnable(GL_POLYGON_SMOOTH);

  if(!m_initialized) {
    m_grid->initialize();
    m_frames->initialize();
    m_joints->initialize();
    m_lights->initialize();
    m_terrain->initialize();

    m_initialized = true;
    emit initialized();
  }

  /*
    initialize uniforms on each body / material, initialize buffers and vertex
    array objects
  */
  if(m_context) {
    // FILE* debug = m_context->modelParameters->debugFilePointer;


    m_bodies->loadContext();
    m_lights->loadContext();
    m_terrain->loadContext();

    // printf("active light count : %d\n", context->activeLightCount);
    updateGL();
  }
}


/*! @brief updateGL function needs to be called on mouse and screen resize events

*/
void RenderWindow::updateGL() {

  if(m_initialized) {
    m_view.setToIdentity();

    QVector3D eye(-m_xPan, -m_yPan + m_camDistance, -m_zPan);
    QVector3D center(-m_xPan, -m_yPan, -m_zPan);
    QVector3D up(0.f, 0.f, -1.f);

    m_view.lookAt(eye, center, up);
    m_view.rotate(m_xRot, 1.0, 0.0, 0.0);
    m_view.rotate(m_zRot, 0.0, 0.0, -1.0);

    m_projection.setToIdentity();
    const qreal retinaScale = devicePixelRatio();
    if(m_initialized) {
      glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    }
    float aspectRatio = (float)width() / (float)height();
    if (m_perspective) {
      m_projection.perspective(
        60,
        (float)width() / (float)height(),
        0.01,
        10000);
    } else {
      float h = 0.5 * m_camDistance;
      float w = aspectRatio * h;
      m_projection.ortho(-w, w, -h, h, 0.01, 10000);
    }
    checkError("after updateGL");
  }
}


/*!
  @brief set joint to be incremented
*/
void RenderWindow::setJoint(
  s2_joint_t* joint,
  int qIndex,
  int dqIndex,
  double delq,
  s2_joint_enum_t jointType) {

  m_joints->set(
    joint,
    qIndex,
    dqIndex,
    delq,
    jointType);
}


/*!
  @brief set joint to be incremented
*/
void RenderWindow::setJointTarget(
  s2_joint_t* joint,
  int qIndex,
  int dqIndex,
  double targetVal,
  s2_joint_enum_t jointType) {

  m_joints->setTarget(
    joint,
    qIndex,
    dqIndex,
    targetVal,
    jointType);
}


/*!
  @brief clear joint to be incremented
*/
void RenderWindow::clearJoint() {
  m_joints->clear();
}


/*!
@brief move joint incrementally if user specifies a joint is to be moved
using joint slider interface.

This function should not be called while simulation is running, or while
simulation is being played back.
*/
void RenderWindow::updateJoint() {
  m_joints->update();
}


/*!
@brief used to start playback of a simulation after computation has finished
*/
void RenderWindow::startSimulationPlayback() {
  m_timer->start(int(1000 * m_context->modelParameters->timeSpan));
  setSimulationRunning(true);
  // qDebug() << "starting simulation playback for t=" <<  m_context->modelParameters->timeSpan;

}


/*!
@brief used to start playback of a simulation after computation has finished
at time t
*/
void RenderWindow::startSimulationPlayback(double t) {
  double timeRemaining = m_context->modelParameters->timeSpan - t;
  m_timer->start(int(1000 * timeRemaining));
  setSimulationRunning(true);
  // qDebug() << "starting simulation playback";

}


/*!
@brief used to stop playback of a simulation after computation has finished
*/
void RenderWindow::stopSimulationPlayback() {
  setSimulationRunning(false);
  emit simulationStopped();
  // qDebug() << "stopping simulation playback";
}


/*!
@brief interpolated current position in ydata array for playback of a simulation

if the simulation is running, get the current time from the timer,
use tspan and tstep to find the current position in the ydata array
use timer to interpolate between two positions and set q and dq arrays,
then update DH
*/
void RenderWindow::interpolateCurrentPosition() {
  if(m_context && m_timer->remainingTime() != -1) {

    int neqn = 0;
    int stepCount = 0;
    double* ydata = NULL;

    switch(m_context->modelParameters->integrationType) {
      case ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      case ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      default: {
        exit(-1);
      } break;
    }

    /* find current time, and time at knot point before and
    after time */
    // qDebug() << "remaining time: " << m_timer->remainingTime();
    double t =
      fmax(0, m_context->modelParameters->timeSpan -
      (double)m_timer->remainingTime() / 1000.0);

    double t0 = t - fmod(t, m_context->modelParameters->timeStep);
    double t1 = t0 + m_context->modelParameters->timeStep;
    // t1 = fmin(t1, m_context->modelParameters->timeSpan);

    /* find index of first knot point */
    int i0 = floor(t0 / m_context->modelParameters->timeStep);
    i0 = fmin(i0, stepCount - 1);
    int i1 = fmin(i0 + 1, stepCount - 1);
    // i1 = fmax(i1, i0);

    double p = (t - t0) / (t1 - t0);

    // qDebug() << "interpolating, t=" << t << ", t0=" << t0 << "t1=" << t1 << "i0" << i0 << "i1:" << i1 << "p:" << p << "stepCount" << stepCount;
    // printf("i0: ");
    // s2_array_print(m_context->qCount, ydata + i0 * neqn);
    // printf("i1: ");
    // s2_array_print(m_context->qCount, ydata + i1 * neqn);

    /* yOut = y0 + (y1 - y0) / (t1 - t0) * (t - t0) */
    s2_array_lerp(
      m_context->qCount + m_context->dqCount,
      p,
      ydata + i0 * neqn,
      ydata + i1 * neqn,
      m_context->q);

    s2_updateDH(m_context);
    s2_updateGLReferenceFrames(m_context);
    emit simulationUpdated(t);
  }
}


/*!
@brief interpolated current position in ydata array for playback of a simulation
for a given time, t

use tspan and tstep to find the current position in the ydata array
use timer to interpolate between two positions and set q and dq arrays,
then update DH
*/
void RenderWindow::interpolateCurrentPosition(double t) {
  if(m_context) {

    int neqn = 0;
    int stepCount = 0;
    double* ydata = NULL;

    switch(m_context->modelParameters->integrationType) {
      case ODE113: {
        s2_numericalintegrator_ode113_t* numericalIntegrator =
          (s2_numericalintegrator_ode113_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      case ODE113B: {
        s2_numericalintegrator_ode113b_t* numericalIntegrator =
          (s2_numericalintegrator_ode113b_t*) m_context->numericalIntegrator;
        neqn = numericalIntegrator->neqn;
        stepCount = numericalIntegrator->stepCount;
        ydata = numericalIntegrator->ydata;
      } break;
      default: {
        exit(-1);
      };
    }

    double t0 = t - fmod(t, m_context->modelParameters->timeStep);
    double t1 = t0 + m_context->modelParameters->timeStep;

    /* find index of first knot point */
    int i0 = floor(t0 / m_context->modelParameters->timeStep);
    int i1 = fmin(i0 + 1, stepCount);

    double p = (t - t0) / (t1 - t0);

    // qDebug() << "interpolating, t=" << t << ", t0=" << t0 << "t1=" << t1 << "i0" << i0 << "i1:" << i1 << "p:" << p;

    /*
      yOut = y0 + (y1 - y0) / (t1 - t0) * (t - t0)

    */
    s2_array_lerp(
      neqn,
      p,
      ydata + i0 * neqn,
      ydata + i1 * neqn,
      m_context->q);
    // emit simulationUpdated(t);

    s2_updateDH(m_context);
    s2_updateGLReferenceFrames(m_context);
  }
}


/*!
@brief set simulation running boolean to true or false
*/
void RenderWindow::setSimulationRunning(bool val) {
  m_simulationRunning = val;
}


/*!
@brief main render loop
*/
void RenderWindow::render() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(
    m_backgroundColor[0],
    m_backgroundColor[1],
    m_backgroundColor[2],
    0.0f);

  if(m_initialized && isAnimating()) {

    /* if the simulation is running, get the current state of the mechanism
    from ydata array and update mechanism for display */
    if(m_simulationRunning) {
      interpolateCurrentPosition();
    } else {
      /* update position of current joint if driven by user */
      m_joints->update();
    }

    m_frames->draw();
    m_terrain->draw();
    m_joints->draw();


        case LINE: {
          m_programLine->bind();
          m_bufferObjects.at(i)->vao->bind();
          double* d = (double*)(m_bufferObjects.at(i)->transform);
          QMatrix4x4 model = QMatrix4x4(d[0], d[3], d[6], d[9],
                                        d[1], d[4], d[7], d[10],
                                        d[2], d[5], d[8], d[11],
                                        0.f,  0.f,  0.f,  1.f);

          s2_material_t* material = m_bufferObjects.at(i)->material;
          QMatrix4x4 mv = m_view * model;

          /* set model * view * projection matrix */
          m_programLine->setUniformValue(
            m_uniformsLine.mvp,
            m_projection * mv);
          checkError("line: after set mvp");

          m_programLine->setUniformValue(
            m_uniformsLine.colorUniform,
            m_planeLineColor[0],
            m_planeLineColor[1],
            m_planeLineColor[2],
            m_planeLineColor[3]);
          checkError("after set uniform color");

          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          checkError("after set polygon mode");

          glDrawElements(
            GL_LINES,
            2 * material->faceCount,
            GL_UNSIGNED_INT,
            0);
          checkError("after draw elements");

          m_bufferObjects.at(i)->vao->release();
          checkError("after vao release");

          m_programLine->release();
          checkError("after flat release");
        }
      }
    }
  }
}
