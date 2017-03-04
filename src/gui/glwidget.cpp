#include "glwidget.hpp"
#include "../star2.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QMatrix4x4>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

// #include <cmath>
// #include <QFileDialog>
// #include <QResizeEvent>
// #include <iostream>
// #include <QVector3D>
// #include <QMatrix3x3>
// #include <QGenericMatrix>

GLWidget::GLWidget(QWidget* parent) : QGLWidget(parent) {

    setAnimating(true);

    /*
      set initial background color
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

    /* set initial plane display colors */
    m_planeFaceColor[0] = 36.f / 255.f;
    m_planeFaceColor[1] = 36.f / 255.f;
    m_planeFaceColor[2] = 36.f / 255.f;
    m_planeFaceColor[3] = 0.5f;

    m_planeLineColor[0] = 36.f / 255.f;
    m_planeLineColor[1] = 36.f / 255.f;
    m_planeLineColor[2] = 36.f / 255.f;
    m_planeLineColor[3] = 1.0f;

    /* initialize m_context to NULL */
    m_context = NULL;

    /* initialize Camera, display variables */
    m_xRot = 0;
    m_yRot = 0;
    m_zRot = 0;

    m_xPan = 0;
    m_yPan = 0;
    m_zPan = 0;

    m_x = 0;
    m_y = 0;

    m_initialized = false;
    m_simulationRunning = false;
    m_camDistance = 3.0;
    m_bufferObjects = QVector<BufferObjects*>();
    m_perspective = true;

    m_lightParameters.posLight = new GLfloat[30];
    memset(m_lightParameters.posLight, 0, 30 * sizeof(GLfloat));

    m_lightParameters.KaLight = new GLfloat[30];
    memset(m_lightParameters.KaLight, 0, 30 * sizeof(GLfloat));

    m_lightParameters.KdLight = new GLfloat[30];
    memset(m_lightParameters.KdLight, 0, 30 * sizeof(GLfloat));

    m_lightParameters.KsLight = new GLfloat[30];
    memset(m_lightParameters.KsLight, 0, 30 * sizeof(GLfloat));

    m_lightParameters.directional = new GLint[10];
    memset(m_lightParameters.directional, 0, 10 * sizeof(GLfloat));

    m_lightParameters.lightCount = 0;

    m_joint.active = false;
    m_joint.joint = NULL;
    m_joint.counter = 0;
    m_joint.qIndex = -1;
    m_joint.dqIndex = -1;
    m_joint.maxCount = 1000;
    m_joint.delq = 0;
    m_joint.targetVal = 0;
    m_joint.isPositive = true;
    m_joint.jointType = PRIMITIVE;
    m_joint.moveType = INCREMENT;

    m_timer = new QTimer(this);
    m_timer->setTimerType(Qt::PreciseTimer);
    m_timer->setSingleShot(true);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(stopSimulationPlayback()));

    //m_drawVertexNormals = false;
}

void GLWidget::resizeGL(int width, int height) {

}

/*!
@brief handler for mouse move events

@param event a QMouseEvent
*/
void GLWidget::mouseMoveEvent(QMouseEvent* event) {
  int dx = event->x() - m_lastPos.x();
  int dy = event->y() - m_lastPos.y();

  if (event->buttons() & Qt::LeftButton) {
    setXRotation(m_xRot + 8 * dy);
    setYRotation(m_yRot + 8 * dx);
  } else if (event->buttons() & Qt::RightButton) {
    m_xPan += dx/120.0;
    m_yPan -= dy/120.0;
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
void GLWidget::mousePressEvent(QMouseEvent* event) {
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
void GLWidget::mouseReleaseEvent(QMouseEvent* event) {
  emit mouseReleaseEvent();
}

/*!
@brief handler for mouse scroll wheel events

@param event a QMouseEvent
*/
void GLWidget::wheelEvent(QWheelEvent* event) {
  m_camDistance += (event->delta() / 360.0);
  if (m_camDistance < 0) m_camDistance = 0;
  updateGL();
}

/*!
@brief toggles display of vertex normals

*/
// void GLWidget::toggleVertexNormalDisplay() {
//   m_drawVertexNormals = !m_drawVertexNormals;
// }

/*!
@brief get position of cursor

@return cursor position as a string "x,y=0.00,0.00"
*/
QString GLWidget::getCursorPositionText() {
  QString str;
  QTextStream(&str) << "x,y=" << m_x << "," << m_y;
  return str;
}


/*!
@brief initialize lights
*/
void RenderWindow::initializeLights() {

  /* @TODO move lights to uniform buffer blocks
    m_funcs->glGetActiveUniformBlockName();
  */
  qDebug() << "initializing lights";
  qDebug() << "lightCount: " << m_context->activeLightCount;
  if(m_context->activeLightCount > 10)
    m_lightParameters.lightCount = 10;
  else m_lightParameters.lightCount = m_context->activeLightCount;

  for(unsigned int i = 0; i < m_lightParameters.lightCount; i++) {
    QVector4D position;
    s2_light_t* light = m_context->lightPointers[i];
    s2_displacement_t* d = light->parentFramePointer->dAbsolute;
    switch(light->lightType) {
      case POINT: {
        m_lightParameters.directional[i] = 0;
        m_lightParameters.posLight[i * 3 + 0] = d->T.x;
        m_lightParameters.posLight[i * 3 + 1] = d->T.y;
        m_lightParameters.posLight[i * 3 + 2] = d->T.z;
      } break;
      case DIRECTIONAL: {
        m_lightParameters.directional[i] = 1;
        m_lightParameters.posLight[i * 3 + 0] = d->T.x;
        m_lightParameters.posLight[i * 3 + 1] = d->T.x;
        m_lightParameters.posLight[i * 3 + 2] = d->T.x;
      }
      break;
    }

    for(int j = 0; j < 3; j++) {
      m_lightParameters.KaLight[3 * i + j] =
        m_context->lightPointers[i]->glLight.Ka[j];
      m_lightParameters.KdLight[3 * i + j] =
        m_context->lightPointers[i]->glLight.Kd[j];
      m_lightParameters.KsLight[3 * i + j] =
        m_context->lightPointers[i]->glLight.Ks[j];
    }
  }
}


// void RenderWindow::updateLights( QVector<Light*> lightArray )
// {
//     if( lightArray.size()>10 ) lightParam.numLights = 10;
//     else lightParam.numLights = lightArray.size();
//
//     // qDebug()<< lightArray.size();
//
//     for(unsigned int i=0; i < lightParam.numLights; i++)
//     {
//         QVector4D position;
//         switch( lightArray[i]->lightType )
//         {
//             case POINT:
//             {
//                 lightParam.directional[i] = 0;
//                 position = lightArray[i]->transform * QVector4D(0,0,0,1);
//
//             }
//             break;
//             case DIRECTIONAL:
//             {
//                 lightParam.directional[i] = 1;
//                 QVector4D zVector = QVector4D(0.f, 0.f, 1.f, 1.f);
//                 position = lightArray[i]->transform * QVector4D(0,0,0,1);
//                 //position = QMatrix4x4(lightArray[i]->transform.normalMatrix()) * zVector;
//             }
//             break;
//         }
//
//         lightParam.posLight[3*i] = position.x();
//         lightParam.posLight[3*i+1] = position.y();
//         lightParam.posLight[3*i+2] = position.z();
//
//         for( int j=0; j<3; j++)
//         {
//             lightParam.KaLight[3*i+j] = lightArray[i]->Ka[j];
//             lightParam.KdLight[3*i+j] = lightArray[i]->Kd[j];
//             lightParam.KsLight[3*i+j] = lightArray[i]->Ks[j];
//         }
//     }
// }

void GLWidget::setXRotation(int angle) {
  angle = s2_normalizeDegrees(angle);
  if(angle != m_xRot) {
    m_xRot = angle;
    emit xRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::setYRotation(int angle) {
  angle = s2_normalizeDegrees(angle);
  if(angle != m_yRot) {
    m_yRot = angle;
    emit yRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::setZRotation(int angle) {
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
void GLWidget::setPerspective() {
  m_perspective = true;
  updateGL();
}


/*!
@brief set camera projection to orthogonal

*/
void GLWidget::setOrthogonal() {
  m_perspective = false;
  updateGL();
}



/*! @brief set camera projection to perspective

*/
// void GLWidget::setPerspective() {
//     m_perspective = true;
//     updateGL();
// }

/*! @brief set camera projection to orthogonal

*/
// void GLWidget::setOrthogonal() {
//     m_perspective = false;
//     updateGL();
// }

// void RenderWindow::setWireFrame()
// {
//     m_shadingType = WIREFRAME;
//     updateGL();
// }

// void RenderWindow::setFlatShaded()
// {
//     m_shadingType = FLAT;
//     updateGL();
// }
// void RenderWindow::setGouraudShaded()
// {
//     m_shadingType = GOURAUD;
//     updateGL();
// }
// void RenderWindow::setPhongShaded()
// {
//     m_shadingType = PHONG;
//     updateGL();
// }

/* @brief load a Context data structure for rendering

*/
/* @brief load a Context data structure for rendering

*/
void GLWidget::loadContext(
  s2_context_t* context) {
  m_context = context;
  initialize();
}

/* @brief check error in OpenGL

*/
void GLWidget::checkError(
  const QString &prefix) {

  GLenum glErr = glGetError();

  while( glErr != GL_NO_ERROR ) {
    QString error;
    switch (glErr) {
    case GL_NO_ERROR:
      error = "NO_ERROR";
      break;
    case GL_INVALID_OPERATION:
      error = "INVALID_OPERATION";
      break;
    case GL_INVALID_ENUM:
      error = "INVALID_ENUM";
      break;
    case GL_INVALID_VALUE:
      error = "INVALID_VALUE";
      break;
    case GL_OUT_OF_MEMORY:
      error = "OUT_OF_MEMORY";
      break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      error = "INVALID_FRAMEBUFFER_OPERATION";
      break;
    }

    if(error != "NO_ERROR") {
      qDebug() << prefix << ":" << error;
    }
    glErr = glGetError();
  }
}

const GLchar* GLWidget::getShaderSource(
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

  /*
    debugging statements
  */
  // qDebug() << shaderSource;
  // printf("QString\n");
  // qDebug() << fileContents;
  // printf("char array <\n");
  // printf("%s\n", out);
  // printf(">\n");
  return out;
}


void GLWidget::initializeProgramUniforms(
  ProgramUniforms* programUniforms,
  QOpenGLShaderProgram* shaderProgram) {

  shaderProgram->bind();

  programUniforms->directional = shaderProgram->uniformLocation("directional");
  checkError("after set uniform location directional");
  // qDebug() << "directional: " << programUniforms->directional;

  programUniforms->posLight = shaderProgram->uniformLocation("posLight");
  checkError("after set uniform location posLight");
  // qDebug() << "posLight: " << programUniforms->posLight;

  programUniforms->lightCount = shaderProgram->uniformLocation("lightCount");
  checkError("after set uniform location lightCount");
  // qDebug() << "lightCount: " << programUniforms->lightCount;

  programUniforms->Ka = shaderProgram->uniformLocation("Ka");
  checkError("after set uniform location Ka");
  // qDebug() << "Ka: " << programUniforms->Ka;

  programUniforms->Kd = shaderProgram->uniformLocation("Kd");
  checkError("after set uniform location Kd");
  // qDebug() << "Kd: " << programUniforms->Kd;

  programUniforms->Ks = shaderProgram->uniformLocation("Ks");
  checkError("after set uniform location Ks");
  // qDebug() << "Ks: " << programUniforms->Ks;

  // programUniforms->m = shaderProgram->uniformLocation("m");
  // // qDebug() << "m: " << programUniforms->m;
  // checkError("after set uniform location m");

  programUniforms->v = shaderProgram->uniformLocation("v");
  // qDebug() << "v: " << programUniforms->v;
  checkError("after set uniform location v");

  programUniforms->mv = shaderProgram->uniformLocation("mv");
  // qDebug() << "mv: " << programUniforms->mv;
  checkError("after set uniform location mv");

  programUniforms->mvp = shaderProgram->uniformLocation("mvp");
  // qDebug() << "mvp: " << programUniforms->mvp;
  checkError("after set uniform location mvp");

  programUniforms->nm = shaderProgram->uniformLocation("nm");
  // qDebug() << "nm: " << programUniforms->nm;
  checkError("after set uniform location nm");

  programUniforms->KaMaterial = shaderProgram->uniformLocation("KaMaterial");
  // qDebug() << "KaMaterial: " << programUniforms->KaMaterial;
  checkError("after set uniform location KaMaterial");

  programUniforms->KdMaterial = shaderProgram->uniformLocation("KdMaterial");
  // qDebug() << "KdMaterial: " << programUniforms->KdMaterial;
  checkError("after set uniform location KdMaterial");

  programUniforms->KsMaterial = shaderProgram->uniformLocation("KsMaterial");
  // qDebug() << "KsMaterial: " << programUniforms->KsMaterial;
  checkError("after set uniform location KsMaterial");

  programUniforms->Ns = shaderProgram->uniformLocation("Ns");
  // qDebug() << "Ns: " << programUniforms->Ns;
  checkError("after set uniform location Ns");
  shaderProgram->release();
}


/*!
@brief initialize opengl properties for a Body datastructure

*/
void GLWidget::initializeBody(
  s2_body_t* body) {

  for(int i = 0; i < body->materialCount; i++) {
    s2_material_t* material = body->materials + i;
    printf("initializing material [%d]\n", i);
    initializeMaterial(material, m_context->d + body->origin->dIndex);
  }
}



/*!
@brief initialize all shader handles on a given material

*/
void GLWidget::initializeMaterial(
  s2_material_t* material,
  s2_displacement_t* origin) {

  m_programPhong->bind();
  BufferObjects* bufferObjects = new BufferObjects;
  bufferObjects->vao = new QOpenGLVertexArrayObject(this);
  bufferObjects->vao->create();

  bufferObjects->vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects->vertexBuffer->create();

  bufferObjects->vertexNormalBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects->vertexNormalBuffer->create();

  bufferObjects->ibo =
    new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  bufferObjects->ibo->create();

  checkError("before generating vertex arrays");

  bufferObjects->vao->bind();

  bufferObjects->vertexBuffer->bind();
  bufferObjects->vertexBuffer->allocate(
    3 * material->vertexCount * sizeof(GLfloat));
  bufferObjects->vertexBuffer->write(
    0,
    material->vertices,
    3 * material->vertexCount * sizeof(GLfloat));

  bufferObjects->vertexNormalBuffer->bind();
  bufferObjects->vertexNormalBuffer->allocate(
    3 * material->vertexCount * sizeof(GLfloat));
  bufferObjects->vertexNormalBuffer->write(
    0,
    material->vertexNormals,
    3 * material->vertexCount * sizeof(GLfloat));

  bufferObjects->ibo->bind();
  bufferObjects->ibo->allocate(
    material->faces,
    3 * material->faceCount * sizeof(GLuint));

  bufferObjects->material = material;
  bufferObjects->transform = origin;

  setAttributeLocation(
    bufferObjects,
    &m_uniformsLine,
    m_programLine);

  setAttributeLocation(
    bufferObjects,
    &m_uniformsFlat,
    m_programFlat);

  // setAttributeLocation(
  //   bufferObjects,
  //   &m_uniformsGouraud,
  //   m_programGouraud);

  setAttributeLocation(
    bufferObjects,
    &m_uniformsPhong,
    m_programPhong);

  checkError("after set attribute location");
  m_bufferObjects.append(bufferObjects);

}


/*!
@brief initialize attributes for each shader program

*/
void GLWidget::setAttributeLocation(
  BufferObjects* bufferObjects,
  ProgramUniforms* uniforms,
  QOpenGLShaderProgram* shaderProgram) {

  shaderProgram->bind();

  bufferObjects->vao->bind();

  uniforms->position =
    shaderProgram->attributeLocation("posAttr");
  uniforms->normal =
    shaderProgram->attributeLocation("normAttr");

  printf("  uniform position: %d\n", uniforms->position);
  printf("  normal position: %d\n", uniforms->normal);

  shaderProgram->bind();
  bufferObjects->vertexBuffer->bind();

  shaderProgram->enableAttributeArray(
    uniforms->position);
  shaderProgram->setAttributeBuffer(
    uniforms->position,
    GL_FLOAT,
    0,
    3);

  if(uniforms->normal != -1) {
    bufferObjects->vertexNormalBuffer->bind();
    shaderProgram->enableAttributeArray(
      uniforms->normal);
    shaderProgram->setAttributeBuffer(
      uniforms->normal,
      GL_FLOAT,
      0,
      3);
  }

  bufferObjects->vao->release();
  shaderProgram->release();
}


/*!
@brief compile shaders
*/
void GLWidget::compileShaders() {
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
  m_programLine->bind();
  m_uniformsLine.mvp = m_programLine->uniformLocation("mvp");
  m_uniformsLine.colorUniform = m_programLine->uniformLocation("colorUniform");
  m_programLine->release();
  checkError("after release program frame");


  qDebug() << "compiling frame shader program";
  m_programFrame = new QOpenGLShaderProgram(this);
  m_programFrame->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/frame.vert");
  m_programFrame->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/frame.frag");
  m_programFrame->link();
  checkError("after link frame program");

  m_programFrame->bind();
  m_uniformsFrame.mvp = m_programFrame->uniformLocation("mvp");
  m_programFrame->release();
  checkError("after release program frame");


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

  initializeProgramUniforms(
    &m_uniformsFlat,
    m_programFlat);

  qDebug() << "compiling phong";
  m_programPhong = new QOpenGLShaderProgram(this);
  m_programPhong->addShaderFromSourceFile(
    QOpenGLShader::Vertex,
    ":/shaders/phong.vert");
  m_programPhong->addShaderFromSourceFile(
    QOpenGLShader::Fragment,
    ":/shaders/phong.frag");
  m_programPhong->link();

  initializeProgramUniforms(
    &m_uniformsPhong,
    m_programPhong);

  checkError("after compiling shaders");
  qDebug() << "shaders compiled";
}




/*!
@brief initialize frames
*/
void GLWidget::initializeFrames() {

  /* set vertices */
  m_uniformsFrame.vertices = new GLfloat[18];
  m_uniformsFrame.vertices[0] = 0.f;
  m_uniformsFrame.vertices[1] = 0.f;
  m_uniformsFrame.vertices[2] = 0.f;

  m_uniformsFrame.vertices[3] = 0.5f;
  m_uniformsFrame.vertices[4] = 0.f;
  m_uniformsFrame.vertices[5] = 0.f;

  m_uniformsFrame.vertices[6] = 0.f;
  m_uniformsFrame.vertices[7] = 0.f;
  m_uniformsFrame.vertices[8] = 0.f;

  m_uniformsFrame.vertices[9] = 0.f;
  m_uniformsFrame.vertices[10] = 0.5f;
  m_uniformsFrame.vertices[11] = 0.f;

  m_uniformsFrame.vertices[12] = 0.f;
  m_uniformsFrame.vertices[13] = 0.f;
  m_uniformsFrame.vertices[14] = 0.f;

  m_uniformsFrame.vertices[15] = 0.f;
  m_uniformsFrame.vertices[16] = 0.f;
  m_uniformsFrame.vertices[17] = 0.5f;

  /* set colors */
  m_uniformsFrame.colors = new GLfloat[18];
  m_uniformsFrame.colors[0] = 1.f;
  m_uniformsFrame.colors[1] = 0.f;
  m_uniformsFrame.colors[2] = 0.f;

  m_uniformsFrame.colors[3] = 1.f;
  m_uniformsFrame.colors[4] = 0.f;
  m_uniformsFrame.colors[5] = 0.f;

  m_uniformsFrame.colors[6] = 0.f;
  m_uniformsFrame.colors[7] = 1.f;
  m_uniformsFrame.colors[8] = 0.f;

  m_uniformsFrame.colors[9] = 0.f;
  m_uniformsFrame.colors[10] = 1.f;
  m_uniformsFrame.colors[11] = 0.f;

  m_uniformsFrame.colors[12] = 0.f;
  m_uniformsFrame.colors[13] = 0.f;
  m_uniformsFrame.colors[14] = 1.f;

  m_uniformsFrame.colors[15] = 0.f;
  m_uniformsFrame.colors[16] = 0.f;
  m_uniformsFrame.colors[17] = 1.f;

  /* set indices */
  m_uniformsFrame.indices = new GLuint[6];
  m_uniformsFrame.indices[0] = 0;
  m_uniformsFrame.indices[1] = 1;
  m_uniformsFrame.indices[2] = 2;
  m_uniformsFrame.indices[3] = 3;
  m_uniformsFrame.indices[4] = 4;
  m_uniformsFrame.indices[5] = 5;

  m_programFrame->bind();
  checkError("after frame program bind");

  m_uniformsFrame.vao = new QOpenGLVertexArrayObject(this);
  m_uniformsFrame.vao->create();
  m_uniformsFrame.vao->bind();

  m_uniformsFrame.vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_uniformsFrame.vbo->create();

  m_uniformsFrame.vbo->bind();
  m_uniformsFrame.vbo->allocate(2 * 18 * sizeof(GLfloat));
  m_uniformsFrame.vbo->write(
    0, m_uniformsFrame.vertices, 18 * sizeof(GLfloat));
  m_uniformsFrame.vbo->write(
    18 * sizeof(GLfloat), m_uniformsFrame.colors, 18 * sizeof(GLfloat));

  m_uniformsFrame.ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  m_uniformsFrame.ibo->create();
  m_uniformsFrame.ibo->bind();
  m_uniformsFrame.ibo->allocate(m_uniformsFrame.indices, 6 * sizeof(GLuint));

  m_uniformsFrame.position = m_programFrame->attributeLocation("posAttr");
  m_programFrame->enableAttributeArray(m_uniformsFrame.position);
  m_programFrame->setAttributeBuffer(m_uniformsFrame.position, GL_FLOAT, 0, 3);

  m_uniformsFrame.color = m_programFrame->attributeLocation("colorAttr");
  m_programFrame->enableAttributeArray(m_uniformsFrame.color);
  m_programFrame->setAttributeBuffer(
    m_uniformsFrame.color, GL_FLOAT, 18 * sizeof(GLfloat), 3);

  m_programFrame->release();

  checkError("after set up frame shader");
}





/*!
@brief initialize grid
*/
void GLWidget::initializeGrid(
  int gridSize,
  float scaleFactor) {

  m_grid = new Grid();

  /* grid size must be an even number */
  if(gridSize % 2 != 0) {
    gridSize++;
  }
  m_grid->gridSize = gridSize;

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

  /*
    set up line segments
  */
  m_grid->lineBufferObject = new BufferObjects;

  m_grid->lineBufferObject->material = s2_materials(1);
  m_grid->lineBufferObject->transform = s2_displacement();

  m_grid->lineBufferObject->material->shadingType = LINE;
  m_grid->lineBufferObject->material->Ns = 10;

  /* set up vertices and line segments for grid */
  m_grid->lineBufferObject->material->vertexCount = 4 * m_grid->gridSize;
  m_grid->lineBufferObject->material->faceCount = 2 * (m_grid->gridSize + 1);

  m_grid->lineBufferObject->material->vertices =
    new GLfloat[3 * m_grid->lineBufferObject->material->vertexCount];
  m_grid->lineBufferObject->material->vertexNormals =
    new GLfloat[3 * m_grid->lineBufferObject->material->vertexCount];
  m_grid->lineBufferObject->material->faces =
    new GLuint[2 * m_grid->lineBufferObject->material->faceCount];

  /* set vertices for vertical line segments */
  int gridIndex = 0;
  for(int i = 0; i <= m_grid->gridSize; i++) {
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      (float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;

    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      -(float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;
  }

  // for(int i = 0; i < gridIndex; i++) {
  //   printf("%f, %f, %f\n",
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 0],
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 1],
  //     m_grid->lineBufferObject->material->vertices[i * 3 + 2]);
  // }

  // qDebug() << "gridIndex: " << gridIndex;

  /* set vertices for remaining horizontal line segments */
  for(int i = 1; i < m_grid->gridSize; i++) {
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      -(float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;

    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 0] =
      (float)(m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 1] =
      (float)(i - m_grid->gridSize / 2);
    m_grid->lineBufferObject->material->vertices[gridIndex * 3 + 2] = 1e-6;
    m_grid->lineBufferObject->material->faces[gridIndex] = gridIndex;
    gridIndex++;
  }

  /* set top horizontal line segment */
  m_grid->lineBufferObject->material->faces[gridIndex++] = 0;
  m_grid->lineBufferObject->material->faces[gridIndex++] =
    m_grid->gridSize * 2;

  /* set bottom horizontal line segment */
  m_grid->lineBufferObject->material->faces[gridIndex++] = 1;
  m_grid->lineBufferObject->material->faces[gridIndex++] =
    m_grid->gridSize * 2 + 1;

  // qDebug() << "gridIndex final " << gridIndex;
  // qDebug() << "face count" << m_grid->lineBufferObject->material->faceCount;


  m_programLine->bind();
  // checkError("after line program bind");

  /* initialize vao and vbo */
  m_grid->lineBufferObject->vao = new QOpenGLVertexArrayObject(this);
  m_grid->lineBufferObject->vao->create();
  m_grid->lineBufferObject->vao->bind();

  // checkError("after initialize vao");

  // checkError("after set attribute posAttr in line program");

  m_grid->lineBufferObject->vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_grid->lineBufferObject->vertexBuffer->create();

  m_grid->lineBufferObject->vertexBuffer->bind();
  m_grid->lineBufferObject->vertexBuffer->allocate(
    m_grid->lineBufferObject->material->vertices,
    3 * m_grid->lineBufferObject->material->vertexCount * sizeof(GLfloat));
  // checkError("after initialize vbo");
  // m_grid->lineBufferObject->vertexNormalBuffer =
  //   new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  // m_grid->lineBufferObject->vertexNormalBuffer->create();
  //
  // m_grid->lineBufferObject->vertexNormalBuffer->allocate(
  //   m_grid->lineBufferObject->material->vertexNormals,
  //   m_grid->lineBufferObject->material->vertexCount * sizeof(GLfloat));

  m_grid->lineBufferObject->ibo =
    new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  m_grid->lineBufferObject->ibo->create();
  m_grid->lineBufferObject->ibo->bind();

  // checkError("after create ibo in line program");

  // for(int i = 0; i < m_grid->lineBufferObject->material->faceCount * 2; i++) {
  //   printf("[%d]\n", m_grid->lineBufferObject->material->faces[i]);
  // }

  m_grid->lineBufferObject->ibo->allocate(
    m_grid->lineBufferObject->material->faces,
    2 * m_grid->lineBufferObject->material->faceCount * sizeof(GLuint));
  checkError("after set ibo in line program");

  m_uniformsLine.position =
    m_programLine->attributeLocation("posAttr");
  m_programLine->enableAttributeArray(m_uniformsLine.position);
  m_programLine->setAttributeBuffer(
    m_uniformsLine.position,
    GL_FLOAT,
    0,
    3);

  m_programLine->release();

  m_bufferObjects.append(m_grid->lineBufferObject);
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

  m_joint.active = true;
  m_joint.counter = 0;
  m_joint.joint = joint;
  m_joint.qIndex = qIndex;
  m_joint.dqIndex = dqIndex;
  m_joint.delq = delq;
  m_joint.targetVal = -1;
  m_joint.jointType = jointType;
  m_joint.moveType = INCREMENT;
  m_joint.isPositive = true;
  m_joint.maxCount = 1000;
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

  m_joint.active = true;
  m_joint.counter = 0;
  m_joint.joint = joint;
  m_joint.qIndex = qIndex;
  m_joint.dqIndex = dqIndex;
  m_joint.delq = 0.01;
  m_joint.targetVal = targetVal;
  m_joint.jointType = jointType;
  m_joint.moveType = TARGET;

  if(targetVal - m_context->q[qIndex] > 0) {
    m_joint.isPositive = true;
  } else {
    m_joint.isPositive = false;
  }

  /* allow up to 1000 steps for each radian or unit of translation */
  m_joint.maxCount = abs(targetVal - m_context->q[qIndex]) * 1000;
}


/*!
  @brief clear joint to be incremented
*/
void RenderWindow::clearJoint() {
  m_joint.active = false;
  m_joint.counter = 0;
  // m_joint.joint = nullptr;
  m_joint.qIndex = -1;
  m_joint.dqIndex = -1;
  m_joint.delq = -1;
  m_joint.jointType = INVALID;
}


/*!
@brief move joint incrementally if user specifies a joint is to be moved
using joint slider interface.

This function should not be called while simulation is running, or while
simulation is being played back.
*/
void RenderWindow::updateJoint() {

  if(m_joint.active) {
    m_joint.counter++;

    /* determine if joint is in a substructure, if it is not constrained
    then no need to do Newton-Raphson iteration */
    if(m_joint.joint->substructurePointer != NULL) {
      if(m_joint.jointType == SPHERICAL) {
      /*! @TODO convert euler angle sequence to quaternion and update
          4 entries in q array */
      } else {
        /* specify variable as independent and update the value in q array.
        Perform Newton-Raphson iteration with current variable as a
        specifid coordinate */
        switch(m_joint.moveType) {
          case TARGET: {
            if(m_joint.counter > m_joint.maxCount) {
              /* if the number of steps allotted for the move has been
              exceeded, stop incrementing the joint */

              clearJoint();
            } else {

              double delq =
                m_joint.delq * pow(m_joint.joint->mobilityNumber[0], 2);

              if(m_joint.isPositive) {
                if(
                  m_context->q[m_joint.qIndex] + delq >
                  m_joint.targetVal) {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint.joint->substructurePointer->substructureIndex,
                    m_joint.dqIndex,
                    m_joint.targetVal,
                    PRIMITIVE,
                    TARGET);
                  // qDebug() << "before clearjoint";

                  clearJoint();
                } else {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint.joint->substructurePointer->substructureIndex,
                    m_joint.dqIndex,
                    delq,
                    PRIMITIVE,
                    INCREMENT);
                }
              } else {
                if(
                  m_context->q[m_joint.qIndex] + delq < m_joint.targetVal) {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint.joint->substructurePointer->substructureIndex,
                    m_joint.dqIndex,
                    m_joint.targetVal,
                    PRIMITIVE,
                    TARGET);

                  // qDebug() << "before clearjoint";
                  clearJoint();
                } else {

                  s2_newtonRaphson_increment_specified(
                    m_context,
                    m_joint.joint->substructurePointer->substructureIndex,
                    m_joint.dqIndex,
                    -delq,
                    PRIMITIVE,
                    INCREMENT);
                }
              }
            }
          } break;
          case INCREMENT: {

            int returnVal = s2_newtonRaphson_increment_specified(
              m_context,
              m_joint.joint->substructurePointer->substructureIndex,
              m_joint.dqIndex,
              m_joint.delq * pow(m_joint.joint->mobilityNumber[0], 2),
              TX,
              INCREMENT);
            if(returnVal){
              clearJoint();
            }
          } break;
        }

        s2_mobility_compute_substructure(
          m_joint.joint->substructurePointer);

        s2_mobility_computeGlobalMobility(m_context);
      }

    } else {
      /* update a joint that is not in a substructure */

      if(m_joint.jointType == SPHERICAL) {

        /*! @TODO convert euler angle sequence to quaternion and update
         4 entries in q array */

      } else {

        /* otherwise, update a single value in q array */
        m_context->q[m_joint.qIndex] += m_joint.delq * 0.5;
      }
      s2_updateDH(m_context);
      s2_updateGLReferenceFrames(m_context);
    }
    emit jointChanged();
  }
}


/*!
@brief used to start playback of a simulation after computation has finished
*/
void RenderWindow::startSimulationPlayback() {
  m_timer->start(int(1000 * m_context->modelParameters->timeSpan));
  setSimulationRunning(true);
  qDebug() << "starting simulation playback for t=" <<  m_context->modelParameters->timeSpan;

}


/*!
@brief used to start playback of a simulation after computation has finished
*/
void RenderWindow::startSimulationPlayback(double timeRemaining) {
  m_timer->start(int(1000 * timeRemaining));
  setSimulationRunning(true);
  qDebug() << "starting simulation playback";

}


/*!
@brief used to stop playback of a simulation after computation has finished
*/
void RenderWindow::stopSimulationPlayback() {
  setSimulationRunning(false);
  qDebug() << "stopping simulation playback";
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


    /* find current time, and time at knot point before and
    after time */
    // qDebug() << "remaining time: " << m_timer->remainingTime();
    double t =
      fmax(0, m_context->modelParameters->timeSpan -
      (double)m_timer->remainingTime() / 1000.0);

    double t0 = t - fmod(t, m_context->modelParameters->timeStep);
    double t1 = t0 + m_context->modelParameters->timeStep;

    /* find index of first knot point */
    int i0 = floor(t0 / m_context->modelParameters->timeStep);
    int i1 = fmin(i0 + 1, m_context->numericalIntegrator->stepCount);

    double p = (t - t0) / (t1 - t0);

    qDebug() << "interpolating, t=" << t << ", t0=" << t0 << "t1=" << t1 << "i0" << i0 << "i1:" << i1 << "p:" << p;

    /*
      yOut = y0 + (y1 - y0) / (t1 - t0) * (t - t0)

    */
    s2_array_lerp(
      m_context->numericalIntegrator->neqn,
      p,
      m_context->numericalIntegrator->ydata +
        i0 * m_context->numericalIntegrator->neqn,
      m_context->numericalIntegrator->ydata +
        i1 * m_context->numericalIntegrator->neqn,
      m_context->q);

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
      updateJoint();
    }

    /* draw coordinate systems */
    m_programFrame->bind();
    m_uniformsFrame.vao->bind();
    checkError("after bind vao coordinate systems");
    for(int i = 0; i < m_frames.size(); i++) {
      double* d = (double*)(m_frames.at(i));
      QMatrix4x4 model =
        QMatrix4x4((float)d[0], (float)d[3], (float)d[6], (float)d[9],
                   (float)d[1], (float)d[4], (float)d[7], (float)d[10],
                   (float)d[2], (float)d[5], (float)d[8], (float)d[11],
                   0.f,  0.f,  0.f,  1.f);
      QMatrix4x4 mvp = m_projection * m_view * model;

      /* set model * view * projection matrix */
      m_programFrame->setUniformValue(
        m_uniformsFrame.mvp,
        mvp);
      checkError("after set mvp coordinate systems");
      glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
      checkError("after draw elements coordinate systems");
    }
    m_uniformsFrame.vao->release();
    m_programFrame->release();

    /* draw geometry */
    for(int i = 0; i < m_bufferObjects.size(); i++ ) {
      switch(m_bufferObjects.at(i)->material->shadingType) {
        case PHONG: {
          m_programPhong->bind();
          m_bufferObjects.at(i)->vao->bind();
          double* d = (double*)(m_bufferObjects.at(i)->transform);
          QMatrix4x4 model = QMatrix4x4(d[0], d[3], d[6], d[9],
                                        d[1], d[4], d[7], d[10],
                                        d[2], d[5], d[8], d[11],
                                        0.f,  0.f,  0.f,  1.f);

          s2_material_t* material = m_bufferObjects.at(i)->material;
          QMatrix4x4 mv = m_view * model;

          /* set model * view * projection matrix */
          m_programPhong->setUniformValue(
            m_uniformsPhong.mvp,
            m_projection * mv);
          checkError("after set mvp phong");

          /* set model * view matrix */
          m_programPhong->setUniformValue(
            m_uniformsPhong.mv, mv);
          checkError("after set mv phong");

          /* set view matrix */
          m_programPhong->setUniformValue(
            m_uniformsPhong.v,
            m_view);
          checkError("after set v phong");

          /* set normal matrix */
          m_programPhong->setUniformValue(
            m_uniformsPhong.nm,
            mv.normalMatrix());
          checkError("after set nm phong");

          /* set Ka for lights */
          m_programPhong->setUniformValueArray(
            m_uniformsPhong.Ka,
            m_lightParameters.KaLight,
            10,
            3);

          /* set Ks for lights */
          m_programPhong->setUniformValueArray(
            m_uniformsPhong.Ks,
            m_lightParameters.KsLight,
            10,
            3);
          checkError("after set KsLight phong");

          /* set Kd for lights */
          m_programPhong->setUniformValueArray(
            m_uniformsPhong.Kd,
            m_lightParameters.KdLight,
            10,
            3);
          checkError("after set KdLight phong");

          /* set directional switch for lights */
          m_programPhong->setUniformValueArray(
            m_uniformsPhong.directional,
            m_lightParameters.directional,
            10);
          checkError("after set directional phong");

          /* set positions of lights */
          m_programPhong->setUniformValueArray(
            m_uniformsPhong.posLight,
            m_lightParameters.posLight,
            10,
            3);
          // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
          checkError("after set poslight phong");

          /* set number of lights */
          m_programPhong->setUniformValue(
            m_uniformsPhong.lightCount,
            m_lightParameters.lightCount);
          // qDebug() << "light count:" << lightParameters.lightCount;
          checkError("after set light count phong");

          /* set Kd properties for material */
          m_programPhong->setUniformValue(
            m_uniformsPhong.KdMaterial,
            material->Kd[0],
            material->Kd[1],
            material->Kd[2]);
          checkError("after set KdMaterial phong");

          /* set Ks properties for material */
          m_programPhong->setUniformValue(
            m_uniformsPhong.KsMaterial,
            material->Ks[0],
            material->Ks[1],
            material->Ks[2]);
          checkError("after set KsMaterial phong");

          /* set Ka properties for material */
          m_programPhong->setUniformValue(
            m_uniformsPhong.KaMaterial,
            material->Ka[0],
            material->Ka[1],
            material->Ka[2]);
          checkError("after set KaMaterial phong");

          // qDebug() << "KaMaterial: " << material->Ka[0]
          // << material->Ka[1] << material->Ka[2];

          /* set specularity property for material */
          m_programPhong->setUniformValue(
            m_uniformsPhong.Ns,
            material->Ns);
          checkError("after set Ns phong");

          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          checkError("after set polygon mode phong");

          glDrawElements(
            GL_TRIANGLES,
            3 * material->faceCount,
            GL_UNSIGNED_INT,
            0);
          checkError("after draw elements phong");

          m_bufferObjects.at(i)->vao->release();
          checkError("after vao release phong");

          m_programPhong->release();
          checkError("after phong release phong");

        } break;
        case FLAT: {
          m_programFlat->bind();
          m_bufferObjects.at(i)->vao->bind();
          double* d = (double*)(m_bufferObjects.at(i)->transform);
          QMatrix4x4 model = QMatrix4x4(d[0], d[3], d[6], d[9],
                                        d[1], d[4], d[7], d[10],
                                        d[2], d[5], d[8], d[11],
                                        0.f,  0.f,  0.f,  1.f);

          s2_material_t* material = m_bufferObjects.at(i)->material;
          QMatrix4x4 mv = m_view * model;

          /* set model * view * projection matrix */
          m_programFlat->setUniformValue(
            m_uniformsFlat.mvp,
            m_projection * mv);
          checkError("after set mvp");

          /* set model * view matrix */
          m_programFlat->setUniformValue(
            m_uniformsFlat.mv, mv);
          checkError("after set mv");

          /* set view matrix */
          m_programFlat->setUniformValue(
            m_uniformsFlat.v,
            m_view);
          checkError("after set v");

          /* set normal matrix */
          m_programFlat->setUniformValue(
            m_uniformsFlat.nm,
            mv.normalMatrix());
          checkError("after set nm");

          /* set Ka for lights */
          m_programFlat->setUniformValueArray(
            m_uniformsFlat.Ka,
            m_lightParameters.KaLight,
            10,
            3);

          /* set Ks for lights */
          m_programFlat->setUniformValueArray(
            m_uniformsFlat.Ks,
            m_lightParameters.KsLight,
            10,
            3);
          checkError("after set KsLight");

          /* set Kd for lights */
          m_programFlat->setUniformValueArray(
            m_uniformsFlat.Kd,
            m_lightParameters.KdLight,
            10,
            3);
          checkError("after set KdLight");

          /* set directional switch for lights */
          m_programFlat->setUniformValueArray(
            m_uniformsFlat.directional,
            m_lightParameters.directional,
            10);
          checkError("after set directional");

          /* set positions of lights */
          m_programFlat->setUniformValueArray(
            m_uniformsFlat.posLight,
            m_lightParameters.posLight,
            10,
            3);
          // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
          checkError("after set poslight");

          /* set number of lights */
          m_programFlat->setUniformValue(
            m_uniformsFlat.lightCount,
            m_lightParameters.lightCount);
          // qDebug() << "light count:" << lightParameters.lightCount;
          checkError("after set light count");

          /* set Kd properties for material */
          m_programFlat->setUniformValue(
            m_uniformsFlat.KdMaterial,
            material->Kd[0],
            material->Kd[1],
            material->Kd[2],
            material->Kd[3]);
          checkError("after set KdMaterial");

          /* set Ks properties for material */
          m_programFlat->setUniformValue(
            m_uniformsFlat.KsMaterial,
            material->Ks[0],
            material->Ks[1],
            material->Ks[2],
            material->Ks[3]);
          checkError("after set KsMaterial");

          /* set Ka properties for material */
          m_programFlat->setUniformValue(
            m_uniformsFlat.KaMaterial,
            material->Ka[0],
            material->Ka[1],
            material->Ka[2],
            material->Ka[3]);
          checkError("after set KaMaterial");

          // qDebug() << "KaMaterial: " << material->Ka[0]
          // << material->Ka[1] << material->Ka[2];

          /* set specularity property for material */
          m_programFlat->setUniformValue(
            m_uniformsFlat.Ns,
            material->Ns);
          checkError("after set Ns");

          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          checkError("after set polygon mode");

          glDrawElements(
            GL_TRIANGLES,
            3 * material->faceCount,
            GL_UNSIGNED_INT,
            0);
          checkError("after draw elements");

          m_bufferObjects.at(i)->vao->release();
          checkError("after vao release");

          m_programFlat->release();
          checkError("after flat release");
        } break;
        case GOURAUD: {
          m_programGouraud->bind();

          m_programGouraud->release();
        } break;
        case WIREFRAME: {
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
          checkError("after set mvp");

          /* set model * view matrix */
          m_programLine->setUniformValue(
            m_uniformsLine.mv, mv);
          checkError("after set mv");

          /* set view matrix */
          m_programLine->setUniformValue(
            m_uniformsLine.v,
            m_view);
          checkError("after set v");

          /* set normal matrix */
          m_programLine->setUniformValue(
            m_uniformsLine.nm,
            mv.normalMatrix());
          checkError("after set nm");

          /* set Ka for lights */
          m_programLine->setUniformValueArray(
            m_uniformsLine.Ka,
            m_lightParameters.KaLight,
            10,
            3);

          /* set Ks for lights */
          m_programFlat->setUniformValueArray(
            m_uniformsLine.Ks,
            m_lightParameters.KsLight,
            10,
            3);
          checkError("after set KsLight");

          /* set Kd for lights */
          m_programLine->setUniformValueArray(
            m_uniformsLine.Kd,
            m_lightParameters.KdLight,
            10,
            3);
          checkError("after set KdLight");

          /* set directional switch for lights */
          m_programLine->setUniformValueArray(
            m_uniformsLine.directional,
            m_lightParameters.directional,
            10);
          checkError("after set directional");

          /* set positions of lights */
          m_programLine->setUniformValueArray(
            m_uniformsLine.posLight,
            m_lightParameters.posLight,
            10,
            3);
          // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
          checkError("after set poslight");

          /* set number of lights */
          m_programLine->setUniformValue(
            m_uniformsLine.lightCount,
            m_lightParameters.lightCount);
          // qDebug() << "light count:" << lightParameters.lightCount;
          checkError("after set light count");

          /* set Kd properties for material */
          m_programLine->setUniformValue(
            m_uniformsLine.KdMaterial,
            material->Kd[0],
            material->Kd[1],
            material->Kd[2],
            material->Kd[3]);
          checkError("after set KdMaterial");

          /* set Ks properties for material */
          m_programLine->setUniformValue(
            m_uniformsLine.KsMaterial,
            material->Ks[0],
            material->Ks[1],
            material->Ks[2],
            material->Ks[3]);
          checkError("after set KsMaterial");

          /* set Ka properties for material */
          m_programLine->setUniformValue(
            m_uniformsLine.KaMaterial,
            material->Ka[0],
            material->Ka[1],
            material->Ka[2],
            material->Ka[3]);
          checkError("after set KaMaterial");

          // qDebug() << "KaMaterial: " << material->Ka[0]
          // << material->Ka[1] << material->Ka[2];

          /* set specularity property for material */
          m_programLine->setUniformValue(
            m_uniformsLine.Ns,
            material->Ns);
          checkError("after set Ns");

          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          checkError("after set polygon mode");

          glDrawElements(
            GL_TRIANGLES,
            3 * material->faceCount,
            GL_UNSIGNED_INT,
            0);
          checkError("after draw elements");

          m_bufferObjects.at(i)->vao->release();
          checkError("after vao release");

          m_programLine->release();
          checkError("after flat release");

        } break;
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
    compileShaders();
    initializeFrames();

    /* push an identity matrix onto the vector of coordinate systems to
    represent the origin */
    m_frames.append(s2_displacement());

    /* set up grid for floor plane */
    initializeGrid(100, 1.f);

    m_initialized = true;
  }

  /*
    initialize uniforms on each body / material, initialize buffers and vertex
    array objects
  */
  if(m_context) {
    qDebug() << "initializing context";

    m_context->viewport = s2_camera();
    for(int i = 0; i < m_context->activeBodyCount; i++) {
      s2_body_t* body = m_context->nonVirtualBodyPointers[i];
      printf("initializing body: ");
      s2_stackstr_print(body->name);
      initializeBody(body);
    }
    initializeLights();

    // printf("active light count : %d\n", context->activeLightCount);
    updateGL();
  }
}



/*! @brief Initialize a star context datastructure

*/
void GLWidget::initialize() {

  /*
    initialize uniforms on each body / material, initialize buffers and vertex
    array objects
  */

  if(m_context) {
    while(!m_initialized) {
      sleep(10);
    }
    qDebug() << "INITIALIZING";
    // s2_glInitialize(m_context);
    m_context->programPhong = m_programPhong;
    m_context->programLine = m_programLine;
    m_context->programGouraud = m_programGouraud;
    m_context->programFlat = m_programFlat;
    updateGL();
  }
}

/*! @brief updateGL function needs to be called on mouse and screen resize events

*/
void GLWidget::updateGL() {

  m_view.setToIdentity();

  QVector3D eye(-m_xPan, -m_yPan, -m_zPan + m_camDistance);
  QVector3D center(-m_xPan, -m_yPan, -m_zPan);
  QVector3D up(0, 1, 0);

  m_view.lookAt(eye, center, up);
  m_view.rotate(m_xRot / 16.0, 1.0, 0.0, 0.0);
  m_view.rotate(m_yRot / 16.0, 0.0, 1.0, 0.0);

  m_projection.setToIdentity();
  const qreal retinaScale = devicePixelRatio();
  if(m_initialized) {
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
  }
  float aspectRatio = (float)width() / (float)height();
  if (m_perspective) {
    m_projection.perspective( 60, (float)width() / (float)height(), 0.01, 10000);
  } else {
    float h = 0.5 * m_camDistance;
    float w = aspectRatio * h;
    m_projection.ortho(-w, w, -h, h, 0.01, 10000);
  }
  if(m_context) {
    cblas_scopy(16, m_view.data(), 1, m_context->viewport->view, 1);
    cblas_scopy(16, m_projection.data(), 1, m_context->viewport->projection, 1);
    // s2_PrintMat4f(m_context->viewport->projection);
    // s2_PrintMat4f(m_context->viewport->view);
  }
}

void GLWidget::paintGL() {

  if(isAnimating()) {
      ++m_frame;
  }
}
