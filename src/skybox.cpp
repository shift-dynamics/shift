#include "skybox.h"
#include "skyboxshaderprogram.h"

#include <QHBoxLayout>
#include <QOpenGLBuffer>
#include <QOpenGLDebugLogger>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector>
#include <QtOpenGL/QGLWidget>

void Shift::Skybox::initialize() {
  m_vao = new QOpenGLVertexArrayObject(this);
  m_vao->create();

  m_program->bind();

  /* load textures */
  m_cubeTexture = new QOpenGLTexture(QOpenGLTexture::TargetCubeMap);
  m_cubeTexture->setWrapMode(QOpenGLTexture::ClampToEdge);
  m_cubeTexture->setMinMagFilters(QOpenGLTexture::Linear,
                                  QOpenGLTexture::Linear);
  checkError("after new cube texture");

  QImage up = QImage(":/textures/top.png")
                .convertToFormat(QImage::Format_RGBA8888)
                .mirrored(true, true);
  QImage dn =
    QImage(":/textures/bottom.png").convertToFormat(QImage::Format_RGBA8888);
  QImage lf = QImage(":/textures/right.png")
                .convertToFormat(QImage::Format_RGBA8888)
                .mirrored(false, false);
  QImage rt = QImage(":/textures/left.png")
                .convertToFormat(QImage::Format_RGBA8888)
                .mirrored(false, false);
  QImage ft = QImage(":/textures/front.png")
                .convertToFormat(QImage::Format_RGBA8888)
                .mirrored(false, false);
  QImage bk = QImage(":/textures/back.png")
                .convertToFormat(QImage::Format_RGBA8888)
                .mirrored(false, false);

  qDebug() << "width:" << lf.width() << " height: " << lf.height();

  m_cubeTexture->setFormat(QOpenGLTexture::RGB32F);
  m_cubeTexture->setSize(lf.width(), lf.height());
  m_cubeTexture->allocateStorage();
  checkError("after allocate storage");

  m_cubeTexture->setData(0, 0, QOpenGLTexture::CubeMapNegativeX,
                         QOpenGLTexture::RGBA, QOpenGLTexture::UInt8,
                         lf.constBits());
  m_cubeTexture->setData(0, 0, QOpenGLTexture::CubeMapPositiveX,
                         QOpenGLTexture::RGBA, QOpenGLTexture::UInt8,
                         rt.constBits());
  m_cubeTexture->setData(0, 0, QOpenGLTexture::CubeMapNegativeY,
                         QOpenGLTexture::RGBA, QOpenGLTexture::UInt8,
                         dn.constBits());
  m_cubeTexture->setData(0, 0, QOpenGLTexture::CubeMapPositiveY,
                         QOpenGLTexture::RGBA, QOpenGLTexture::UInt8,
                         up.constBits());
  m_cubeTexture->setData(0, 0, QOpenGLTexture::CubeMapNegativeZ,
                         QOpenGLTexture::RGBA, QOpenGLTexture::UInt8,
                         ft.constBits());
  m_cubeTexture->setData(0, 0, QOpenGLTexture::CubeMapPositiveZ,
                         QOpenGLTexture::RGBA, QOpenGLTexture::UInt8,
                         bk.constBits());

  QOpenGLShaderProgram* shaderProgram = m_program->getProgram();
  shaderProgram->setUniformValue("m_texture_cube_u", 1);
  m_cubeTexture->bind();
  m_cubeTexture->release();

  m_vao->release();
  m_program->release();

  m_initialized = true;
}

void Shift::Skybox::draw() {
  if(m_initialized && m_drawEnabled) {
    m_program->bind();
    m_vao->bind();
    QOpenGLShaderProgram* shaderProgram = m_program->getProgram();
    shaderProgram->setUniformValue("m_texture_cube_u", 1);
    m_cubeTexture->bind();
    checkError("skybox: after bind texture");
    m_program->setTransform();
    checkError("skybox: after set transform");
    glDisable(GL_DEPTH_TEST);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    checkError("skybox: after draw arrays");
    glEnable(GL_DEPTH_TEST);
    m_cubeTexture->release();
    m_vao->release();
    m_program->release();
  }
}
