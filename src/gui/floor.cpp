#include "floor.hpp"
#include "floorshaderprogram.hpp"
#include "floorshadowshaderprogram.hpp"

void Shift::Floor::initialize() {
  if(!m_initialized) {
    QImage* checkerBoard = new QImage(2, 2, QImage::Format_RGBA8888);

    QRgb value;
    value = qRgb(191, 191, 192);
    checkerBoard->setPixel(0, 1, value);
    checkerBoard->setPixel(1, 0, value);

    value = qRgb(249, 249, 249);
    checkerBoard->setPixel(0, 0, value);
    checkerBoard->setPixel(1, 1, value);

    m_texture_diffuse = new QOpenGLTexture(*checkerBoard);
    checkError("Floor::initialize(): after new diffuse texture");

    m_texture_diffuse->setWrapMode(QOpenGLTexture::Repeat);
    checkError("Floor::initialize(): after set wrap mode");

    m_texture_diffuse->setMinMagFilters(QOpenGLTexture::Nearest,
                                        QOpenGLTexture::Nearest);
    checkError("Floor::initialize(): after set min mag filters");
    m_initialized = true;
    enable(true);
  }
}

void Shift::Floor::draw() {
  if(m_initialized && m_drawEnabled) {
    m_program->bind();
    m_program->bindVao();

    m_program->bindDiffuseTexture(m_texture_diffuse);

    QMatrix4x4 model;
    model.scale(m_scaleFactor);

    m_program->setTransform(model);
    checkError("Floor::draw(): after set transform");

    m_program->setScale(m_scaleFactor);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    checkError("Floor::draw(): after draw arrays");

    m_texture_diffuse->release();

    m_program->releaseVao();
    m_program->release();
  }
}

void Shift::Floor::drawShadows() {}
