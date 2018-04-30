#include "floor.h"
#include "floorshaderprogram.h"
#include "floorshadowshaderprogram.h"
#include "grid.h"
#include "lights.h"

void Shift::Floor::initialize() {
  if(!m_initialized) {
    QImage checkerBoard(2, 2, QImage::Format_RGBA8888);

    QRgb value;
    value = qRgb(191, 191, 192);
    checkerBoard.setPixel(0, 1, value);
    checkerBoard.setPixel(1, 0, value);

    value = qRgb(249, 249, 249);
    checkerBoard.setPixel(0, 0, value);
    checkerBoard.setPixel(1, 1, value);

    m_texture_diffuse =
      new QOpenGLTexture(checkerBoard, QOpenGLTexture::DontGenerateMipMaps);
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
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glDisable(GL_CULL_FACE);

    m_program->bind();
    checkError("Floor::draw(): after program bind");

    m_program->bindVao();
    checkError("Floor::draw(): after bind vao");

    m_program->bindDiffuseTexture(m_texture_diffuse);

    QMatrix4x4 model;
    model.scale(m_scaleFactor);

    m_program->setTransform(model);
    checkError("Floor::draw(): after set transform");

    m_program->setScale(m_scaleFactor);

    m_program->setLights(m_lights);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    checkError("Floor::draw(): after draw arrays");

    m_texture_diffuse->release();
    checkError("Floor::draw(): after release texture diffuse");

    m_program->release();
  }
}

void Shift::Floor::drawShadows() {
  if(m_initialized && m_drawEnabled) {
    glDisable(GL_CULL_FACE);

    m_programShadow->bind();
    checkError("Floor::drawShadows(): after program bind");

    m_programShadow->bindVao();
    checkError("Floor::drawShadows(): after program bind vao");

    QMatrix4x4 model;
    model.scale(m_scaleFactor);

    m_programShadow->setTransform(model);
    checkError("Floor::drawShadows(): after set transform");

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    checkError("Floor::draw(): after draw");

    m_programShadow->release();
    checkError("Floor::drawShadows(): after program release");
  }
}
