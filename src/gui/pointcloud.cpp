#include "bodies.hpp"
#include "renderwindow.hpp"
#include "pointcloudshaderprogram.hpp"
#include "shadowshaderprogram.hpp"
#include "lights.hpp"
#include "pointcloud.hpp"
#include "../star2.h"

/*!
@brief sets draw style, WIREFRAME, PHONG, and FLAT are the only
*/
void s2_PointCloud::setDrawStyle(s2_point_shading_enum_t shadingType) {
  switch(shadingType) {
    case S2_POINT_SHADING_SPRITE: {
      m_shadingType = S2_POINT_SHADING_SPRITE;
    } break;
    case S2_POINT_SHADING_PHONG: {
      m_shadingType = S2_POINT_SHADING_PHONG;
    }
    default: {
      m_shadingType = S2_POINT_SHADING_SIMPLE;
    } break;
  }
}


/*!
@brief sets pointer to lights datastructure
*/
void s2_PointCloud::setLights(s2_Lights* lights) {
  m_lights = lights;
}


/*!
@brief set up program uniforms for s2_PointCloud
*/
void s2_PointCloud::initialize() {
  m_program = new PointCloudShaderProgram(m_renderWindow);
}


/*!
@brief load context data structure
*/
void s2_PointCloud::loadContext() {
  if(m_renderWindow->getContext()) {
    m_context = m_renderWindow->getContext();
    for(int i = 0; i < m_context->activePointCloudCount; i++) {
      s2_pointcloud_t* pointcloud = m_context->pointCloudPointers[i];
      initializePointCloud(pointcloud);
    }
    enable(true);
  }
}


/*!
@brief initialize opengl properties for a s2_material_t datastructure

*/
void s2_PointCloud::initializePointCloud(
  s2_pointcloud_t* pointcloud) {

  // m_programPhong->bind();
  pointcloud_buffer_t bufferObjects;
  bufferObjects.vao = new QOpenGLVertexArrayObject(m_renderWindow);
  bufferObjects.vao->create();

  bufferObjects.vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects.vertexBuffer->create();

  checkError("bodies: after create vertex buffers");

  bufferObjects.vao->bind();
  checkError("bodies: after bind vao");


  bufferObjects.vertexBuffer->bind();
  bufferObjects.vertexBuffer->allocate(
    6 * pointcloud->vertexCount * sizeof(GLfloat));
  bufferObjects.vertexBuffer->write(
    0,
    pointcloud->vertices,
    3 * pointcloud->vertexCount * sizeof(GLfloat));

  bufferObjects.colorBuffer->bind();
  bufferObjects.colorBuffer->allocate(
    3 * pointcloud->vertexCount * sizeof(GLfloat));
  bufferObjects.colorBuffer->write(
    0,
    pointcloud->colors,
    3 * pointcloud->vertexCount * sizeof(GLfloat));

  bufferObjects.pointcloud = pointcloud;
  bufferObjects.transform = pointcloud->frame->dAbsolute;

  setAttributeLocations(
    m_program->getShaderProgram(S2_POINT_SHADING_PHONG)->getProgram(),
    m_program->getPositionAttribute(S2_POINT_SHADING_PHONG),
    m_program->getColorAttribute(S2_POINT_SHADING_PHONG),
    &bufferObjects);
  checkError("bodies: after set attribute locations phong");

  setAttributeLocations(
    m_program->getShaderProgram(S2_POINT_SHADING_SPRITE)->getProgram(),
    m_program->getPositionAttribute(S2_POINT_SHADING_SPRITE),
    m_program->getColorAttribute(S2_POINT_SHADING_SPRITE),
    &bufferObjects);
  checkError("bodies: after set attribute locations sprite");

  setAttributeLocations(
    m_program->getShaderProgram(S2_POINT_SHADING_SIMPLE)->getProgram(),
    m_program->getPositionAttribute(S2_POINT_SHADING_SIMPLE),
    m_program->getColorAttribute(S2_POINT_SHADING_SIMPLE),
    &bufferObjects);
  checkError("bodies: after set attribute locations flat");

  m_bufferObjects.append(bufferObjects);
  checkError("bodies: after append buffer objects");

  bufferObjects.vao->release();
  checkError("bodies: after release vao");

}


/*!
@brief
*/
void s2_PointCloud::draw() {

  if(m_drawEnabled && m_renderWindow->isInitialized()) {

    m_program->bind();
    switch(m_shadingType) {
      case S2_POINT_SHADING_PHONG: {

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(
          GL_TEXTURE_2D_ARRAY, m_renderWindow->getLights()->getDepthTexture());
        m_program->bindDepthBuffer();
        checkError("pointcloud: after bind depth buffer uniform");

        m_program->bindTextures();
        checkError("pointcloud: after bind normal uniform");

        m_program->setLights(m_lights);
        checkError("bodies: after set lights");

        /* draw geometry */
        for(int i = 0; i < m_bufferObjects.size(); i++) {

          m_bufferObjects.at(i).vao->bind();
          checkError("bodies: after bind vao phong");

          m_program->setTransform(m_bufferObjects.at(i).transform);
          checkError("bodies: after set transform phong shader");

          glDrawElements(
            GL_POINTS,
            3 * m_bufferObjects.at(i).pointcloud->vertexCount,
            GL_UNSIGNED_INT,
            0);
          checkError("bodies: after draw elements phong shader");

          m_bufferObjects.at(i).vao->release();
          // checkError("after vao release phong");
        }
        // checkError("after phong release phong");
      } break;
      case S2_POINT_SHADING_SPRITE: {

        for(int i = 0; i < m_bufferObjects.size(); i++) {

          m_bufferObjects.at(i).vao->bind();
          checkError("bodies: after bind vao flat");

          m_program->setTransform(m_bufferObjects.at(i).transform);
          checkError("bodies: after set transform flat shader");

          m_program->bindTextures();
          checkError("pointcloud: after bind normal uniform");

          glDrawElements(
            GL_POINTS,
            3 * m_bufferObjects.at(i).pointcloud->vertexCount,
            GL_UNSIGNED_INT,
            0);
          checkError("bodies: after draw elements flat");

          m_bufferObjects.at(i).vao->release();
          checkError("bodies: after vao release flat");
        }
      } break;
      case S2_POINT_SHADING_SIMPLE: {

      }
      default: {
      } break;
    }
    m_program->release();
  }
}


/*!
@brief
*/
// void s2_PointCloud::drawShadows() {
//
//   if(m_drawEnabled && m_renderWindow->isInitialized()) {
//     m_programShadow->bind();
//     checkError("bodies: after bind shadow program");
//
//     /* draw geometry */
//     for(int i = 0; i < m_bufferObjects.size(); i++) {
//
//       m_bufferObjects.at(i).vao->bind();
//       checkError("bodies: after bind vao shadow");
//
//       m_programShadow->setTransform(m_bufferObjects.at(i).transform);
//       checkError("bodies: after set transform shadow shader");
//
//       glDrawElements(
//         GL_TRIANGLES,
//         3 * m_bufferObjects.at(i).material->faceCount,
//         GL_UNSIGNED_INT,
//         0);
//       checkError("bodies: after draw elements shadow shader");
//
//       m_bufferObjects.at(i).vao->release();
//       checkError("bodies: after release shadow vao");
//     }
//     m_programShadow->release();
//     checkError("bodies: after release shadow shader");
//   }
// }


/*!
@brief initialize attributes for each shader program

*/
void s2_PointCloud::setAttributeLocations(
  QOpenGLShaderProgram* program,
  GLint position_a,
  GLint color_a,
  pointcloud_buffer_t* bufferObjects) {

  program->bind();

  // bufferObjects->vao->bind();

  // uniforms->position_a = program->attributeLocation("position_a");
  // uniforms->normal_a = program->attributeLocation("normal_a");

  // printf("  uniform position: %d\n", uniforms->position);
  // printf("  normal position: %d\n", uniforms->normal);

  bufferObjects->vertexBuffer->bind();
  program->enableAttributeArray(position_a);
  program->setAttributeBuffer(position_a, GL_FLOAT, 0, 3);

  if(color_a != -1) {
    bufferObjects->colorBuffer->bind();
    program->enableAttributeArray(color_a);
    program->setAttributeBuffer(color_a, GL_FLOAT, 0, 3);
  }

  // bufferObjects->vao->release();
  program->release();
}


/*!
@brief free memory for context datastructure
*/
void s2_PointCloud::clearContext() {
  enable(false);
  m_context = NULL;
  m_bufferObjects.clear();
}
