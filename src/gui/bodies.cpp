#include "bodies.hpp"
#include "renderwindow.hpp"
#include "phongshaderprogram.hpp"
#include "flatshaderprogram.hpp"
#include "simpleshaderprogram.hpp"
#include "shadowshaderprogram.hpp"
#include "lights.hpp"


/*!
@brief sets draw style, WIREFRAME, PHONG, and FLAT are the only
*/
void s2_Bodies::setDrawStyle(s2_shading_enum_t shadingType) {
  switch(shadingType) {
    case S2_SHADING_WIREFRAME: {
      m_shadingType = S2_SHADING_WIREFRAME;
    } break;
    case S2_SHADING_FLAT: {
      m_shadingType = S2_SHADING_FLAT;
    }
    default: {
      m_shadingType = S2_SHADING_PHONG;
    } break;
  }
}


/*!
@brief sets pointer to lights datastructure
*/
void s2_Bodies::setLights(s2_Lights* lights) {
  m_lights = lights;
}


/*!
@brief set pointer to flat shader class
*/
void s2_Bodies::setFlatShader(FlatShaderProgram* program) {
  m_programFlat = program;
}


/*!
@brief set pointer to phong shader class
*/
void s2_Bodies::setPhongShader(PhongShaderProgram* program) {
  m_programPhong = program;
}


/*!
@brief set pointer to phong shader class
*/
void s2_Bodies::setLineShader(SimpleShaderProgram* program) {
  m_programLine = program;
}


/*!
@brief set pointer to phong shader class
*/
void s2_Bodies::setShadowShader(ShadowShaderProgram* program) {
  m_programShadow = program;
}


/*!
@brief set up program uniforms for s2_Bodies
*/
void s2_Bodies::initialize() {

}


/*!
@brief load context data structure
*/
void s2_Bodies::loadContext() {
  if(m_renderWindow->getContext()) {
    m_context = m_renderWindow->getContext();
    m_context->viewport = s2_camera();
    for(int i = 0; i < m_context->activeBodyCount; i++) {
      s2_body_t* body = m_context->nonVirtualBodyPointers[i];
      initializeBody(body);
    }
    enable(true);
  }
}


/*!
@brief initialize opengl properties for a s2_body_t datastructure

*/
void s2_Bodies::initializeBody(
  s2_body_t* body) {

  for(int i = 0; i < body->materialCount; i++) {
    s2_material_t* material = body->materials + i;
    // printf("initializing material [%d]\n", i);
    initializeMaterial(material, body, m_context->d + body->origin->dIndex);
  }
  initializeConvexHull(&body->convexHulls[0], body);
}


/*!
@brief initialize opengl properties for a s2_material_t datastructure

*/
void s2_Bodies::initializeMaterial(
  s2_material_t* material,
  s2_body_t* body,
  s2_displacement_t* origin) {

  // m_programPhong->bind();
  material_buffer_t bufferObjects;
  bufferObjects.body = body;
  bufferObjects.vao = new QOpenGLVertexArrayObject(m_renderWindow);
  bufferObjects.vao->create();

  bufferObjects.vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects.vertexBuffer->create();

  bufferObjects.vertexNormalBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects.vertexNormalBuffer->create();

  bufferObjects.ibo =
    new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  bufferObjects.ibo->create();

  checkError("bodies: after create vertex buffers");

  bufferObjects.vao->bind();
  checkError("bodies: after bind vao");

  bufferObjects.vertexBuffer->bind();
  bufferObjects.vertexBuffer->allocate(
    3 * material->vertexCount * sizeof(GLfloat));
  bufferObjects.vertexBuffer->write(
    0,
    material->vertices,
    3 * material->vertexCount * sizeof(GLfloat));

  bufferObjects.vertexNormalBuffer->bind();
  bufferObjects.vertexNormalBuffer->allocate(
    3 * material->vertexCount * sizeof(GLfloat));
  bufferObjects.vertexNormalBuffer->write(
    0,
    material->vertexNormals,
    3 * material->vertexCount * sizeof(GLfloat));

  bufferObjects.ibo->bind();
  bufferObjects.ibo->allocate(
    material->faces,
    3 * material->faceCount * sizeof(GLuint));

  bufferObjects.material = material;
  bufferObjects.transform = origin;

  setMaterialAttributeLocations(
    m_programPhong->getProgram(),
    m_programPhong->getPositionAttribute(),
    m_programPhong->getNormalAttribute(),
    &bufferObjects);
  checkError("bodies: after set attribute locations phong");

  setMaterialAttributeLocations(
    m_programFlat->getProgram(),
    m_programFlat->getPositionAttribute(),
    m_programFlat->getNormalAttribute(),
    &bufferObjects);
  checkError("bodies: after set attribute locations flat");

  m_materialBufferObjects.append(bufferObjects);
  checkError("bodies: after append buffer objects");

  bufferObjects.vao->release();
  checkError("bodies: after release vao");

}


/*!
@brief initialize opengl properties for a s2_convexhull_t datastructure

*/
void s2_Bodies::initializeConvexHull(
  s2_convexhull_t* convexhull,
  s2_body_t* body) {

  s2_displacement_t* origin = m_context->d + body->origin->dIndex;
  // m_programPhong->bind();
  convexhull_buffer_t bufferObjects;
  bufferObjects.vao = new QOpenGLVertexArrayObject(m_renderWindow);
  bufferObjects.vao->create();

  bufferObjects.vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects.vertexBuffer->create();

  bufferObjects.indexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  bufferObjects.indexBuffer->create();

  checkError("bodies: after create convexhull vertex buffers");

  bufferObjects.vao->bind();
  checkError("bodies: after bind vao");

  bufferObjects.vertexBuffer->bind();
  bufferObjects.vertexBuffer->allocate(
    3 * convexhull->vertexCount * sizeof(GLfloat));
  bufferObjects.vertexBuffer->write(
    0,
    convexhull->vertices,
    3 * convexhull->vertexCount * sizeof(GLfloat));

  bufferObjects.indexBuffer->bind();
  bufferObjects.indexBuffer->allocate(
    convexhull->faces,
    3 * convexhull->faceCount * sizeof(GLuint));

  bufferObjects.convexhull = convexhull;
  bufferObjects.transform = origin;

  setConvexHullAttributeLocations(
    m_programLine->getProgram(),
    m_programLine->getPositionAttribute(),
    &bufferObjects);
  checkError("bodies: after set attribute locations flat");

  m_convexhullBufferObjects.append(bufferObjects);
  checkError("bodies: after append convex hull buffer objects");

  bufferObjects.vao->release();
  checkError("bodies: after release vao");

}



/*!
@brief
*/
void s2_Bodies::draw() {

  if(m_drawEnabled && m_renderWindow->isInitialized()) {

    if(m_shadingType == S2_SHADING_WIREFRAME) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    switch(m_shadingType) {
      case S2_SHADING_WIREFRAME:
      case S2_SHADING_PHONG: {
        m_programPhong->bind();

        glActiveTexture(GL_TEXTURE0);

        glBindTexture(
          GL_TEXTURE_2D_ARRAY, m_renderWindow->getLights()->getDepthTexture());
        checkError("bodies: after bind texture");

        m_programPhong->bindDepthBuffer();
        checkError("bodies: after bind depth buffer uniform");

        m_programPhong->setLights(m_lights);
        checkError("bodies: after set lights");

        /* draw geometry */
        for(int i = 0; i < m_materialBufferObjects.size(); i++) {

          if(m_materialBufferObjects.at(i).body->drawEnabled &&
            m_materialBufferObjects.at(i).body->isVisible) {

            m_materialBufferObjects.at(i).vao->bind();
            checkError("bodies: after bind vao phong");

            m_programPhong->setTransform(m_materialBufferObjects.at(i).transform);
            checkError("bodies: after set transform phong shader");

            m_programPhong->setMaterial(m_materialBufferObjects.at(i).material);
            checkError("bodies: after set material phong shader");

            glDrawElements(
              GL_TRIANGLES,
              3 * m_materialBufferObjects.at(i).material->faceCount,
              GL_UNSIGNED_INT,
              0);
            checkError("bodies: after draw elements phong shader");

            m_materialBufferObjects.at(i).vao->release();
            // checkError("after vao release phong");
          }
        }
        m_programPhong->release();
        // checkError("after phong release phong");
      } break;
      case S2_SHADING_FLAT: {
        m_programFlat->bind();
        m_programFlat->setLights(m_lights);

        for(int i = 0; i < m_materialBufferObjects.size(); i++) {
          if(m_materialBufferObjects.at(i).body->drawEnabled) {

            m_materialBufferObjects.at(i).vao->bind();
            checkError("bodies: after bind vao flat");

            m_programFlat->setTransform(m_materialBufferObjects.at(i).transform);
            checkError("bodies: after set transform flat shader");

            m_programFlat->setMaterial(m_materialBufferObjects.at(i).material);
            checkError("bodies: after set material flat shader");

            glDrawElements(
              GL_TRIANGLES,
              3 * m_materialBufferObjects.at(i).material->faceCount,
              GL_UNSIGNED_INT,
              0);
            checkError("bodies: after draw elements flat");

            m_materialBufferObjects.at(i).vao->release();
            checkError("bodies: after vao release flat");
          }
        }
        m_programFlat->release();
        checkError("bodies: after flat release flat");
      } break;
      default: {
      } break;
    }

    if(m_drawConvexHullEnabled) {
      m_programLine->bind();
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      for(int i = 0; i < m_convexhullBufferObjects.size(); i++) {
        if(m_convexhullBufferObjects.at(i).convexhull->drawEnabled) {

          m_convexhullBufferObjects.at(i).vao->bind();
          checkError("bodies: after bind vao flat");

          m_programLine->setTransform(m_convexhullBufferObjects.at(i).transform);
          checkError("bodies: after set transform line shader");

          GLfloat* color =
            m_convexhullBufferObjects.at(i).body->convexHullColor;
          m_programLine->setColor(color[0], color[1], color[2], 1.f);
          m_programLine->setScale(1.f);
          checkError("bodies: after set material line shader");

          glDrawElements(
            GL_TRIANGLES,
            3 * m_convexhullBufferObjects.at(i).convexhull->faceCount,
            GL_UNSIGNED_INT,
            0);
          checkError("bodies: after draw elements flat");

          m_convexhullBufferObjects.at(i).vao->release();
          checkError("bodies: after vao release flat");
        }
      }
      m_programLine->release();
    }

    if(m_shadingType == S2_SHADING_WIREFRAME) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

  }
}


/*!
@brief
*/
void s2_Bodies::drawShadows() {

  if(m_drawEnabled && m_renderWindow->isInitialized()) {
    m_programShadow->bind();
    checkError("bodies: after bind shadow program");

    /* draw geometry */
    for(int i = 0; i < m_materialBufferObjects.size(); i++) {

      if(m_materialBufferObjects.at(i).body->isVisible) {

        m_materialBufferObjects.at(i).vao->bind();
        checkError("bodies: after bind vao shadow");

        m_programShadow->setTransform(m_materialBufferObjects.at(i).transform);
        checkError("bodies: after set transform shadow shader");

        glDrawElements(
          GL_TRIANGLES,
          3 * m_materialBufferObjects.at(i).material->faceCount,
          GL_UNSIGNED_INT,
          0);
        checkError("bodies: after draw elements shadow shader");

        m_materialBufferObjects.at(i).vao->release();
        checkError("bodies: after release shadow vao");
      }
    }
    m_programShadow->release();
    checkError("bodies: after release shadow shader");
  }
}


/*!
@brief initialize attributes for each shader program

*/
void s2_Bodies::setMaterialAttributeLocations(
  QOpenGLShaderProgram* program,
  GLint position_a,
  GLint normal_a,
  material_buffer_t* bufferObjects) {

  program->bind();

  // bufferObjects->vao->bind();

  // uniforms->position_a = program->attributeLocation("position_a");
  // uniforms->normal_a = program->attributeLocation("normal_a");

  // printf("  uniform position: %d\n", uniforms->position);
  // printf("  normal position: %d\n", uniforms->normal);

  bufferObjects->vertexBuffer->bind();
  program->enableAttributeArray(position_a);
  program->setAttributeBuffer(position_a, GL_FLOAT, 0, 3);

  if(normal_a != -1) {
    bufferObjects->vertexNormalBuffer->bind();
    program->enableAttributeArray(normal_a);
    program->setAttributeBuffer(normal_a, GL_FLOAT, 0, 3);
  }

  // bufferObjects->vao->release();
  program->release();
}


/*!
@brief initialize attributes for each shader program

*/
void s2_Bodies::setConvexHullAttributeLocations(
  QOpenGLShaderProgram* program,
  GLint position_a,
  convexhull_buffer_t* bufferObjects) {

  program->bind();

  bufferObjects->vao->bind();

  // uniforms->position_a = program->attributeLocation("position_a");
  // uniforms->normal_a = program->attributeLocation("normal_a");

  // printf("  uniform position: %d\n", uniforms->position);
  // printf("  normal position: %d\n", uniforms->normal);

  bufferObjects->vertexBuffer->bind();
  program->enableAttributeArray(position_a);
  program->setAttributeBuffer(position_a, GL_FLOAT, 0, 3);

  bufferObjects->vao->release();
  program->release();
}



/*!
@brief free memory for context datastructure
*/
void s2_Bodies::clearContext() {
  enable(false);
  m_context = NULL;
  m_materialBufferObjects.clear();
  m_convexhullBufferObjects.clear();
}
