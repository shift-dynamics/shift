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
    initializeMaterial(material, m_context->d + body->origin->dIndex);
  }
}


/*!
@brief initialize opengl properties for a s2_material_t datastructure

*/
void s2_Bodies::initializeMaterial(
  s2_material_t* material,
  s2_displacement_t* origin) {

  // m_programPhong->bind();
  buffer_objects_t bufferObjects;
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

  setAttributeLocations(
    m_programPhong->getProgram(),
    m_programPhong->getPositionAttribute(),
    m_programPhong->getNormalAttribute(),
    &bufferObjects);
  checkError("bodies: after set attribute locations phong");

  setAttributeLocations(
    m_programFlat->getProgram(),
    m_programFlat->getPositionAttribute(),
    m_programFlat->getNormalAttribute(),
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
        for(int i = 0; i < m_bufferObjects.size(); i++) {

          m_bufferObjects.at(i).vao->bind();
          checkError("bodies: after bind vao phong");

          m_programPhong->setTransform(m_bufferObjects.at(i).transform);
          checkError("bodies: after set transform phong shader");

          m_programPhong->setMaterial(m_bufferObjects.at(i).material);
          checkError("bodies: after set material phong shader");

          glDrawElements(
            GL_TRIANGLES,
            3 * m_bufferObjects.at(i).material->faceCount,
            GL_UNSIGNED_INT,
            0);
          checkError("bodies: after draw elements phong shader");

          m_bufferObjects.at(i).vao->release();
          // checkError("after vao release phong");
        }
        m_programPhong->release();
        // checkError("after phong release phong");
      } break;
      case S2_SHADING_FLAT: {
        m_programFlat->bind();
        m_programFlat->setLights(m_lights);

        for(int i = 0; i < m_bufferObjects.size(); i++) {

          m_bufferObjects.at(i).vao->bind();
          checkError("bodies: after bind vao flat");

          m_programFlat->setTransform(m_bufferObjects.at(i).transform);
          checkError("bodies: after set transform flat shader");

          m_programFlat->setMaterial(m_bufferObjects.at(i).material);
          checkError("bodies: after set material flat shader");

          glDrawElements(
            GL_TRIANGLES,
            3 * m_bufferObjects.at(i).material->faceCount,
            GL_UNSIGNED_INT,
            0);
          checkError("bodies: after draw elements flat");

          m_bufferObjects.at(i).vao->release();
          checkError("bodies: after vao release flat");
        }
        m_programFlat->release();
        checkError("bodies: after flat release flat");
      } break;
      default: {
      } break;
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
    for(int i = 0; i < m_bufferObjects.size(); i++) {

      m_bufferObjects.at(i).vao->bind();
      checkError("bodies: after bind vao shadow");

      m_programShadow->setTransform(m_bufferObjects.at(i).transform);
      checkError("bodies: after set transform shadow shader");

      glDrawElements(
        GL_TRIANGLES,
        3 * m_bufferObjects.at(i).material->faceCount,
        GL_UNSIGNED_INT,
        0);
      checkError("bodies: after draw elements shadow shader");

      m_bufferObjects.at(i).vao->release();
      checkError("bodies: after release shadow vao");
    }
    m_programShadow->release();
    checkError("bodies: after release shadow shader");
  }
}


/*!
@brief initialize attributes for each shader program

*/
void s2_Bodies::setAttributeLocations(
  QOpenGLShaderProgram* program,
  GLint position_a,
  GLint normal_a,
  buffer_objects_t* bufferObjects) {

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
@brief free memory for context datastructure
*/
void s2_Bodies::clearContext() {
  enable(false);
  m_context = NULL;
  m_bufferObjects.clear();
}
