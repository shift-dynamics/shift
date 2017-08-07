#include "bodies.hpp"
#include "renderwindow.hpp"

/*!
@brief set up program uniforms for s2_Bodies
*/
void s2_Bodies::initialize() {
  setUniformLocations(m_programFlat, &m_uniformsFlat);
  setUniformLocations(m_programPhong, &m_uniformsPhong);

}

void s2_Bodies::loadContext() {
  if(m_renderWindow->getContext()) {
    m_context = m_renderWindow->getContext();
    m_context->viewport = s2_camera();
    for(int i = 0; i < m_context->activeBodyCount; i++) {
      s2_body_t* body = m_context->nonVirtualBodyPointers[i];
      initializeBody(body);
    }
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

  m_programPhong->bind();
  BufferObjects bufferObjects;
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

  checkError("before generating vertex arrays");

  bufferObjects.vao->bind();

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

  bufferObjects.vao->bind();

  setAttributeLocations(m_programPhong, &m_uniformsPhong, &bufferObjects);
  checkError("after set attribute location phong");

  setAttributeLocations(m_programFlat, &m_uniformsFlat, &bufferObjects);
  checkError("after set attribute location flat");
  m_bufferObjects.append(bufferObjects);

}


void s2_Bodies::draw() {

  /* draw geometry */
  for(int i = 0; i < m_bufferObjects.size(); i++ ) {

    /* override phong shaded models with wireframe if
    global wireframe shading type is selected */
    s2_shading_enum_t shadingType =
      m_bufferObjects.at(i).material->shadingType;

    switch(shadingType) {
      case PHONG: {
        m_programPhong->bind();
        m_bufferObjects.at(i).vao->bind();

        QMatrix4x4 model =
          s2_displacementToQMatrix4x4(m_bufferObjects.at(i).transform);
        QMatrix4x4 view = m_renderWindow->getViewMatrix();
        s2_material_t* material = m_bufferObjects.at(i).material;
        QMatrix4x4 mv = view * model;

        /* set model * view * projection matrix */
        m_programPhong->setUniformValue(
          m_uniformsPhong.mvp_u,
          m_renderWindow->getProjectionMatrix() * mv);
        // checkError("after set mvp phong");

        /* set model * view matrix */
        m_programPhong->setUniformValue(
          m_uniformsPhong.mv_u, mv);
        // checkError("after set mv phong");

        /* set view matrix */
        m_programPhong->setUniformValue(
          m_uniformsPhong.v_u,
          view);
        // checkError("after set v phong");

        /* set normal matrix */
        m_programPhong->setUniformValue(
          m_uniformsPhong.nm_u,
          mv.normalMatrix());
        // checkError("after set nm phong");

        /* set Ka for lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.Ka_u,
          m_lights->getKa(),
          10,
          3);

        /* set Ks for lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.Ks_u,
          m_lights->getKs(),
          10,
          3);
        // checkError("after set KsLight phong");

        /* set Kd for lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.Kd_u,
          m_lights->getKd(),
          10,
          3);
        // checkError("after set KdLight phong");

        /* set directional switch for lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.directional_u,
          m_lights->getDirectional(),
          10);
        // checkError("after set directional phong");

        /* set positions of lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.posLight_u,
          m_lights->getPosition(),
          10,
          3);
        // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
        // checkError("after set poslight phong");

        /* set number of lights */
        m_programPhong->setUniformValue(
          m_uniformsPhong.lightCount_u,
          m_lights->getCount());
        // qDebug() << "light count:" << lightParameters.lightCount;
        // checkError("after set light count phong");

        /* set Kd properties for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.KdMaterial_u,
          material->Kd[0],
          material->Kd[1],
          material->Kd[2]);
        // checkError("after set KdMaterial phong");

        /* set Ks properties for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.KsMaterial_u,
          material->Ks[0],
          material->Ks[1],
          material->Ks[2]);
        // checkError("after set KsMaterial phong");

        /* set Ka properties for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.KaMaterial_u,
          material->Ka[0],
          material->Ka[1],
          material->Ka[2]);
        // checkError("after set KaMaterial phong");

        // qDebug() << "KaMaterial: " << material->Ka[0]
        // << material->Ka[1] << material->Ka[2];

        /* set specularity property for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.Ns_u,
          material->Ns);
        // checkError("after set Ns phong");
        if(m_shadingType == WIREFRAME) {
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        } else {
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
        // checkError("after set polygon mode phong");

        glDrawElements(
          GL_TRIANGLES,
          3 * material->faceCount,
          GL_UNSIGNED_INT,
          0);
        // checkError("after draw elements phong");

        m_bufferObjects.at(i).vao->release();
        // checkError("after vao release phong");

        m_programPhong->release();
        // checkError("after phong release phong");

      } break;
      case FLAT: {
        m_programFlat->bind();
        m_bufferObjects.at(i).vao->bind();
        QMatrix4x4 model =
          s2_displacementToQMatrix4x4(m_bufferObjects.at(i).transform);
        s2_material_t* material = m_bufferObjects.at(i).material;
        QMatrix4x4 view = m_renderWindow->getViewMatrix();
        QMatrix4x4 mv = view * model;

        /* set model * view * projection matrix */
        m_programFlat->setUniformValue(
          m_uniformsFlat.mvp_u,
          m_renderWindow->getProjectionMatrix() * mv);
        checkError("after set mvp");

        /* set model * view matrix */
        m_programFlat->setUniformValue(
          m_uniformsFlat.mv_u, mv);
        checkError("after set mv");

        /* set view matrix */
        m_programFlat->setUniformValue(
          m_uniformsFlat.v_u,
          view);
        checkError("after set v");

        /* set normal matrix */
        m_programFlat->setUniformValue(
          m_uniformsFlat.nm_u,
          mv.normalMatrix());
        checkError("after set nm");

        /* set Ka for lights */
        m_programFlat->setUniformValueArray(
          m_uniformsFlat.Ka_u,
          m_lights->getKa(),
          10,
          3);

        /* set Ks for lights */
        m_programFlat->setUniformValueArray(
          m_uniformsFlat.Ks_u,
          m_lights->getKs(),
          10,
          3);
        checkError("after set KsLight");

        /* set Kd for lights */
        m_programFlat->setUniformValueArray(
          m_uniformsFlat.Kd_u,
          m_lights->getKd(),
          10,
          3);
        checkError("after set KdLight");

        /* set directional switch for lights */
        m_programFlat->setUniformValueArray(
          m_uniformsFlat.directional_u,
          m_lights->getDirectional(),
          10);
        checkError("after set directional");

        /* set positions of lights */
        m_programFlat->setUniformValueArray(
          m_uniformsFlat.posLight_u,
          m_lights->getPosition(),
          10,
          3);
        // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
        checkError("after set poslight");

        /* set number of lights */
        m_programFlat->setUniformValue(
          m_uniformsFlat.lightCount_u,
          m_lights->getCount());

        // qDebug() << "light count:" << lightParameters.lightCount;
        checkError("after set light count");

        /* set Kd properties for material */
        m_programFlat->setUniformValue(
          m_uniformsFlat.KdMaterial_u,
          material->Kd[0],
          material->Kd[1],
          material->Kd[2],
          material->Kd[3]);
        checkError("after set KdMaterial");

        /* set Ks properties for material */
        m_programFlat->setUniformValue(
          m_uniformsFlat.KsMaterial_u,
          material->Ks[0],
          material->Ks[1],
          material->Ks[2],
          material->Ks[3]);
        checkError("after set KsMaterial");

        /* set Ka properties for material */
        m_programFlat->setUniformValue(
          m_uniformsFlat.KaMaterial_u,
          material->Ka[0],
          material->Ka[1],
          material->Ka[2],
          material->Ka[3]);
        checkError("after set KaMaterial");

        // qDebug() << "KaMaterial: " << material->Ka[0]
        // << material->Ka[1] << material->Ka[2];

        /* set specularity property for material */
        m_programFlat->setUniformValue(
          m_uniformsFlat.Ns_u,
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

        m_bufferObjects.at(i).vao->release();
        checkError("after vao release");

        m_programFlat->release();
        checkError("after flat release");
      } break;
      default: {

      } break;
    }
  }
}


/*!
@brief initialize program uniforms
*/
void s2_Bodies::setUniformLocations(
  QOpenGLShaderProgram* program,
  UniformLocations* uniforms) {

  program->bind();

  uniforms->directional_u = program->uniformLocation("directional");
  checkError("after set uniform location directional");
  // qDebug() << "directional: " << programUniforms->directional;

  uniforms->posLight_u = program->uniformLocation("posLight");
  checkError("after set uniform location posLight");
  // qDebug() << "posLight: " << programUniforms->posLight;

  uniforms->lightCount_u = program->uniformLocation("lightCount");
  checkError("after set uniform location lightCount");
  // qDebug() << "lightCount: " << programUniforms->lightCount;

  uniforms->Ka_u = program->uniformLocation("Ka");
  checkError("after set uniform location Ka");
  // qDebug() << "Ka: " << programUniforms->Ka;

  uniforms->Kd_u = program->uniformLocation("Kd");
  checkError("after set uniform location Kd");
  // qDebug() << "Kd: " << programUniforms->Kd;

  uniforms->Ks_u = program->uniformLocation("Ks");
  checkError("after set uniform location Ks");
  // qDebug() << "Ks: " << programUniforms->Ks;

  // uniforms->m_u = program->uniformLocation("m");
  // // qDebug() << "m: " << programUniforms->m;
  // checkError("after set uniform location m");

  uniforms->v_u = program->uniformLocation("v");
  // qDebug() << "v: " << programUniforms->v;
  checkError("after set uniform location v");

  uniforms->mv_u = program->uniformLocation("mv");
  // qDebug() << "mv: " << programUniforms->mv;
  checkError("after set uniform location mv");

  uniforms->mvp_u = program->uniformLocation("mvp");
  // qDebug() << "mvp: " << programUniforms->mvp;
  checkError("after set uniform location mvp");

  uniforms->nm_u = program->uniformLocation("nm");
  // qDebug() << "nm: " << programUniforms->nm;
  checkError("after set uniform location nm");

  uniforms->KaMaterial_u = program->uniformLocation("KaMaterial");
  // qDebug() << "KaMaterial: " << programUniforms->KaMaterial;
  checkError("after set uniform location KaMaterial");

  uniforms->KdMaterial_u = program->uniformLocation("KdMaterial");
  // qDebug() << "KdMaterial: " << programUniforms->KdMaterial;
  checkError("after set uniform location KdMaterial");

  uniforms->KsMaterial_u = program->uniformLocation("KsMaterial");
  // qDebug() << "KsMaterial: " << programUniforms->KsMaterial;
  checkError("after set uniform location KsMaterial");

  uniforms->Ns_u = program->uniformLocation("Ns");
  // qDebug() << "Ns: " << programUniforms->Ns;
  checkError("after set uniform location Ns");
  program->release();
}


/*!
@brief initialize attributes for each shader program

*/
void s2_Bodies::setAttributeLocations(
  QOpenGLShaderProgram* program,
  UniformLocations* uniforms,
  BufferObjects* bufferObjects) {

  program->bind();

  bufferObjects->vao->bind();

  uniforms->position_a =
    program->attributeLocation("posAttr");
  uniforms->normal_a =
    program->attributeLocation("normAttr");

  // printf("  uniform position: %d\n", uniforms->position);
  // printf("  normal position: %d\n", uniforms->normal);

  program->bind();
  bufferObjects->vertexBuffer->bind();

  program->enableAttributeArray(
    uniforms->position_a);
  program->setAttributeBuffer(
    uniforms->position_a,
    GL_FLOAT,
    0,
    3);

  if(uniforms->normal_a != -1) {
    bufferObjects->vertexNormalBuffer->bind();
    program->enableAttributeArray(
      uniforms->normal_a);
    program->setAttributeBuffer(
      uniforms->normal_a,
      GL_FLOAT,
      0,
      3);
  }

  bufferObjects->vao->release();
  program->release();
}
