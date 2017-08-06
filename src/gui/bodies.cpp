#include "bodies.hpp"

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
void RenderWindow::initializeMaterial(
  s2_material_t* material,
  s2_displacement_t* origin) {

  m_program->bind();
  BufferObjects bufferObjects;
  bufferObjects.vao = new QOpenGLVertexArrayObject(this);
  bufferObjects.vao->create();

  bufferObjects.vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects.vertexBuffer->create();

  bufferObjects.ibovertexNormalBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  bufferObjects.ibovertexNormalBuffer->create();

  bufferObjects.ibo =
    new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  bufferObjects.ibo->create();

  checkError("before generating vertex arrays");

  bufferObjects->vao->bind();

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

  m_program->bind();

  bufferObjects.vao->bind();

  m_posAttr = m_program->attributeLocation("posAttr");
  m_normAttr = m_program->attributeLocation("normAttr");

  bufferObjects.vertexBuffer->bind();

  m_program->enableAttributeArray(m_posAttr);
  m_program->setAttributeBuffer(
    positionAttr,
    GL_FLOAT,
    0,
    3);

  if(uniforms->normal != -1) {
    bufferObjects.vertexNormalBuffer->bind();
    shaderProgram->enableAttributeArray(
      uniforms->normal);
    shaderProgram->setAttributeBuffer(
      uniforms->normal,
      GL_FLOAT,
      0,
      3);
  }

  bufferObjects.vao->release();
  m_program->release();

  checkError("after set attribute location");
  m_bufferObjects.append(bufferObjects);

}

s2_Bodies::draw() {

  /* draw geometry */
  for(int i = 0; i < m_bufferObjects.size(); i++ ) {

    /* override phong shaded models with wireframe if
    global wireframe shading type is selected */
    s2_shading_enum_t shadingType =
      m_bufferObjects.at(i)->material->shadingType;
    if(m_shadingType == WIREFRAME) {
      shadingType = WIREFRAME;
    }

    switch(shadingType) {
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
        // checkError("after set mvp phong");

        /* set model * view matrix */
        m_programPhong->setUniformValue(
          m_uniformsPhong.mv, mv);
        // checkError("after set mv phong");

        /* set view matrix */
        m_programPhong->setUniformValue(
          m_uniformsPhong.v,
          m_view);
        // checkError("after set v phong");

        /* set normal matrix */
        m_programPhong->setUniformValue(
          m_uniformsPhong.nm,
          mv.normalMatrix());
        // checkError("after set nm phong");

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
        // checkError("after set KsLight phong");

        /* set Kd for lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.Kd,
          m_lightParameters.KdLight,
          10,
          3);
        // checkError("after set KdLight phong");

        /* set directional switch for lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.directional,
          m_lightParameters.directional,
          10);
        // checkError("after set directional phong");

        /* set positions of lights */
        m_programPhong->setUniformValueArray(
          m_uniformsPhong.posLight,
          m_lightParameters.posLight,
          10,
          3);
        // qDebug() << lightParameters.posLight[0] << lightParameters.posLight[1] << lightParameters.posLight[2];
        // checkError("after set poslight phong");

        /* set number of lights */
        m_programPhong->setUniformValue(
          m_uniformsPhong.lightCount,
          m_lightParameters.lightCount);
        // qDebug() << "light count:" << lightParameters.lightCount;
        // checkError("after set light count phong");

        /* set Kd properties for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.KdMaterial,
          material->Kd[0],
          material->Kd[1],
          material->Kd[2]);
        // checkError("after set KdMaterial phong");

        /* set Ks properties for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.KsMaterial,
          material->Ks[0],
          material->Ks[1],
          material->Ks[2]);
        // checkError("after set KsMaterial phong");

        /* set Ka properties for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.KaMaterial,
          material->Ka[0],
          material->Ka[1],
          material->Ka[2]);
        // checkError("after set KaMaterial phong");

        // qDebug() << "KaMaterial: " << material->Ka[0]
        // << material->Ka[1] << material->Ka[2];

        /* set specularity property for material */
        m_programPhong->setUniformValue(
          m_uniformsPhong.Ns,
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

        m_bufferObjects.at(i)->vao->release();
        // checkError("after vao release phong");

        m_programPhong->release();
        // checkError("after phong release phong");

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

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
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
      default: {

      } break;
    }
  }
