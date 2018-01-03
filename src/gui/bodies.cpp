#include "bodies.hpp"
#include "renderwindow.hpp"
#include "phongshaderprogram.hpp"
#include "flatshaderprogram.hpp"
#include "simpleshaderprogram.hpp"
#include "shadowshaderprogram.hpp"
#include "normalshaderprogram.hpp"
#include "lights.hpp"

#include <assimp/cimport.h>        // Plain-C interface to assimp loader
#include <assimp/scene.h>          // Output data structure
#include <assimp/postprocess.h>    // Post processing flags
#include <assimp/material.h>

/*!
@brief sets draw style, WIREFRAME, PHONG, and FLAT are the three supported draw styles
*/
void s2_Bodies::setDrawStyle(s2_shading_enum_t shadingType) {
  switch(shadingType) {
    case S2_SHADING_WIREFRAME: {
      m_shadingType = S2_SHADING_WIREFRAME;
      break;
    }
    case S2_SHADING_FLAT: {
      m_shadingType = S2_SHADING_FLAT;
      break;
    }
    default: {
      m_shadingType = S2_SHADING_PHONG;
      break;
    }
  }
}


void s2_Bodies::loadGeometryPrimitives() {

  QFile sphere_data(":/geometry/sphere.obj");
  if(!sphere_data.open(QIODevice::ReadOnly | QIODevice::Text)) {
    qDebug() << "failed to open qresource file :/geometry/sphere.obj";
    exit(-1);
  }
  QByteArray sphere = sphere_data.readAll();
  sphere_data.close();

  const aiScene* scene = aiImportFileFromMemory(
                           sphere.constData(),
                           sphere.size(),
                           aiProcess_CalcTangentSpace |
                           aiProcess_Triangulate |
                           aiProcess_SortByPType,
                           ".obj");

  /* If loading was not successful, report import error and exit */
  if(!scene) {
    printf("%s\n", aiGetErrorString());
    printf("error, exiting.\n");
    exit(-1);
    return;
  }

  aiMesh* mesh = scene->mMeshes[0];

  /* load vertices */
  m_sphereBufferObject.vertices =
    (GLfloat*)malloc(3 * mesh->mNumVertices * sizeof(GLfloat));
  m_sphereBufferObject.vertexCount = mesh->mNumVertices;
  for(uint j = 0; j < mesh->mNumVertices; j++) {
    m_sphereBufferObject.vertices[3 * j + 0] = mesh->mVertices[j].x;
    m_sphereBufferObject.vertices[3 * j + 1] = mesh->mVertices[j].y;
    m_sphereBufferObject.vertices[3 * j + 2] = mesh->mVertices[j].z;
  }

  /* load vertex normals */
  m_sphereBufferObject.vertexNormals =
    (GLfloat*)malloc(3 * mesh->mNumVertices * sizeof(GLfloat));
  for(uint j = 0; j < mesh->mNumVertices; j++) {
    m_sphereBufferObject.vertexNormals[3 * j + 0] = mesh->mNormals[j].x;
    m_sphereBufferObject.vertexNormals[3 * j + 1] = mesh->mNormals[j].y;
    m_sphereBufferObject.vertexNormals[3 * j + 2] = mesh->mNormals[j].z;
  }

  /* load faces */
  m_sphereBufferObject.faceCount = mesh->mNumFaces;
  m_sphereBufferObject.faces =
    (GLuint*)malloc(3 * mesh->mNumFaces * sizeof(GLuint));
  for(int j = 0; j < mesh->mNumFaces; j++) {
    m_sphereBufferObject.faces[j * 3 + 0] = mesh->mFaces[j].mIndices[0];
    m_sphereBufferObject.faces[j * 3 + 1] = mesh->mFaces[j].mIndices[1];
    m_sphereBufferObject.faces[j * 3 + 2] = mesh->mFaces[j].mIndices[2];
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
@brief set pointer to normal shader class
*/
void s2_Bodies::setNormalShader(NormalShaderProgram* program) {
  m_programNormal = program;
}


/*!
@brief set up program uniforms for s2_Bodies
*/
void s2_Bodies::initialize() {
  loadGeometryPrimitives();
}


void s2_Bodies::initializeSphere() {
  m_programPhong->bind();
  m_sphereBufferObject.vao = new QOpenGLVertexArrayObject(m_renderWindow);
  m_sphereBufferObject.vao->create();

  m_sphereBufferObject.vertexBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_sphereBufferObject.vertexBuffer->create();

  m_sphereBufferObject.vertexNormalBuffer =
    new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  m_sphereBufferObject.vertexNormalBuffer->create();

  m_sphereBufferObject.ibo =
    new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  m_sphereBufferObject.ibo->create();
  checkError("bodies: after create sphere vertex buffers");

  m_sphereBufferObject.vao->bind();
  checkError("bodies: after bind sphere vao");

  m_sphereBufferObject.vertexBuffer->bind();
  m_sphereBufferObject.vertexBuffer->allocate(
    3 * m_sphereBufferObject.vertexCount * sizeof(GLfloat));
  m_sphereBufferObject.vertexBuffer->write(
    0,
    m_sphereBufferObject.vertices,
    3 * m_sphereBufferObject.vertexCount * sizeof(GLfloat));

  m_sphereBufferObject.vertexNormalBuffer->bind();
  m_sphereBufferObject.vertexNormalBuffer->allocate(
    3 * m_sphereBufferObject.vertexCount * sizeof(GLfloat));
  m_sphereBufferObject.vertexNormalBuffer->write(
    0,
    m_sphereBufferObject.vertexNormals,
    3 * m_sphereBufferObject.vertexCount * sizeof(GLfloat));

  m_sphereBufferObject.ibo->bind();
  m_sphereBufferObject.ibo->allocate(
    m_sphereBufferObject.faces,
    3 * m_sphereBufferObject.faceCount * sizeof(GLuint));


  // bufferObjects->vao->bind();

  // uniforms->position_a = program->attributeLocation("position_a");
  // uniforms->normal_a = program->attributeLocation("normal_a");

  // printf("  uniform position: %d\n", uniforms->position);
  // printf("  normal position: %d\n", uniforms->normal);

  m_sphereBufferObject.vertexBuffer->bind();

  m_programPhong->getProgram()->enableAttributeArray(
    m_programPhong->getPositionAttribute());
  m_programPhong->getProgram()->setAttributeBuffer(
    m_programPhong->getPositionAttribute(), GL_FLOAT, 0, 3);

  m_sphereBufferObject.vertexNormalBuffer->bind();
  m_programPhong->getProgram()->enableAttributeArray(
    m_programPhong->getNormalAttribute());
  m_programPhong->getProgram()->setAttributeBuffer(
    m_programPhong->getNormalAttribute(), GL_FLOAT, 0, 3);

  m_programPhong->release();
  m_sphereBufferObject.initialized = true;
}


/*!
@brief load context data structure
*/
void s2_Bodies::loadContext() {
  if(m_renderWindow->getContext()) {
    if(!m_sphereBufferObject.initialized) {
      initializeSphere();
    }

    m_context = m_renderWindow->getContext();
    m_context->viewport = s2_camera();
    initializeMaterials();
    initializeConvexHulls();
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

  setMaterialTransforms(body);
  setCollisionObjectTransforms(body);
}


/*!
@brief initialize opengl properties for a s2_material_t datastructure

*/
void s2_Bodies::initializeMaterials() {

  for(int i = 0; i < m_context->materialCount; i++) {

    // m_programPhong->bind();
    s2_material_t* material = &m_context->materials[i];
    material_buffer_t* bufferObjects = new material_buffer_t;
    bufferObjects->vao = new QOpenGLVertexArrayObject(m_renderWindow);
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

    checkError("bodies: after create vertex buffers");

    bufferObjects->vao->bind();
    checkError("bodies: after bind vao");

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

    setMaterialAttributeLocations(
      m_programPhong->getProgram(),
      m_programPhong->getPositionAttribute(),
      m_programPhong->getNormalAttribute(),
      bufferObjects);
    checkError("bodies: after set attribute locations phong");

    setMaterialAttributeLocations(
      m_programFlat->getProgram(),
      m_programFlat->getPositionAttribute(),
      m_programFlat->getNormalAttribute(),
      bufferObjects);
    checkError("bodies: after set attribute locations flat");

    m_materialBufferObjects.append(bufferObjects);
    checkError("bodies: after append buffer objects");

    if(bufferObjects->material->GLuniforms.Kd[3] < 1.0) {
      m_translucentBufferObjects.append(bufferObjects);
    } else {
      m_opaqueBufferObjects.append(bufferObjects);
    }

    bufferObjects->vao->release();
    checkError("bodies: after release vao");
  }
}


void s2_Bodies::setMaterialTransforms(s2_body_t* body) {
  for(
    int i = body->materialIndex;
    i < body->materialIndex + body->materialCount;
    i++) {

    m_materialBufferObjects[i]->transforms.push_back(body->origin->dAbsolute);
    m_materialBufferObjects[i]->bodies.push_back(body);
  }
}


/*!
@brief initialize opengl properties for a s2_convexhull_t datastructure

*/
void s2_Bodies::initializeConvexHulls() {

  qDebug() << "convexhullCount " << m_context->convexhullCount;
  for(int i = 0; i < m_context->convexhullCount; i++) {

    s2_convexhull_t* convexhull = &m_context->convexhulls[i];

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

    qDebug() << "vertex count: " << convexhull->vertexCount;

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
}


/*!
@brief initialize collision objects

assigns transforms to collision buffer objects
*/
void s2_Bodies::setCollisionObjectTransforms(s2_body_t* body) {

  for(int i = 0; i < body->collisionObjectCount; i++) {
    s2_collisionobject_t* object = &body->collisionObjects[i];

    switch(object->type) {
      case S2_COLLISIONOBJECT_CONVEXHULL: {
        m_convexhullBufferObjects[object->index].transforms.push_back(
          body->origin->dAbsolute);
        m_convexhullBufferObjects[object->index].bodies.push_back(body);
        break;
      }
      case S2_COLLISIONOBJECT_SPHERE: {
        m_sphereBufferObject.sphereTransforms.push_back(&object->dAbsolute);
        m_sphereBufferObject.sphereBodies.push_back(body);
        break;
      }
      case S2_COLLISIONOBJECT_ELLIPSOID: {
        m_sphereBufferObject.ellipsoidTransforms.push_back(&object->dAbsolute);
        m_sphereBufferObject.ellipsoidBodies.push_back(body);
        break;
      }
      default: {
        break;
      }
    }
  }
}



/*!
@brief top level draw function
*/
void s2_Bodies::draw() {

  if(m_drawEnabled && m_renderWindow->isInitialized()) {

    glEnable(GL_DEPTH_TEST);
    if(m_shadingType == S2_SHADING_WIREFRAME) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if(m_drawNormalsEnabled) {
      m_programNormal->bind();
      checkError("bodies: after bind normal program");
      for(int i = 0; i < m_materialBufferObjects.size(); i++) {

        bool drawAnyEnabled = false;
        for(int j = 0; j < m_materialBufferObjects.at(i)->bodies.size(); j++) {
          if(m_materialBufferObjects.at(i)->bodies.at(j)->drawEnabled) {
            drawAnyEnabled = true;
            break;
          }
        }

        if(drawAnyEnabled) {
          m_materialBufferObjects.at(i)->vao->bind();
          checkError("bodies: after bind vao normal");

          m_programNormal->setScale(m_normalScale);
          checkError("bodies: after set transform normal shader");

          m_programNormal->setColor(
            m_normalColor[0], m_normalColor[1], m_normalColor[2]);

          for(
            int j = 0;
            j < m_materialBufferObjects.at(i)->transforms.size();
            j++) {

            if(m_materialBufferObjects.at(i)->bodies.at(j)->drawEnabled) {
              m_programNormal->setTransform(
                m_materialBufferObjects.at(i)->transforms.at(j));
              glDrawElements(
                GL_TRIANGLES,
                3 * m_materialBufferObjects.at(i)->material->faceCount,
                GL_UNSIGNED_INT,
                0);
              checkError("bodies: after draw elements normal");
            }
          }

          m_materialBufferObjects.at(i)->vao->release();
          checkError("bodies: after vao release normal");
        }
      }
      m_programNormal->release();
      checkError("bodies: after release normal program");
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

        /* draw opaque geometry */
        for(int i = 0; i < m_opaqueBufferObjects.size(); i++) {
          bool anyDrawEnabled = false;
          for(int j = 0; j < m_opaqueBufferObjects[i]->bodies.size(); j++) {
            if(
              m_opaqueBufferObjects[i]->bodies.at(j)->drawEnabled &&
              m_opaqueBufferObjects[i]->bodies.at(j)->isVisible) {
              anyDrawEnabled = true;
              break;
            }
          }

          if(anyDrawEnabled) {
            m_opaqueBufferObjects.at(i)->vao->bind();
            checkError("bodies: after bind vao phong");

            m_programPhong->setMaterial(m_opaqueBufferObjects.at(i)->material);
            checkError("bodies: after set material phong shader");

            for(int j = 0; j < m_opaqueBufferObjects.at(i)->transforms.size(); j++) {

              if(
                m_opaqueBufferObjects.at(i)->bodies.at(j)->drawEnabled &&
                m_opaqueBufferObjects.at(i)->bodies.at(j)->isVisible) {

                m_programPhong->setTransform(
                  m_opaqueBufferObjects.at(i)->transforms.at(j));
                checkError("bodies: after set transform phong shader");

                s2_stackstr_print(stdout, m_opaqueBufferObjects.at(i)->bodies.at(j)->name);
                s2_displacement_print(stdout, m_opaqueBufferObjects.at(i)->transforms.at(j));
                printf("\n");


                glDrawElements(
                  GL_TRIANGLES,
                  3 * m_opaqueBufferObjects.at(i)->material->faceCount,
                  GL_UNSIGNED_INT,
                  0);
                checkError("bodies: after draw elements phong shader");
              }
            }

            m_opaqueBufferObjects.at(i)->vao->release();
            // checkError("after vao release phong");
          }
        }

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        // glDisable(GL_CULL_FACE);


        /* draw translucent geometry */
        for(int i = 0; i < m_translucentBufferObjects.size(); i++) {

          bool anyDrawEnabled = false;
          for(int j = 0; j < m_translucentBufferObjects.at(i)->bodies.size(); j++) {
            if(
              m_translucentBufferObjects.at(i)->bodies.at(j)->drawEnabled &&
              m_translucentBufferObjects.at(i)->bodies.at(j)->isVisible) {
              anyDrawEnabled = true;
              break;
            }
          }

          if(anyDrawEnabled) {

            m_translucentBufferObjects.at(i)->vao->bind();
            checkError("bodies: after bind vao phong");

            m_programPhong->setMaterial(m_translucentBufferObjects.at(i)->material);
            checkError("bodies: after set material phong shader");

            for(
              int j = 0;
              j < m_translucentBufferObjects.at(i)->transforms.size();
              j++) {

              if(
                m_translucentBufferObjects.at(i)->bodies.at(j)->drawEnabled &&
                m_translucentBufferObjects.at(i)->bodies.at(j)->isVisible) {

                m_programPhong->setTransform(
                  m_translucentBufferObjects.at(i)->transforms.at(j));
                checkError("bodies: after set transform phong shader");

                glDrawElements(
                  GL_TRIANGLES,
                  3 * m_translucentBufferObjects.at(i)->material->faceCount,
                  GL_UNSIGNED_INT,
                  0);
                checkError("bodies: after draw elements phong shader");
              }
            }

            m_translucentBufferObjects.at(i)->vao->release();
            // checkError("after vao release phong");
          }
        }
        // glEnable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        m_programPhong->release();
        // checkError("after phong release phong");
      }
      break;
      case S2_SHADING_FLAT: {
        m_programFlat->bind();
        m_programFlat->setLights(m_lights);

        for(int i = 0; i < m_opaqueBufferObjects.size(); i++) {
          bool anyDrawEnabled = false;
          for(int j = 0; j < m_opaqueBufferObjects.at(i)->bodies.size(); j++) {
            if(
              m_opaqueBufferObjects.at(i)->bodies.at(j)->drawEnabled &&
              m_opaqueBufferObjects.at(i)->bodies.at(j)->isVisible) {
              anyDrawEnabled = true;
              break;
            }
          }
          if(anyDrawEnabled) {

            m_opaqueBufferObjects.at(i)->vao->bind();
            checkError("bodies: after bind vao flat");

            m_programFlat->setMaterial(m_opaqueBufferObjects.at(i)->material);
            checkError("bodies: after set material flat shader");

            for(
              int j = 0;
              j < m_opaqueBufferObjects.at(i)->transforms.size();
              j++) {

              if(
                m_opaqueBufferObjects.at(i)->bodies.at(j)->drawEnabled &&
                m_opaqueBufferObjects.at(i)->bodies.at(j)->isVisible) {

                m_programFlat->setTransform(
                  m_opaqueBufferObjects.at(i)->transforms.at(j));
                checkError("bodies: after set transform flat shader");

                glDrawElements(
                  GL_TRIANGLES,
                  3 * m_opaqueBufferObjects.at(i)->material->faceCount,
                  GL_UNSIGNED_INT,
                  0);
                checkError("bodies: after draw elements flat");
              }
            }

            m_opaqueBufferObjects.at(i)->vao->release();
            checkError("bodies: after vao release flat");
          }
        }

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        for(int i = 0; i < m_translucentBufferObjects.size(); i++) {
          bool anyDrawEnabled = false;
          for(int j = 0; j < m_translucentBufferObjects.at(i)->bodies.size(); j++) {
            if(
              m_translucentBufferObjects.at(i)->bodies.at(j)->drawEnabled &&
              m_translucentBufferObjects.at(i)->bodies.at(j)->isVisible) {
              anyDrawEnabled = true;
              break;
            }
          }
          if(anyDrawEnabled) {

            m_translucentBufferObjects.at(i)->vao->bind();
            checkError("bodies: after bind vao flat");

            m_programFlat->setMaterial(m_translucentBufferObjects.at(i)->material);
            checkError("bodies: after set material flat shader");

            for(
              int j = 0;
              j < m_translucentBufferObjects.at(i)->transforms.size();
              j++) {

              if(
                m_translucentBufferObjects.at(i)->bodies.at(j)->drawEnabled &&
                m_translucentBufferObjects.at(i)->bodies.at(j)->isVisible) {

                m_programFlat->setTransform(
                  m_translucentBufferObjects.at(i)->transforms.at(j));
                checkError("bodies: after set transform flat shader");

                glDrawElements(
                  GL_TRIANGLES,
                  3 * m_translucentBufferObjects.at(i)->material->faceCount,
                  GL_UNSIGNED_INT,
                  0);
                checkError("bodies: after draw elements flat");
              }
            }

            m_translucentBufferObjects.at(i)->vao->release();
            checkError("bodies: after vao release flat");
          }
        }
        glDisable(GL_BLEND);
        m_programFlat->release();
        checkError("bodies: after flat release flat");
      }
      break;
      default: {
      } break;
    }

    if(m_drawConvexHullEnabled) {
      m_programLine->bind();
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      for(int i = 0; i < m_convexhullBufferObjects.size(); i++) {
        bool drawAnyEnabled = false;
        for(
          int j = 0;
          j < m_convexhullBufferObjects.at(i).bodies.size();
          j++) {

          s2_body_t* body = m_convexhullBufferObjects.at(i).bodies.at(j);
          if(body->collisionObjectDrawEnabled) {
            drawAnyEnabled = true;
            break;
          }
        }

        if(drawAnyEnabled) {
          m_convexhullBufferObjects.at(i).vao->bind();
          checkError("bodies: after bind vao flat");

          m_programLine->setScale(1.f);
          checkError("bodies: after set material line shader");

          // qDebug() << "drawing " <<
          //   m_convexhullBufferObjects.at(i).transforms.size() << "convex hulls";

          for(
            int j = 0;
            j < m_convexhullBufferObjects.at(i).transforms.size();
            j++) {

            s2_body_t* body = m_convexhullBufferObjects.at(i).bodies.at(j);

            if(body->collisionObjectDrawEnabled) {
              GLfloat* color = body->collisionObjects[0].color;
              m_programLine->setColor(color[0], color[1], color[2], 1.f);

              // qDebug() << "color" << color[0] << color[1] << color[2];
              // s2_displacement_print(stdout, m_convexhullBufferObjects.at(i).transforms.at(j));

              m_programLine->setTransform(
                m_convexhullBufferObjects.at(i).transforms.at(j));
              checkError("bodies: after set transform line shader");

              glDrawElements(
                GL_TRIANGLES,
                3 * m_convexhullBufferObjects.at(i).convexhull->faceCount,
                GL_UNSIGNED_INT,
                0);
              checkError("bodies: after draw elements flat");
            }
          }

          m_convexhullBufferObjects.at(i).vao->release();
          checkError("bodies: after vao release flat");
        }
      }
      m_programLine->release();
    }

    // if(m_drawBoundingSpheresEnabled) {
    //   m_programPhong->bind();
    //   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    //   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //   glEnable(GL_BLEND);
    //   glEnable(GL_CULL_FACE);
    //
    //   m_sphere.vao->bind();
    //   checkError("after bind bounding sphere vao");
    //
    //   m_programPhong->setKd(m_sphere.Kd);
    //   m_programPhong->setKa(m_sphere.Ka);
    //   m_programPhong->setKs(m_sphere.Ks);
    //   m_programPhong->setNs(1000.0);
    //
    //   s2_displacement_t transform;
    //   s2_displacement_setIdentity(&transform);
    //
    //   for(int i = 0; i < m_context->bodyCount; i++) {
    //
    //     s2_sphere_t* sphere = &m_context->boundingSpheres[i];
    //     for(int i = 0; i < 3; i++) {
    //       s2_displacement_set(&transform, i, i, sphere->radius);
    //     }
    //     transform.T = sphere->centerGlobal;
    //
    //     // s2_displacement_print(stdout, &transform);
    //
    //     m_programPhong->setTransform(&transform);
    //     checkError("bounding spheres: after set transform");
    //
    //     glDrawElements(
    //       GL_TRIANGLES,
    //       3 * m_sphere.faceCount,
    //       GL_UNSIGNED_INT,
    //       0);
    //     checkError("bounding spheres: after draw elements");
    //   }
    //
    //   m_sphere.vao->release();
    //   glDisable(GL_BLEND);
    //   m_programPhong->release();
    // }
    //
    // if(m_shadingType == S2_SHADING_WIREFRAME) {
    //   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // } else {
    //   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // }
  }
}


/*!
@brief renders depth buffer in each light's frame
*/
void s2_Bodies::drawShadows() {

  if(m_drawEnabled && m_renderWindow->isInitialized()) {
    m_programShadow->bind();
    checkError("bodies: after bind shadow program");

    /* draw geometry */
    for(int i = 0; i < m_materialBufferObjects.size(); i++) {

      bool drawAnyEnabled = false;
      for(int j = 0; j < m_materialBufferObjects.at(i)->bodies.size(); j++) {
        s2_body_t* body = m_materialBufferObjects.at(i)->bodies.at(j);
        if(body->isVisible) {
          drawAnyEnabled = true;
          break;
        }
      }

      if(drawAnyEnabled) {

        m_materialBufferObjects.at(i)->vao->bind();
        checkError("bodies: after bind vao shadow");

        for(
          int j = 0;
          j < m_materialBufferObjects.at(i)->transforms.size();
          j++) {

          if(m_materialBufferObjects.at(i)->bodies.at(j)->isVisible) {

            m_programShadow->setTransform(
              m_materialBufferObjects.at(i)->transforms.at(j));
            checkError("bodies: after set transform shadow shader");

            glDrawElements(
              GL_TRIANGLES,
              3 * m_materialBufferObjects.at(i)->material->faceCount,
              GL_UNSIGNED_INT,
              0);
            checkError("bodies: after draw elements shadow shader");
          }
        }

        m_materialBufferObjects.at(i)->vao->release();
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
  m_opaqueBufferObjects.clear();
  m_translucentBufferObjects.clear();
  m_convexhullBufferObjects.clear();
}
