#include "renderwindow.h"
#include <QOpenGLDebugLogger>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QScreen>
#include <QMouseEvent>
#include <cmath>
#include <QFileDialog>
#include <QResizeEvent>
#include <iostream>

////////////////////
/// RenderWindow ///
////////////////////

RenderWindow::RenderWindow()
    : OpenGLWindow()
    //, m_program(0)
    , m_frame(0)
{
    setAnimating( true );

    //default background color
    m_color_bg[0]=0.0; 
    m_color_bg[1]=0.0; 
    m_color_bg[2]=0.0;

    // default object color
    // m_color_obj[0]=0.2; 
    // m_color_obj[1]=0.2; 
    // m_color_obj[2]=0.2;
    
    //initialize rotation, pan and camDistance variables
    m_xRot = 0; m_yRot = 0; m_zRot = 0;
    m_xPan = 0; m_yPan = 0; m_zPan = 0;
    m_x=0; m_y=0;
    m_camDistance = 2.0;

    m_perspective = true;
    m_shaderType = SPRITE;

    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;

    m_vertexMatrix = QMatrix4x4( 
        1/fx,     0,  0, 0,
        0,    -1/fy,  0, 0,
        0,       0,  0, a,
        -cx/fx, cy/fy, -1, b );

    m_rgbMatrix = QMatrix4x4(         
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02 );
}

///////////////////
// Mouse events ///
///////////////////
void RenderWindow::mouseMoveEvent( QMouseEvent *event )
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) 
    {
        setXRotation(m_xRot + 8 * dy);
        setYRotation(m_yRot + 8 * dx);
    } 
    else if (event->buttons() & Qt::RightButton) 
    {
        m_xPan += dx/120.0;
        m_yPan -= dy/120.0;
        updateGL();
    }

    m_lastPos = event->pos();
    m_x = event->pos().x();
    m_y = event->pos().y();
    emit mouseMove();
}

void RenderWindow::mousePressEvent( QMouseEvent *event )
{
    m_lastPos = event->pos();
    if(event->button() == Qt::RightButton)
    {
        emit mouseRightPressEvent();
    }

    if(event->button() == Qt::LeftButton)
    {
        emit mouseLeftPressEvent();
    }
}

void RenderWindow::mouseReleaseEvent(QMouseEvent *event)
{
    emit mouseReleaseEvent();
}

void RenderWindow::wheelEvent( QWheelEvent *event )
{
    m_camDistance += (event->delta() / 360.0);
    if (m_camDistance<0) m_camDistance = 0;
    updateGL();
}

// Miscellaneous methods
void RenderWindow::qNormalizeAngle( int &angle )
{
    while ( angle < 0 )
        angle += 360 * 16;
    while ( angle > 360 * 16 )
        angle -= 360 * 16;
}

QString RenderWindow::getCursorPositionText()
{
    QString str;
    QTextStream(&str) << "x,y=" << m_x << "," << m_y;
    //qDebug() << x << y << str;
    return str;
}

void RenderWindow::changeBackgroundColor( int red, int green, int blue )
{
    m_color_bg[0] = float( red )/255;
    m_color_bg[1] = float( green )/255;
    m_color_bg[2] = float( blue )/255;
}

//void RenderWindow::changeObjectColor( int red, int green, int blue, Model* model )
// void RenderWindow::changeObjectColor( int red, int green, int blue, Model* model )
// {   
//     for( int i=0; i<model->clen/3; i++ )
//     {
//         model->colors[3*i] = float( red )/255;
//         model->colors[3*i+1] = float( green )/255;
//         model->colors[3*i+2] = float( blue )/255;
//     }

//     qDebug() << model->name << "color changed";
//     //model->m_vbo->write( model->vlen*sizeof(GLfloat), model->colors, model->clen * sizeof(GLfloat) );
//     m_program->bind();    
//     model->m_vao->bind();    
//     model->m_vbo->bind();
//     model->m_vbo->write( model->vlen*sizeof(GLfloat), model->colors, model->clen * sizeof(GLfloat) );
//     model->m_vao->release();
//     m_program->release();
// }

// Slots
void RenderWindow::setXRotation( int angle )
{
    qNormalizeAngle( angle );
    if( angle != m_xRot ) 
    {
        m_xRot = angle;
        emit xRotationChanged( angle );
        updateGL();
    }
}

void RenderWindow::setYRotation( int angle )
{
    qNormalizeAngle( angle );
    if( angle != m_yRot ) 
    {
        m_yRot = angle;
        emit yRotationChanged( angle );
        updateGL();
    }
}

void RenderWindow::setZRotation( int angle )
{
    qNormalizeAngle(angle);
    if( angle != m_zRot ) 
    {
        m_zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}

void RenderWindow::setPerspective()
{
    m_perspective = true;
    updateGL();
}

void RenderWindow::setOrthogonal()
{
    m_perspective = false;
    updateGL();
}

void RenderWindow::loadFrame( Frame* frame )
{
    m_frameArray.append( frame );
}

//OpenGL Window
void RenderWindow::checkError( const QString &prefix )
{
    /// if OpenGL Logging is enabled or active
    /// then use that instead.

    if( isOpenGLLogging() ) 
    {
        return;
    }

    GLenum glErr = glGetError();

    while( glErr != GL_NO_ERROR ) 
    {
        QString error;
        switch ( glErr )
        {
        case GL_NO_ERROR:               error="NO_ERROR";               break;
        case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
        case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
        case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
        case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
        }

        if( error != "NO_ERROR" ) 
        {
            qDebug() << prefix << ":" << error;
        }
        glErr = glGetError();
    }
}

void RenderWindow::setShader( ShaderType shaderType )
{
    m_shaderType = shaderType;
}

void RenderWindow::setPixelView()
{
    m_shaderType = PIXEL;
}

void RenderWindow::setSpriteView()
{
    m_shaderType = SPRITE;
}

void RenderWindow::setPhongView()
{
    m_shaderType = PHONG;
}

void RenderWindow::initializeShader( QOpenGLShaderProgram * shaderProgram, ShaderAttributes * shaderAttributes, int vlen )
{
    // initialize attributes for pixel shader
    shaderAttributes->m_posAttr = shaderProgram->attributeLocation( "posAttr" );
    shaderAttributes->m_colAttr = shaderProgram->attributeLocation( "colAttr" );
    shaderAttributes->m_matrixUniform = shaderProgram->uniformLocation( "mvp" );
    shaderAttributes->m_mv = shaderProgram->uniformLocation( "mv" );
    
    shaderProgram->bind();
    shaderProgram->enableAttributeArray( shaderAttributes->m_posAttr );
    shaderProgram->setAttributeBuffer( shaderAttributes->m_posAttr, GL_FLOAT, 0, 3 );
    shaderProgram->enableAttributeArray( shaderAttributes->m_colAttr );
    shaderProgram->setAttributeBuffer( shaderAttributes->m_colAttr, GL_FLOAT, vlen * sizeof(GLfloat), 3 );
    shaderProgram->release();
}

void RenderWindow::initializeFrame( Frame * frame )
{

    m_program->bind();

    frame->m_vao = new QOpenGLVertexArrayObject( this );
    frame->m_vao->create();

    frame->m_vbo = new QOpenGLBuffer( QOpenGLBuffer::VertexBuffer );
    frame->m_vbo->create();

    frame->m_ibo = new QOpenGLBuffer( QOpenGLBuffer::IndexBuffer );
    frame->m_ibo->create();

    frame->m_vao->bind();
    frame->m_vbo->bind();
    frame->m_vbo->allocate((frame->vlen + frame->clen)*sizeof(GLfloat));
    frame->m_vbo->write( 0, frame->vertices, frame->vlen * sizeof(GLfloat) );
    frame->m_vbo->write( frame->vlen*sizeof(GLfloat), frame->colors, frame->clen * sizeof(GLfloat) );

    checkError("after vertex buffer allocation");

    frame->m_ibo->bind();
    frame->m_ibo->allocate(frame->indices, frame->ilen*sizeof(GLuint));

    checkError("after index buffer allocation");

    m_program->release();

    /// initialize shaders
    frame->pixelShaderAttributes = new ShaderAttributes();
    frame->spriteShaderAttributes = new ShaderAttributes();
    frame->phongShaderAttributes = new ShaderAttributes();
    
    initializeShader( m_program, frame->pixelShaderAttributes, frame->vlen );
    initializeShader( m_programSprite, frame->spriteShaderAttributes, frame->vlen );
    initializeShader( m_programPhong, frame->phongShaderAttributes, frame->vlen );

    frame->phongShaderAttributes->m_v = m_programPhong->uniformLocation( "v" );
    frame->m_vao->release();

    qDebug() << "frame initialized";
    frame->initialized = true;
}

void RenderWindow::initialize()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor( m_color_bg[0], m_color_bg[1], m_color_bg[2], 0.0f );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_PROGRAM_POINT_SIZE );

    m_program = new QOpenGLShaderProgram( this );
    m_program->addShaderFromSourceFile( QOpenGLShader::Vertex, ":/shader/simple.vert" );
    m_program->addShaderFromSourceFile( QOpenGLShader::Fragment, ":/shader/simple.frag" );
    m_program->link();

    m_programSprite = new QOpenGLShaderProgram( this );
    m_programSprite->addShaderFromSourceFile( QOpenGLShader::Vertex, ":/shader/sprite.vert" );
    m_programSprite->addShaderFromSourceFile( QOpenGLShader::Fragment, ":/shader/sprite.frag" );
    m_programSprite->link();

    m_programPhong = new QOpenGLShaderProgram( this );
    m_programPhong->addShaderFromSourceFile( QOpenGLShader::Vertex, ":/shader/phong.vert" );
    m_programPhong->addShaderFromSourceFile( QOpenGLShader::Fragment, ":/shader/phong.frag" );
    m_programPhong->link();

    m_normalTexture = new QOpenGLTexture( QImage(":/shader/sphere_normal_map.png") );
    m_normalTexture->setWrapMode(QOpenGLTexture::ClampToEdge);
    m_normalTexture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);

    m_sphereTexture = new QOpenGLTexture( QImage(":/shader/sphere.png") );
    m_sphereTexture->setWrapMode(QOpenGLTexture::ClampToEdge);
    m_sphereTexture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);

    /// bind textures
    m_programSprite->bind();
    m_sphereTexture->bind();
    m_programSprite->setUniformValue("tex", 0);
    m_sphereTexture->release();
    m_programSprite->release();

    m_programPhong->bind();
    m_normalTexture->bind();
    m_programPhong->setUniformValue("tex", 0);
    m_normalTexture->release();
    m_programPhong->release();

    checkError( "state initialized" );
    qDebug() << "shaders initialized" ;
    updateGL();
}

void RenderWindow::updateGL()
{
    /// View Matrix
    /// Needs to be called on mouse events AND screen resize
    m_view.setToIdentity();

    QVector3D eye( -m_xPan, -m_yPan, -m_zPan + m_camDistance );
    QVector3D center( -m_xPan, -m_yPan, -m_zPan );
    QVector3D up( 0, 1, 0 );

    m_view.lookAt( eye, center, up );
    m_view.rotate( m_xRot / 16.0, 1.0, 0.0, 0.0 );
    m_view.rotate( m_yRot / 16.0, 0.0, 1.0, 0.0 );

    /// Projection matrix
    /// projection only needs update when user changes perspective or when screen is resized
    m_projection.setToIdentity();

    const qreal retinaScale = devicePixelRatio();
    glViewport( 0, 0, width() * retinaScale, height() * retinaScale );
    float aspectRatio = (float)width()/(float)height();
    
    if ( m_perspective )
    {
        m_projection.perspective( 60, (float)width()/(float)height(), 0.1, 10000 );
    }
    else
    {
        float h = 0.5 * m_camDistance;
        float w = aspectRatio * h;
        m_projection.ortho ( -w,w,-h,h, 0.01, 10000 );
    }
}

void RenderWindow::render()
{
    ///////////////////////////////////////////////////////////////////////////////
    ///  set up projection/view again, only necessary if size of screen changes
    ///  or user changes the projection type (orthogonal or perspective)
    ///  for better performance, this call should be moved to a resize function
    ///  i tried to implement this, but the program segfaults when i  
    ///  reimplement resizeEvent in either this file or in openglwindow.cpp.
    ///  try to figure this out later, meanwhile, call:

    updateGL();

    /// ...at every time step.
    /////////////////////////////////////////////////////////////////////////

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor( m_color_bg[0], m_color_bg[1], m_color_bg[2], 0.0f );

    /// animate 
    //m_frameArray[0]->transform.rotate( 10.0f / screen()->refreshRate(), 0, 1, 0 ); /// rotate if animation is on.
    
    QMatrix4x4 projection_view = m_projection * m_view;
    for( int i=0; i<m_frameArray.count(); i++ )
    {
        if(!(m_frameArray[i]->initialized))
        {
            initializeFrame( m_frameArray[i] );
        }
        switch( m_shaderType )
        {
            case PIXEL:
            {
                m_program->bind();
                m_program->setUniformValue( m_frameArray[i]->pixelShaderAttributes->m_matrixUniform, projection_view * m_frameArray[i]->transform );
                m_program->setUniformValue( m_frameArray[i]->pixelShaderAttributes->m_mv, m_view * m_frameArray[i]->transform );
                m_frameArray[i]->m_vao->bind();
                glDrawElements( GL_POINTS, m_frameArray[i]->ilen, GL_UNSIGNED_INT, 0 );
                m_frameArray[i]->m_vao->release();
                m_program->release();
                checkError("after program release");
            }
            break;
            
            case SPRITE:
            {
                //qDebug() << "Rendering sprite";
                m_programSprite->bind();
                m_sphereTexture->bind();
                m_programSprite->setUniformValue( m_frameArray[i]->spriteShaderAttributes->m_matrixUniform, projection_view * m_frameArray[i]->transform );
                m_programSprite->setUniformValue( m_frameArray[i]->spriteShaderAttributes->m_mv, m_view * m_frameArray[i]->transform );
                m_frameArray[i]->m_vao->bind();

                glDrawElements( GL_POINTS, m_frameArray[i]->ilen, GL_UNSIGNED_INT, 0 );

                m_frameArray[i]->m_vao->release();
                m_sphereTexture->release();
                m_programSprite->release();
                checkError("after program release");
            }
            break;
            
            case PHONG:
            {
                                //qDebug() << "Rendering Phong";
                m_programPhong->bind();
                m_normalTexture->bind();
                m_programPhong->setUniformValue( m_frameArray[i]->phongShaderAttributes->m_matrixUniform, projection_view * m_frameArray[i]->transform );
                m_programPhong->setUniformValue( m_frameArray[i]->phongShaderAttributes->m_mv, m_view * m_frameArray[i]->transform );
                m_programPhong->setUniformValue( m_frameArray[i]->phongShaderAttributes->m_v, m_view );
                m_frameArray[i]->m_vao->bind();

                glDrawElements( GL_POINTS, m_frameArray[i]->ilen, GL_UNSIGNED_INT, 0 );

                m_frameArray[i]->m_vao->release();
                m_normalTexture->release();
                m_programPhong->release();
                                        checkError("after program release");
            }
            break;

        }


        checkError("after program release");

    }

    if( isAnimating() ) 
    {
        ++m_frame;
    }
}


