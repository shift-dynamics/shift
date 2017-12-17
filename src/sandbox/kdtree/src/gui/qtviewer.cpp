#include <QtGui>
#include <QPushButton>
#include <QFileDialog>
#include <QColorDialog>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QStackedLayout>
#include <QMessageBox>
#include <QLabel>
#include <QMargins>
#include <QMenuBar>
#include <QStatusBar>
#include <QKeySequence>
#include <QTreeWidget>
#include <QComboBox>
#include <QResizeEvent>
#include <QStatusBar>

#if defined(ENABLE_QTSCRIPT)
#include <QtScript>
#include <QScriptEngine>
#endif

#include <iostream>
#include <string>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

#include <string>
#include <vector>
#include <map>

#include <stdlib.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <cxcore.h>
#include <highgui.h>

#include "qtviewer.h"
#include "renderwindow.h"
#include "tiny_obj_loader.h"
#include "matrixToEulerAngles.h"

qtviewer::qtviewer(QWidget* parent) : QMainWindow(parent) {
  //currentSelectedModel = NULL;
  //currentFileName = "default.json";
  createMenus();
  layout()->setContentsMargins(0, 0, 0, 0);
  layout()->setMargin(0);
}

qtviewer::~qtviewer() {}

void qtviewer::createMenus() {
  ///////////////
  /// MenuBar ///
  ///////////////

  setWindowTitle(tr("Kinect Point Cloud Viewer"));

  /*
      File Menu
  */
  QMenu* fileMenu = menuBar()->addMenu("&File");

  // save dialog
  // QAction *actionOpen = new QAction("Open Scene...", this);
  // actionOpen->setShortcut(QKeySequence::Open);
  // fileMenu->addAction( actionOpen );

  // QAction *actionSave = new QAction("Save Scene", this);
  // actionSave->setShortcut(QKeySequence::Save);
  // fileMenu->addAction( actionSave );

  // QAction *actionSaveAs = new QAction("Save Scene As..", this);
  // actionSaveAs->setShortcut(QKeySequence::SaveAs);
  // fileMenu->addAction( actionSaveAs );

  //fileMenu->addSeparator();

  // add quit dialog
  QAction* actionQuit = new QAction("Quit", this);
  actionQuit->setShortcut(QKeySequence::Quit);
  fileMenu->addAction(actionQuit);

  /*
      View Menu
  */
  QMenu* viewMenu = menuBar()->addMenu("&View");

  QMenu* projectionMenu = viewMenu->addMenu("&Camera Projection");
  QActionGroup* perspectiveGroup = new QActionGroup(this);

  QAction* actionSetOrthogonal = new QAction("Orthogonal", this);
  actionSetOrthogonal->setCheckable(true);
  perspectiveGroup->addAction(actionSetOrthogonal);

  projectionMenu->addAction(actionSetOrthogonal);

  QAction* actionSetPerspective = new QAction("Perspective", this);
  projectionMenu->addAction(actionSetPerspective);
  actionSetPerspective->setCheckable(true);
  actionSetPerspective->setChecked(true);
  perspectiveGroup->addAction(actionSetPerspective);

  viewMenu->addSeparator();

  // point display menu items
  QActionGroup* pointDisplayGroup = new QActionGroup(this);

  QMenu* pointDisplayMenu = viewMenu->addMenu("&Point Display");

  QAction* actionSetPixelView = new QAction("Single Pixel", this);
  pointDisplayGroup->addAction(actionSetPixelView);
  actionSetPixelView->setCheckable(true);
  pointDisplayMenu->addAction(actionSetPixelView);

  QAction* actionSetSpriteView = new QAction("Scaled Sprites", this);
  pointDisplayGroup->addAction(actionSetSpriteView);
  actionSetSpriteView->setCheckable(true);
  actionSetSpriteView->setChecked(true);
  pointDisplayMenu->addAction(actionSetSpriteView);

  QAction* actionSetPhongView = new QAction("Phong Shaded Sprites", this);
  pointDisplayGroup->addAction(actionSetPhongView);
  actionSetPhongView->setCheckable(true);
  pointDisplayMenu->addAction(actionSetPhongView);

  /*
      Help Menu
  */
  QMenu* helpMenu = menuBar()->addMenu("&Help");

  QAction* actionAbout = new QAction("About", this);
  helpMenu->addAction(actionAbout);

  QAction* actionControls = new QAction("Controls", this);
  helpMenu->addAction(actionControls);

  ////////////////////
  /// Window Layout///
  ////////////////////

  // main window
  QGroupBox* mainBox = new QGroupBox;
  mainBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  mainBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QStackedLayout* mainLayout = new QStackedLayout(mainBox);
  mainLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  mainLayout->setMargin(0);
  mainLayout->setSpacing(0);
  mainBox->resize(800, 600);

  // GL viewport
  glwidget = new RenderWindowWidget(mainBox);
  glwidget->setMinimumHeight(200);
  glwidget->setMinimumWidth(200);
  glwidget->setContentsMargins(QMargins(0, 0, 0, 0));

  mainLayout->addWidget(glwidget);

  RenderWindow* renWin = glwidget->GetRenderWindow();
  renWin->loadFrame(readFrameFromFile(":/scene/depth0.png", ":/scene/rgb0.png"));

  setCentralWidget(mainBox);

  /////////////////////////
  /// Signals and Slots ///
  /////////////////////////
  // buttons
  QObject::connect(qApp, SIGNAL(resizeEvent()), renWin, SLOT(updateGL()));
  // QObject::connect( backgroundColorButton, SIGNAL( clicked() ), this, SLOT( backgroundColorSelector() ) );
  // QObject::connect( objectColorButton, SIGNAL( clicked() ), this, SLOT( objectColorSelector() ) );

  // cursor dialog, lower left
  QObject::connect(renWin, SIGNAL(mouseMove()), this, SLOT(updateCursorPositionText()));
  QObject::connect(glwidget, SIGNAL(mouseOut()), this, SLOT(clearCursorPositionText()));

  // file menu actions
  // QObject::connect( actionOpen, SIGNAL( triggered() ), this, SLOT( openFile() ) );

  // QObject::connect( actionSave, SIGNAL( triggered() ), this, SLOT( saveFile()) );
  // QObject::connect( actionSaveAs, SIGNAL( triggered() ), this, SLOT( saveFileAs()) );
  QObject::connect(actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  // view menu actions
  QObject::connect(actionSetPerspective, SIGNAL(triggered()), renWin, SLOT(setPerspective()));
  QObject::connect(actionSetOrthogonal, SIGNAL(triggered()), renWin, SLOT(setOrthogonal()));
  QObject::connect(actionSetPixelView, SIGNAL(triggered()), renWin, SLOT(setPixelView()));
  QObject::connect(actionSetSpriteView, SIGNAL(triggered()), renWin, SLOT(setSpriteView()));
  QObject::connect(actionSetPhongView, SIGNAL(triggered()), renWin, SLOT(setPhongView()));


  // help menu actions
  QObject::connect(actionAbout, SIGNAL(triggered()), this, SLOT(about()));
  QObject::connect(actionControls, SIGNAL(triggered()), this, SLOT(controls()));

}


void qtviewer::updateTransform() {
  // currentSelectedModel->transform.setToIdentity();
  // currentSelectedModel->transform.rotate( currentSelectedModel->rotation.x(), 1, 0, 0);
  // currentSelectedModel->transform.rotate( currentSelectedModel->rotation.y(), 0, 1, 0);
  // currentSelectedModel->transform.rotate( currentSelectedModel->rotation.z(), 0, 0, 1);
  // currentSelectedModel->transform.translate( currentSelectedModel->translation );
  // currentSelectedModel->transform.scale( currentSelectedModel->s );
}

// void qtviewer::setRx( int val)
// {
//     if(currentSelectedModel!=NULL)
//     {
//         currentSelectedModel->rotation.setX( val/16.0f);
//         qDebug() << currentSelectedModel->transform;
//         updateTransform();
//     }
//     emit rxChanged( 0 );
// }


// void qtviewer::setRy( int val )
// {
//     if(currentSelectedModel!=NULL)
//     {
//         currentSelectedModel->rotation.setY( val/16.0f);
//         qDebug() << currentSelectedModel->transform;
//         updateTransform();
//     }
//     emit ryChanged( 0 );
// }


// void qtviewer::setRz( int val)
// {
//     if(currentSelectedModel!=NULL)
//     {
//         currentSelectedModel->rotation.setZ( val/16.0f);
//         qDebug() << currentSelectedModel->transform;
//         updateTransform();
//     }
//     emit rzChanged( 0 );
// }


// void qtviewer::setTx( int val )
// {
//     if(currentSelectedModel!=NULL)
//     {
//         currentSelectedModel->translation.setX( val/10.0f );
//         qDebug() << currentSelectedModel->transform;
//         updateTransform();
//     }
//     emit txChanged( 0 );
// }


// void qtviewer::setTy( int val )
// {
//     if(currentSelectedModel!=NULL)
//     {
//         currentSelectedModel->translation.setY( val/10.0f );
//         qDebug() << currentSelectedModel->transform;
//         updateTransform();
//     }
//     emit tyChanged( 0 );
// }


// void qtviewer::setTz( int val )
// {
//     if(currentSelectedModel!=NULL)
//     {
//         currentSelectedModel->translation.setZ( val/10.0f );
//         qDebug() << currentSelectedModel->transform;
//         updateTransform();
//     }
//     emit tzChanged( 0 );
// }


// void qtviewer::setS( int val )
// {
//     if( currentSelectedModel!=NULL )
//     {
//         currentSelectedModel->s = val/10.0f;
//         qDebug() << currentSelectedModel->transform;
//         updateTransform();
//     }
//     emit sChanged( 0 );
// }


// void qtviewer::getSelectedTreeItem()
// {
//     ModelTreeItem* item = static_cast<ModelTreeItem*>( treeWidget->currentItem() );
//     qDebug() << item->UserType << item->text(0);

//     currentSelectedModel = item->model;

//     if(item->model!=NULL)
//     {
//         qDebug() << currentSelectedModel->name;
//         rxSlider->setValue(currentSelectedModel->rotation.x()*16.0f);
//         rySlider->setValue(currentSelectedModel->rotation.y()*16.0f);
//         rzSlider->setValue(currentSelectedModel->rotation.z()*16.0f);
//         txSlider->setValue(currentSelectedModel->translation.x()*10.0f);
//         tySlider->setValue(currentSelectedModel->translation.y()*10.0f);
//         tzSlider->setValue(currentSelectedModel->translation.z()*10.0f);
//         scaleSlider->setValue(currentSelectedModel->s*10.0f);
//     }
// }

// Tried implementing resize event as shown below (and variations thereof )
// to initiate a resizeGL function. Cannot get it to work, try to fix this later
// void qtviewer::resizeEvent( QResizeEvent * event )
// {
//     emit resized();
// }

void qtviewer::updateCursorPositionText() {
  RenderWindow* renWin = glwidget->GetRenderWindow();
  QString positionText = renWin->getCursorPositionText();
  this->statusBar()->showMessage(positionText);
  //qDebug() << positionText;
  //cursorPositionLabel->setText( positionText );

}


void qtviewer::clearCursorPositionText() {
  this->statusBar()->showMessage("");
}


void qtviewer::openFile() {
//     RenderWindow* renWin = glwidget->GetRenderWindow();
//     renWin->setAnimating(false);
//     QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
//                                                  "",
//                                                  tr("JSON Files (*.json)"));
//     //QMessageBox::information( this, "File Selected", fileName.length() == 0 ? "No File Selected" : fileName );
//     if(fileName.length()!=0)
//     {
//         currentFileName = fileName;
//         modelArray.clear();
//         sceneRead( currentFileName, scenes );
//         populateTree();
//         renWin->loadModelArray( modelArray );
//         renWin->initialize();
//     }
//     renWin->setAnimating(true);

}


// void qtviewer::populateTree()
// {
//     treeWidget->clear();

//     for(int i = 0; i < scenes.size(); ++i)
//     {
//         //QTreeWidgetItem * item = new QTreeWidgetItem((QTreeWidget*)0, QStringList( scenes[i]->name ), 0);
//         ModelTreeItem* item = new ModelTreeItem( 1001 );
//         item->setText( 0, scenes[i]->name );
//         treeWidget->addTopLevelItem( item );
//         for(int j = 0; j < scenes[i]->root.size(); ++j)
//         {
//             qDebug() << scenes[i]->root[j]->type;
//             if (scenes[i]->root[j]->type == "model")
//             {
//                 Model* m = static_cast<Model*>(scenes[i]->root[j]->children[0]);
//                 qDebug() << m->name;
//                 ModelTreeItem* subItem = new ModelTreeItem( 1002 );
//                 subItem->setText( 0, m->name );
//                 subItem->model = m;
//                 item->addChild( subItem );
//             }
//         }
//     }
// }

void qtviewer::about() {
  QString fileContents;
  QFile file(":/about/about.md");
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();
  QMessageBox::about(this, "About", fileContents);
}

void qtviewer::controls() {
  QString fileContents;
  QFile file(":/about/controls.md");
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();
  QMessageBox::about(this, "Controls", fileContents);
}

// void qtviewer::saveFile()
// {
//     sceneWrite( currentFileName, scenes );
// }


// void qtviewer::saveFileAs()
// {
//     RenderWindow* renWin = glwidget->GetRenderWindow();
//     renWin->setAnimating(false);
//     QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
//                                                  "",
//                                                 tr("JSON Files (*.json)"));
//     //QMessageBox::information( this, "File Selected", fileName.length() == 0 ? "No File Selected" : fileName );
//     renWin->setAnimating(true);
//     sceneWrite( fileName, scenes );
// }


// void qtviewer::backgroundColorSelector()
// {
//     RenderWindow* renWin = glwidget->GetRenderWindow();
//     renWin->setAnimating( false );
//     QColor color = QColorDialog::getColor( Qt::black, this, "Select Color", QColorDialog::DontUseNativeDialog );
//     if( color.isValid() )
//     {
//         qDebug( "ok r:%i g:%i b:%i\n", color.red(), color.green(), color.blue() );
//         renWin->changeBackgroundColor( color.red(), color.green(), color.blue() );
//     }
//     renWin->setAnimating( true );
// }


// void qtviewer::objectColorSelector()
// {
//     RenderWindow* renWin = glwidget->GetRenderWindow();
//     renWin->setAnimating( false );

//     if( currentSelectedModel==NULL )
//     {
//         QMessageBox::information( this, "Alert", "No object was selected. Select an object from the model tree." );
//     }
//     else
//     {
//         QColor color = QColorDialog::getColor( Qt::white, this, "Select Color", QColorDialog::DontUseNativeDialog );
//         if( color.isValid() )
//         {
//             qDebug() << currentSelectedModel->name;
//             qDebug( "ok r:%i g:%i b:%i\n", color.red(), color.green(), color.blue() );
//             renWin->changeObjectColor( color.red(), color.green(), color.blue(), currentSelectedModel );
//         }
//     }
//         renWin->setAnimating( true );
// }


// QSlider* qtviewer::createSlider()
// {
//     QSlider *slider = new QSlider(Qt::Horizontal);
//     slider->setRange(-180 *16, 180 * 16);
//     slider->setSingleStep(16);
//     slider->setPageStep(15 * 16);
//     slider->setTickInterval(15 * 16);
//     slider->setTickPosition(QSlider::TicksAbove);
//     slider->setContentsMargins( QMargins(0, 0, 0, 0 ) );
//     slider->setValue( 0 * 16 );
//     return slider;
// }


// Model* qtviewer::parseModel(const QJsonObject& model)
// {
//     Model* out = new Model();

//     out->name = model["name"].toString();
//     out->fileName = model["filename"].toString();
//     QJsonArray jmatrix = model["matrix"].toArray();

//     if( jmatrix.size() != 16 )
//     {
//         qDebug() << "name: " << out->name
//                  << " file: " << out->fileName
//                  << " matrix length: " << jmatrix.size()
//                  << " not 16";
//         out->transform.setToIdentity();
//     }
//     else
//     {
//         float dlist[16];
//         for(int i = 0; i < jmatrix.size(); ++i)
//         {
//             dlist[i] = (float)jmatrix[i].toDouble();
//         }
//         out->transform = QMatrix4x4( dlist );
//     }

//     //extract uniform scalar term
//     out->s = ExtractScalar( out->transform );

//     //extract translation
//     out->translation = ExtractTranslation( out->transform );

//     QMatrix4x4 temp = out->transform;
//     temp.scale( 1/out->s );
//     ExtractEulerAngles( temp, &(out->rotation) );

//     qDebug() << "\n###################\n name: " << out->name;
//     qDebug() << " file: " << out->fileName;
//     qDebug() << " initial transform loaded:  \n" << out->transform;
//     qDebug() << " cardan angles: " << out->rotation;
//     qDebug() << " transform with uniform scaling removed: \n" << temp;
//     qDebug() << " scale: " << out->s;
//     qDebug() << " translation: " << out->translation;

//     readOBJ( out );
//     currentSelectedModel = out;
//     updateTransform();
//     return out;
// }


// void qtviewer::readOBJ( Model* model )
// {
//     /// Modified from tiny obj loader example to
//     /// convert tinyobj object to my "Model" datastructure
//     std::string inputfile = model->fileName.toUtf8().constData();
//     std::vector<tinyobj::shape_t> shapes;
//     std::vector<tinyobj::material_t> materials;

//     std::string err = tinyobj::LoadObj( shapes, materials, inputfile.c_str() );

//     if( !err.empty() )
//     {
//         std::cerr << err << std::endl;
//         std::cout << "no OBJ file specified!";
//     }

//     else
//     {

//         // get total number of indices, vertices from all shapes
//         size_t ilen = 0;
//         size_t vlen = 0;

//         for( size_t i=0; i<shapes.size(); i++ )
//         {
//             ilen += shapes[i].mesh.indices.size();
//             vlen += shapes[i].mesh.positions.size();
//         }

//         model->ilen = (int) ilen;
//         model->vlen = (int) vlen;

//         std::cout << "# of shapes    : " << shapes.size() << std::endl;
//         std::cout << "# of materials : " << materials.size() << std::endl;
//         std::cout << "# of vertices : " << vlen/3 << std::endl;
//         std::cout << "# of indices : " << ilen/3 << std::endl;

//         int j=0;
//         model->faces = new GLuint[model->ilen];
//         for( int i=0; i<(int)shapes.size(); i++ )
//         {
//             for( int k=0; k<(int)shapes[i].mesh.indices.size(); k++ )
//             {
//                 model->faces[j++] = (GLuint)(shapes[i].mesh.indices[k]);
//             }
//         }

//         j=0;
//         model->vertices = new GLfloat[model->vlen];
//         for( size_t i=0; i<shapes.size(); i++ )
//         {
//             for( int k=0; k<(int)shapes[i].mesh.positions.size(); k++ )
//             {
//                 model->vertices[j++] = (GLfloat)(shapes[i].mesh.positions[k]);
//             }
//         }

//         model->clen = model->ilen;
//         model->colors = new GLfloat[model->clen];

//         int r = rand() % 100;
//         int g = rand() % 100;
//         int b = rand() % 100;

//         for( int i=0; i<model->clen/3; i++ )
//         {
//             model->colors[i*3] = r/100.0f;
//             model->colors[i*3+1] = g/100.0f;
//             model->colors[i*3+2] = b/100.0f;
//         }

//         /// print debugging statements only if model is below a certain size, otherwise large models take a
//         /// long time to load: verify input from obj loader

//         if( model->vlen<100)
//         {
//             for ( unsigned int i = 0; i<shapes.size(); i++ )
//             {
//                 for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++)
//                 {
//                     qDebug("  idx[%ld] = %d, %d, %d. mat_id = %d\n",
//                               f,
//                               shapes[i].mesh.indices[3*f+0], shapes[i].mesh.indices[3*f+1],
//                               shapes[i].mesh.indices[3*f+2], shapes[i].mesh.material_ids[f] );
//                 }

//                 qDebug("shape[i].vertices: %ld\n", shapes[i].mesh.positions.size());

//                 for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++)
//                 {
//                     qDebug("  v[%ld] = (%f, %f, %f)\n", v,
//                     shapes[i].mesh.positions[3*v+0],
//                     shapes[i].mesh.positions[3*v+1],
//                     shapes[i].mesh.positions[3*v+2]);
//                 }
//             }

//             // verify that whats loaded in to model data structure is the same as what
//             // tinyobjloader loaded
//             for( int i = 0; i<model->ilen/3; i++ )
//             {
//                 qDebug("indice[%d]: (%d, %d, %d)", i, model->faces[i*3], model->faces[i*3+1], model->faces[i*3+2]);
//             }

//             for( int i = 0; i<model->vlen/3; i++ )
//             {
//                 qDebug("vertex[%d]: (%f, %f, %f)", i, model->vertices[i*3], model->vertices[i*3+1], model->vertices[i*3+2]);
//             }
//         }

//         // materials currently not used
//         // for (size_t i = 0; i < materials.size(); i++)
//         // {
//         //   qDebug("material[%ld].name = %s\n", i, materials[i].name.c_str());
//         //   qDebug("  material.Ka = (%f, %f ,%f)\n", materials[i].ambient[0], materials[i].ambient[1], materials[i].ambient[2]);
//         //   qDebug("  material.Kd = (%f, %f ,%f)\n", materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]);
//         //   qDebug("  material.Ks = (%f, %f ,%f)\n", materials[i].specular[0], materials[i].specular[1], materials[i].specular[2]);
//         //   qDebug("  material.Tr = (%f, %f ,%f)\n", materials[i].transmittance[0], materials[i].transmittance[1], materials[i].transmittance[2]);
//         //   qDebug("  material.Ke = (%f, %f ,%f)\n", materials[i].emission[0], materials[i].emission[1], materials[i].emission[2]);
//         //   qDebug("  material.Ns = %f\n", materials[i].shininess);
//         //   qDebug("  material.Ni = %f\n", materials[i].ior);
//         //   qDebug("  material.dissolve = %f\n", materials[i].dissolve);
//         //   qDebug("  material.illum = %d\n", materials[i].illum);
//         //   qDebug("  material.map_Ka = %s\n", materials[i].ambient_texname.c_str());
//         //   qDebug("  material.map_Kd = %s\n", materials[i].diffuse_texname.c_str());
//         //   qDebug("  material.map_Ks = %s\n", materials[i].specular_texname.c_str());
//         //   qDebug("  material.map_Ns = %s\n", materials[i].normal_texname.c_str());
//         //   std::map<std::string, std::string>::const_iterator it(materials[i].unknown_parameter.begin());
//         //   std::map<std::string, std::string>::const_iterator itEnd(materials[i].unknown_parameter.end());
//         //   for (; it != itEnd; it++)
//         //   {
//         //     qDebug("  material.%s = %s\n", it->first.c_str(), it->second.c_str());
//         //   }
//         //   qDebug("\n");
//         // }
//         modelArray.append( model );
//     }
// }


// Node* qtviewer::parseNode( const QJsonObject& parentNode )
// {
//     Node* out = new Node();
//     out->type = parentNode["type"].toString();

//     /// detected a model
//     if(out->type == "model")
//     {
//         out->children.push_back( parseModel(parentNode) );
//     }
//     else if(out->type == "node")
//     {

//         QJsonArray nodeArray = parentNode["children"].toArray();
//         for(int i = 0; i < nodeArray.size(); ++i)
//         {
//             out->children.push_back(parseNode(nodeArray[i].toObject()));
//         }
//     }
//     return out;
// }




// void qtviewer::sceneRead( const QString &filename, QVector<Scene*>& scenes )
// {
//     scenes.clear();
//     QString fileContents;

//     QFile file( filename );

//     file.open(QIODevice::ReadOnly | QIODevice::Text);
//     fileContents = file.readAll();
//     file.close();

//     QJsonDocument doc = QJsonDocument::fromJson(fileContents.toUtf8());

//     QJsonArray sceneList = doc.array();

//     for( int i = 0; i < sceneList.size(); ++i )
//     {
//         Scene* scene = new Scene();

//         QJsonObject object = sceneList[i].toObject();
//         scene->name = object["name"].toString();
//         scene->description = object["description"].toString();

//         QJsonArray rootNode = object["root"].toArray();

//         qDebug() << "\n\nscene: " << i << " " << scene->name << " " << scene->description;

//         for(int i = 0; i < rootNode.size(); ++i)
//         {
//             scene->root.push_back(parseNode(rootNode[i].toObject()));
//         }
//         scenes.push_back(scene);
//     }
// }


// void qtviewer::sceneReadFromStr( const QString &fileContents, QVector<Scene*>& scenes )
// {
//     QJsonDocument doc = QJsonDocument::fromJson(fileContents.toUtf8());

//     QJsonArray sceneList = doc.array();

//     modelArray.clear();

//     for( int i = 0; i < sceneList.size(); ++i ) {

//         Scene* scene = new Scene();

//         QJsonObject object = sceneList[i].toObject();
//         scene->name = object["name"].toString();
//         scene->description = object["description"].toString();

//         QJsonArray rootNode = object["root"].toArray();

//         qDebug() << "scene: " << i << " " << scene->name << " " << scene->description;

//         for(int i = 0; i < rootNode.size(); ++i)
//         {
//             scene->root.push_back( parseNode(rootNode[i].toObject()) );
//         }
//         scenes.push_back( scene );
//     }
// }

Frame* qtviewer::readFrameFromFile(const QString& depthFileName, const QString& rgbFileName) {
  Frame* frame = new Frame();
  frame->transform.setToIdentity();

  cv::Mat depthMat;//(cv::Size(640,480),CV_16UC1);
  depthMat = cv::imread("../scene/fulldepth0.png", CV_16UC1);
  //QImage depthMap = QImage(":/scene/fulldepth0.png");
  QImage rgb = QImage(":/scene/rgb0.png");

  float fx = 594.21f;
  float fy = 591.04f;
  float a = -0.0030711f;
  float b = 3.3309495f;
  float cx = 339.5f;
  float cy = 242.7f;

  QMatrix4x4 m_vertexMatrix = QMatrix4x4(
                                1 / fx,     0,  0, 0,
                                0,    -1 / fy,  0, 0,
                                0,       0,  0, a,
                                -cx / fx, cy / fy, -1, b);


  //count vertices
  frame->ilen = 0;
  for(int i = 0; i < 480; i++) {
    for(int j = 0; j < 640; j++) {
      if(depthMat.at<short>(i, j) > 0.0 && depthMat.at<short>(i, j) < 2047.0) {
        frame->ilen++;
      }
    }
  }

  frame->indices = new GLint[frame->ilen];
  frame->vlen = frame->ilen * 3;
  frame->vertices = new GLfloat[frame->vlen];
  frame->clen = frame->vlen;
  frame->colors = new GLfloat[frame->vlen];

  float fx_rgb = 5.2921508098293293e+02;
  float fy_rgb = 5.2556393630057437e+02;
  float cx_rgb = 3.2894272028759258e+02;
  float cy_rgb = 2.6748068171871557e+02;
  float k1_rgb = 2.6451622333009589e-01;
  float k2_rgb = -8.3990749424620825e-01;
  float p1_rgb = -1.9922302173693159e-03;
  float p2_rgb = 1.4371995932897616e-03;
  float k3_rgb = 9.1192465078713847e-01;

  float fx_d = 5.9421434211923247e+02;
  float fy_d = 5.9104053696870778e+02;
  float cx_d = 3.3930780975300314e+02;
  float cy_d = 2.4273913761751615e+02;
  float k1_d = -2.6386489753128833e-01;
  float k2_d = 9.9966832163729757e-01;
  float p1_d = -7.6275862143610667e-04;
  float p2_d = 5.0350940090814270e-03;
  float k3_d = -1.3053628089976321e+00;

  QMatrix4x4 R = QMatrix4x4(9.9984628826577793e-01, 1.2635359098409581e-03, -1.7487233004436643e-02, 0.0,
                            -1.4779096108364480e-03, 9.9992385683542895e-01, -1.2251380107679535e-02, 0.0,
                            1.7470421412464927e-02, 1.2275341476520762e-02, 9.9977202419716948e-01, 0.0,
                            0.0, 0.0, 0.0, 1.0);

  QVector4D t = QVector4D(1.9985242312092553e-02, -7.4423738761617583e-04,
                          -1.0916736334336222e-02, 0.0);


  int k = 0;
  for(int y = 0; y < 480; y++) {
    for(int x = 0; x < 640; x++) {
      short depth;
      if((depth = depthMat.at<short>(y, x)) > 0.0) {
        if(depth < 2047.0) {
          float depthVal = 1.0f / (depth * -0.0030711016f + 3.3309495161f);

          frame->vertices[k++] = (x - cx_d) * depthVal / fx_d;
          frame->vertices[k++] = (y - cy_d) * depthVal / fy_d;
          frame->vertices[k++] = depthVal;
        }
      }
    }
  }

  frame->indices = new GLint[frame->ilen];
  for(int i = 0; i < frame->ilen; i++) {
    frame->indices[i] = (GLint)i;
    qDebug() << frame->vertices[3 * i + 0] << frame->vertices[3 * i + 1] << frame->vertices[3 * i + 2];
  }


  k = 0;
  for(int i = 0; i < frame->ilen; i++) {
    GLfloat point_x = 9.9984628826577793e-01 * frame->vertices[3 * i + 0] +
                      1.2635359098409581e-03 * frame->vertices[3 * i + 1] +
                      -1.7487233004436643e-02 * frame->vertices[3 * i + 2] + 1.9985242312092553e-02;

    GLfloat point_y = -1.4779096108364480e-03 * frame->vertices[3 * i + 0] +
                      9.9992385683542895e-01 * frame->vertices[3 * i + 1] +
                      -1.2251380107679535e-02 * frame->vertices[3 * i + 2] + -7.4423738761617583e-04;

    GLfloat point_z = 1.7470421412464927e-02 * frame->vertices[3 * i + 0] +
                      1.2275341476520762e-02 * frame->vertices[3 * i + 1] +
                      9.9977202419716948e-01 * frame->vertices[3 * i + 2] + -1.0916736334336222e-02;

    int rgb_x = (point_x * fx_rgb / point_z) + cx_rgb;
    int rgb_y = (point_y * fy_rgb / point_z) + cy_rgb;

    QColor pixel = rgb.pixel(rgb_x, rgb_y);
    frame->colors[3 * i + 0] = pixel.red() / 255.0;
    frame->colors[3 * i + 1] = pixel.green() / 255.0;
    frame->colors[3 * i + 2] = pixel.blue() / 255.0;
  }

  frame->initialized = false;
  return frame;
}


// QJsonObject qtviewer::writeNode( Node* node )
// {

//     QJsonObject out;
//     out["type"] = node->type;

//     if( node->type == "model" )
//     {

//         Model* m = static_cast<Model*>(node->children[0]);

//         out["name"] = m->name;
//         out["filename"] = m->fileName;

//         qDebug() << out["name"] << " " << out["filename"];

//         QJsonArray data;
//         for( int i = 0; i < 16; ++i )
//         {
//             data.push_back((float)m->transform.data()[i]);
//         }

//         out["matrix"] = data;
//     }
//     else if( node->type == "node" )
//     {
//         QJsonArray children;
//         for( int i = 0; i < node->children.size(); ++i ) {
//             children.push_back(writeNode(node->children[i]));
//         }
//         out["children"] = children;
//     }
//     return out;
// }


// void qtviewer::sceneWrite( const QString &filename, QVector<Scene *> &scenes )
// {
//     QFile file( filename );

//     file.open(QIODevice::WriteOnly | QIODevice::Text);

//     QJsonDocument doc;

//     QJsonArray sceneList;
//     for(int i = 0; i < scenes.size(); ++i)
//     {
//         QJsonObject scene;
//         scene["name"] = scenes[i]->name;
//         scene["description"] = scenes[i]->description;
//         qDebug() << "writing : " << scene["name"] << " " << scene["description"];
//         QJsonArray rootNode;
//         for(int j = 0; j < scenes[i]->root.size(); ++j) {
//             QJsonValue value = writeNode(scenes[i]->root[j]);
//             rootNode.push_back(value);
//         }
//         scene["root"] = rootNode;
//         sceneList.push_back(scene);
//     }

//     doc.setArray(sceneList);
//     file.write(doc.toJson());
//     file.close();

// }
