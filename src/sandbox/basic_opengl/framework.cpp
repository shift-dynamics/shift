#include <QtGui>
#include <QPushButton>
#include <QFileDialog>
#include <QColorDialog>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QLabel>
#include <QMargins>
#include <QMenuBar>
#include <QStatusBar>
#include <QKeySequence>
#include <QTreeWidget>
#include <QComboBox>
#include <QResizeEvent>
#include <QStringList>

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


#include "framework.hpp"
#include "renderwindow.hpp"
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

/*!
@brief constructor for BasicFramework class
*/
BasicFramework::BasicFramework(
  QWidget* parent) : QMainWindow(parent) {

  m_currentSelectedModel = NULL;
  m_currentFileName = "default.json";
  m_currentFilePath = ".";
  createMenus();
  createLayout();
}

/*!
@brief destructor for BasicFramework class, any resources that are malloced
in the class should be freed here
*/
BasicFramework::~BasicFramework() {

  /* free resources created with calls to malloc here */
}

/*!
@brief sets up menus on menubar
*/
void BasicFramework::createMenus() {

  setWindowTitle(tr("Basic OpenGL"));

  /*
    File Menu
  */
  QMenu* fileMenu = menuBar()->addMenu("&File");

  /* Open Scene */
  m_actionOpen = new QAction("Open Scene...", this);
  m_actionOpen->setShortcut(QKeySequence::Open);
  fileMenu->addAction(m_actionOpen);

  /* Save */
  m_actionSave = new QAction("Save Scene", this);
  m_actionSave->setShortcut(QKeySequence::Save);
  fileMenu->addAction(m_actionSave);

  /* Save As */
  m_actionSaveAs = new QAction("Save Scene As..", this);
  m_actionSaveAs->setShortcut(QKeySequence::SaveAs);
  fileMenu->addAction(m_actionSaveAs);

  fileMenu->addSeparator();

  /* Quit */
  m_actionQuit = new QAction("Quit", this);
  m_actionQuit->setShortcut(QKeySequence::Quit);
  fileMenu->addAction(m_actionQuit);

  /*
    View Menu
  */
  QMenu* viewMenu = menuBar()->addMenu("&View");

  /* Orthogonal */
  m_actionSetOrthogonal = new QAction("Orthogonal", this);
  viewMenu->addAction(m_actionSetOrthogonal);

  /* Perspective */
  m_actionSetPerspective = new QAction("Perspective", this);
  viewMenu->addAction(m_actionSetPerspective);

  viewMenu->addSeparator();

  /* Wireframe */
  m_actionSetWireframe = new QAction("Wireframe", this);
  viewMenu->addAction(m_actionSetWireframe);

  /* Flat */
  m_actionSetFlatShaded = new QAction("Flat", this);
  viewMenu->addAction(m_actionSetFlatShaded);

  /*
    Help Menu
  */
  QMenu* helpMenu = menuBar()->addMenu("&Help");

  /* About */
  m_actionAbout = new QAction("About", this);
  helpMenu->addAction(m_actionAbout);

  /* Credit */
  m_actionCredit = new QAction("Credit", this);
  helpMenu->addAction(m_actionCredit);

}

void BasicFramework::addSlider(
  const char* label,
  QSlider* slider,
  QVBoxLayout* sliderLayout) {

  QGroupBox* sliderBox = new QGroupBox();
  QHBoxLayout* boxLayout = new QHBoxLayout(sliderBox);
  boxLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  QLabel* sliderLabel = new QLabel(label);
  boxLayout->addWidget(sliderLabel);
  boxLayout->addWidget(slider);
  sliderLayout->addWidget(sliderBox);
}
/*!
  @brief set up window layout
*/
void BasicFramework::createLayout() {

  /* main window */
  QGroupBox* mainBox = new QGroupBox;
  mainBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QHBoxLayout* mainLayout = new QHBoxLayout(mainBox);
  mainLayout->setContentsMargins(QMargins(0, 0, 0, 0));

  /* left window */
  QGroupBox* leftBox = new QGroupBox;
  QVBoxLayout* leftLayout = new QVBoxLayout(leftBox);
  leftLayout->setContentsMargins(QMargins(0, 0, 0, 0));

  /* OpenGL viewport */
  glwidget = new RenderWindowWidget(leftBox);
  glwidget->setMinimumHeight(200);
  glwidget->setMinimumWidth(200);
  glwidget->setContentsMargins(QMargins(0, 0, 0, 0));
  leftLayout->addWidget(glwidget);
  RenderWindow* renWin = glwidget->getRenderWindow();

  /* text label below viewport */
  cursorPositionLabel = new QLabel("");
  cursorPositionLabel->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
  cursorPositionLabel->setFixedHeight(20);
  leftLayout->addWidget(cursorPositionLabel);

  /* right window */
  QGroupBox* rightBox = new QGroupBox();
  QVBoxLayout* rightLayout = new QVBoxLayout(rightBox );
  rightBox->setFixedWidth(300);

  /* tree widget */
  treeWidget = new QTreeWidget();
  treeWidget->setAlternatingRowColors(true);
  treeWidget->setColumnCount(1);

  treeWidget->setEditTriggers(
    QAbstractItemView::SelectedClicked |
    QAbstractItemView::DoubleClicked);
  treeWidget->setHeaderLabel("Model Browser");
  rightLayout->addWidget(treeWidget);

  renWin->loadModelArray(m_modelArray);

  /* change object color push button */
  QPushButton* objectColorButton = new QPushButton();
  objectColorButton->setText("select object color");

  /* change background color push button */
  QPushButton* backgroundColorButton = new QPushButton();
  backgroundColorButton->setText("select background color");

  QGroupBox* sliderBox = new QGroupBox();
  QVBoxLayout* sliderLayout = new QVBoxLayout(sliderBox);
  sliderBox->setContentsMargins(QMargins(0, 0, 0, 0));

  /* rotation sliders */
  m_rxSlider = createSlider();
  addSlider("rx", m_rxSlider, sliderLayout);

  m_rySlider = createSlider();
  addSlider("ry", m_rySlider, sliderLayout);

  m_rzSlider = createSlider();
  addSlider("rz", m_rzSlider, sliderLayout);

  /* translation sliders */
  m_txSlider = createSlider();
  m_txSlider->setRange(-100.0, 100.0);
  addSlider("tx", m_txSlider, sliderLayout);

  m_tySlider = createSlider();
  m_tySlider->setRange(-100.0, 100.0);
  addSlider("ty", m_tySlider, sliderLayout);

  m_tzSlider = createSlider();
  m_tzSlider->setRange(-100.0, 100.0);
  addSlider("tz", m_tzSlider, sliderLayout);

  /* scale slider */
  m_scaleSlider = createSlider();
  m_scaleSlider->setRange(0, 100);
  m_scaleSlider->setValue(1.0);
  addSlider("s", m_scaleSlider, sliderLayout);

  rightLayout->addWidget(objectColorButton);
  rightLayout->addWidget(backgroundColorButton);

  sliderLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  rightLayout->addWidget(sliderBox);
  rightLayout->addStretch();

  mainLayout->addWidget(leftBox);
  mainLayout->addWidget(rightBox);

  setCentralWidget(mainBox);

  /*
    Connect signals and slots
  */
  // QObject::connect( qApp, SIGNAL( resizeEvent() ), renWin, SLOT( updateGL() ));
  QObject::connect(
    backgroundColorButton,
    SIGNAL(clicked()),
    this,
    SLOT(backgroundColorSelector()));

  QObject::connect(
    objectColorButton,
    SIGNAL(clicked()),
    this,
    SLOT(objectColorSelector()));

  /* cursor dialog, lower left */
  QObject::connect(
    renWin,
    SIGNAL(mouseMove()),
    this,
    SLOT(updateCursorPositionText()));

  QObject::connect(
    glwidget,
    SIGNAL(mouseOut()),
    this,
    SLOT(clearCursorPositionText()));

  /* file menu actions */
  QObject::connect(
    m_actionOpen,
    SIGNAL(triggered()),
    this,
    SLOT(openFile()));
  QObject::connect(
    m_actionSave,
    SIGNAL(triggered()),
    this,
    SLOT(saveFile()));
  QObject::connect(
    m_actionSaveAs,
    SIGNAL(triggered()),
    this,
    SLOT(saveFileAs()));
  QObject::connect(
    m_actionQuit,
    SIGNAL(triggered()),
    qApp,
    SLOT(quit()));

  /* view menu actions */
  QObject::connect(
    m_actionSetPerspective,
    SIGNAL(triggered()),
    renWin,
    SLOT(setPerspective()));
  QObject::connect(
    m_actionSetOrthogonal,
    SIGNAL(triggered()),
    renWin,
    SLOT(setOrthogonal()));
  QObject::connect(
    m_actionSetWireframe,
    SIGNAL(triggered()),
    renWin,
    SLOT(setWireFrame()));
  QObject::connect(
    m_actionSetFlatShaded,
    SIGNAL(triggered()),
    renWin,
    SLOT(setFlatShaded()));

  /* help menu actions */
  QObject::connect(
    m_actionAbout,
    SIGNAL(triggered()),
    this,
    SLOT(about()));
  QObject::connect(
    m_actionCredit,
    SIGNAL(triggered()),
    this,
    SLOT(credit()));

  /* qtreewidget actions */
  QObject::connect(
    treeWidget,
    SIGNAL(itemSelectionChanged()),
    this,
    SLOT(getSelectedTreeItem()));

  /* sliders */
  QObject::connect(
    m_rxSlider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setRx(int)));
  QObject::connect(
    m_rySlider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setRy(int)));
  QObject::connect(
    m_rzSlider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setRz(int)));
  QObject::connect(
    m_txSlider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setTx(int)));
  QObject::connect(
    m_tySlider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setTy(int)));
  QObject::connect(
    m_tzSlider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setTz(int)));
  QObject::connect(
    m_scaleSlider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setS(int)));
}

/*!
  @brief update a SE3 matrix
*/
void BasicFramework::updateTransform() {
  m_currentSelectedModel->transform.setToIdentity();
  m_currentSelectedModel->transform.rotate(
    m_currentSelectedModel->rotation.x(), 1, 0, 0);
  m_currentSelectedModel->transform.rotate(
    m_currentSelectedModel->rotation.y(), 0, 1, 0);
  m_currentSelectedModel->transform.rotate(
    m_currentSelectedModel->rotation.z(), 0, 0, 1);
  m_currentSelectedModel->transform.translate(
    m_currentSelectedModel->translation);
  m_currentSelectedModel->transform.scale(m_currentSelectedModel->scalar);
}

/*!
  @brief set rotation about x axis
*/
void BasicFramework::setRx(int val) {
  if(m_currentSelectedModel != NULL) {
    m_currentSelectedModel->rotation.setX(val / 16.0f);
    qDebug() << m_currentSelectedModel->transform;
    updateTransform();
  }
  emit rxChanged(0);
}

/*!
  @brief set rotation about y axis
*/
void BasicFramework::setRy(int val) {
  if(m_currentSelectedModel!=NULL) {
    m_currentSelectedModel->rotation.setY( val/16.0f);
    qDebug() << m_currentSelectedModel->transform;
    updateTransform();
  }
  emit ryChanged( 0 );
}

/*!
  @brief set rotation about z axis
*/
void BasicFramework::setRz( int val) {
  if(m_currentSelectedModel != NULL) {
      m_currentSelectedModel->rotation.setZ(val / 16.0f);
      qDebug() << m_currentSelectedModel->transform;
      updateTransform();
  }
  emit rzChanged(0);
}

/*!
  @brief set translation along x axis
*/
void BasicFramework::setTx(int val) {
  if(m_currentSelectedModel != NULL) {
    m_currentSelectedModel->translation.setX(val / 10.0f);
    qDebug() << m_currentSelectedModel->transform;
    updateTransform();
  }
  emit txChanged(0);
}

/*!
  @brief set translation along y axis
*/
void BasicFramework::setTy(int val) {
  if(m_currentSelectedModel != NULL) {
    m_currentSelectedModel->translation.setY(val / 10.0f);
    qDebug() << m_currentSelectedModel->transform;
    updateTransform();
  }
  emit tyChanged(0);
}

/*!
  @brief set translation along z axis
*/
void BasicFramework::setTz(int val) {
  if(m_currentSelectedModel != NULL) {
    m_currentSelectedModel->translation.setZ(val / 10.0f);
    qDebug() << m_currentSelectedModel->transform;
    updateTransform();
  }
  emit tzChanged(0);
}

/*!
  @brief set scalar
*/
void BasicFramework::setS(int val) {
  if(m_currentSelectedModel != NULL) {
    m_currentSelectedModel->scalar = val / 10.0f;
    qDebug() << m_currentSelectedModel->transform;
    updateTransform();
  }
  emit sChanged(0);
}

/*!
  @brief retrieve current selected tree item
*/
void BasicFramework::getSelectedTreeItem() {
  ModelTreeItem* item = static_cast<ModelTreeItem*>(treeWidget->currentItem());
  qDebug() << item->UserType << item->text(0);

  m_currentSelectedModel = item->model;

  if(item->model!=NULL) {
    qDebug() << m_currentSelectedModel->name;
    m_rxSlider->setValue(m_currentSelectedModel->rotation.x() * 16.0f);
    m_rySlider->setValue(m_currentSelectedModel->rotation.y() * 16.0f);
    m_rzSlider->setValue(m_currentSelectedModel->rotation.z() * 16.0f);
    m_txSlider->setValue(m_currentSelectedModel->translation.x() * 10.0f);
    m_tySlider->setValue(m_currentSelectedModel->translation.y() * 10.0f);
    m_tzSlider->setValue(m_currentSelectedModel->translation.z() * 10.0f);
    m_scaleSlider->setValue(m_currentSelectedModel->scalar * 10.0f);
  }
}

/*!
  @brief update text at bottom of screen showing x,y position of cursor
*/
void BasicFramework::updateCursorPositionText() {
  RenderWindow* renWin = glwidget->getRenderWindow();
  QString positionText = renWin->getCursorPositionText();
  cursorPositionLabel->setText(positionText);
}

/*!
  @brief clear text showing x,y position of cursor
*/
void BasicFramework::clearCursorPositionText() {
  cursorPositionLabel->clear();
}

/*!
  @brief open a file
*/
void BasicFramework::openFile() {
  RenderWindow* renWin = glwidget->getRenderWindow();
  renWin->setAnimating(false);
  QString fileName =
    QFileDialog::getOpenFileName(
      this,
      tr("Open File"),
      "",
      tr("JSON Files (*.json)"));
  //QMessageBox::information( this, "File Selected", fileName.length() == 0 ? "No File Selected" : fileName );
  if(fileName.length() != 0) {
    m_currentFileName = fileName;
    QStringList filePathList = fileName.split("/");
    filePathList.removeLast();
    m_currentFilePath = filePathList.join("/");
    m_modelArray.clear();
    parseScene(m_currentFileName, m_scenes);
    populateTree();
    renWin->loadModelArray(m_modelArray);
    renWin->initialize();
  }
  renWin->setAnimating(true);

}

/*!
  @brief populate model tree
*/
void BasicFramework::populateTree() {
  treeWidget->clear();

  for(int i = 0; i < m_scenes.size(); ++i) {
    //QTreeWidgetItem * item = new QTreeWidgetItem((QTreeWidget*)0, QStringList( scenes[i]->name ), 0);
    ModelTreeItem* item = new ModelTreeItem(1001);
    item->setText(0, m_scenes[i]->name);
    treeWidget->addTopLevelItem(item);
    for(int j = 0; j < m_scenes[i]->root.size(); ++j) {
      qDebug() << m_scenes[i]->root[j]->type;
      if(m_scenes[i]->root[j]->type == "model") {
        Model* m = static_cast<Model*>(m_scenes[i]->root[j]->children[0]);
        qDebug() << m->name;
        ModelTreeItem* subItem = new ModelTreeItem(1002);
        subItem->setText( 0, m->name );
        subItem->model = m;
        item->addChild(subItem);
      }
    }
  }
}

/*!
  @brief about dialog
*/
void BasicFramework::about() {
  QString fileContents;
  QFile file(":/html/about.html");
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();
  QMessageBox::about(this, "About", fileContents);
}

/*!
  @brief credit dialog
*/
void BasicFramework::credit() {
  QString fileContents;
  QFile file(":/html/credit.html");
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();
  QMessageBox::about(this, "Credit", fileContents);
}

/*!
  @brief save a file
*/
void BasicFramework::saveFile() {
  sceneWrite(m_currentFileName, m_scenes);
}

/*!
  @brief save file as
*/
void BasicFramework::saveFileAs() {
  RenderWindow* renWin = glwidget->getRenderWindow();
  renWin->setAnimating(false);
  QString fileName =
    QFileDialog::getSaveFileName(
      this,
      tr("Save File"),
      "",
      tr("JSON Files (*.json)"));
  renWin->setAnimating(true);
  sceneWrite(fileName, m_scenes);
}

/*!
  @brief set background color
*/
void BasicFramework::backgroundColorSelector() {
  RenderWindow* renWin = glwidget->getRenderWindow();
  renWin->setAnimating( false );
  QColor color =
    QColorDialog::getColor(
      Qt::black,
      this,
      "Select Color",
      QColorDialog::DontUseNativeDialog);
  if(color.isValid()) {
    qDebug("ok r:%f g:%f b:%f\n", color.redF(), color.greenF(), color.blueF());
    renWin->changeBackgroundColor(color.redF(), color.greenF(), color.blueF());
  }
  renWin->setAnimating( true );
}

/*!
  @brief select object color
*/
void BasicFramework::objectColorSelector() {
  RenderWindow* renWin = glwidget->getRenderWindow();
  renWin->setAnimating(false);

  if(m_currentSelectedModel==NULL) {
    QMessageBox::information(
      this,
      "Alert",
      "No object was selected. Select an object from the model tree.");
  } else {
    QColor color =
      QColorDialog::getColor(
        Qt::white,
        this,
        "Select Color",
        QColorDialog::DontUseNativeDialog);
    if(color.isValid()) {
      qDebug() << m_currentSelectedModel->name;
      qDebug("ok r:%i g:%i b:%i\n", color.red(), color.green(), color.blue());
      renWin->changeObjectColor(
        color.red(),
        color.green(),
        color.blue(),
        m_currentSelectedModel);
    }
  }
  renWin->setAnimating( true );
}

/*!
  @brief create a slider
*/
QSlider* BasicFramework::createSlider() {
  QSlider *slider = new QSlider(Qt::Horizontal);
  slider->setRange(-180 * 16, 180 * 16);
  slider->setSingleStep(16);
  slider->setPageStep(15 * 16);
  slider->setTickInterval(15 * 16);
  slider->setTickPosition(QSlider::TicksAbove);
  slider->setContentsMargins(QMargins(0, 0, 0, 0));
  slider->setValue( 0 * 16 );
  return slider;
}

/*!
  @brief parse a "model" node in a simple JSON scene graph
*/
Model* BasicFramework::parseModel(
  const QJsonObject& model) {

  Model* out = new Model();

  out->name = model["name"].toString();
  out->fileName = model["filename"].toString();
  QJsonArray rotation = model["rotation"].toArray();
  QJsonArray translation = model["translation"].toArray();
  for(int i = 0; i < 3; i++) {
    out->rotation[i] = (float)rotation[i].toDouble();
    out->translation[i] = (float)translation[i].toDouble();
  }
  out->scalar = (float)model["scalar"].toDouble();

  QJsonArray color = model["color"].toArray();
  out->color.setRedF((float)color[0].toDouble());
  out->color.setGreenF((float)color[1].toDouble());
  out->color.setBlueF((float)color[2].toDouble());
  qDebug() << "color: " << out->color;

  m_currentSelectedModel = out;
  updateTransform();
  parseGeometry(out);

  return out;
}

/*!
  @brief parse geometry specified in a JSON scene graph

*/
bool BasicFramework::parseGeometry(
  Model* model) {

  /* Create an instance of the Importer class */
  Assimp::Importer importer;

  QStringList list;
  list.append(m_currentFilePath);
  list.append(model->fileName);
  QString newFileName = list.join("/");

  const aiScene* scene = importer.ReadFile(
    newFileName.toUtf8().constData(),
    aiProcess_CalcTangentSpace       |
    aiProcess_Triangulate            |
    aiProcess_JoinIdenticalVertices  |
    aiProcess_SortByPType);

  if(!scene) {
    std::cerr << importer.GetErrorString() << std::endl;
    return false;
  }

  /* get total number of indices, vertices from all shapes */
  model->ilen = 0;
  model->vlen = 0;


  /* each mesh could be loaded into its own material, but for this simple
  example, all meshes are flattened into a single "model" datastructure; the
  color is loaded from the JSON file, rather than loaded from the material
  definition */
  for(unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    const aiMesh* mesh = scene->mMeshes[n];

    model->ilen += mesh->mNumFaces * 3;
    model->vlen += mesh->mNumVertices * 3;
  }

  qDebug() << "ilen:" << model->ilen;
  qDebug() << "vlen:" << model->vlen;

  model->faces = new GLuint[model->ilen];
  model->vertices = new GLfloat[model->vlen];

  int j = 0;
  for(unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    const aiMesh* mesh = scene->mMeshes[n];
    for(unsigned int k= 0; k < mesh->mNumFaces; k++) {
      for(int l = 0; l < 3; l++) {
        model->faces[j++] = (GLuint)(mesh->mFaces[k].mIndices[l]);
      }
    }
  }
  qDebug() << "faces" << model->faces;

  j = 0;
  for(unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    const aiMesh* mesh = scene->mMeshes[n];
    for(unsigned int k= 0; k < mesh->mNumVertices; k++) {
      for(int l = 0; l < 3; l++) {
        model->vertices[j++] = (GLfloat)(mesh->mVertices[k][l]);
      }
    }
  }
  // qDebug() << "vertices" << model->vertices;


  model->clen = model->vlen;
  model->colors = new GLfloat[model->clen];
  // qDebug() << "color in model:" << model->color;
  // qDebug() << "red " << model->color.redF();


  for(int i = 0; i < model->clen / 3; i++) {
    model->colors[i * 3 + 0] = model->color.redF();
    model->colors[i * 3 + 1] = model->color.greenF();
    model->colors[i * 3 + 2] = model->color.blueF();
  }

  m_modelArray.append(model);
  return true;
}

/*!
  @brief parse a node in a json scene graph
*/
Node* BasicFramework::parseNode( const QJsonObject& parentNode )
{
    Node* out = new Node();
    out->type = parentNode["type"].toString();

    if(out->type == "model") {
        out->children.push_back( parseModel(parentNode) );
    }
    else if(out->type == "node") {
      QJsonArray nodeArray = parentNode["children"].toArray();
      for(int i = 0; i < nodeArray.size(); ++i) {
          out->children.push_back(parseNode(nodeArray[i].toObject()));
      }
    }
    return out;
}

/*!
  @brief parse a scene specified in a JSON file
*/
void BasicFramework::parseScene(
  const QString &filename,
  QVector<Scene*>& scenes) {

  QStringList filepathList = filename.split("/");
  filepathList.removeAt(filepathList.size() - 1);
  QString filepath = filepathList.join("/");
  qDebug() << filepath;
  scenes.clear();
  QString fileContents;

  QFile file( filename );

  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();

  QJsonDocument doc = QJsonDocument::fromJson(fileContents.toUtf8());

  QJsonArray sceneList = doc.array();

  for(int i = 0; i < sceneList.size(); ++i) {
    Scene* scene = new Scene();

    QJsonObject object = sceneList[i].toObject();
    scene->name = object["name"].toString();
    scene->description = object["description"].toString();

    QJsonArray rootNode = object["root"].toArray();

    qDebug() << "\n\nscene: " << i << " " << scene->name << " " << scene->description;

    for(int i = 0; i < rootNode.size(); ++i) {
      scene->root.push_back(parseNode(rootNode[i].toObject()));
    }
    scenes.push_back(scene);
  }
}

/*!
  @brief write a single node to a simple JSON scene graph
*/
QJsonObject BasicFramework::writeNode(
  Node* node) {

  QJsonObject out;
  out["type"] = node->type;

  if(node->type == "model") {

    Model* m = static_cast<Model*>(node->children[0]);

    out["name"] = m->name;
    out["filename"] = m->fileName;
    out["scalar"] = m->scalar;
    QJsonArray rotation;
    QJsonArray translation;
    for(int i = 0; i < 3; i++) {
      rotation.push_back((float)m->rotation[i]);
      rotation.push_back((float)m->translation[i]);
    }
    out["rotation"] = rotation;
    out["translation"] = translation;

    QJsonArray color;
    color.push_back(m->color.redF());
    color.push_back(m->color.blueF());
    color.push_back(m->color.greenF());
    out["color"] = color;
  } else if(node->type == "node") {
    QJsonArray children;
    for( int i = 0; i < node->children.size(); ++i ) {
        children.push_back(writeNode(node->children[i]));
    }
    out["children"] = children;
  }
  return out;
}

/*!
  @brief write a scene to a JSON file
*/
void BasicFramework::sceneWrite(
  const QString &filename,
  QVector<Scene *> &scenes) {

  QFile file( filename );

  file.open(QIODevice::WriteOnly | QIODevice::Text);

  QJsonDocument doc;

  QJsonArray sceneList;
  for(int i = 0; i < scenes.size(); ++i) {
    QJsonObject scene;
    scene["name"] = scenes[i]->name;
    scene["description"] = scenes[i]->description;
    qDebug() << "writing : " << scene["name"] << " " << scene["description"];
    QJsonArray rootNode;
    for(int j = 0; j < scenes[i]->root.size(); ++j) {
        QJsonValue value = writeNode(scenes[i]->root[j]);
        rootNode.push_back(value);
    }
    scene["root"] = rootNode;
    sceneList.push_back(scene);
  }

  doc.setArray(sceneList);
  file.write(doc.toJson());
  file.close();
}
