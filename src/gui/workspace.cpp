/**
@file
@brief A Qt class for laying out a workspace to interface with star2
@author Kristopher Wehage
@date 2016
@version 1.0
@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/

#include <QColorDialog>
#include <QComboBox>
#include <QDockWidget>
#include <QFileDialog>
#include <QGenericMatrix>
#include <QGroupBox>
#include <QIcon>
#include <QKeySequence>
#include <QLabel>
#include <QMargins>
#include <QMatrix4x4>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTextBrowser>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QVector3D>
#include <QPalette>
#include <QShortcut>
#include <QProgressBar>
#include <QSvgWidget>
#include <QSpacerItem>

#include <fstream>
#include <iostream>
#include <map>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <stdio.h>

#include "workspace.hpp"
#include "renderwindow.hpp"
#include "joints.hpp"
#include "bodies.hpp"
#include "lights.hpp"
#include "frames.hpp"
#include "terrain.hpp"
#include "grid.hpp"
#include "pointcloud.hpp"
#include "../shift.h"

/*!
@brief class constructor for Workspace application

*/
Workspace::Workspace(QWidget* parent) : QMainWindow(parent) {

  m_currentSelectedItem = NULL;
  m_renderWindow = NULL;

  m_cursorPositionLabel = new QLabel();
  m_context = NULL;
  m_simulationRunning = false;
  m_sliders.prismaticRange = 0.2;
  m_sliders.revoluteRange = 0.2;
  initializeLayout();
  initializeMenus();
  m_mobilityFile = fopen("mobility.dat", "w");
  m_currentFileName = "untitled.json";

}


/*! @brief class destructor

*/
Workspace::~Workspace() {}


/*!
  @brief add context-dependent dynamic scene box
*/
void Workspace::addDynamicSceneBox() {
  m_scenePanel.groupBox = new QGroupBox;
  m_scenePanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));
  QGridLayout* sceneLayout = new QGridLayout(m_scenePanel.groupBox);
  sceneLayout->setMargin(0);
  sceneLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  sceneLayout->setSpacing(0);
  m_scenePanel.title = new QLabel(
    "Select an object from the treewidget to edit its properties.");
  m_scenePanel.title->setWordWrap(true);
  sceneLayout->addWidget(m_scenePanel.title, 0, 0, 0);
  m_scenePanel.groupBox->setVisible(true);
  m_dynamicPanelLayout->addWidget(m_scenePanel.groupBox);
}


/*!
  @brief add all sliders to slider group box
*/
void Workspace::addSliders() {

  m_sliders.groupBox = new QGroupBox;
  m_sliders.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QVBoxLayout* sliderLayout = new QVBoxLayout(m_sliders.groupBox);
  sliderLayout->setContentsMargins(QMargins(0, 0, 0, 0));

  /*
    translation sliders
  */
  double singleStep = m_sliders.prismaticRange / 100.0;
  addSlider(
    singleStep,
    &m_sliders.tx,
    sliderLayout,
    "Tx");

  addSlider(
    singleStep,
    &m_sliders.ty,
    sliderLayout,
    "Ty");

  addSlider(
    singleStep,
    &m_sliders.tz,
    sliderLayout,
    "Tz");

  /*
    rotation sliders
  */
  singleStep = m_sliders.revoluteRange / 100.0;

  addSlider(
    singleStep,
    &m_sliders.rx,
    sliderLayout,
    "Rx");

  addSlider(
    singleStep,
    &m_sliders.ry,
    sliderLayout,
    "Ry");

  addSlider(
    singleStep,
    &m_sliders.rz,
    sliderLayout,
    "Rz");

  m_dynamicPanelLayout->addWidget(m_sliders.groupBox);

  QObject::connect(
    m_sliders.tx.slider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setTx(int)));

  QObject::connect(
    m_sliders.ty.slider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setTy(int)));

  QObject::connect(
    m_sliders.tz.slider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setTz(int)));

  QObject::connect(
    m_sliders.rx.slider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setRx(int)));

  QObject::connect(
    m_sliders.ry.slider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setRy(int)));

  QObject::connect(
    m_sliders.rz.slider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(setRz(int)));

  /* connect spinbox value changed to recenter slider */
  QObject::connect(
    m_sliders.tx.spinBox,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setTxSpinBox(double)));

  QObject::connect(
    m_sliders.ty.spinBox,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setTySpinBox(double)));

  QObject::connect(
    m_sliders.tz.spinBox,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setTzSpinBox(double)));

  QObject::connect(
    m_sliders.rx.spinBox,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setRxSpinBox(double)));

  QObject::connect(
    m_sliders.ry.spinBox,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setRySpinBox(double)));

  QObject::connect(
    m_sliders.rz.spinBox,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setRzSpinBox(double)));

  /* connect mouse clicked button to setup initial factorization and
  remove constraint violations */
  QObject::connect(
    m_sliders.tx.slider,
    SIGNAL(sliderPressed()),
    this,
    SLOT(onclickTx()));

  QObject::connect(
    m_sliders.ty.slider,
    SIGNAL(sliderPressed()),
    this,
    SLOT(onclickTy()));

  QObject::connect(
    m_sliders.tz.slider,
    SIGNAL(sliderPressed()),
    this,
    SLOT(onclickTz()));

  QObject::connect(
    m_sliders.rx.slider,
    SIGNAL(sliderPressed()),
    this,
    SLOT(onclickRx()));

  QObject::connect(
    m_sliders.ry.slider,
    SIGNAL(sliderPressed()),
    this,
    SLOT(onclickRy()));

  QObject::connect(
    m_sliders.rz.slider,
    SIGNAL(sliderPressed()),
    this,
    SLOT(onclickRz()));

  /* connect mouse button released to recenter slider function */
  QObject::connect(
    m_sliders.tx.slider,
    SIGNAL(sliderReleased()),
    this,
    SLOT(recenterTx()));

  QObject::connect(
    m_sliders.ty.slider,
    SIGNAL(sliderReleased()),
    this,
    SLOT(recenterTy()));

  QObject::connect(
    m_sliders.tz.slider,
    SIGNAL(sliderReleased()),
    this,
    SLOT(recenterTz()));

  QObject::connect(
    m_sliders.rx.slider,
    SIGNAL(sliderReleased()),
    this,
    SLOT(recenterRx()));

  QObject::connect(
    m_sliders.ry.slider,
    SIGNAL(sliderReleased()),
    this,
    SLOT(recenterRy()));

  QObject::connect(
    m_sliders.rz.slider,
    SIGNAL(sliderReleased()),
    this,
    SLOT(recenterRz()));
}


/*!
  @brief add a slider to slider group box
*/
void Workspace::addSlider(
  double singleStep,
  SliderHandle* sliderHandle,
  QLayout* layout,
  const char* label) {

  sliderHandle->singleStep = singleStep;

  sliderHandle->box = new QGroupBox();
  QGridLayout* sliderLayout = new QGridLayout(sliderHandle->box);
  sliderLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  QLabel* sliderLabel = new QLabel(label);
  sliderLayout->addWidget(sliderLabel, 0, 0);
  sliderLayout->setColumnStretch(1, 1);

  sliderHandle->spinBox = new QDoubleSpinBox();
  sliderHandle->spinBox->setValue(0);
  sliderHandle->spinBox->setDecimals(4);
  sliderHandle->spinBox->setMinimum(-10000000);
  sliderHandle->spinBox->setMaximum(10000000);
  sliderHandle->spinBox->setKeyboardTracking(false);
  sliderHandle->spinBox->setSingleStep(0.01);
  sliderLayout->addWidget(sliderHandle->spinBox, 0, 1);

  sliderHandle->slider = new QSlider(Qt::Horizontal);
  sliderHandle->slider->setRange(-100, 100);
  sliderHandle->slider->setSingleStep(1);
  sliderHandle->slider->setPageStep(5);
  sliderHandle->slider->setTickPosition(QSlider::NoTicks);
  sliderHandle->slider->setContentsMargins(QMargins(0, 0, 0, 0));
  sliderHandle->slider->setValue(0.0 * sliderHandle->singleStep);
  sliderLayout->addWidget(sliderHandle->slider, 1, 0, 2, 0);

  layout->addWidget(sliderHandle->box);
  sliderHandle->box->setVisible(false);
}


/*!
  @brief add dynamic assembly box
*/
void Workspace::addAssemblyBox() {
  m_assemblyPanel.groupBox = new QGroupBox;
  m_assemblyPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));
  QGridLayout* assemblyLayout = new QGridLayout(m_assemblyPanel.groupBox);

  assemblyLayout->setMargin(0);
  assemblyLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  assemblyLayout->setSpacing(0);

  m_assemblyPanel.title = new QLabel("");
  m_assemblyPanel.title->setWordWrap(true);
  assemblyLayout->addWidget(m_assemblyPanel.title, 0, 0, 0);

  m_assemblyPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_scenePanel.groupBox);
}


/*!
  @brief add dynamic joint group box
*/
void Workspace::addJointGroupBox() {
  m_jointGroupPanel.groupBox = new QGroupBox;
  QGridLayout* jointGroupLayout = new QGridLayout(m_jointGroupPanel.groupBox);

  jointGroupLayout->setMargin(0);
  jointGroupLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  jointGroupLayout->setSpacing(0);

  m_jointGroupPanel.title = new QLabel("");
  m_jointGroupPanel.title->setWordWrap(true);
  jointGroupLayout->addWidget(m_jointGroupPanel.title, 0, 0, 0);

  m_jointGroupPanel.addJointButton = new QPushButton();
  m_jointGroupPanel.addJointButton->setText("add joint");
  jointGroupLayout->addWidget(m_jointGroupPanel.addJointButton, 1, 0, 0);

  m_jointGroupPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_jointGroupPanel.groupBox);

}


/*!
  @brief add dynamic box for modifying joints or displaying properties
*/
void Workspace::addJointBox() {
  m_jointPanel.groupBox = new QGroupBox;
  QGridLayout* jointLayout = new
  QGridLayout(m_jointPanel.groupBox);
  jointLayout->setMargin(0);
  jointLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  jointLayout->setSpacing(0);
  m_jointPanel.title = new QLabel("");
  m_jointPanel.title->setWordWrap(true);
  jointLayout->addWidget(m_jointPanel.title, 0, 0, 0);
  m_jointPanel.tx = new QLabel("");
  m_jointPanel.ty = new QLabel("");
  m_jointPanel.tz = new QLabel("");
  m_jointPanel.rx = new QLabel("");
  m_jointPanel.ry = new QLabel("");
  m_jointPanel.rz = new QLabel("");
  jointLayout->addWidget(m_jointPanel.tx, 1, 0, 0);
  jointLayout->addWidget(m_jointPanel.ty, 1, 1, 0);
  jointLayout->addWidget(m_jointPanel.tz, 1, 2, 0);
  jointLayout->addWidget(m_jointPanel.rx, 1, 3, 0);
  jointLayout->addWidget(m_jointPanel.ry, 1, 4, 0);
  jointLayout->addWidget(m_jointPanel.rz, 1, 5, 0);

  m_jointPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_jointPanel.groupBox);
}


/*!
  @brief add dynamic body group box
*/
void Workspace::addBodyGroupBox() {

  m_bodyGroupPanel.groupBox = new QGroupBox;
  m_bodyGroupPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QGridLayout* bodyGroupLayout = new QGridLayout(m_bodyGroupPanel.groupBox);
  bodyGroupLayout->setMargin(0);
  bodyGroupLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  bodyGroupLayout->setSpacing(0);

  m_bodyGroupPanel.title = new QLabel("");
  m_bodyGroupPanel.title->setWordWrap(true);
  bodyGroupLayout->addWidget(m_bodyGroupPanel.title, 0, 0, 0);

  m_bodyGroupPanel.addBodyButton = new QPushButton();
  m_bodyGroupPanel.addBodyButton->setText("add body");
  bodyGroupLayout->addWidget(m_bodyGroupPanel.addBodyButton, 1, 0, 0);

  m_bodyGroupPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_bodyGroupPanel.groupBox);
}


/*!
  @brief add dynamic body box
*/
void Workspace::addBodyBox() {
  m_bodyPanel.groupBox = new QGroupBox;
  m_bodyPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QGridLayout* materialColorLayout = new QGridLayout(m_bodyPanel.groupBox);
  materialColorLayout->setMargin(0);
  materialColorLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  materialColorLayout->setSpacing(0);

  m_bodyPanel.addFrame = new QPushButton();
  m_bodyPanel.addFrame->setText("add frame");

  m_bodyPanel.materialColorButton = new QPushButton();
  m_bodyPanel.materialColorButton->setText("select color");
  m_bodyPanel.materialColorDropDown = new QComboBox();
  m_bodyPanel.materialDropDown = new QComboBox();
  m_bodyPanel.materialColorDropDown->addItems(
    QStringList() << "ambient" << "diffuse" << "specular");
  QLabel* materialLabel = new QLabel("material");
  QLabel* colorLabel = new QLabel("color");
  QLabel* NsLabel1 = new QLabel("Ns");
  m_bodyPanel.nsModelSpinBox = new QDoubleSpinBox();
  m_bodyPanel.nsModelSpinBox->setRange(0, 10000);
  materialColorLayout->addWidget(materialLabel, 0, 0, 0);
  materialColorLayout->addWidget(m_bodyPanel.materialDropDown, 1, 0, 1, 4, 0);
  materialColorLayout->addWidget(NsLabel1, 2, 2, 0);
  materialColorLayout->addWidget(m_bodyPanel.nsModelSpinBox, 2, 3, 0);
  materialColorLayout->addWidget(colorLabel, 2, 0, 0);
  materialColorLayout->addWidget(
    m_bodyPanel.materialColorDropDown, 3, 0, 1, 2, 0);
  materialColorLayout->addWidget(
    m_bodyPanel.materialColorButton, 3, 2, 1, 2, 0);
  m_bodyPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_bodyPanel.groupBox);

  QObject::connect(
    m_bodyPanel.nsModelSpinBox,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(updateMaterialNs(double)));

  QObject::connect(
    m_bodyPanel.materialColorButton,
    SIGNAL(clicked()),
    this,
    SLOT(materialColorSelector()));
}


/*!
  @brief add dynamic frame box
*/
void Workspace::addFrameBox() {
  m_framePanel.groupBox = new QGroupBox;
  m_framePanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QGridLayout* frameLayout = new QGridLayout(m_framePanel.groupBox);
  frameLayout->setMargin(0);
  frameLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  frameLayout->setSpacing(0);

  m_framePanel.m00 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m00, 0, 0, 0);

  m_framePanel.m01 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m01, 0, 1, 0);

  m_framePanel.m02 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m02, 0, 2, 0);

  m_framePanel.m03 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m03, 0, 3, 0);

  m_framePanel.m10 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m10, 1, 0, 0);

  m_framePanel.m11 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m11, 1, 1, 0);

  m_framePanel.m12 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m12, 1, 2, 0);

  m_framePanel.m13 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m13, 1, 3, 0);

  m_framePanel.m20 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m20, 2, 0, 0);

  m_framePanel.m21 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m21, 2, 1, 0);

  m_framePanel.m22 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m22, 2, 2, 0);

  m_framePanel.m23 = new QLabel("");
  frameLayout->addWidget(m_framePanel.m23, 2, 3, 0);

  m_framePanel.m30 = new QLabel("0");
  frameLayout->addWidget(m_framePanel.m30, 3, 0, 0);

  m_framePanel.m31 = new QLabel("0");
  frameLayout->addWidget(m_framePanel.m31, 3, 1, 0);

  m_framePanel.m32 = new QLabel("0");
  frameLayout->addWidget(m_framePanel.m32, 3, 2, 0);

  m_framePanel.m33 = new QLabel("1");
  frameLayout->addWidget(m_framePanel.m33, 3, 3, 0);

  m_framePanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_framePanel.groupBox);
}


/*!
  @brief add restraint group box
*/
void Workspace::addRestraintGroupBox() {

  m_restraintGroupPanel.groupBox = new QGroupBox;
  m_restraintGroupPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QGridLayout* restraintGroupLayout =
    new QGridLayout(m_restraintGroupPanel.groupBox);
  restraintGroupLayout->setMargin(0);
  restraintGroupLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  restraintGroupLayout->setSpacing(0);


  m_restraintGroupPanel.addRestraintButton = new QPushButton();
  m_restraintGroupPanel.addRestraintButton->setText("add restraint");
  restraintGroupLayout->addWidget(
    m_restraintGroupPanel.addRestraintButton, 1, 0, 0);

  m_restraintGroupPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_restraintGroupPanel.groupBox);
}


/*!
  @brief add dynamic restraint box
*/
void Workspace::addRestraintBox() {
  m_restraintPanel.groupBox = new QGroupBox;
  m_restraintPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));
  m_restraintPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_restraintPanel.groupBox);
}


/*!
  @brief add dynamic io group box
*/
void Workspace::addIoGroupBox() {
  m_ioGroupPanel.groupBox = new QGroupBox;
  m_ioGroupPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QGridLayout* ioGroupLayout = new QGridLayout(m_ioGroupPanel.groupBox);
  ioGroupLayout->setMargin(0);
  ioGroupLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  ioGroupLayout->setSpacing(0);

  m_ioGroupPanel.addIoButton = new QPushButton();
  m_ioGroupPanel.addIoButton->setText("add input/output");
  ioGroupLayout->addWidget(m_ioGroupPanel.addIoButton, 1, 0, 0);

  m_ioGroupPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_ioGroupPanel.groupBox);

}


/*!
  @brief add dynamic io box
*/
void Workspace::addIoBox() {
  m_ioPanel.groupBox = new QGroupBox;
  m_ioPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));
  m_ioPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_ioPanel.groupBox);
}



/*!
  @brief add dynamic light group box
*/
void Workspace::addLightGroupBox() {
  m_lightGroupPanel.groupBox = new QGroupBox;
  m_lightGroupPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QGridLayout* lightGroupLayout = new QGridLayout(m_lightGroupPanel.groupBox);
  lightGroupLayout->setMargin(0);
  lightGroupLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  lightGroupLayout->setSpacing(0);

  m_lightGroupPanel.addLightButton = new QPushButton();
  m_lightGroupPanel.addLightButton->setText("add light");
  lightGroupLayout->addWidget(m_lightGroupPanel.addLightButton, 1, 0, 0);

  m_lightGroupPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_lightGroupPanel.groupBox);

  QObject::connect(
    m_lightGroupPanel.addLightButton,
    SIGNAL(clicked()),
    this,
    SLOT(addLight()));
}


/*!
  @brief add light box
*/
void Workspace::addLightBox() {
  /* Group box for changing light color */
  m_lightPanel.groupBox = new QGroupBox;
  m_lightPanel.groupBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QGridLayout* lightColorLayout = new QGridLayout(m_lightPanel.groupBox);

  m_lightPanel.deleteLightButton = new QPushButton();
  m_lightPanel.deleteLightButton->setText("delete light");
  lightColorLayout->addWidget(m_lightPanel.deleteLightButton, 0, 0, 1, 4, 0);
  QLabel* spacerLabel = new QLabel(" ");

  lightColorLayout->addWidget(spacerLabel, 1, 0, 1, 4, 0);

  m_lightPanel.lightColorButton = new QPushButton();
  m_lightPanel.lightColorButton->setText("select color");
  m_lightPanel.lightTypeDropDown = new QComboBox();
  m_lightPanel.lightTypeDropDown->addItems(
    QStringList() << "point" << "directional");
  QLabel* lightLabel = new QLabel("light type");
  lightColorLayout->addWidget(lightLabel, 2, 0, 0);
  lightColorLayout->addWidget(m_lightPanel.lightTypeDropDown, 3, 0, 1, 4, 0);

  // NsLightSpinBox = new QDoubleSpinBox();
  // NsLightSpinBox->setRange(0, 10000);
  // lightColorLayout->addWidget(NsLightSpinBox, 2, 3, 0);
  QLabel* colorLabel = new QLabel("Ns");
  lightColorLayout->addWidget(colorLabel, 4, 0, 0);
  m_lightPanel.lightColorDropDown = new QComboBox();
  // lightColorLayout->addWidget(NsLabel2, 2, 2, 0);
  m_lightPanel.lightColorDropDown->addItems(
    QStringList() << "ambient" << "diffuse" << "specular");
  lightColorLayout->addWidget(m_lightPanel.lightColorDropDown, 5, 0, 1, 2, 0);
  lightColorLayout->addWidget(m_lightPanel.lightColorButton, 5, 2, 1, 2, 0);

  m_lightPanel.groupBox->setVisible(false);
  m_dynamicPanelLayout->addWidget(m_lightPanel.groupBox);

  QObject::connect(
    m_lightPanel.lightColorButton,
    SIGNAL(clicked()),
    this,
    SLOT(lightColorSelector()));

  QObject::connect(
    m_lightPanel.deleteLightButton,
    SIGNAL(clicked()),
    this,
    SLOT(deleteLight()));

  QObject::connect(
    m_lightPanel.lightTypeDropDown,
    SIGNAL(activated(int)),
    this,
    SLOT(updateCurrentLightType(int)));

}


/*!
@brief add box to display model parameters
*/
void Workspace::addModelParametersBox() {
  m_modelParametersPanel.groupBox = new QGroupBox();
  m_modelParametersPanel.groupBox->setVisible(false);
  QGridLayout* modelParametersLayout =
    new QGridLayout(m_modelParametersPanel.groupBox);

  QLabel* integratorLabel = new QLabel("Integrator Type");
  modelParametersLayout->addWidget(
    integratorLabel, 0, 0, 0);
  m_modelParametersPanel.integratorType = new QComboBox();
  m_modelParametersPanel.integratorType->addItems(
    QStringList() << "ode113" << "ode113b");
  modelParametersLayout->addWidget(
    m_modelParametersPanel.integratorType, 0, 1, 0);

  QObject::connect(
    m_modelParametersPanel.integratorType,
    SIGNAL(currentIndexChanged(int)),
    this,
    SLOT(setIntegratorType(int)));

  QLabel* timeSpanLabel = new QLabel("Time Span");
  modelParametersLayout->addWidget(
    timeSpanLabel, 1, 0, 0);
  m_modelParametersPanel.timeSpan = new QDoubleSpinBox();
  m_modelParametersPanel.timeSpan->setValue(0);
  m_modelParametersPanel.timeSpan->setDecimals(4);
  m_modelParametersPanel.timeSpan->setMinimum(0);
  m_modelParametersPanel.timeSpan->setMaximum(10000000);
  m_modelParametersPanel.timeSpan->setKeyboardTracking(false);
  m_modelParametersPanel.timeSpan->setSingleStep(0.01);
  modelParametersLayout->addWidget(
    m_modelParametersPanel.timeSpan, 1, 1, 0);
  QObject::connect(
    m_modelParametersPanel.timeSpan,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setTimeSpan(double)));

  QLabel* timeStepLabel = new QLabel("Time Step");
  modelParametersLayout->addWidget(
    timeStepLabel, 2, 0, 0);
  m_modelParametersPanel.timeStep = new QDoubleSpinBox();
  m_modelParametersPanel.timeStep->setValue(0);
  m_modelParametersPanel.timeStep->setDecimals(4);
  m_modelParametersPanel.timeStep->setMinimum(0);
  m_modelParametersPanel.timeStep->setMaximum(10);
  m_modelParametersPanel.timeStep->setKeyboardTracking(false);
  m_modelParametersPanel.timeStep->setSingleStep(0.001);
  modelParametersLayout->addWidget(
    m_modelParametersPanel.timeStep, 2, 1, 0);
  QObject::connect(
    m_modelParametersPanel.timeStep,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setTimeStep(double)));

  QLabel* relativeErrorLabel = new QLabel("Relative Error");
  modelParametersLayout->addWidget(
    relativeErrorLabel, 3, 0, 0);
  m_modelParametersPanel.relativeError = new QDoubleSpinBox();
  m_modelParametersPanel.relativeError->setValue(0);
  m_modelParametersPanel.relativeError->setDecimals(12);
  m_modelParametersPanel.relativeError->setMinimum(0);
  m_modelParametersPanel.relativeError->setMaximum(1);
  m_modelParametersPanel.relativeError->setKeyboardTracking(false);
  m_modelParametersPanel.relativeError->setSingleStep(0.000000001);
  modelParametersLayout->addWidget(
    m_modelParametersPanel.relativeError, 3, 1, 0);
  QObject::connect(
    m_modelParametersPanel.relativeError,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setRelativeError(double)));

  QLabel* absoluteErrorLabel = new QLabel("Absolute Error");
  modelParametersLayout->addWidget(
    absoluteErrorLabel, 4, 0, 0);
  m_modelParametersPanel.absoluteError = new QDoubleSpinBox();
  m_modelParametersPanel.absoluteError->setValue(0);
  m_modelParametersPanel.absoluteError->setDecimals(12);
  m_modelParametersPanel.absoluteError->setMinimum(0);
  m_modelParametersPanel.absoluteError->setMaximum(1);
  m_modelParametersPanel.absoluteError->setKeyboardTracking(false);
  m_modelParametersPanel.absoluteError->setSingleStep(0.000000001);
  modelParametersLayout->addWidget(
    m_modelParametersPanel.absoluteError, 4, 1, 0);
  QObject::connect(
    m_modelParametersPanel.absoluteError,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setAbsoluteError(double)));

  QLabel* omegaNLabel = new QLabel("\u03C9<sub>n</sub>");
  modelParametersLayout->addWidget(
    omegaNLabel, 5, 0, 0);
  m_modelParametersPanel.omegaN = new QDoubleSpinBox();
  m_modelParametersPanel.omegaN->setValue(0);
  m_modelParametersPanel.omegaN->setDecimals(4);
  m_modelParametersPanel.omegaN->setMinimum(0);
  m_modelParametersPanel.omegaN->setMaximum(1000);
  m_modelParametersPanel.omegaN->setKeyboardTracking(false);
  m_modelParametersPanel.omegaN->setSingleStep(0.01);
  modelParametersLayout->addWidget(
    m_modelParametersPanel.omegaN, 5, 1, 0);
  QObject::connect(
    m_modelParametersPanel.omegaN,
    SIGNAL(valueChanged(double)),
    this,
    SLOT(setOmegaN(double)));

  QPushButton* runButton = new QPushButton("&Run", this);
  modelParametersLayout->addWidget(runButton, 6, 0, 1, 2);
  QObject::connect(
    runButton,
    SIGNAL(clicked()),
    this,
    SLOT(runSimulation()));

  QShortcut* runShortcut =
    new QShortcut(QKeySequence(tr("Ctrl+R", "Run")), this);
  QObject::connect(
    runShortcut,
    SIGNAL(activated()),
    this,
    SLOT(runSimulation()));
}


/*!
@brief initialize left panel
*/
void Workspace::initializeLeftPanel() {
  m_tabwidget = new MyTabWidget();
  m_tabwidget->setContentsMargins(QMargins(0, 0, 0, 0));

  /* set up scene panel, add to first tab of tab widget */
  QGroupBox* scenePanelBox = new QGroupBox();
  scenePanelBox->setContentsMargins(QMargins(0, 0, 0, 0));

  QVBoxLayout* scenePanelLayout = new QVBoxLayout(scenePanelBox);
  scenePanelLayout->setMargin(0);
  scenePanelLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  scenePanelLayout->setSpacing(0);

  QPixmap sceneIconPixMap(":/icons/scene.png");
  m_tabwidget->addTab(scenePanelBox, QIcon(sceneIconPixMap), tr("Scene"));

  /* add vertical splitter to assembly tab */
  QSplitter* scenePanelSplitter = new QSplitter();
  scenePanelSplitter->setContentsMargins(QMargins(0, 0, 0, 0));
  scenePanelSplitter->setOrientation(Qt::Vertical);
  scenePanelLayout->addWidget(scenePanelSplitter);

  /* add tree widget to top half of scene panel */
  m_treeWidget = new QTreeWidget();
  m_treeWidget->setContentsMargins(QMargins(0, 0, 0, 0));
  m_treeWidget->setIconSize(QSize(24, 24));
  m_treeWidget->setAlternatingRowColors(true);
  m_treeWidget->setColumnCount(3);
  m_treeWidget->setEditTriggers(
    QAbstractItemView::SelectedClicked |
    QAbstractItemView::DoubleClicked);
  m_treeWidget->setHeaderLabels(
    QStringList() << "" << "" << "value" << "mobility");
  m_treeWidget->resizeColumnToContents(1);
  m_treeWidget->setColumnWidth(0, 250);
  scenePanelSplitter->addWidget(m_treeWidget);

  /* add "dynamic group box" to bottom half of scene panel */
  m_dynamicGroupBox = new QGroupBox;
  m_dynamicGroupBox->setContentsMargins(QMargins(0, 0, 0, 0));
  m_dynamicPanelLayout = new QVBoxLayout(m_dynamicGroupBox);
  m_dynamicPanelLayout->setMargin(0);
  m_dynamicPanelLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  m_dynamicPanelLayout->setSpacing(0);

  scenePanelSplitter->addWidget(m_dynamicGroupBox);
  scenePanelSplitter->setHandleWidth(1);

  addDynamicSceneBox();
  addAssemblyBox();
  addJointGroupBox();
  addJointBox();
  addBodyGroupBox();
  addLightGroupBox();
  addLightBox();
  addBodyBox();
  addFrameBox();
  addRestraintGroupBox();
  addRestraintBox();
  addIoGroupBox();
  addIoBox();
  addSliders();

  m_dynamicPanelLayout->addStretch();

  /* add motion tab to layout */
  QGroupBox* motionPanelBox = new QGroupBox();
  motionPanelBox->setContentsMargins(QMargins(0, 0, 0, 0));
  QPixmap clapperIconPixMap(":/icons/clapper.png");
  m_tabwidget->addTab(motionPanelBox, QIcon(clapperIconPixMap), tr("Motion"));

  QVBoxLayout* motionPanelLayout = new QVBoxLayout(motionPanelBox);
  motionPanelLayout->setMargin(0);
  motionPanelLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  motionPanelLayout->setSpacing(0);

  QSplitter* motionPanelSplitter = new QSplitter();
  motionPanelSplitter->setOrientation(Qt::Vertical);
  motionPanelSplitter->setHandleWidth(1);
  motionPanelLayout->addWidget(motionPanelSplitter);

  addModelParametersBox();

  QGroupBox* motionSubBox = new QGroupBox();
  motionSubBox->setContentsMargins(QMargins(0, 0, 0, 0));
  QVBoxLayout* motionSubLayout = new QVBoxLayout(motionSubBox);
  motionSubLayout->setMargin(0);
  motionSubLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  motionSubLayout->setSpacing(0);
  motionSubLayout->addWidget(m_modelParametersPanel.groupBox);
  motionSubLayout->addStretch();
  motionPanelSplitter->addWidget(motionSubBox);
}


/*! @brief initialize layout

*/
void Workspace::initializeLayout() {
  /* main window and layout */
  QGroupBox* mainBox = new QGroupBox;

  mainBox->setContentsMargins(QMargins(0, 0, 0, 0));
  // mainBox->layout()->setContentsMargins(QMargins(0, 0, 0, 0));

  QVBoxLayout* mainLayout = new QVBoxLayout(mainBox);
  mainLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  mainLayout->setMargin(0);
  mainLayout->setSpacing(0);

  /* set up left panel */
  initializeLeftPanel();

  /* GL viewport */
  QGroupBox* rightPanelBox = new QGroupBox;
  rightPanelBox->setContentsMargins(QMargins(0, 0, 0, 0));
  QVBoxLayout* rightPanelLayout = new QVBoxLayout(rightPanelBox);
  rightPanelLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  rightPanelLayout->setMargin(0);
  rightPanelLayout->setSpacing(0);

  QGroupBox* renderWindowBox = new QGroupBox;
  RenderWindowWidget* glwidget = new RenderWindowWidget(renderWindowBox);
  m_renderWindow = glwidget->getRenderWindow();
  glwidget->setMinimumHeight(200);
  glwidget->setMinimumWidth(200);


  QSizePolicy policy = glwidget->sizePolicy();
  policy.setHorizontalStretch(3);
  glwidget->setSizePolicy(policy);
  renderWindowBox->setContentsMargins(QMargins(0, 0, 0, 0));
  glwidget->setContentsMargins(QMargins(0, 0, 0, 0));

  rightPanelLayout->addWidget(glwidget);


  /* initialize splitter and tab layout */

  QSplitter* mainSplitter = new QSplitter();
  mainSplitter->setContentsMargins(QMargins(0, 0, 0, 0));
  mainSplitter->setHandleWidth(1);
  mainSplitter->setOpaqueResize(true);
  mainSplitter->setCollapsible(1, true);
  mainLayout->addWidget(mainSplitter);

  /* add notification panel */
  QGroupBox* notificationPanelBox = new QGroupBox();
  notificationPanelBox->setContentsMargins(QMargins(0, 0, 0, 0));
  m_notificationPanelLayout = new QGridLayout(notificationPanelBox);
  m_notificationPanelLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  m_notificationPanelLayout->setSpacing(0);
  m_notificationPanelLayout->setMargin(0);

  m_cursorPositionLabel->setContentsMargins(QMargins(0, 0, 0, 0));
  m_notificationPanelLayout->addWidget(m_cursorPositionLabel, 0, 0);
  m_notificationPanelLayout->setColumnMinimumWidth(0, 100);

  m_progressBar = new QProgressBar;
  m_notificationPanelLayout->addWidget(m_progressBar, 0, 2);
  m_notificationPanelLayout->setColumnMinimumWidth(2, 300);
  m_progressBar->setVisible(false);

  m_playbackPanel.playIcon = QIcon(":/icons/play_button.png");
  m_playbackPanel.pauseIcon = QIcon(":/icons/pause_button.png");

  m_playbackPanel.groupBox = new QGroupBox();
  QGridLayout* playbackLayout = new QGridLayout(m_playbackPanel.groupBox);

  m_playbackPanel.playPauseButton = new QPushButton();
  m_playbackPanel.playPauseButton->setIcon(m_playbackPanel.playIcon);

  QObject::connect(
    m_playbackPanel.playPauseButton,
    SIGNAL(clicked()),
    this,
    SLOT(playPauseSimulation()));

  playbackLayout->addWidget(m_playbackPanel.playPauseButton, 0, 0);

  m_playbackPanel.slider = new QSlider(Qt::Horizontal);
  m_playbackPanel.singleStep = 100;
  m_playbackPanel.slider->setTickPosition(QSlider::NoTicks);
  m_playbackPanel.slider->setSingleStep(m_playbackPanel.singleStep);
  QObject::connect(
    m_playbackPanel.slider,
    SIGNAL(valueChanged(int)),
    this,
    SLOT(interpModel(int)));

  playbackLayout->addWidget(m_playbackPanel.slider, 0, 1);
  m_notificationPanelLayout->addWidget(m_playbackPanel.groupBox, 0, 2);
  m_playbackPanel.groupBox->setVisible(false);

  // notificationPanelBox->setFixedHeight(40);
  // notificationPanelBox->setMaximumHeight(40);

  m_notificationPanelLayout->setColumnStretch(0, 2);
  m_notificationPanelLayout->setColumnStretch(2, 2);

  QGroupBox* leftPanelBox = new QGroupBox();
  leftPanelBox->setContentsMargins(QMargins(0, 0, 0, 0));
  QVBoxLayout* leftPanelLayout = new QVBoxLayout(leftPanelBox);
  leftPanelLayout->setContentsMargins(QMargins(0, 0, 0, 0));
  leftPanelLayout->setMargin(0);
  leftPanelLayout->setSpacing(0);
  leftPanelLayout->addWidget(m_tabwidget);
  leftPanelLayout->addWidget(notificationPanelBox);

  mainSplitter->addWidget(leftPanelBox);
  mainSplitter->addWidget(glwidget);
  mainSplitter->setContentsMargins(QMargins(0, 0, 0, 0));

  setCentralWidget(mainSplitter);

  /*
    Signals and Slots
  */

  // QObject::connect(backgroundColorButton, SIGNAL(clicked()),
  //                  this, SLOT(backgroundColorSelector() ) );
  QObject::connect(
    m_renderWindow,
    SIGNAL(mouseMove()),
    this,
    SLOT(updateCursorPositionText()));

  QObject::connect(
    m_renderWindow,
    SIGNAL(mouseOut()),
    this,
    SLOT(clearCursorPositionText()));

  /* qtreewidget actions */
  QObject::connect(
    m_treeWidget,
    SIGNAL(itemSelectionChanged()),
    this,
    SLOT(getSelectedTreeItem()));

  QObject::connect(
    m_treeWidget,
    SIGNAL(itemClicked(QTreeWidgetItem*, int)),
    this,
    SLOT(toggleVisibility(QTreeWidgetItem*, int)));

  /* update opengl widget on window resize */
  QObject::connect(
    this,
    SIGNAL(windowResized()),
    m_renderWindow,
    SLOT(updateGL()));

  QObject::connect(
    mainSplitter,
    SIGNAL(splitterMoved(int, int)),
    m_renderWindow,
    SLOT(updateGL()));

  s2_Joints* joints = m_renderWindow->getJoints();
  QObject::connect(
    joints,
    SIGNAL(jointChanged()),
    this,
    SLOT(updateTree()));

  QObject::connect(
    joints,
    SIGNAL(jointChanged()),
    this,
    SLOT(updateH()));

  QObject::connect(
    m_renderWindow,
    SIGNAL(simulationUpdated(double)),
    this,
    SLOT(updateSimulationTimer(double)));

  QObject::connect(
    m_renderWindow,
    SIGNAL(simulationStopped()),
    this,
    SLOT(setPlaybackPaused()));

  QObject::connect(
    m_renderWindow,
    SIGNAL(initialized()),
    this,
    SLOT(openFileAtLoad()));

}


/*!

*/
void Workspace::toggleVisibility(QTreeWidgetItem* item, int column) {
  if(column == 1) {
    MyTreeItem* currentItem =
      static_cast<MyTreeItem*>(m_treeWidget->currentItem());
    switch(currentItem->m_userType) {
      case 1002: {
        /* assembly */
        s2_assembly_t* assembly =
          (s2_assembly_t*)(currentItem->m_object);
      }
      break;
      case 1003: {
        /* body folder */
        s2_assembly_t* assembly =
          (s2_assembly_t*)(currentItem->m_object);
        for(
          s2_body_t* bodyHead = assembly->bodyHead;
          bodyHead;
          bodyHead = bodyHead->next) {

          bodyHead->isVisible = !bodyHead->isVisible;

        }
        for(int i = 0; i < item->childCount(); ++i) {
          MyTreeItem* child = static_cast<MyTreeItem*>(item->child(i));
          s2_body_t* body = (s2_body_t*)(child->m_object);
          body->isVisible = !body->isVisible;
          if(body->isVisible) {
            item->setIcon(1, QIcon(":/icons/visible.png"));
          } else {
            item->setIcon(1, QIcon(":/icons/invisible.png"));
          }
        }
      }
      break;
      case 1004: {
        /* body */
        s2_body_t* body = (s2_body_t*)(currentItem->m_object);
        body->isVisible = !body->isVisible;
        updateItemDrawState(
          currentItem,
          body->isVisible,
          m_renderWindow->getBodies()->isEnabled());
      }
      break;
      case 1005: {
        /* frame */
        s2_frame_t* frame = (s2_frame_t*)(currentItem->m_object);
        qDebug() << "toggling visibility of frame";
        s2_stackstr_print(stdout, frame->name);
        frame->isVisible = !frame->isVisible;
        updateItemDrawState(
          currentItem,
          frame->isVisible,
          m_renderWindow->getFrames()->isEnabled());
      }
      break;
      case 1006: {
        /* joint folder */


      } break;
      case 1007: {
        /* joint */
        s2_joint_t* joint = (s2_joint_t*)(currentItem->m_object);
        qDebug() << "toggling visibility of frame";
        s2_stackstr_print(stdout, joint->name);
        joint->isVisible = !joint->isVisible;
        updateItemDrawState(
          currentItem,
          joint->isVisible,
          m_renderWindow->getJoints()->isEnabled());
      }
      break;
      case 1008: {
        /* light folder */

      } break;
      case 1009: {
        /* light */

      } break;
      case 1010: {
        /* restraint folder */

      } break;
      case 1011: {
        /* restraint */

      } break;
      case 1012: {
        /* io folder */

      } break;
      case 1013: {
        /* io */

      } break;
      case 1014: {
        /* terrain folder */
      } break;
      case 1015: {
        /* terrain */
        s2_terrain_t* terrain = (s2_terrain_t*)(currentItem->m_object);
        terrain->isVisible = !terrain->isVisible;
        updateItemDrawState(
          currentItem,
          terrain->isVisible,
          m_renderWindow->getTerrain()->isEnabled());
      }
      break;
      case 1016: {
        /* point cloud folder */
      } break;
      case 1017: {
        /* point cloud */
        s2_pointcloud_t* pointcloud = (s2_pointcloud_t*)(currentItem->m_object);
        pointcloud->isVisible = !pointcloud->isVisible;
        updateItemDrawState(
          currentItem,
          pointcloud->isVisible,
          m_renderWindow->getPointClouds()->isEnabled());
      }
      break;


      default: {

      } break;
    }
  }
}

void Workspace::resizeEvent(QResizeEvent* event) {
  m_renderWindow->updateGL();
}


void Workspace::initializeMenus() {

  setWindowTitle("Shift");
  QMenu* fileMenu = menuBar()->addMenu("&File");

  /* save dialog */
  QAction* actionOpen = new QAction("Open Model...", this);
  actionOpen->setShortcut(QKeySequence::Open);
  fileMenu->addAction(actionOpen);

  QAction* actionSave = new QAction("Save Model", this);
  actionSave->setShortcut(QKeySequence::Save);
  fileMenu->addAction(actionSave);

  QAction* actionSaveAs = new QAction("Save Model As..", this);
  actionSaveAs->setShortcut(QKeySequence::SaveAs);
  fileMenu->addAction(actionSaveAs);

  fileMenu->addSeparator();

  /* add quit dialog */
  QAction* actionQuit = new QAction("Quit", this);
#ifdef Q_WS_MACX
  actionQuit->setShortcut(QKeySequence::Quit);
#else
  actionQuit->setShortcut(Qt::Key_Q | Qt::CTRL);
#endif
  fileMenu->addAction(actionQuit);

  QMenu* viewMenu = menuBar()->addMenu("&View");

  QActionGroup* perspectiveGroup = new QActionGroup(this);

  QAction* actionSetOrthogonal = new QAction("Orthogonal", this);
  actionSetOrthogonal->setCheckable(true);
  perspectiveGroup->addAction(actionSetOrthogonal);

  QAction* actionSetPerspective = new QAction("Perspective", this);
  perspectiveGroup->addAction(actionSetPerspective);
  actionSetPerspective->setCheckable(true);
  actionSetPerspective->setChecked(true);

  // viewMenu->addActions(perspectiveGroup);
  viewMenu->addAction(actionSetOrthogonal);
  viewMenu->addAction(actionSetPerspective);

  viewMenu->addSeparator();

  QActionGroup* shadingGroup = new QActionGroup(this);

  QAction* actionSetWireframe = new QAction("Wireframe", this);
  viewMenu->addAction(actionSetWireframe);

  QAction* actionSetFlatShaded = new QAction("Flat Shaded", this);
  viewMenu->addAction(actionSetFlatShaded);

  QAction* actionSetPhongShaded = new QAction("Phong Shaded", this);
  viewMenu->addAction(actionSetPhongShaded);

  shadingGroup->addAction(actionSetWireframe);
  actionSetWireframe->setCheckable(true);
  shadingGroup->addAction(actionSetFlatShaded);
  actionSetFlatShaded->setCheckable(true);

  shadingGroup->addAction(actionSetPhongShaded);
  actionSetPhongShaded->setCheckable(true);
  actionSetPhongShaded->setChecked(true);

  viewMenu->addSeparator();

  // QAction* actionToggleVertexNormals =
  //     new QAction("Display Vertex Normals", this);
  // viewMenu->addAction(actionToggleVertexNormals);
  // actionToggleVertexNormals->setCheckable(true);
  // actionToggleVertexNormals->setChecked(false);

  QAction* m_actionToggleDrawFrames =
    new QAction("Frames", this);
  viewMenu->addAction(m_actionToggleDrawFrames);
  m_actionToggleDrawFrames->setIcon(QIcon(":/icons/frame.png"));
  m_actionToggleDrawFrames->setCheckable(true);
  m_actionToggleDrawFrames->setChecked(false);

  QAction* m_actionToggleDrawJoints =
    new QAction("Joints", this);
  viewMenu->addAction(m_actionToggleDrawJoints);
  m_actionToggleDrawJoints->setIcon(QIcon(":/icons/joint.png"));
  m_actionToggleDrawJoints->setCheckable(true);
  m_actionToggleDrawJoints->setChecked(false);

  m_actionToggleDrawTerrain =
    new QAction("Terrain", this);
  viewMenu->addAction(m_actionToggleDrawTerrain);
  m_actionToggleDrawTerrain->setIcon(QIcon(":/icons/terrain.png"));
  m_actionToggleDrawTerrain->setCheckable(true);
  m_actionToggleDrawTerrain->setChecked(true);

  m_actionToggleDrawGrid = new QAction("Grid", this);
  viewMenu->addAction(m_actionToggleDrawGrid);
  m_actionToggleDrawGrid->setIcon(QIcon(":/icons/grid.png"));
  m_actionToggleDrawGrid->setCheckable(true);
  m_actionToggleDrawGrid->setChecked(true);

  m_actionToggleDrawShadows =
    new QAction("Shadows", this);
  viewMenu->addAction(m_actionToggleDrawShadows);
  m_actionToggleDrawShadows->setIcon(QIcon(":/icons/shadow.png"));
  m_actionToggleDrawShadows->setCheckable(true);
  m_actionToggleDrawShadows->setChecked(true);

  m_actionToggleDrawConvexHulls =
    new QAction("Collision Geometry", this);
  viewMenu->addAction(m_actionToggleDrawConvexHulls);
  m_actionToggleDrawConvexHulls->setIcon(QIcon(":/icons/convexhull.png"));
  m_actionToggleDrawConvexHulls->setCheckable(true);
  m_actionToggleDrawConvexHulls->setChecked(false);

  m_actionToggleDrawNormals =
    new QAction("Normals", this);
  viewMenu->addAction(m_actionToggleDrawNormals);
  m_actionToggleDrawNormals->setIcon(QIcon(":/icons/normal.png"));
  m_actionToggleDrawNormals->setCheckable(true);
  m_actionToggleDrawNormals->setChecked(false);

  // m_actionToggleDrawBoundingSpheres =
  //   new QAction("Bounding Spheres", this);
  // viewMenu->addAction(m_actionToggleDrawBoundingSpheres);
  // m_actionToggleDrawBoundingSpheres->setIcon(QIcon(":/icons/sphere.png"));
  // m_actionToggleDrawBoundingSpheres->setCheckable(true);
  // m_actionToggleDrawBoundingSpheres->setChecked(false);

  QMenu* helpMenu = menuBar()->addMenu("&Help");

  QAction* actionAbout = new QAction("About this program ...", this);
  helpMenu->addAction(actionAbout);

  QAction* actionLicense = new QAction("License", this);
  helpMenu->addAction(actionLicense);

  /* file menu actions */
  QObject::connect(actionOpen, SIGNAL(triggered()), this, SLOT(openFile()));
  QObject::connect(actionSave, SIGNAL(triggered()), this, SLOT(saveFile()));
  QObject::connect(actionSaveAs, SIGNAL(triggered()), this, SLOT(saveFileAs()));
  QObject::connect(actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  /* view menu actions */
  QObject::connect(
    actionSetPerspective,
    SIGNAL(triggered()),
    m_renderWindow,
    SLOT(setPerspective()));

  QObject::connect(
    actionSetOrthogonal,
    SIGNAL(triggered()),
    m_renderWindow,
    SLOT(setOrthogonal()));

  QObject::connect(
    actionSetWireframe,
    SIGNAL(triggered()),
    m_renderWindow->getBodies(),
    SLOT(setWireframe()));

  QObject::connect(
    actionSetFlatShaded,
    SIGNAL(triggered()),
    m_renderWindow->getBodies(),
    SLOT(setFlatShaded()));

  QObject::connect(
    actionSetPhongShaded,
    SIGNAL(triggered()),
    m_renderWindow->getBodies(),
    SLOT(setPhongShaded()));

  QObject::connect(
    m_actionToggleDrawFrames,
    SIGNAL(triggered()),
    m_renderWindow->getFrames(),
    SLOT(toggleDraw()));

  QObject::connect(
    m_actionToggleDrawFrames,
    SIGNAL(triggered()),
    this,
    SLOT(updateDrawState()));

  QObject::connect(
    m_actionToggleDrawJoints,
    SIGNAL(triggered()),
    m_renderWindow->getJoints(),
    SLOT(toggleDraw()));

  QObject::connect(
    m_actionToggleDrawJoints,
    SIGNAL(triggered()),
    this,
    SLOT(updateDrawState()));

  QObject::connect(
    m_actionToggleDrawTerrain,
    SIGNAL(triggered()),
    m_renderWindow->getTerrain(),
    SLOT(toggleDraw()));

  QObject::connect(
    m_actionToggleDrawTerrain,
    SIGNAL(triggered()),
    this,
    SLOT(updateDrawState()));

  QObject::connect(
    m_actionToggleDrawGrid,
    SIGNAL(triggered()),
    m_renderWindow->getGrid(),
    SLOT(toggleDraw()));

  QObject::connect(
    m_actionToggleDrawGrid,
    SIGNAL(triggered()),
    this,
    SLOT(updateDrawState()));

  QObject::connect(
    m_actionToggleDrawShadows,
    SIGNAL(triggered()),
    m_renderWindow->getLights(),
    SLOT(toggleDraw()));

  QObject::connect(
    m_actionToggleDrawConvexHulls,
    SIGNAL(triggered()),
    m_renderWindow->getBodies(),
    SLOT(toggleDrawConvexHulls()));


  QObject::connect(
    m_actionToggleDrawNormals,
    SIGNAL(triggered()),
    m_renderWindow->getBodies(),
    SLOT(toggleDrawNormals()));

  // QObject::connect(
  //   m_actionToggleDrawBoundingSpheres,
  //   SIGNAL(triggered()),
  //   m_renderWindow->getBodies(),
  //   SLOT(toggleDrawBoundingSpheres()));

  QObject::connect(
    m_actionToggleDrawConvexHulls,
    SIGNAL(triggered()),
    this,
    SLOT(updateDrawState()));


  /*
    help menu actions
  */
  QObject::connect(actionAbout, SIGNAL(triggered()), this, SLOT(about()));
  QObject::connect(actionLicense, SIGNAL(triggered()), this, SLOT(license()));
}


/*!
  @brief add a light to assembly

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

  should be accessible only from "light group" pane (1008)
*/
void Workspace::addLight() {
  if(m_currentSelectedItem->m_userType == 1008) {
    // create and initialize new light
    // Light * light = new Light();
    // for( int i=0; i<3; i++ )
    // {
    //     light->Kd[i]=0.2;
    //     light->Ka[i]=0.4;
    //     light->Ks[i]=0.6;
    // }
    // light->rotation.setX(0.f);
    // light->rotation.setY(0.f);
    // light->rotation.setZ(0.f);
    // light->translation.setY(0.f);
    // light->translation.setX(0.f);
    // light->translation.setZ(3.f);
    //
    // light->transform.setToIdentity();
    // light->transform.translate( light->translation );
    // light->transform.rotate( light->rotation.x(), 1, 0, 0);
    // light->transform.rotate( light->rotation.y(), 0, 1, 0);
    // light->transform.rotate( light->rotation.z(), 0, 0, 1);

    // append light to scenes
    // get current selected scene, make sure a scene is selected

    // RenderWindow* renWin = glwidget->GetRenderWindow();
    // QString name =
    //     QString("light").append(
    //         QString::number(renWin->lightParam.numLights));
    //
    // renWin->lightParam.numLights++;
    // light->name = name;
    // light->type = POINT;

    // qDebug() << "adding light: " << name;
    // int sceneIndex = treeWidget->indexOfTopLevelItem(currentItem);
    // qDebug() << "adding light to scene" << scenes[sceneIndex]->name;
    // Node* out = new Node();
    // out->type = QString("light");
    // out->children.push_back( light );
    // scenes[sceneIndex]->root.push_back( out );
    // populateTree();
    // lightArray.append( light );
    // renWin->updateLights(lightArray );
  }
}


/*!
  @brief delete light from assembly

  should only be accessible from "light" pane (1009)
*/
void Workspace::deleteLight() {

  if(m_currentSelectedItem->m_userType ==  1009) {

    //     RenderWindow* renWin = glwidget->GetRenderWindow();
    //     renWin->updateLights( lightArray );
    //     populateTree();
  }
}


/*!
  @brief update specular highlight for material

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

  should only be accessible from "body" pane (1004)
*/
void Workspace::updateMaterialNs(double Ns) {
  if(m_currentSelectedItem->m_userType == 1004) {
    // Model* currentModel = static_cast<Model*>(m_currentSelectedItem->object);
    // get current selected material
    // currentModel->materials[materialDropDown->currentIndex()].Ns = Ns;
  }
}


/*!
  @brief update current light type

  0 for point
  1 for directional

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

  should only be accessible from "light" pane (1009)
*/
void Workspace::updateCurrentLightType(int lightType) {
  if(m_currentSelectedItem->m_userType == 1009) {
    s2_light_t* light = (s2_light_t*)m_currentSelectedItem->m_object;

    //     currentLight->lightType = (LightType) lightType;
    //     RenderWindow* renWin = glwidget->GetRenderWindow();
    //     renWin->updateLights( lightArray );
  }
}


/*!
  @brief update transform of shape matrices

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

  should only be accessible from panes: "frame" (1005) and "joint" (1007 -- for rigid joints only)
*/
void Workspace::updateTransform() {
  if(m_currentSelectedItem->m_userType == 1005) {

    s2_frame_t* frame = (s2_frame_t*)m_currentSelectedItem->m_object;
    // Model* currentModel = static_cast<Model*>(m_currentSelectedItem->object);
    // currentModel->transform.setToIdentity();
    // currentModel->transform.translate( currentModel->translation );
    // currentModel->transform.rotate( currentModel->rotation.x(), 1, 0, 0);
    // currentModel->transform.rotate( currentModel->rotation.y(), 0, 1, 0);
    // currentModel->transform.rotate( currentModel->rotation.z(), 0, 0, 1);

  } else if(m_currentSelectedItem->m_userType == 1007) {
    s2_joint_t* joint = (s2_joint_t*)m_currentSelectedItem->m_object;

  }
}

/*!
  @brief recenters Tx slider with current value of m_currentValue
*/
void Workspace::recenterTx() {
  recenterSlider(&m_sliders.tx);
}

/*!
  @brief recenters Tx slider with current value of m_currentValue
*/
void Workspace::recenterTy() {
  recenterSlider(&m_sliders.ty);
}

/*!
  @brief recenters Tx slider with current value of m_currentValue
*/
void Workspace::recenterTz() {
  recenterSlider(&m_sliders.tz);
}

/*!
  @brief recenters Tx slider with current value of m_currentValue
*/
void Workspace::recenterRx() {
  recenterSlider(&m_sliders.rx);
}

/*!
  @brief recenters Tx slider with current value of m_currentValue
*/
void Workspace::recenterRy() {
  recenterSlider(&m_sliders.ry);
}

/*!
  @brief recenters Tx slider with current value of m_currentValue
*/
void Workspace::recenterRz() {
  recenterSlider(&m_sliders.rz);
}


/*!
  @brief updates Tx for prismatic joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setTx(int val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;

      double delq = (double)val * m_sliders.tx.singleStep;

      // qDebug() << delq;

      m_renderWindow->setJoint(
        joint,
        joint->qIndex,
        joint->dqIndex,
        delq,
        S2_JOINT_PRIMITIVE);
      m_sliders.tx.spinBox->blockSignals(true);
      m_sliders.tx.spinBox->setValue(m_context->q[joint->qIndex] + delq);
      m_sliders.tx.spinBox->blockSignals(false);
    }
    emit txChanged(0);
  }
}


/*!
  @brief updates Ty for prismatic joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setTy(int val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;

      int qIndex = -1;
      int dqIndex = -1;
      switch(joint->jointType) {
        case S2_JOINT_FLOAT: {
          qIndex = joint->qIndex + 1;
          dqIndex = joint->dqIndex + 1;
        }
        break;
        default: {
          qIndex = joint->qIndex;
          dqIndex = joint->dqIndex;
        }
        break;
      }

      double delq = (double)val * m_sliders.ty.singleStep;
      m_sliders.ty.spinBox->blockSignals(true);
      m_sliders.ty.spinBox->setValue(m_context->q[qIndex] + delq);
      m_sliders.ty.spinBox->blockSignals(false);

      m_renderWindow->setJoint(
        joint,
        qIndex,
        dqIndex,
        delq,
        S2_JOINT_PRIMITIVE);
    }
    emit tyChanged(0);
  }
}


/*!
  @brief updates Tz for prismatic joints, spherical joints and float joints
  also used to update shape matrices
*/
void Workspace::setTz(int val) {
  if(m_currentSelectedItem != NULL) {
    switch(m_currentSelectedItem->m_userType) {
      case S2_MODELTREE_FRAME: {
        updateTransform();
        break;
      }
      case S2_MODELTREE_JOINT: {
        s2_joint_t* joint = (s2_joint_t*)m_currentSelectedItem->m_object;
        int qIndex = -1;
        int dqIndex = -1;
        switch(joint->jointType) {
          case S2_JOINT_FLOAT: {
            qIndex = joint->qIndex + 2;
            dqIndex = joint->dqIndex + 2;
          }
          break;
          default: {
            qIndex = joint->qIndex;
            dqIndex = joint->dqIndex;
          }
          break;
        }
        double delq = (double)val * m_sliders.tz.singleStep;
        m_sliders.tz.spinBox->blockSignals(true);
        m_sliders.tz.spinBox->setValue(m_context->q[qIndex] + delq);
        m_sliders.tz.spinBox->blockSignals(false);

        m_renderWindow->setJoint(
          joint,
          qIndex,
          dqIndex,
          delq,
          S2_JOINT_PRIMITIVE);
        break;
      }
      default: {
        break;
      }
    }
    emit tzChanged(0);
  }
}


/*!
  @brief updates Rx for revolute joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io
*/
void Workspace::setRx(int val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
      double delq = (double)val * m_sliders.rx.singleStep;
      switch(joint->jointType) {
        case S2_JOINT_SPHERICAL: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex,
            joint->dqIndex,
            delq,
            S2_JOINT_SPHERICAL);
          m_sliders.rx.spinBox->blockSignals(true);
          m_sliders.rx.spinBox->setValue(m_context->q[joint->qIndex] + delq);
          m_sliders.rx.spinBox->blockSignals(false);
        }
        break;
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex + 3,
            joint->dqIndex + 3,
            delq,
            S2_JOINT_SPHERICAL);
          m_sliders.rx.spinBox->blockSignals(true);
          m_sliders.rx.spinBox->setValue(
            m_context->q[joint->qIndex + 3] + delq);
          m_sliders.rx.spinBox->blockSignals(false);
        }
        break;
        default: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex,
            joint->dqIndex,
            delq,
            S2_JOINT_PRIMITIVE);
          m_sliders.rx.spinBox->blockSignals(true);
          m_sliders.rx.spinBox->setValue(m_context->q[joint->qIndex] + delq);
          m_sliders.rx.spinBox->blockSignals(false);
        }
        break;
      }
    }
    emit rxChanged(0);
  }
}


/*!
  @brief updates Rx for revolute joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io
*/
void Workspace::setRy(int val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
      double delq = (double)val * m_sliders.ry.singleStep;
      switch(joint->jointType) {
        case S2_JOINT_SPHERICAL: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex,
            joint->dqIndex,
            delq,
            S2_JOINT_SPHERICAL);
          m_sliders.ry.spinBox->blockSignals(true);
          m_sliders.ry.spinBox->setValue(m_context->q[joint->qIndex] + delq);
          m_sliders.ry.spinBox->blockSignals(false);
        }
        break;
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex + 3,
            joint->dqIndex + 3,
            delq,
            S2_JOINT_SPHERICAL);
          m_sliders.ry.spinBox->blockSignals(true);
          m_sliders.ry.spinBox->setValue(
            m_context->q[joint->qIndex + 3] + delq);
          m_sliders.ry.spinBox->blockSignals(false);
        }
        break;
        default: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex,
            joint->dqIndex,
            delq,
            S2_JOINT_PRIMITIVE);
          m_sliders.ry.spinBox->blockSignals(true);
          m_sliders.ry.spinBox->setValue(m_context->q[joint->qIndex] + delq);
          m_sliders.ry.spinBox->blockSignals(false);
        }
        break;
      }
    }
    emit ryChanged(0);
  }
}


/*!
  @brief updates Rx for revolute joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setRz(int val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */

      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
      double delq = (double)val * m_sliders.rz.singleStep;
      switch(joint->jointType) {
        case S2_JOINT_SPHERICAL: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex,
            joint->dqIndex,
            delq,
            S2_JOINT_SPHERICAL);
          m_sliders.rz.spinBox->blockSignals(true);
          m_sliders.rz.spinBox->setValue(m_context->q[joint->qIndex] + delq);
          m_sliders.rz.spinBox->blockSignals(false);
        }
        break;
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex + 3,
            joint->dqIndex + 3,
            delq,
            S2_JOINT_SPHERICAL);
          m_sliders.rz.spinBox->blockSignals(true);
          m_sliders.rz.spinBox->setValue(
            m_context->q[joint->qIndex + 3] + delq);
          m_sliders.rz.spinBox->blockSignals(false);
        }
        break;
        default: {
          m_renderWindow->setJoint(
            joint,
            joint->qIndex,
            joint->dqIndex,
            delq,
            S2_JOINT_PRIMITIVE);
          m_sliders.rz.spinBox->blockSignals(true);
          m_sliders.rz.spinBox->setValue(m_context->q[joint->qIndex] + delq);
          m_sliders.rz.spinBox->blockSignals(false);
        }
        break;
      }
    }
    emit rzChanged(0);
  }
}


/*!
  @brief resolve constraint violations before using Tx slider
*/
void Workspace::onclickTx() {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      // updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* drive constraint error to zero before subsecquent fast factorization */
      if(m_sliders.tx.substructure) {
        s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
          m_sliders.tx.substructure,
          m_sliders.tx.dqIndex);
      }
    }
  }
}


/*!
  @brief resolve constraint violations before using Ty slider
*/
void Workspace::onclickTy() {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      // updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* drive constraint error to zero before subsecquent fast factorization */
      if(m_sliders.ty.substructure) {
        s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
          m_sliders.ty.substructure,
          m_sliders.ty.dqIndex);
      }
    }
  }
}


/*!
  @brief resolve constraint violations before using Tz slider
*/
void Workspace::onclickTz() {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      // updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* drive constraint error to zero before subsecquent fast factorization */
      if(m_sliders.tz.substructure) {
        s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
          m_sliders.tz.substructure,
          m_sliders.tz.dqIndex);
      }
    }
  }
}


/*!
  @brief resolve constraint violations before using Rx slider
*/
void Workspace::onclickRx() {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      // updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* drive constraint error to zero before subsecquent fast factorization */
      qDebug() << "initialize rx slider";
      if(m_sliders.rx.substructure) {
        s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
          m_sliders.rx.substructure,
          m_sliders.rx.dqIndex);
      }
    }
  }
}


/*!
  @brief resolve constraint violations before using Ry slider
*/
void Workspace::onclickRy() {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      // updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* drive constraint error to zero before subsecquent fast factorization */
      if(m_sliders.ry.substructure) {
        s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
          m_sliders.ry.substructure,
          m_sliders.ry.dqIndex);
      }
    }
  }
}


/*!
  @brief resolve constraint violations before using Rz slider
*/
void Workspace::onclickRz() {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      // updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {

      /* drive constraint error to zero before subsecquent fast factorization */
      if(m_sliders.rz.substructure) {
        s2_substructure_setupNewtonRaphsonSpecifiedCoordinate(
          m_sliders.rz.substructure,
          m_sliders.rz.dqIndex);
      }
    }
  }
}



/*!
  @brief updates Tx for prismatic joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setTxSpinBox(double val) {
  if(m_currentSelectedItem != NULL) {
    switch(m_currentSelectedItem->m_userType) {
      case S2_MODELTREE_FRAME: {
        updateTransform();
        break;
      }
      case S2_MODELTREE_JOINT: {
        updateH();
        /* process a joint */
        s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
        switch(joint->jointType) {
          case S2_JOINT_SPHERICAL: {
            m_renderWindow->setJointTarget(
              joint,
              joint->qIndex,
              joint->dqIndex,
              val,
              S2_JOINT_SPHERICAL);
            break;
          }
          default: {
            m_renderWindow->setJointTarget(
              joint,
              joint->qIndex,
              joint->dqIndex,
              val,
              S2_JOINT_PRIMITIVE);
            break;
          }
        }
        break;
      }
      default: {
        break;
      }
    }
    emit txChanged(0);
  }
}


/*!
  @brief updates Ty for prismatic joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setTySpinBox(double val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {
      updateH();
      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;

      switch(joint->jointType) {
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex + 1,
            joint->dqIndex + 1,
            val,
            S2_JOINT_PRIMITIVE);
        }
        break;
        default: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_PRIMITIVE);
        }
        break;
      }
    }
    emit tyChanged(0);
  }
}


/*!
  @brief updates Tz for prismatic joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body folder
    * 1004: body
    * 1005: frame
    * 1006: joint folder
    * 1007: joint
    * 1008: light folder
    * 1009: light
    * 1010: restraint folder
    * 1011: restraint
    * 1012: io folder
    * 1013: io

*/
void Workspace::setTzSpinBox(double val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {
      updateH();
      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
      switch(joint->jointType) {
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex + 2,
            joint->dqIndex + 2,
            val,
            S2_JOINT_PRIMITIVE);
        }
        break;
        default: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_PRIMITIVE);
        }
        break;
      }
    }
    emit tzChanged(0);
  }
}


/*!
  @brief updates Rx for revolute joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setRxSpinBox(double val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {
      updateH();
      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
      switch(joint->jointType) {
        case S2_JOINT_SPHERICAL: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_SPHERICAL);
        }
        break;
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex + 3,
            joint->dqIndex + 3,
            val,
            S2_JOINT_SPHERICAL);
        }
        break;
        default: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_PRIMITIVE);
        }
        break;
      }
    }
    emit rxChanged(0);
  }
}


/*!
  @brief updates Rx for revolute joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setRySpinBox(double val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {
      updateH();
      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
      switch(joint->jointType) {
        case S2_JOINT_SPHERICAL: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_SPHERICAL);
        }
        break;
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex + 3,
            joint->dqIndex + 3,
            val,
            S2_JOINT_SPHERICAL);
        }
        break;
        default: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_PRIMITIVE);
        }
        break;
      }
    }
    emit ryChanged(0);
  }
}


/*!
  @brief updates Rx for revolute joints, spherical joints and float joints
  also used to update shape matrices

    * 1002: assembly
    * 1003: body group
    * 1004: body
    * 1005: frame
    * 1006: joint group
    * 1007: joint
    * 1008: light group
    * 1009: light
    * 1010: restraint group
    * 1011: restraint
    * 1012: io group
    * 1013: io

*/
void Workspace::setRzSpinBox(double val) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1005) {

      /* process a frame */
      updateTransform();

    } else if(m_currentSelectedItem->m_userType == 1007) {
      updateH();
      /* process a joint */
      s2_joint_t* joint = (s2_joint_t*) m_currentSelectedItem->m_object;
      switch(joint->jointType) {
        case S2_JOINT_SPHERICAL: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_SPHERICAL);
        }
        break;
        case S2_JOINT_FLOAT: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex + 3,
            joint->dqIndex + 3,
            val,
            S2_JOINT_SPHERICAL);
        }
        break;
        default: {
          m_renderWindow->setJointTarget(
            joint,
            joint->qIndex,
            joint->dqIndex,
            val,
            S2_JOINT_PRIMITIVE);
        }
        break;
      }
    }
    emit rzChanged(0);
  }
}


/*!
  @brief populates material color drop down in the body group box
  with body colors
*/
void Workspace::populateMaterialColorDropDown(s2_material_t* material) {

  QPixmap currentColor(40, 40);
  currentColor.fill(QColor(
                      material->GLuniforms.Ka[0] * 255,
                      material->GLuniforms.Ka[1] * 255,
                      material->GLuniforms.Ka[2] * 255));
  m_bodyPanel.materialColorDropDown->setItemIcon(0, QIcon(currentColor));
  currentColor.fill(
    QColor(
      material->GLuniforms.Kd[0] * 255,
      material->GLuniforms.Kd[1] * 255,
      material->GLuniforms.Kd[2] * 255));
  m_bodyPanel.materialColorDropDown->setItemIcon(1, QIcon(currentColor));
  currentColor.fill(
    QColor(
      material->GLuniforms.Ks[0] * 255,
      material->GLuniforms.Ks[1] * 255,
      material->GLuniforms.Ks[2] * 255));
  m_bodyPanel.materialColorDropDown->setItemIcon(2, QIcon(currentColor));
  qDebug() << "setting value" << material->GLuniforms.Ns;
  m_bodyPanel.nsModelSpinBox->setValue((double)material->GLuniforms.Ns);
}


/*!
  @brief populate the color drop down menus in the lightBox
*/
void Workspace::populateLightColorDropDown(s2_light_t* light) {
  QPixmap currentColor(40, 40);
  currentColor.fill(
    QColor(
      light->glLight.Ka[0] * 255,
      light->glLight.Ka[1] * 255,
      light->glLight.Ka[2] * 255));
  m_lightPanel.lightColorDropDown->setItemIcon(0, QIcon(currentColor));
  currentColor.fill(
    QColor(
      light->glLight.Kd[0] * 255,
      light->glLight.Kd[1] * 255,
      light->glLight.Kd[2] * 255));
  m_lightPanel.lightColorDropDown->setItemIcon(1, QIcon(currentColor));
  currentColor.fill(
    QColor(
      light->glLight.Ks[0] * 255,
      light->glLight.Ks[1] * 255,
      light->glLight.Ks[2] * 255));
  m_lightPanel.lightColorDropDown->setItemIcon(2, QIcon(currentColor));
}


/*!
  @brief used to update the current material, currently not implemented

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

  should be accessible only from body pane
*/
void Workspace::updateCurrentMaterial(int index) {
  if(m_currentSelectedItem != NULL) {
    if(m_currentSelectedItem->m_userType == 1004) {
      s2_body_t* body = (s2_body_t*)m_currentSelectedItem->m_object;

      // populate material Dropdown
      qDebug() << "material" << index << "updated";
      // populateMaterialColorDropDown(&currentModel->materials[index]);
    }
  }
}


/*!
@brief initialize a joint slider with default values
*/
void Workspace::setSlider(
  SliderHandle* sliderHandle,
  int qIndex,
  int dqIndex,
  s2_substructure_t* substructure,
  bool isSpherical) {


  sliderHandle->qIndex = qIndex;
  sliderHandle->dqIndex = dqIndex;
  if(substructure != NULL) {
    sliderHandle->substructure = substructure;
  } else {
    sliderHandle->substructure = NULL;
  }
  sliderHandle->isSpherical = isSpherical;
  sliderHandle->spinBoxValue = m_context->q + qIndex;
  sliderHandle->box->setVisible(true);
  recenterSlider(sliderHandle);
}


/*!
@brief update display of influence coefficient matrices
*/
void Workspace::updateH() {
  if(m_currentSelectedItem->m_userType == S2_MODELTREE_JOINT) {
    /* joint */
    s2_joint_t* joint =
      (s2_joint_t*)(m_currentSelectedItem->m_object);
    if(joint != NULL) {
      if(joint->h != NULL) {
        char number[20];
        sprintf(number, "%2.3f", joint->h->m0);
        m_jointPanel.tx->setText(number);
        sprintf(number, "%2.3f", joint->h->m1);
        m_jointPanel.ty->setText(number);
        sprintf(number, "%2.3f", joint->h->m2);
        m_jointPanel.tz->setText(number);
        sprintf(number, "%2.3f", joint->h->m3);
        m_jointPanel.rx->setText(number);
        sprintf(number, "%2.3f", joint->h->m4);
        m_jointPanel.ry->setText(number);
        sprintf(number, "%2.3f", joint->h->m5);
        m_jointPanel.rz->setText(number);
      }
    }
  }
}

/*!
  @brief gets selected tree item and update dynamic window with parameters

  The following userTypes are used
  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

*/
void Workspace::getSelectedTreeItem() {

  clearSubstructureHighlight();

  m_currentSelectedItem =
    static_cast<MyTreeItem*>(m_treeWidget->currentItem());

  // qDebug() << m_currentSelectedItem->m_userType;

  if(m_currentSelectedItem != NULL) {
    clearDynamicBox();

    switch(m_currentSelectedItem->m_userType) {

      case S2_MODELTREE_ASSEMBLY: {
        s2_assembly_t* assembly =
          (s2_assembly_t*)(m_currentSelectedItem->m_object);
        qDebug() << s2_getRootName(assembly->name);
        break;
      }
      case S2_MODELTREE_BODY_GROUP: {
        m_bodyGroupPanel.groupBox->setVisible(true);
        break;
      }
      case S2_MODELTREE_BODY: {
        s2_body_t* body = (s2_body_t*)(m_currentSelectedItem->m_object);
        qDebug() << s2_getRootName(body->name);
        m_bodyPanel.groupBox->setVisible(true);
        m_bodyPanel.materialDropDown->clear();

        /* populate material dropdown menu */
        for(
          int i = body->materialIndex;
          i < body->materialIndex + body->materialCount;
          i++) {
          m_bodyPanel.materialDropDown->addItem(m_context->materials[i].name);
        }
        if(body->materials != NULL) {
          populateMaterialColorDropDown(
            &m_context->materials[body->materialIndex]);
        }
        break;
      }
      case S2_MODELTREE_FRAME: {
        s2_frame_t* frame = (s2_frame_t*)(m_currentSelectedItem->m_object);
        qDebug() << s2_getRootName(frame->name);

        char number[20];
        sprintf(number, "%2.3f", frame->dAbsolute->R.m00);
        m_framePanel.m00->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m01);
        m_framePanel.m01->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m02);
        m_framePanel.m02->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->T.x);
        m_framePanel.m03->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m10);
        m_framePanel.m10->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m11);
        m_framePanel.m11->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m12);
        m_framePanel.m12->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->T.y);
        m_framePanel.m13->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m20);
        m_framePanel.m20->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m21);
        m_framePanel.m21->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->R.m22);
        m_framePanel.m22->setText(number);

        sprintf(number, "%2.3f", frame->dAbsolute->T.z);
        m_framePanel.m23->setText(number);

        m_framePanel.groupBox->setVisible(true);
        break;
      }
      case S2_MODELTREE_JOINT_GROUP: {
        m_jointGroupPanel.groupBox->setVisible(true);
        break;
      }
      case S2_MODELTREE_JOINT: {
        s2_joint_t* joint = (s2_joint_t*)(m_currentSelectedItem->m_object);

        highlightSubstructure(joint->substructurePointer);
        updateH();
        clearSliders();
        m_jointPanel.groupBox->setVisible(true);

        switch(joint->jointType) {
          case S2_JOINT_TX: {
            setSlider(
              &m_sliders.tx,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              false);
            break;
          }
          case S2_JOINT_TY: {
            setSlider(
              &m_sliders.ty,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              false);
            break;
          }
          case S2_JOINT_TZ: {
            setSlider(
              &m_sliders.tz,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              false);
            break;
          }
          case S2_JOINT_RX: {
            setSlider(
              &m_sliders.rx,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              false);
            break;
          }
          case S2_JOINT_RY: {
            setSlider(
              &m_sliders.ry,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              false);
            break;
          }
          case S2_JOINT_RZ: {
            setSlider(
              &m_sliders.rz,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              false);
            break;
          }
          case S2_JOINT_RIGID: {
            break;
          }
          case S2_JOINT_SPHERICAL: {
            setSlider(
              &m_sliders.rx,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              true);
            setSlider(
              &m_sliders.ry,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              true);
            setSlider(
              &m_sliders.rz,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              true);
            break;
          }
          case S2_JOINT_FLOAT: {
            setSlider(
              &m_sliders.tx,
              joint->qIndex,
              joint->dqIndex,
              joint->substructurePointer,
              true);
            setSlider(
              &m_sliders.ty,
              joint->qIndex + 1,
              joint->dqIndex + 1,
              joint->substructurePointer,
              true);
            setSlider(
              &m_sliders.tz,
              joint->qIndex + 2,
              joint->dqIndex + 2,
              joint->substructurePointer,
              true);
            setSlider(
              &m_sliders.rx,
              joint->qIndex + 3,
              joint->dqIndex + 3,
              joint->substructurePointer,
              true);
            setSlider(
              &m_sliders.ry,
              joint->qIndex + 3,
              joint->dqIndex + 3,
              joint->substructurePointer,
              true);
            setSlider(
              &m_sliders.rz,
              joint->qIndex + 3,
              joint->dqIndex + 3,
              joint->substructurePointer,
              true);
            break;
          }
          case S2_JOINT_INVALID:
          case S2_JOINT_PRIMITIVE: {
            break;
          }
        }
        break;
      }
      case S2_MODELTREE_LIGHT_GROUP: {
        m_lightGroupPanel.groupBox->setVisible(true);
        break;
      }
      case S2_MODELTREE_LIGHT: {
        s2_light_t* light = (s2_light_t*) m_currentSelectedItem->m_object;
        m_lightPanel.groupBox->setVisible(true);
        populateLightColorDropDown(light);
        break;
      }
      case S2_MODELTREE_RESTRAINT_GROUP: {
        m_restraintGroupPanel.groupBox->setVisible(true);
        break;
      }
      case S2_MODELTREE_RESTRAINT: {
        s2_restraint_t* restraint =
          (s2_restraint_t*)m_currentSelectedItem->m_object;
        qDebug() << s2_getRootName(restraint->name);
        m_restraintPanel.groupBox->setVisible(true);
        break;
      }
      case S2_MODELTREE_IO_GROUP: {
        m_ioGroupPanel.groupBox->setVisible(true);
        break;
      }
      case S2_MODELTREE_IO: {
        s2_io_t* io = (s2_io_t*) m_currentSelectedItem->m_object;
        qDebug() << s2_getRootName(io->name);
        m_ioPanel.groupBox->setVisible(true);
        break;
      }
      default: {
        break;
      }
    }
  }
}


/*!
  @brief set visible flag of all dynamic group boxes to false
*/
void Workspace::clearDynamicBox() {

  clearSliders();
  m_scenePanel.groupBox->setVisible(false);
  m_assemblyPanel.groupBox->setVisible(false);
  m_jointGroupPanel.groupBox->setVisible(false);
  m_jointPanel.groupBox->setVisible(false);
  m_bodyGroupPanel.groupBox->setVisible(false);
  m_bodyPanel.groupBox->setVisible(false);
  m_framePanel.groupBox->setVisible(false);
  m_restraintGroupPanel.groupBox->setVisible(false);
  m_restraintPanel.groupBox->setVisible(false);
  m_ioGroupPanel.groupBox->setVisible(false);
  m_ioPanel.groupBox->setVisible(false);
  m_lightGroupPanel.groupBox->setVisible(false);
  m_lightPanel.groupBox->setVisible(false);
}


/*!
  @brief set visible flag for all slider boxes to false
*/
void Workspace::clearSliders() {
  m_sliders.rx.box->setVisible(false);
  m_sliders.ry.box->setVisible(false);
  m_sliders.rz.box->setVisible(false);
  m_sliders.tx.box->setVisible(false);
  m_sliders.ty.box->setVisible(false);
  m_sliders.tz.box->setVisible(false);
}


/*!
  @brief used to recenter slider at current joint position
  +/- PI for revolute joints
  +/- 1 for prismatic joints

*/
void Workspace::recenterSlider(
  SliderHandle* sliderHandle) {

  sliderHandle->slider->blockSignals(true);
  if(QApplication::keyboardModifiers() != Qt::ShiftModifier) {
    m_renderWindow->clearJoint();
  }

  sliderHandle->spinBox->blockSignals(true);
  sliderHandle->spinBox->setValue(*sliderHandle->spinBoxValue);
  sliderHandle->spinBox->blockSignals(false);

  sliderHandle->slider->setValue(0);
  sliderHandle->slider->blockSignals(false);
}


/*!
  @brief render x,y cursor position in status bar
*/
void Workspace::updateCursorPositionText() {
  QString positionText = m_renderWindow->getCursorPositionText();
  m_cursorPositionLabel->setText(positionText);
}


/*!
@brief clears cursor position text in status bar
*/
void Workspace::clearCursorPositionText() {
  m_cursorPositionLabel->clear();
}



/*!
@brief open an external model definition

Opens an external model definition file and loads objects recursively
in to treewidget
*/
void Workspace::openFileAtLoad() {

  QStringList arguments = QCoreApplication::arguments();
  if(arguments.count() == 2) {
    QString fileName = arguments.at(1);

    m_renderWindow->setAnimating(false);

    QByteArray byteArray = fileName.toLatin1();

    if(fileName.length() != 0) {
      m_currentFileName = fileName;
      setWindowTitle(fileName + QString(" | Shift"));

      m_context = s2_context_load(byteArray.data());
      s2_context_setTopology(m_context);
      qDebug() << "loading from openFileAtLoad: " << fileName;
      m_renderWindow->loadContext(m_context);
      populateTree();
    }

    if(m_context) {
      FILE* debug = m_context->modelParameters.debugFilePointer;
      s2_context_initializeJacobian(m_context);
      s2_context_evaluateConstraints(m_context);

      /* print bodies */
      for(int i = 0; i < m_context->activeJointCount; i++) {
        fprintf(
          debug,
          "body [%d] isVirtual: %s ",
          i,
          m_context->bodyPointers[i]->isVirtual ? "true" : "false");
        s2_stackstr_print(debug, m_context->bodyPointers[i]->name);

      }

      /* printing constraints */
      fprintf(debug, "chord constraint error:");
      for(int i = 0; i < m_context->chordConstraintCount; i++) {
        fprintf(debug, "[%d] ", i);
        for(int j = 0; j < 6; j++) {
          fprintf(debug, "%0.12e, ", *(m_context->cs + 6 * i + j));
        }
        fprintf(debug, "\n");
      }

      for(int i = 0; i < m_context->substructureCount; i++) {
        s2_substructure_t* substructure = m_context->substructures + i;
        double error = s2_substructure_evaluateConstraintError(substructure);
        fprintf(debug, "constraint violations before load: %0.16e\n", error);
      }

      /* print constraints not in substructures */
      // for(int i = 0; i < m_context->chordConstraintCount; i++) {
      //   s2_chordconstraint_t* constraint = m_context->chordConstraints + i;
      //   printf("[%d] chord constraint\n", i);
      //
      //   printf(" parent body: ");
      //   s2_stackstr_print(constraint->parentBodyPointer->name);
      //   s2_displacement_print(constraint->parentFramePointer->dAbsolute);
      //
      //   printf(" child body: ");
      //   s2_stackstr_print(constraint->childBodyPointer->name);
      //   s2_displacement_print(constraint->childFramePointer->dAbsolute);
      //
      //   s2_displacement_t temp;
      //   s2_product_ddinverse(
      //     constraint->parentFramePointer->dAbsolute,
      //     constraint->childFramePointer->dAbsolute,
      //     &temp);
      //   qDebug() << " error:";
      //   s2_displacement_print(&temp);
      //
      // }

      /* print constraints from within substructures */
      // for(int sub = 0; sub < m_context->substructureCount; sub++) {
      //
      //   s2_substructure_t* substructure = m_context->substructures + sub;
      //   for(int i = 0; i < substructure->chordConstraintCount; i++) {
      //     printf("\n[%d] chord constraint in substructure:\n", i);
      //     s2_displacement_t* parentFrame =
      //       m_context->d + substructure->constraints[i][3];
      //     printf(" parent frame:");
      //     s2_displacement_print(parentFrame);
      //     s2_displacement_t* childFrame =
      //       m_context->d + substructure->constraints[i][2];
      //     qDebug() << " child frame";
      //     s2_displacement_print(childFrame);
      //     s2_displacement_t temp;
      //     s2_product_ddinverse(parentFrame, childFrame, &temp);
      //     qDebug() << " error:";
      //     s2_displacement_print(&temp);
      //   }
      // }

      s2_context_removeConstraintViolations(m_context);
      for(int i = 0; i < m_context->substructureCount; i++) {
        s2_substructure_t* substructure = m_context->substructures + i;
        double error =
          s2_substructure_evaluateConstraintError(substructure);
        printf("constraint violations after Newton-Raphson at load: %g\n", error);
      }

      s2_context_updateDH(m_context);
      s2_context_setJacobianPO1(m_context);

      s2_context_initializeMobility(m_context);
      s2_context_computeMobility(m_context);
      s2_context_printMobility(debug, m_context);

      for(int i = 0; i < m_context->activeJointCount; i++) {
        fprintf(
          m_mobilityFile,
          "q:%s%s",
          s2_getRootName(m_context->jointPointers[i]->name),
          m_context->modelParameters.delimiter);
      }

      for(int i = 0; i < m_context->activeJointCount; i++) {
        if(i != m_context->activeJointCount - 1) {
          fprintf(
            m_mobilityFile,
            "m:%s%s",
            s2_getRootName(m_context->jointPointers[i]->name),
            m_context->modelParameters.delimiter);
        } else {
          fprintf(
            m_mobilityFile,
            "m:%s\n",
            s2_getRootName(m_context->jointPointers[i]->name));
        }
      }

      updateTree();
      loadModelParameters();
      if(m_context->terrainCount == 0) {
        drawTerrainToggleActive(false);
      } else {
        drawTerrainToggleActive(true);
      }
      // s2_context_print(m_context);
    }

    m_renderWindow->setAnimating(true);
  }
}


/*!
@brief open an external model definition

Opens an external model definition file and loads objects recursively
in to treewidget
*/
void Workspace::openFile() {
  m_renderWindow->setAnimating(false);

  QString fileName =
    QFileDialog::getOpenFileName(
      this,
      tr("Open File"),
      "",
      tr("json files (*.json)"));

  QByteArray byteArray = fileName.toLatin1();

  if(fileName.length() != 0) {
    m_currentFileName = fileName;

    setWindowTitle(fileName + QString(" | Shift"));

    m_context = s2_context_load(byteArray.data());
    s2_context_setTopology(m_context);
    qDebug() << "loading from openFile: " << fileName;
    m_renderWindow->loadContext(m_context);
    populateTree();
  }

  if(m_context) {
    FILE* debug = m_context->modelParameters.debugFilePointer;
    s2_context_initializeJacobian(m_context);
    s2_context_evaluateConstraints(m_context);

    /* print bodies */
    for(int i = 0; i < m_context->activeJointCount; i++) {
      printf(
        "body [%d] isVirtual: %s ",
        i,
        m_context->bodyPointers[i]->isVirtual ? "true" : "false");
      s2_stackstr_print(debug, m_context->bodyPointers[i]->name);

    }

    /* printing constraints */
    fprintf(debug, "chord constraint error:");
    for(int i = 0; i < m_context->chordConstraintCount; i++) {
      fprintf(debug, "[%d] ", i);
      for(int j = 0; j < 6; j++) {
        fprintf(debug, "%0.12e, ", *(m_context->cs + 6 * i + j));
      }
      printf("\n");
    }

    for(int i = 0; i < m_context->substructureCount; i++) {
      s2_substructure_t* substructure = m_context->substructures + i;
      double error = s2_substructure_evaluateConstraintError(substructure);
      fprintf(debug, "constraint violations before load: %0.16e\n", error);
    }

    /* print constraints not in substructures */
    // for(int i = 0; i < m_context->chordConstraintCount; i++) {
    //   s2_chordconstraint_t* constraint = m_context->chordConstraints + i;
    //   printf("[%d] chord constraint\n", i);
    //
    //   printf(" parent body: ");
    //   s2_stackstr_print(constraint->parentBodyPointer->name);
    //   s2_displacement_print(constraint->parentFramePointer->dAbsolute);
    //
    //   printf(" child body: ");
    //   s2_stackstr_print(constraint->childBodyPointer->name);
    //   s2_displacement_print(constraint->childFramePointer->dAbsolute);
    //
    //   s2_displacement_t temp;
    //   s2_product_ddinverse(
    //     constraint->parentFramePointer->dAbsolute,
    //     constraint->childFramePointer->dAbsolute,
    //     &temp);
    //   qDebug() << " error:";
    //   s2_displacement_print(&temp);
    //
    // }

    /* print constraints from within substructures */
    // for(int sub = 0; sub < m_context->substructureCount; sub++) {
    //
    //   s2_substructure_t* substructure = m_context->substructures + sub;
    //   for(int i = 0; i < substructure->chordConstraintCount; i++) {
    //     printf("\n[%d] chord constraint in substructure:\n", i);
    //     s2_displacement_t* parentFrame =
    //       m_context->d + substructure->constraints[i][3];
    //     printf(" parent frame:");
    //     s2_displacement_print(parentFrame);
    //     s2_displacement_t* childFrame =
    //       m_context->d + substructure->constraints[i][2];
    //     qDebug() << " child frame";
    //     s2_displacement_print(childFrame);
    //     s2_displacement_t temp;
    //     s2_product_ddinverse(parentFrame, childFrame, &temp);
    //     qDebug() << " error:";
    //     s2_displacement_print(&temp);
    //   }
    // }

    s2_context_removeConstraintViolations(m_context);
    for(int i = 0; i < m_context->substructureCount; i++) {
      s2_substructure_t* substructure = m_context->substructures + i;
      double error = s2_substructure_evaluateConstraintError(substructure);
      printf("constraint violations after Newton-Raphson at load: %g\n", error);
    }

    s2_context_updateDH(m_context);
    s2_context_setJacobianPO1(m_context);

    s2_context_initializeMobility(m_context);
    s2_context_computeMobility(m_context);
    s2_context_printMobility(debug, m_context);

    for(int i = 0; i < m_context->activeJointCount; i++) {
      if(m_context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
        fprintf(
          m_mobilityFile,
          "q:%s%s",
          s2_getRootName(m_context->jointPointers[i]->name),
          m_context->modelParameters.delimiter);
      }
    }

    for(int i = 0; i < m_context->activeJointCount; i++) {
      if(i != m_context->activeJointCount - 1) {
        if(m_context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
          fprintf(
            m_mobilityFile,
            "m:%s%s",
            s2_getRootName(m_context->jointPointers[i]->name),
            m_context->modelParameters.delimiter);
        }
      } else {
        if(m_context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
          fprintf(
            m_mobilityFile,
            "m:%s\n",
            s2_getRootName(m_context->jointPointers[i]->name));
        }
      }
    }

    updateTree();
    loadModelParameters();
    if(m_context->terrainCount == 0) {
      drawTerrainToggleActive(false);
    } else {
      drawTerrainToggleActive(true);
    }
    // s2_context_print(m_context);
  }

  m_renderWindow->setAnimating(true);
}


/*!
@brief write line of joint variables and mobility numbers to m_mobilityFile,
called after s2_context_computeMobility
*/
void Workspace::writeMobility() {

  /* write states */
  for(int i = 0; i < m_context->qCount; i++) {
    fprintf(
      m_mobilityFile,
      "%0.12f%s",
      m_context->q[i],
      m_context->modelParameters.delimiter);
  }

  /* write mobility */
  for(int i = 0; i < m_context->dqCount - 1; i++) {
    fprintf(
      m_mobilityFile,
      "%0.12f%s",
      m_context->mobilityNumbers[i],
      m_context->modelParameters.delimiter);
  }
  if(m_context->dqCount > 0) {
    fprintf(
      m_mobilityFile,
      "%0.12f\n",
      m_context->mobilityNumbers[m_context->dqCount - 1]);
  }
}


/*!
  @brief populates model tree recursively

  The following userTypes are used
  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io
*/
void Workspace::populateTreeRecursive(
  s2_assembly_t* assemblyHead,
  QTreeWidgetItem* widget) {

  s2_assembly_t* subAssemblyHead = assemblyHead->subAssemblyHead;

  while(subAssemblyHead) {

    /* add assembly */
    MyTreeItem* subAssembly = new MyTreeItem(S2_MODELTREE_ASSEMBLY); // assembly
    subAssembly->setText(0, s2_getRootName(subAssemblyHead->name));
    subAssembly->setIcon(0, QIcon(":/icons/assembly.png"));
    subAssembly->m_object = subAssemblyHead;
    widget->addChild(subAssembly);

    /* add bodies */
    MyTreeItem* bodies = new MyTreeItem(S2_MODELTREE_BODY_GROUP); // body group
    bodies->setText(0, "bodies");
    bodies->setIcon(0, QIcon(":/icons/body_folder.png"));
    subAssembly->addChild(bodies);

    s2_body_t* bodyHead = subAssemblyHead->bodyHead;
    while(bodyHead) {
      MyTreeItem* item = new MyTreeItem(S2_MODELTREE_BODY); // body
      item->setText(0, s2_getRootName(bodyHead->name));
      item->setIcon(0, QIcon(":/icons/body.png"));
      item->setIcon(1, QIcon(":/icons/visible.png"));
      item->m_object = bodyHead;
      bodies->addChild(item);

      /* add frames */
      s2_frame_t* frameHead = bodyHead->frameHead;
      while(frameHead) {
        MyTreeItem* subItem = new MyTreeItem(S2_MODELTREE_FRAME); // frame
        subItem->setText(0, s2_getRootName(frameHead->name));
        subItem->m_object = frameHead;
        subItem->setIcon(0, QIcon(":/icons/frame.png"));
        subItem->setIcon(1, QIcon(":/icons/invisible.png"));
        item->addChild(subItem);
        frameHead = frameHead->next;
      }
      item->setExpanded(false);
      bodyHead = bodyHead->next;
    }
    bodies->setExpanded(true);

    /* add joints */
    MyTreeItem* joints = new MyTreeItem(S2_MODELTREE_JOINT_GROUP); // joint group
    joints->setText(0, "joints");
    joints->setIcon(0, QIcon(":/icons/joint_folder.png"));

    subAssembly->addChild(joints);

    s2_joint_t* jointHead = subAssemblyHead->jointHead;
    while(jointHead) {
      MyTreeItem* item = new MyTreeItem(S2_MODELTREE_JOINT); // joint
      item->setText(0, s2_getRootName(jointHead->name));
      qDebug() << "loading joint" << s2_getRootName(jointHead->name);

      /* set joint value in second column */
      // char numberString[50];
      // if(jointHead->primitiveDisplacement != NULL) {
      //   sprintf(numberString, "%0.6f", *(jointHead->primitiveDisplacement));
      //   item->setText(1, numberString);
      // }

      item->setIcon(0, QIcon(":/icons/joint.png"));
      item->setIcon(1, QIcon(":/icons/invisible.png"));
      item->m_object = jointHead;
      joints->addChild(item);
      jointHead = jointHead->next;
    }
    joints->setExpanded(true);

    /* add lights */
    MyTreeItem* lights = new MyTreeItem(S2_MODELTREE_LIGHT_GROUP); // light group
    lights->setText(0, "lights");
    lights->setIcon(0, QIcon(":/icons/light_folder.png"));
    subAssembly->addChild(lights);

    s2_light_t* lightHead = subAssemblyHead->lightHead;
    while(lightHead) {
      MyTreeItem* item = new MyTreeItem(S2_MODELTREE_LIGHT); // light
      item->setText(0, s2_getRootName(lightHead->name));
      item->setIcon(0, QIcon(":/icons/light.png"));
      item->setIcon(1, QIcon(":/icons/invisible.png"));
      item->m_object = lightHead;
      lights->addChild(item);
      lightHead = lightHead->next;
    }
    lights->setExpanded(false);

    /* add restraints */
    MyTreeItem* restraints = new MyTreeItem(S2_MODELTREE_RESTRAINT_GROUP); // restraint group
    restraints->setText(0, "restraints");
    restraints->setExpanded(true);
    restraints->setIcon(0, QIcon(":/icons/restraint_folder.png"));
    subAssembly->addChild(restraints);

    s2_restraint_t* restraintHead = subAssemblyHead->restraintHead;
    while(restraintHead) {
      MyTreeItem* item = new MyTreeItem(S2_MODELTREE_RESTRAINT); // restraint
      item->setText(0, s2_getRootName(restraintHead->name));
      item->setIcon(0, QIcon(":/icons/restraint.png"));
      item->setIcon(1, QIcon(":/icons/invisible.png"));
      item->m_object = restraintHead;
      restraints->addChild(item);
      restraintHead = restraintHead->next;
    }

    /* add input / outputs */
    MyTreeItem* io = new MyTreeItem(S2_MODELTREE_IO_GROUP); /* io group */
    io->setText(0, "i/o");
    io->setExpanded(true);
    io->setIcon(0, QIcon(":/icons/io_folder.png"));
    subAssembly->addChild(io);

    /*! @TODO load io data structures */
    // Device* deviceHead = subAssemblyHead->deviceHead;
    // while(deviceHead) {
    //   MyTreeItem* item = new MyTreeItem(1013); /* io */
    //   item->setText(0, deviceHead->name);
    //   devices->addChild(item);
    //   devices->m_object = deviceHead;
    //   deviceHead = deviceHead->next;
    // }

    MyTreeItem* pointCloudFolder = new MyTreeItem(1016);
    pointCloudFolder->setText(0, "pointclouds");
    pointCloudFolder->setExpanded(true);
    pointCloudFolder->setIcon(0, QIcon(":/icons/pointcloud_folder.png"));
    subAssembly->addChild(pointCloudFolder);

    s2_pointcloud_t* pointCloudHead = subAssemblyHead->pointCloudHead;
    while(pointCloudHead) {
      MyTreeItem* item = new MyTreeItem(1017);
      item->setText(0, s2_getRootName(pointCloudHead->name));
      item->setIcon(0, QIcon(":/icons/pointcloud.png"));
      item->setIcon(1, QIcon(":/icons/visible.png"));
      item->m_object = pointCloudHead;
      pointCloudFolder->addChild(item);
      pointCloudHead = pointCloudHead->next;
    }

    /* add subassemblies */
    populateTreeRecursive(subAssemblyHead, subAssembly);
    subAssembly->setExpanded(true);
    subAssemblyHead = subAssemblyHead->next;
  }
}

/*!
@brief disables turning on/off terrain maps if no terrain maps are loaded
*/
void Workspace::drawTerrainToggleActive(bool active) {
  m_actionToggleDrawTerrain->setEnabled(active);
  m_actionToggleDrawTerrain->setChecked(active);
  updateDrawState();
}

/*!
  @brief populate model treewidget with s2_context_t data structure

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

*/
void Workspace::populateTree() {

  m_treeWidget->clear();

  /* load fixed properties */
  MyTreeItem* fixed = new MyTreeItem(1002);
  fixed->setText(0, "fixed");
  fixed->setIcon(0, QIcon(":/icons/anchor.png"));
  fixed->m_object = m_context->assembly;

  m_treeWidget->invisibleRootItem()->addChild(fixed);

  /* load fixed frames */
  MyTreeItem* fixedFrames = new MyTreeItem(1014); // frames group
  fixedFrames->setText(0, "frames");
  fixedFrames->setIcon(0, QIcon(":/icons/frame_folder.png"));
  fixed->addChild(fixedFrames);

  s2_frame_t* frameHead = m_context->assembly->bodyHead->frameHead;
  while(frameHead) {
    MyTreeItem* item = new MyTreeItem(1005); // frame
    item->setText(0, s2_getRootName(frameHead->name));
    item->setIcon(0, QIcon(":/icons/frame.png"));
    item->m_object = frameHead;
    fixedFrames->addChild(item);
    frameHead = frameHead->next;
  }
  fixedFrames->setExpanded(false);

  /* load fixed lights */
  MyTreeItem* fixedLights = new MyTreeItem(1008); // light group
  fixedLights->setText(0, "lights");
  fixedLights->setIcon(0, QIcon(":/icons/light_folder.png"));
  fixed->addChild(fixedLights);

  s2_light_t* lightHead = m_context->assembly->lightHead;
  while(lightHead) {
    MyTreeItem* item = new MyTreeItem(1009); // frame
    item->setText(0, s2_getRootName(lightHead->name));
    item->setIcon(0, QIcon(":/icons/light.png"));
    item->m_object = lightHead;
    fixedLights->addChild(item);
    lightHead = lightHead->next;
  }
  fixedLights->setExpanded(false);
  fixed->setExpanded(true);

  /* load fixed terrains */
  MyTreeItem* fixedTerrainsFolder = new MyTreeItem(1014);
  fixedTerrainsFolder->setText(0, "terrain");
  fixedTerrainsFolder->setIcon(0, QIcon(":/icons/terrain_folder.png"));
  fixed->addChild(fixedTerrainsFolder);
  for(int i = 0; i < m_context->terrainCount; i++) {
    s2_terrain_t* terrain = m_context->terrains[i];
    MyTreeItem* item = new MyTreeItem(1015);
    item->setText(0, s2_getRootName(terrain->name));
    item->setIcon(0, QIcon(":/icons/terrain.png"));
    item->m_object = terrain;
    fixedTerrainsFolder->addChild(item);
  }

  /* load all other bodies, joints, frames */
  populateTreeRecursive(m_context->assembly, m_treeWidget->invisibleRootItem());
  updateDrawState();
  m_currentSelectedItem =
    static_cast<MyTreeItem*>(m_treeWidget->currentItem());
}


/*!
  @brief highlight treewidget items belonging to current substructure
*/
void Workspace::highlightSubstructure(s2_substructure_t* substructure) {

  QTreeWidgetItemIterator it(m_treeWidget);
  QPalette palette = this->palette();

  while(*it) {
    MyTreeItem* item = static_cast<MyTreeItem*>(*it);
    if(item->m_userType == 1007) {
      s2_joint_t* joint = (s2_joint_t*)item->m_object;
      if(joint->substructurePointer != NULL &&
          joint->substructurePointer == substructure) {
        if(palette.isBrushSet(QPalette::Active, QPalette::HighlightedText)) {
          item->setForeground(0, QBrush(palette.highlightedText()));
        } else {
          item->setForeground(0, Qt::red);
        }
      } else {
        item->setForeground(0, palette.text());
      }
    }
    ++it;
  }
}


/*!
@brief clear treewidget highlighting
*/
void Workspace::clearSubstructureHighlight() {

  QTreeWidgetItemIterator it(m_treeWidget);

  while(*it) {
    MyTreeItem* item = static_cast<MyTreeItem*>(*it);
    if(item->m_userType == 1007) {
      QPalette palette = this->palette();
      item->setForeground(0, palette.text());
    }
    ++it;
  }
}


/*!
@brief updates joint angles displayed in tree
*/
void Workspace::updateTree() {

  QTreeWidgetItemIterator it(m_treeWidget);

  while(*it) {
    char numberString[100];
    MyTreeItem* item = static_cast<MyTreeItem*>(*it);
    switch(item->m_userType) {
      case S2_MODELTREE_ASSEMBLY: {
        if(item->text(0) == QString("fixed")) {
          sprintf(
            numberString,
            "%0.4f",
            m_context->mobility);
          (*it)->setText(3, numberString);
        }
        break;
      }
      case S2_MODELTREE_JOINT: {
        s2_joint_t* joint = (s2_joint_t*)item->m_object;
        qDebug() << "updating joint" << s2_getRootName(joint->name) << " dof:" << joint->dof;
        if(joint->primitiveDisplacement != NULL) {
          sprintf(
            numberString,
            "%0.4f",
            *(joint->primitiveDisplacement));
          (*it)->setText(2, numberString);
        }
        if(joint->dof == 0) {
          sprintf(numberString, "0");
        } else if(joint->dof == 1) {
          sprintf(
            numberString,
            "%0.4f",
            joint->mobilityNumber[0]);
        } else if(joint->dof == 3) {
          sprintf(
            numberString,
            "%0.4f, %0.4f, %0.4f",
            joint->mobilityNumber[0],
            joint->mobilityNumber[1],
            joint->mobilityNumber[2]);
        } else if(joint->dof == 6) {
          sprintf(
            numberString,
            "%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f",
            joint->mobilityNumber[0],
            joint->mobilityNumber[1],
            joint->mobilityNumber[2],
            joint->mobilityNumber[3],
            joint->mobilityNumber[4],
            joint->mobilityNumber[5]);
        }
        (*it)->setText(3, numberString);
        break;
      }
      default: {
        break;
      }
    }
    ++it;
  }
  writeMobility();
}


void Workspace::updateItemDrawState(
  MyTreeItem* item,
  bool isVisible,
  bool isCategoryEnabled) {
  if(isVisible && isCategoryEnabled) {
    item->setIcon(1, QIcon(":/icons/visible.png"));
  } else if(isVisible && !isCategoryEnabled) {
    item->setIcon(1, QIcon(":/icons/visible_grey.png"));
  } else if(!isVisible && isCategoryEnabled) {
    item->setIcon(1, QIcon(":/icons/invisible.png"));
  } else {
    item->setIcon(1, QIcon(":/icons/invisible_grey.png"));
  }
}
/*!
@brief toggle draw state
*/
void Workspace::updateDrawState() {

  QTreeWidgetItemIterator it(m_treeWidget);
  while(*it) {
    MyTreeItem* item = static_cast<MyTreeItem*>(*it);
    switch(item->m_userType) {
      case 1002: {
        /* assembly */
        s2_assembly_t* assembly =
          (s2_assembly_t*)(item->m_object);
      }
      break;
      case 1003: {
        /* body folder */
      } break;
      case 1004: {
        /* body */
        s2_body_t* body = (s2_body_t*)(item->m_object);
        updateItemDrawState(
          item,
          body->isVisible,
          m_renderWindow->getBodies()->isEnabled());
      }
      break;
      case 1005: {
        /* frame */
        s2_frame_t* frame = (s2_frame_t*)(item->m_object);
        updateItemDrawState(
          item,
          frame->isVisible,
          m_renderWindow->getFrames()->isEnabled());
      }
      break;
      case 1006: {
        /* joint folder */


      } break;
      case 1007: {
        /* joint */
        s2_joint_t* joint = (s2_joint_t*)(item->m_object);
        updateItemDrawState(
          item,
          joint->isVisible,
          m_renderWindow->getJoints()->isEnabled());
      }
      break;
      case 1008: {
        /* light folder */

      } break;
      case 1009: {
        /* light */
        s2_light_t* light = (s2_light_t*)(item->m_object);
        updateItemDrawState(
          item,
          light->isVisible,
          false);
      }
      break;
      case 1010: {
        /* restraint folder */

      } break;
      case 1011: {
        /* restraint */

      } break;
      case 1012: {
        /* io folder */

      } break;
      case 1013: {
        /* io */

      } break;
      case 1014: {
        /* terrain folder */
      } break;
      case 1015: {
        /* terrain */
        s2_terrain_t* terrain = (s2_terrain_t*)(item->m_object);
        updateItemDrawState(
          item,
          terrain->isVisible,
          m_renderWindow->getTerrain()->isEnabled());
      }
      break;
      case 1016: {
        /* point cloud folder */
      } break;
      case 1017: {
        s2_pointcloud_t* pointcloud = (s2_pointcloud_t*)(item->m_object);
        updateItemDrawState(
          item,
          pointcloud->isVisible,
          m_renderWindow->getPointClouds()->isEnabled());
        /* point cloud */
      }
      break;
      default: {

      } break;
    }
    ++it;
  }


}


/*!
  @brief display license dialog
*/
void Workspace::license() {

  QString fileContents;
  QFile file(":/html/license.html");
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();

  QMessageBox messageBox;
  messageBox.setWindowTitle("License");
  QTextBrowser scrollArea;
  QGridLayout* layout = (QGridLayout*)messageBox.layout();
  layout->addWidget(&scrollArea, 0, 0, 1, 1);
  scrollArea.setText(fileContents);
  scrollArea.setFixedSize(800, 800);
  messageBox.exec();
}


/*!
  @brief display about dialog
*/
void Workspace::about() {
  QString fileContents;
  QFile file(":/html/about.html");
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  fileContents = file.readAll();
  file.close();

  QMessageBox messageBox;
  messageBox.setWindowTitle("About");
  QTextBrowser scrollArea;
  QGridLayout* layout = (QGridLayout*)messageBox.layout();
  layout->addWidget(&scrollArea, 0, 0, 1, 1);
  scrollArea.setText(fileContents);
  scrollArea.setFixedSize(800, 800);
  messageBox.exec();
}


/*!
  @brief save changes to json configuration file, currently not implemented

  @TODO add feature to allow saving modified context
  datastructure
*/
void Workspace::saveFile() {
  // sceneWrite(currentFileName, scenes);
}


/*!
  @brief save changes to json configuration file, currently not implemented
*/
void Workspace::saveFileAs() {
  m_renderWindow->setAnimating(false);
  QString fileName = QFileDialog::getSaveFileName(
                       this,
                       tr("Save File"),
                       "",
                       tr("JSON Files (*.json)"));
  QMessageBox::information(
    this,
    "File Selected",
    fileName.length() == 0 ? "No File Selected" : fileName);
  // sceneWrite(fileName, m_context);
  m_renderWindow->setAnimating(true);
}


/*!
  @brief display color selection dialog to change material color

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

  valid only for body pane (1004)

*/
void Workspace::materialColorSelector() {
  m_renderWindow->setAnimating(false);
  if(
    m_currentSelectedItem != NULL &&
    m_currentSelectedItem->m_userType == 1004) {

    QColor color =
      QColorDialog::getColor(
        Qt::white,
        this,
        "Select Color",
        QColorDialog::DontUseNativeDialog);

    if(color.isValid()) {
      s2_body_t* body = (s2_body_t*)m_currentSelectedItem->m_object;
      s2_material_t* material =
        body->materials + m_bodyPanel.materialDropDown->currentIndex();
      s2_color_enum_t colorType =
        (s2_color_enum_t)m_bodyPanel.materialColorDropDown->currentIndex();
      switch(colorType) {
        case S2_COLOR_AMBIENT: {
          qDebug() << "ambient";
          material->GLuniforms.Ka[0] = color.red() / 255.f;
          material->GLuniforms.Ka[1] = color.green() / 255.f;
          material->GLuniforms.Ka[2] = color.blue() / 255.f;
        }
        break;
        case S2_COLOR_DIFFUSE: {
          qDebug() << "diffuse";
          material->GLuniforms.Kd[0] = color.red() / 255.f;
          material->GLuniforms.Kd[1] = color.green() / 255.f;
          material->GLuniforms.Kd[2] = color.blue() / 255.f;
        }
        break;
        case S2_COLOR_SPECULAR: {
          qDebug() << "specular";
          material->GLuniforms.Ks[0] = color.red() / 255.f;
          material->GLuniforms.Ks[1] = color.green() / 255.f;
          material->GLuniforms.Ks[2] = color.blue() / 255.f;
        }
        break;
        case S2_COLOR_EMISSIVE: {
          qDebug() << "emissive currently not handled";
        }
        break;
      }
      populateMaterialColorDropDown(material);

      // qDebug(
      //   "ok r:%i g:%i b:%i\n",
      //   color.red(), color.green(), color.blue());
    }
  }
  m_renderWindow->setAnimating(true);
}


/*!
  @brief light color selector

  * 1002: assembly
  * 1003: body group
  * 1004: body
  * 1005: frame
  * 1006: joint group
  * 1007: joint
  * 1008: light group
  * 1009: light
  * 1010: restraint group
  * 1011: restraint
  * 1012: io group
  * 1013: io

  valid only for light pane (1009)
*/
void Workspace::lightColorSelector() {
  m_renderWindow->setAnimating(false);
  if(
    m_currentSelectedItem != NULL &&
    m_currentSelectedItem->m_userType == 1009) {

    QColor color =
      QColorDialog::getColor(
        Qt::white,
        this,
        "Select Color",
        QColorDialog::DontUseNativeDialog);

    if(color.isValid()) {
      s2_light_t* light = (s2_light_t*)m_currentSelectedItem->m_object;
      s2_color_enum_t colorType =
        (s2_color_enum_t)m_bodyPanel.materialColorDropDown->currentIndex();
      switch(colorType) {
        case S2_COLOR_AMBIENT: {
          qDebug() << "ambient";
          light->glLight.Ka[0] = color.red() / 255.f;
          light->glLight.Ka[1] = color.green() / 255.f;
          light->glLight.Ka[2] = color.blue() / 255.f;
        }
        break;
        case S2_COLOR_DIFFUSE: {
          qDebug() << "diffuse";
          light->glLight.Kd[0] = color.red() / 255.f;
          light->glLight.Kd[1] = color.green() / 255.f;
          light->glLight.Kd[2] = color.blue() / 255.f;
        }
        break;
        case S2_COLOR_SPECULAR: {
          qDebug() << "specular";
          light->glLight.Ks[0] = color.red() / 255.f;
          light->glLight.Ks[1] = color.green() / 255.f;
          light->glLight.Ks[2] = color.blue() / 255.f;
        }
        break;
        case S2_COLOR_EMISSIVE: {
          qDebug() << "emissive currently not handled";
        }
        break;
      }
      populateLightColorDropDown(light);

      qDebug(
        "ok r:%i g:%i b:%i\n",
        color.red(), color.green(), color.blue());
    }
  }
  m_renderWindow->setAnimating(true);
}


/*!
@brief set integrator type in context data structure
*/
void Workspace::setIntegratorType(int val) {
  switch(val) {
    case 0: {
      m_context->modelParameters.integrationType =
        S2_NUMERICALINTEGRATOR_ODE113;
    }
    break;
    case 1: {
      m_context->modelParameters.integrationType =
        S2_NUMERICALINTEGRATOR_ODE113B;
    }
    break;
  }
}


/*!
@brief sets time span in context data structure
*/
void Workspace::setTimeSpan(double timeSpan) {
  if(m_context != NULL) {
    m_context->modelParameters.timeSpan = timeSpan;
  }
}


/*!
@brief sets time step in context data structure
*/
void Workspace::setTimeStep(double timeStep) {
  if(m_context != NULL) {
    m_context->modelParameters.timeStep = timeStep;
  }
}


/*!
@brief sets relative error in context data structure
*/
void Workspace::setRelativeError(double relativeError) {
  if(m_context != NULL) {
    m_context->modelParameters.relativeError = relativeError;
  }
}


/*!
@brief sets absolute error in context data structure
*/
void Workspace::setAbsoluteError(double absoluteError) {
  if(m_context != NULL) {
    m_context->modelParameters.absoluteError = absoluteError;
  }
}


/*!
@brief sets omega n in context data structure
*/
void Workspace::setOmegaN(double omegaN) {
  if(m_context != NULL) {
    s2_context_setOmegaN(m_context, omegaN);
  }
}


/*!
@brief sets output filename in context data structure
*/
void Workspace::setOutputFileName(const QString& fileName) {
  if(m_context != NULL) {

  }
}


/*!
@brief update simulation timer
*/
void Workspace::updateSimulationTimer(double t) {
  m_playbackPanel.time = t;
  m_playbackPanel.slider->blockSignals(true);
  m_playbackPanel.slider->setValue(t * m_playbackPanel.singleStep);
  m_playbackPanel.slider->blockSignals(false);
}


/*!
@brief loads model parameters from context data structure to display in
model parameters box
*/
void Workspace::loadModelParameters() {
  if(m_context != NULL) {
    m_modelParametersPanel.groupBox->setVisible(true);
    m_modelParametersPanel.absoluteError->setValue(
      m_context->modelParameters.absoluteError);

    m_modelParametersPanel.relativeError->setValue(
      m_context->modelParameters.relativeError);

    qDebug() << "omega_N" << m_context->modelParameters.baumgarteOmegaN;

    m_modelParametersPanel.omegaN->setValue(
      m_context->modelParameters.baumgarteOmegaN);

    m_modelParametersPanel.timeSpan->setValue(
      m_context->modelParameters.timeSpan);

    m_modelParametersPanel.timeStep->setValue(
      m_context->modelParameters.timeStep);
  }
}


/*!
@brief toggle visibility of playback box
*/
void Workspace::setPlaybackVisible(bool val) {
  m_playbackPanel.groupBox->setVisible(val);
}


/*!
@brief toggle visibility of loading dialog
*/
void Workspace::setLoadingVisible(bool val) {
  m_progressBar->setVisible(val);
}


/*!
@brief set playback panel to paused state
*/
void Workspace::setPlaybackPaused() {
  m_playbackPanel.playPauseButton->setIcon(
    m_playbackPanel.playIcon);
  m_simulationRunning = false;
}


/*!
@brief interpolate slider position in render window
*/
void Workspace::interpModel(int val) {
  double t = (double) val / (double)m_playbackPanel.singleStep;
  m_renderWindow->stopSimulationPlayback();
  m_renderWindow->interpolateCurrentPosition(t);
  m_playbackPanel.time = t;
}


/*!
@brief run simulation
*/
void Workspace::runSimulation() {
  if(m_context != NULL) {

    setPlaybackVisible(false);
    setLoadingVisible(true);

    printf("\n##initializing equations\n");
    s2_context_initializeEquations(m_context);

    m_renderWindow->setAnimating(false);

    printf("\nintegrating equations:\n");
    s2_context_integrateEquations(m_context);

    setLoadingVisible(false);
    setPlaybackVisible(true);

    m_playbackPanel.playPauseButton->setIcon(
      m_playbackPanel.pauseIcon);

    m_simulationRunning = true;
    m_playbackPanel.time = 0;

    m_playbackPanel.slider->setRange(
      0.0,
      m_playbackPanel.singleStep * m_context->modelParameters.timeSpan);

    m_renderWindow->setAnimating(true);
    m_renderWindow->startSimulationPlayback();
  }
}


/*!
@brief play/pause simulation
*/
void Workspace::playPauseSimulation() {
  if(m_context != NULL) {

    if(m_simulationRunning) {
      m_simulationRunning = false;
      setLoadingVisible(false);
      setPlaybackVisible(true);
      m_playbackPanel.playPauseButton->setIcon(m_playbackPanel.playIcon);
      m_renderWindow->setAnimating(true);
      m_renderWindow->stopSimulationPlayback();
    } else {
      m_simulationRunning = true;
      setLoadingVisible(false);
      setPlaybackVisible(true);
      m_playbackPanel.playPauseButton->setIcon(m_playbackPanel.pauseIcon);
      m_renderWindow->setAnimating(true);
      m_renderWindow->startSimulationPlayback(m_playbackPanel.time);
    }
  }
}
