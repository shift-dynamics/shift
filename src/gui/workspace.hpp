#ifndef _SHIFT_WORKSPACE_H_
#define _SHIFT_WORKSPACE_H_

#include <QApplication>
#include <QComboBox>
#include <QDesktopWidget>
#include <QGroupBox>
#include <QLabel>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QPixmap>
#include <QProgressBar>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QVector3D>
#include <QVector4D>
#include <QWidget>
#include <QtGui>

#include "../shift.h"
#include "renderwindow.hpp"

class QSlider;

namespace Shift {

typedef struct SliderHandle {
  QGroupBox* box;
  QSlider* slider;
  QLabel* minLabel;
  QLabel* maxLabel;
  QDoubleSpinBox* spinBox;
  double* spinBoxValue;
  int qIndex;
  int dqIndex;
  bool isSpherical;
  s2_substructure_t* substructure = NULL;
  double singleStep;
} SliderHandle;

enum s2_modeltree_enum_t {
  S2_MODELTREE_ASSEMBLY = 1002,
  S2_MODELTREE_BODY_GROUP = 1003,
  S2_MODELTREE_BODY = 1004,
  S2_MODELTREE_FRAME = 1005,
  S2_MODELTREE_JOINT_GROUP = 1006,
  S2_MODELTREE_JOINT = 1007,
  S2_MODELTREE_LIGHT_GROUP = 1008,
  S2_MODELTREE_LIGHT = 1009,
  S2_MODELTREE_RESTRAINT_GROUP = 1010,
  S2_MODELTREE_RESTRAINT = 1011,
  S2_MODELTREE_IO_GROUP = 1012,
  S2_MODELTREE_IO = 1013,
  S2_MODELTREE_TERRAIN_GROUP = 1014,
  S2_MODELTREE_TERRAIN = 1015,
  S2_MODELTREE_POINTCLOUD_GROUP = 1016,
  S2_MODELTREE_POINTCLOUD = 1017
};

class MyTabWidget : public QTabWidget {
private:
  QSize sizeHint() const {
    // QDesktopWidget *d = QApplication::desktop();
    // return QSize(d->width()/2, d->height());
    return QSize(600, 400);
  }
};

class MyTreeItem : public QTreeWidgetItem {
public:
  MyTreeItem(int type) {
    m_object = NULL;
    m_userType = type;
  }
  ~MyTreeItem() {}
  QString m_name;
  void* m_object;
  int m_userType;
};

class Workspace : public QMainWindow {
  Q_OBJECT // must be included to add qt meta information

    public : explicit Workspace(QWidget* parent = 0);
  ~Workspace();

  RenderWindow* m_renderWindow;
  QLabel* m_cursorPositionLabel;

  // void resizeEvent(QResizeEvent *event) {
  //   emit resizeWindow();
  // }

protected:
  void resizeEvent(QResizeEvent* event);

private slots:
  void setTx(int);
  void setTy(int);
  void setTz(int);
  void setRx(int);
  void setRy(int);
  void setRz(int);
  void onclickTx();
  void onclickTy();
  void onclickTz();
  void onclickRx();
  void onclickRy();
  void onclickRz();
  void setTxSpinBox(double);
  void setTySpinBox(double);
  void setTzSpinBox(double);
  void setRxSpinBox(double);
  void setRySpinBox(double);
  void setRzSpinBox(double);
  void recenterTx();
  void recenterTy();
  void recenterTz();
  void recenterRx();
  void recenterRy();
  void recenterRz();
  void updateH();
  void setTimeSpan(double);
  void setTimeStep(double);
  void setRelativeError(double);
  void setAbsoluteError(double);
  void setOmegaN(double);
  void setOutputFileName(const QString& str);
  void updateSimulationTimer(double);
  void setPlaybackPaused();
  void interpModel(int);
  void setIntegratorType(int);

  // void backgroundColorSelector();
  void materialColorSelector();
  void lightColorSelector();
  void updateCursorPositionText();
  void clearCursorPositionText();
  void openFile();
  void openFileAtLoad();
  void saveFile();
  void saveFileAs();
  void getSelectedTreeItem();
  void about();
  void license();
  // void controls();
  // void features();
  // void credit();
  void updateCurrentMaterial(int);
  void updateCurrentLightType(int);
  void updateMaterialNs(double);
  void updateTree();
  void addLight();
  void deleteLight();
  void loadModelParameters();
  void runSimulation();
  void playPauseSimulation();
  void setPlaybackVisible(bool);
  void setLoadingVisible(bool);
  void writeMobility();
  void drawTerrainToggleActive(bool);
  void toggleVisibility(QTreeWidgetItem* item, int column);
  void updateDrawState();
  void updateItemDrawState(MyTreeItem* item, bool isVisible,
                           bool isCategoryEnabled);

private:
  QSize minimumSizeHint() const {
    return QSize(50, 50);
  }

  QSize sizeHint() const {
    // QDesktopWidget *d = QApplication::desktop();
    // return QSize(d->width()/2, d->height());
    return QSize(700, 500);
  }

  /* Layout utilities */
  QGridLayout* m_notificationPanelLayout;
  QProgressBar* m_progressBar;
  MyTreeItem* m_currentSelectedItem;
  QTreeWidget* m_treeWidget;

  QGroupBox* m_dynamicGroupBox;
  QVBoxLayout* m_dynamicPanelLayout;

  SliderHandle* m_currentSlider;
  QTabWidget* m_tabwidget;

  QAction* m_actionToggleDrawFrames;
  QAction* m_actionToggleDrawJoints;
  QAction* m_actionToggleDrawTerrain;
  QAction* m_actionToggleDrawGrid;
  QAction* m_actionToggleDrawShadows;
  QAction* m_actionToggleDrawConvexHulls;
  QAction* m_actionToggleDrawNormals;
  QAction* m_actionToggleDrawBoundingSpheres;

  struct m_sliders {
    QGroupBox* groupBox;

    SliderHandle tx;
    SliderHandle ty;
    SliderHandle tz;
    SliderHandle rx;
    SliderHandle ry;
    SliderHandle rz;

    float revoluteRange;
    float prismaticRange;
    double* currentSpinBoxValue;
  } m_sliders;

  struct m_scenePanel {
    QGroupBox* groupBox;
    QLabel* title;
  } m_scenePanel;

  struct m_assemblyPanel {
    QGroupBox* groupBox;
    QLabel* title;
    s2_assembly_t* selectedAssembly;
  } m_assemblyPanel;

  struct m_jointGroupPanel {
    QGroupBox* groupBox;
    QLabel* title;
    QPushButton* addJointButton;
  } m_jointGroupPanel;

  struct m_jointPanel {
    QGroupBox* groupBox;
    QLabel* title;
    QLabel* tx;
    QLabel* ty;
    QLabel* tz;
    QLabel* rx;
    QLabel* ry;
    QLabel* rz;
    s2_joint_t* selectedJoint;
  } m_jointPanel;

  struct m_bodyGroupPanel {
    QGroupBox* groupBox;
    QLabel* title;
    QPushButton* addBodyButton;
  } m_bodyGroupPanel;

  struct m_bodyPanel {
    QGroupBox* groupBox;
    QLabel* title;

    QPushButton* addFrame;
    QPushButton* materialColorButton;
    QComboBox* materialDropDown;
    QComboBox* materialColorDropDown;
    QDoubleSpinBox* nsModelSpinBox;

    s2_body_t* selectedBody;
  } m_bodyPanel;

  struct m_playbackPanel {
    QGroupBox* groupBox;
    QIcon playIcon;
    QIcon pauseIcon;
    QPushButton* playPauseButton;
    QSlider* slider;
    double time;
    int singleStep;
  } m_playbackPanel;

  struct m_framePanel {
    QGroupBox* groupBox;
    QLabel* title;
    QLabel* m00;
    QLabel* m01;
    QLabel* m02;
    QLabel* m03;
    QLabel* m10;
    QLabel* m11;
    QLabel* m12;
    QLabel* m13;
    QLabel* m20;
    QLabel* m21;
    QLabel* m22;
    QLabel* m23;
    QLabel* m30;
    QLabel* m31;
    QLabel* m32;
    QLabel* m33;
    s2_frame_t* selectedFrame;
  } m_framePanel;

  struct m_restraintGroupPanel {
    QGroupBox* groupBox;
    QLabel* title;

    QPushButton* addRestraintButton;
  } m_restraintGroupPanel;

  struct m_restraintPanel {
    QGroupBox* groupBox;
    QLabel* title;

    s2_restraint_t* selectedRestraint;
  } m_restraintPanel;

  struct m_ioGroupPanel {
    QGroupBox* groupBox;
    QLabel* title;

    QPushButton* addIoButton;
  } m_ioGroupPanel;

  struct m_ioPanel {
    QGroupBox* groupBox;
    QLabel* title;

    s2_io_t* selectedIo;
  } m_ioPanel;

  struct m_lightGroupPanel {
    QGroupBox* groupBox;
    QLabel* title;

    QPushButton* addLightButton;
  } m_lightGroupPanel;

  struct m_lightPanel {
    QGroupBox* groupBox;
    QLabel* title;

    QPushButton* lightColorButton;
    QPushButton* deleteLightButton;
    QComboBox* lightTypeDropDown;
    QGroupBox* lightColorBox;
    QComboBox* lightColorDropDown;
    QDoubleSpinBox* nsLightSpinBox;

    s2_light_t* selectedLight;
  } m_lightPanel;

  struct m_modelParametersPanel {
    QGroupBox* groupBox;
    QComboBox* integratorType;
    QDoubleSpinBox* timeSpan;
    QDoubleSpinBox* timeStep;
    QDoubleSpinBox* relativeError;
    QDoubleSpinBox* absoluteError;
    QDoubleSpinBox* omegaN;
    QLabel* title;

  } m_modelParametersPanel;

  QStatusBar* m_statusBar;

  void initializeMenus();
  void initializeLeftPanel();
  void initializeLayout();
  void addSlider(double singleStep, SliderHandle* sliderHandle, QLayout* layout,
                 const char* label);
  void addSliders();
  void addDynamicSceneBox();
  void addModelParametersBox();
  void addAssemblyBox();
  void addJointGroupBox();
  void addJointBox();
  void addBodyGroupBox();
  void addBodyBox();
  void addFrameBox();
  void addRestraintGroupBox();
  void addRestraintBox();
  void addIoGroupBox();
  void addIoBox();
  void addLightGroupBox();
  void addLightBox();
  void populateDynamicBox(int userType);
  void clearDynamicBox();
  void clearSliders();
  void setSlider(SliderHandle* sliderHandle, int qIndex, int dqIndex,
                 s2_substructure_t* substructure, bool isSpherical);
  void recenterSlider(SliderHandle* sliderHandle);

  QString getCursorPositionText();
  // virtual void resizeEvent(QResizeEvent* event);

  /*
    Model utilities
  */
  s2_context_t* m_context;
  QString m_currentFileName;
  FILE* m_mobilityFile;
  bool m_simulationRunning;

  void populateTree();
  void populateTreeRecursive(s2_assembly_t* assemblyHead,
                             QTreeWidgetItem* widget);
  void updateTransform();
  void populateMaterialColorDropDown(s2_material_t* material);
  void populateLightColorDropDown(s2_light_t* light);
  void highlightSubstructure(s2_substructure_t* substructure);
  void clearSubstructureHighlight();

signals:
  void treeWidgetCleared();
  void openFileDialog();
  void txChanged(int);
  void tyChanged(int);
  void tzChanged(int);
  void rxChanged(int);
  void ryChanged(int);
  void rzChanged(int);
  void windowResized();
};

} // namespace Shift
#endif
