#ifndef _BASICFRAMEWORK_HPP_
#define _BASICFRAMEWORK_HPP_

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QVector3D>
#include <QVector4D>
#include <QMatrix4x4>
#include <QJsonDocument>
#include <QWidget>
#include <QTreeWidget>

#include "renderwindow.hpp"

class QSlider;
class GLWidget;

/*!
@brief Custom tree item that derives from QTreeWidgetItem
*/
class ModelTreeItem : public QTreeWidgetItem {

  public:
    ModelTreeItem(int type) {
      model = NULL; UserType=type;}
    ~ModelTreeItem() {}
    Model* model;
    int UserType;
};

/*!
@brief class definition for BasicFramework window, derives from QMainWindow
class
*/
class BasicFramework : public QMainWindow {
  Q_OBJECT /*!< this macro must be included to add qt meta information */

  /* public methods */
  public:
    explicit BasicFramework(QWidget* parent = 0);
    ~BasicFramework();

    RenderWindowWidget* glwidget;
    QLabel* cursorPositionLabel;

  /* signals/slots are Qt's way of communicating between various objects */
  private slots:
    void setRx(int);
    void setRy(int);
    void setRz(int);
    void setTx(int);
    void setTy(int);
    void setTz(int);
    void setS(int);
    void backgroundColorSelector();
    void objectColorSelector();
    void updateCursorPositionText();
    void clearCursorPositionText();
    void openFile();
    void saveFile();
    void saveFileAs();
    void getSelectedTreeItem();
    void about();
    void credit();

  /* private methods - these methods are only called by other methods of the
     class */
  private:
    QSize minimumSizeHint() const {
       return QSize(50, 50);
    }

    QSize sizeHint() const {
       return QSize(700, 500);
    }

    /* Layout utilities */
    void createMenus();
    void createLayout();
    void addSlider(
      const char* label,
      QSlider* slider,
      QVBoxLayout* sliderLayout);
    QTreeWidget* treeWidget;
    QSlider* createSlider();
    QString getCursorPositionText();

    /* list of models */
    Model* m_currentSelectedModel;
    QVector<Model*> m_modelArray;

    /* JSON reader/writer utilities */
    QString m_currentFileName;
    QString m_currentFilePath;
    QVector<Scene*> m_scenes;
    void populateTree();
    Model* parseModel(const QJsonObject& model);
    Node* parseNode(const QJsonObject& parentNode);
    void parseScene(const QString& filename, QVector<Scene*> &scenes);
    QJsonObject writeNode(Node* node);
    void sceneWrite(const QString& filename, QVector<Scene *> &scenes);
    bool parseGeometry(Model* model);
    void updateTransform();

    /* layout objects */
    QSlider* m_rxSlider;
    QSlider* m_rySlider;
    QSlider* m_rzSlider;
    QSlider* m_txSlider;
    QSlider* m_tySlider;
    QSlider* m_tzSlider;
    QSlider* m_scaleSlider;

    /* actions */
    QAction* m_actionOpen;
    QAction* m_actionOpenObject;
    QAction* m_actionSave;
    QAction* m_actionSaveAs;
    QAction* m_actionQuit;
    QAction* m_actionSetOrthogonal;
    QAction* m_actionSetPerspective;
    QAction* m_actionSetWireframe;
    QAction* m_actionSetFlatShaded;
    QAction* m_actionAbout;
    QAction* m_actionCredit;

  /* signals/slots are Qt's way of communicating between objects */
  signals:
    void treeWidgetCleared();
    void openFileDialog();
    // void resizeEvent();
    void rxChanged(int);
    void ryChanged(int);
    void rzChanged(int);
    void txChanged(int);
    void tyChanged(int);
    void tzChanged(int);
    void sChanged(int);

};

#endif
