#ifndef QTVIEWER_H
#define QTVIEWER_H

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QVector3D>
#include <QVector4D>
#include <QMatrix4x4>
#include <QJsonDocument>
#include <QWidget>
#include <QTreeWidget>

#include "renderwindow.h"

class QSlider;
class GLWidget;

// class ModelTreeItem : public QTreeWidgetItem
// {
//     // Q_OBJECT

// public:
//     ModelTreeItem( int type) { model = NULL; UserType=type; }
//     ~ModelTreeItem() {}

//     Model* model;
//     int UserType;
// };


class qtviewer : public QMainWindow 
{
    Q_OBJECT //must be included to add qt meta information

    public:
        explicit qtviewer( QWidget *parent = 0 );
        ~qtviewer();

        RenderWindowWidget* glwidget;
        QLabel* cursorPositionLabel;


    private slots:
        // void setRx( int );
        // void setRy( int );
        // void setRz( int );
        // void setTx( int );
        // void setTy( int );
        // void setTz( int );
        // void setS( int );

        // void initRx();
        // void initRy();
        // void initRz();
        // void initTx();
        // void initTy();
        // void initTz();
        // void initS();

        //void backgroundColorSelector();
        //void objectColorSelector();
        void updateCursorPositionText();
        void clearCursorPositionText();
        void openFile();
        //void saveFile();
        //void saveFileAs();
        //void getSelectedTreeItem();
        void about();
        void controls();
        //void features();
        //void credit();

    private:
         QSize minimumSizeHint() const
         {
             return QSize(50, 50);
         }

         QSize sizeHint() const
         {
             return QSize(700, 500);
         }

        // Layout utilities
        void createMenus();
        void createLayout();

        QString getCursorPositionText();

        //virtual void resizeEvent( QResizeEvent * event );

        // JSON reader/writer utilities
        //QVector<Scene*> scenes;
        // void populateTree();
        
        // Model* parseModel( const QJsonObject& model );
        // Node* parseNode( const QJsonObject& parentNode ) ;
        // void sceneRead( const QString& filename, QVector<Scene *> &scenes );
        // void sceneReadFromStr( const QString& fileContents, QVector<Scene *> &scenes );
        // QJsonObject writeNode( Node* node );
        // void sceneWrite( const QString& filename, QVector<Scene *> &scenes );
        // void readOBJ( Model* model );
        
        Frame* readFrameFromFile( const QString &depthFileName, const QString &rgbFileName );
        void updateTransform();

    signals:
        void treeWidgetCleared();
        void openFileDialog();
        void resizeEvent();
        void rxChanged( int );
        void ryChanged( int );
        void rzChanged( int );
        void txChanged( int );
        void tyChanged( int );
        void tzChanged( int );
        void sChanged( int );
};

#endif
