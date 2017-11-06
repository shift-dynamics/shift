macx {
   QMAKE_MAC_SDK=macosx10.9
}

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qt_viewer

TEMPLATE = app

DEPENDPATH += .
INCLUDEPATH += . \
				 /usr/local/include/libfreenect \
				 /usr/include/opencv


SOURCES += main.cpp \
           qtviewer.cpp \
           openglwindow.cpp \
           renderwindow.cpp \
           tiny_obj_loader.cpp
           
HEADERS  += qtviewer.h \
            openglwindow.h \
            renderwindow.h \
            tiny_obj_loader.h \
            matrixToEulerAngles.h

RESOURCES += resources.qrc

LIBS += -L/usr/local/lib -lfreenect `pkg-config --libs opencv`