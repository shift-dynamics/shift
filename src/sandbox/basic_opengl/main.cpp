#include <QApplication>
#include "framework.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    BasicFramework window;

 	window.setMouseTracking( true );
    window.show();

    return app.exec();
}
