#include <QApplication>
#include "main_window.h"

int main(int argc, char **argv)
{
    qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr"); 
    QApplication app(argc, argv);
    MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    return app.exec();
}