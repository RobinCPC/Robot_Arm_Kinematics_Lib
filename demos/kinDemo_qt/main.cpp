#include "kindemo.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    kinDemo w;
    w.show();

    return a.exec();
}
