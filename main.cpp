#include <QCoreApplication>
#include "apilot.h"


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    apilot _apilot;
    _apilot.Start();

    return a.exec();
}

