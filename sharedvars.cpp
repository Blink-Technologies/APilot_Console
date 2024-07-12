#include "sharedvars.h"

QFile *sharedVars::ff = nullptr;
bool sharedVars::RC_FLAG_ARM = false;
bool sharedVars::RC_FLAG_ARM_PREV = false;
int sharedVars::RC_FLAG_FLT_MODE = 0;
int sharedVars::RC_FLAG_FLT_MODE_PREV =0;



sharedVars::sharedVars() {}

void sharedVars::InitLogFile()
{
    qDebug()<<"APilot Started";
    qDebug()<<"..........................";

    QString FileName = QString("/home/rpi/apilot_logs/%1.log").arg(QDateTime::currentDateTime().toString("ddMMyyyy-hhmmss"));

    sharedVars::ff = new QFile(FileName);
    sharedVars::LogToFile("Welcome to APilot");
    sharedVars::LogToFile(FileName);
    sharedVars::printh("File Created :: " + FileName);
}

void sharedVars::printh(QString str)
{
    qDebug() << str;
    sharedVars::LogToFile(str);
}

void sharedVars::LogToFile(QString str)
{
    str = str + "\n";

    if (!sharedVars::ff->isOpen())
    {
        if (sharedVars::ff->open(QIODevice::Append))
        {
            sharedVars::ff->write(str.toUtf8());
            sharedVars::ff->close();
        }

    }
    else
    {
        sharedVars::ff->close();
    }
}
