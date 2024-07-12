#ifndef SHAREDVARS_H
#define SHAREDVARS_H

#include <QIODevice>
#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/failure/failure.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <QDebug>
#include <QString>
#include <QTimer>
#include <QObject>
#include <QFile>
#include <QDateTime>


using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

class sharedVars
{
public:
    sharedVars();

    static void printh(QString);
    static void LogToFile(QString);
    static void InitLogFile();


    static bool RC_FLAG_ARM, RC_FLAG_ARM_PREV;
    static int RC_FLAG_FLT_MODE, RC_FLAG_FLT_MODE_PREV;

    static QFile *ff;
};

#endif // SHAREDVARS_H
