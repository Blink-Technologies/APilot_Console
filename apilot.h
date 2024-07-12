#ifndef APILOT_H
#define APILOT_H

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
#include "aimode.h"

#define VEHICLE_CONNECTION_PATH "/dev/ttyACM0"

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

enum
{
  st_STABLIZED,
  st_Start_AIMode,
  st_Run_AIMode
};

enum
{
    FLT_MODE_STABLIZE,
    FLT_MODE_AI
};


class apilot : public QObject
{
    Q_OBJECT

signals:



public:
    explicit apilot(QObject *parent = nullptr);

    void Start();
    static void CallBack_Battery(mavsdk::Telemetry::Battery btry);
    static void CallBack_RC_Channels(const mavlink_message_t msg_raw);
    static void CallBack_FlightMode(Telemetry::FlightMode f);
    static void CallBack_AttitudeEuler(Telemetry::EulerAngle an);
    static void CallBack_RCStatus(Telemetry::RcStatus rc);
    static void CallBack_Health(Telemetry::Health h);

    static bool FLAG_ARM;
    static bool FLAG_ARM_PREV;
    static bool FLAG_AI_MODE;
    static int FLAG_FLT_MODE;
    static bool FLAG_AI_SUCCESS;

    static void printh(QString);
    static void LogIntoFile(QString);

public slots:

private:
    void process_rc_channels(const mavlink_message_t &message);
    int InitMav();


    Offboard::Attitude ai_attitude;
    float attitude_params[5];
    int state = st_STABLIZED;

    static QFile *ff;

};

#endif // APILOT_H
