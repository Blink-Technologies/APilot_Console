#ifndef APILOT_H
#define APILOT_H


#include "aimode.h"
#include "sharedvars.h"
#include "a_telem.h"
#include <QTimer>

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

    Telemetry *telemetry;
    Offboard *offboard;
    Action *action;
    MavlinkPassthrough *mavlink_passthrough;

    A_TELEM *_atelem;

public slots:

    void on_Flt_Timer_Tick();

private:

    int InitMav();
    Offboard::Attitude ai_attitude;
    float attitude_params[5];
    int state = st_STABLIZED;
    void FSM();
    QTimer *flt_timer;
};

#endif // APILOT_H
