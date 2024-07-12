#ifndef A_TELEM_H
#define A_TELEM_H

#include <QObject>
#include "sharedvars.h"


class A_TELEM : public QObject
{
    Q_OBJECT
public:
    A_TELEM(Telemetry*, MavlinkPassthrough*);

    Telemetry *telemetry;
    MavlinkPassthrough *mavlink_passthrough;
    static void CallBack_FlightMode(Telemetry::FlightMode);
    static void CallBack_Battery(Telemetry::Battery);
    static void CallBack_RC_Channels(const mavlink_message_t);
    static void CallBack_AttitudeEuler(Telemetry::EulerAngle);
    static void CallBack_RCStatus(Telemetry::RcStatus);
    static void CallBack_Health(Telemetry::Health);
    void Telemetry_Subscribe();
    void Telemetry_Subscribe_RC_Channels();
};

#endif // A_TELEM_H
