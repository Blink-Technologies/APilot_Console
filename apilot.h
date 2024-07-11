#ifndef APILOT_H
#define APILOT_H

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <QDebug>
#include <QString>
#include <QTimer>

#define VEHICLE_CONNECTION_PATH "/dev/ttyACM0"

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

class apilot
{
public:
    apilot();



    static void CallBack_Battery(mavsdk::Telemetry::Battery btry);
    static void CallBack_RC_Channels(const mavlink_message_t msg_raw);
    void Start();
    static void CallBack_FlightMode(Telemetry::FlightMode f);
private:
    void process_rc_channels(const mavlink_message_t &message);
    int InitMav();
};

#endif // APILOT_H
