#include <QCoreApplication>

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

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

int InitMav();
static void CallBack_Battery(Telemetry::Battery);
//static void CallBack_ActuatorOutput(Telemetry::ActuatorOutputStatus);



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    InitMav();
    return a.exec();
}


void process_rc_channels(const mavlink_message_t& message) {
    mavlink_rc_channels_t rc_channels;
    mavlink_msg_rc_channels_decode(&message, &rc_channels);

    qDebug() << "RC Channels \n";
    qDebug() << "Channel count: " << rc_channels.chancount << "\n";

    //for (int i = 0; i < rc_channels.chancount; ++i) {
        //qDebug() << "Channel " << i + 1 << ": " << rc_channels. [i] << std::endl;
    }

int InitMav()
{
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    //QString VehicleSerialPath = "serial:///dev/ttyACM0:115200";

    //Make Serial Connection
    ConnectionResult connection_result = mavsdk.add_serial_connection("/dev/ttyACM0", Mavsdk::DEFAULT_SERIAL_BAUDRATE, false, ForwardingOption::ForwardingOff);

    //ConnectionResult connection_result = mavsdk.add_any_connection(VehicleSerialPath);
    if (connection_result != ConnectionResult::Success) {
        qDebug()<< "Adding connectiConnectionResulton failed:'\n";
        return 10;
    }
    qDebug()<< "Vechicle added at /dev/ttyACM0";


    //Detect Autopilot Heartbeat
    auto system = mavsdk.first_autopilot(3);
    if (!system) {
        qDebug()<<"Timed out waiting for system\n";
        return 1;
    }

    qDebug()<<"Autopilot Detected \n";

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    auto mavlink_passthrough = MavlinkPassthrough{system.value()};

    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    const auto set_battery_result = telemetry.set_rate_battery(1.0);
    //const auto set_act_result = telemetry.set_rate_actuator_output_status(1.0);


    if (set_rate_result != Telemetry::Result::Success) {
        qDebug() << "Setting rate failed: \n";
        return 1;
    }

    if (set_battery_result != Telemetry::Result::Success) {
        qDebug() << "Setting rate battery failed: \n";
        return 1;
    }


    /*
    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position)
    {
        qDebug() << "Altitude: " << position.relative_altitude_m << " m\n";
    });
    */

    /*
    telemetry.subscribe_battery([] (Telemetry::Battery bat)
    {
       qDebug() << "Recieved Battery: \n";
       qDebug()<<"Battery Voltage : " << bat.voltage_v << " \n";
    });
    */

    //telemetry.subscribe_flight_mode([](const FlightModeCallback& callback);



    telemetry.subscribe_battery(CallBack_Battery);
    //telemetry.subscribe_actuator_output_status(CallBack_ActuatorOutput);

    //telemetry.subscribe_actuator_output_status([] (const Telemetry::ActuatorOutputStatusCallback& callback)
    //{
    //
    //});
    //mavlink_passthrough.subscribe_message(65, CallBack_RC_Channels);


    /*
    mavlink_passthrough.subscribe_message(65, [](const mavlink_message_t &msg_raw)
                                          {

                                              qDebug() << "Recieved RC Transmission: \n";

                                              const mavlink_message_t* msg = &msg_raw;

                                              mavlink_rc_channels_t rc_channels;
                                              mavlink_msg_rc_channels_decode(msg, &rc_channels);
                                              qDebug()<< "Number of channels received: " << int(rc_channels.chancount) << "\n";
                                              qDebug() << "Channel 1 : " << rc_channels.chan1_raw << "\n";
                                              qDebug() << "Channel 2 : " << rc_channels.chan2_raw << "\n";
                                              qDebug() << "Channel 3 : " << rc_channels.chan3_raw << "\n";
                                              qDebug() << "Channel 4 : " << rc_channels.chan4_raw << "\n";
                                              qDebug() << "Channel 5 : " << rc_channels.chan5_raw << "\n";
                                              qDebug() << "Channel 6 : " << rc_channels.chan6_raw << "\n";
                                              qDebug() << "Channel 7 : " << rc_channels.chan7_raw << "\n";
                                              qDebug() << "Channel 8 : " << rc_channels.chan8_raw << "\n";

                                          }
                                          );
*/


    MavlinkPassthrough::CommandLong cmd_to_set_rate;

    cmd_to_set_rate.target_sysid = mavlink_passthrough.get_target_sysid();
    cmd_to_set_rate.target_compid = mavlink_passthrough.get_target_compid();
    cmd_to_set_rate.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd_to_set_rate.param1 = 65;
    cmd_to_set_rate.param2 = 1000000;  //1Hz
    cmd_to_set_rate.param3 =0;
    cmd_to_set_rate.param4 =0;
    cmd_to_set_rate.param5 =0;
    cmd_to_set_rate.param6 =0;
    cmd_to_set_rate.param7 =0;

    MavlinkPassthrough::Result res = mavlink_passthrough.send_command_long(cmd_to_set_rate);

    qDebug()<<"Result : "<<(int)res<< "\n";




    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS, [](const mavlink_message_t &msg_raw)
                                          {

                                              qDebug() << "Recieved RC Transmission: \n";

                                              const mavlink_message_t* msg = &msg_raw;

                                              mavlink_rc_channels_t rc_channels;
                                              mavlink_msg_rc_channels_decode(msg, &rc_channels);
                                              qDebug()<< "Number of channels received: " << int(rc_channels.chancount) << "\n";
                                              qDebug() << "Channel 1 : " << rc_channels.chan1_raw;// << "\n";
                                              qDebug() << "Channel 2 : " << rc_channels.chan2_raw;// << "\n";
                                              qDebug() << "Channel 3 : " << rc_channels.chan3_raw;// << "\n";
                                              qDebug() << "Channel 4 : " << rc_channels.chan4_raw;// << "\n";
                                              qDebug() << "Channel 5 : " << rc_channels.chan5_raw;// << "\n";
                                              qDebug() << "Channel 6 : " << rc_channels.chan6_raw;// << "\n";
                                              qDebug() << "Channel 7 : " << rc_channels.chan7_raw;// << "\n";
                                              qDebug() << "Channel 8 : " << rc_channels.chan8_raw;// << "\n";
                                              qDebug() << "Channel 9 : " << rc_channels.chan9_raw;// << "\n";
                                              qDebug() << "Channel 10 : " << rc_channels.chan10_raw;// << "\n";
                                              qDebug() << "Channel 11 : " << rc_channels.chan11_raw;// << "\n";
                                              qDebug() << "Channel 12 : " << rc_channels.chan12_raw;// << "\n";
                                              qDebug() << "Channel 13 : " << rc_channels.chan13_raw;// << "\n";
                                              qDebug() << "Channel 14 : " << rc_channels.chan14_raw;// << "\n";
                                              qDebug() << "Channel 15 : " << rc_channels.chan15_raw;// << "\n";
                                              qDebug() << "Channel 16 : " << rc_channels.chan16_raw;// << "\n";

                                          }
                                          );


    /*
    mavlink_passthrough.subscribe_message(
        MAVLINK_MSG_ID_RC_CHANNELS,
        [](const mavlink_message_t& message) {
            qDebug()<<"RC Recieved \n";
            process_rc_channels(message);
        }
        );
    */

    mavlink_passthrough.subscribe_message(
        34,
        [](const mavlink_message_t& message) {
            qDebug()<<"RC Recieved 34 : " << message.len << "\n";
        }
        );

    mavlink_passthrough.subscribe_message(
        35,
        [](const mavlink_message_t& message) {
            qDebug()<<"RC Recieved 35 : " << message.len << "\n";
        }
        );


    /*
    mavsdk.intercept_incoming_messages_async([](mavlink_message_t& message) {
        qDebug() << "Got message " << (int)message.msgid << "\n";
        return true;
    });
    */



    // Keep the program alive to receive updates
    for (;;) {
        sleep_for(seconds(1));
    }



    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }



    /*

     qDebug() << "Sleeping Now";

    sleep_for(seconds(5));

    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        qDebug() << "Arming failed \n";
    }
    else
    {
        qDebug() << "Arming Success \n";
    }

    sleep_for(seconds(3));

    const Action::Result disarm_result = action.disarm();
    if (disarm_result != Action::Result::Success) {
        qDebug() << "DisArming failed \n";
    }
    else
    {
        qDebug() << "DisArming Success \n";
    }

    */

    //while(1);

    return 0;

}

static void CallBack_Battery(Telemetry::Battery btry)
{
    qDebug() << "Callback :: Battery \n";
    qDebug()<<"Battery Voltage : " << btry.voltage_v << " \n";
}

/*
static void CallBack_ActuatorOutput(Telemetry::ActuatorOutputStatus aa)
{
    qDebug() << "Callback :: Actuator Output Status \n";
    qDebug() <<aa.active;
    //qDebug()<<"Battery Voltage : " << btry.voltage_v << " \n";
}
*/
