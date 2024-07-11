#include "apilot.h"
#include "stdio.h"
#include "stdbool.h"


bool FLAG_ARM = false;
bool FLAG_ARM_PREV = false;

apilot::apilot() {}


void apilot::Start()
{
    qDebug()<<"APilot Started";
    qDebug()<<"..........................";
    InitMav();
}


int apilot::InitMav()
{
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_serial_connection(VEHICLE_CONNECTION_PATH, Mavsdk::DEFAULT_SERIAL_BAUDRATE, false, ForwardingOption::ForwardingOff);

    if (connection_result != ConnectionResult::Success) {
        qDebug()<< "Adding Vechicle Failed \n";
        return -1;
    }
    qDebug()<< "Vechicle added at /dev/ttyACM0";


    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        qDebug()<<"Timed out waiting for system";
        return -2;
    }

    qDebug()<<"Heartbeat Detected Successfully";


    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    auto mavlink_passthrough = MavlinkPassthrough{system.value()};


    Telemetry::FlightMode fm =telemetry.flight_mode();
    qDebug()<<"Inital Flight Mode : " << (int)fm;


    if (telemetry.set_rate_battery(1.0) != Telemetry::Result::Success)
        qDebug() << "Setting Rate failed: Battery";
    else
        qDebug() << "Setting Rate Success: Battery";


    telemetry.subscribe_flight_mode(apilot::CallBack_FlightMode);
    telemetry.subscribe_battery(apilot::CallBack_Battery);

    // Set Rate of RC Channels
    MavlinkPassthrough::CommandLong cmd_to_set_rate;
    cmd_to_set_rate.target_sysid = mavlink_passthrough.get_target_sysid();
    cmd_to_set_rate.target_compid = mavlink_passthrough.get_target_compid();
    cmd_to_set_rate.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd_to_set_rate.param1 = MAVLINK_MSG_ID_RC_CHANNELS;
    cmd_to_set_rate.param2 = 1000000;  //1Hz
    cmd_to_set_rate.param3 =0;
    cmd_to_set_rate.param4 =0;
    cmd_to_set_rate.param5 =0;
    cmd_to_set_rate.param6 =0;
    cmd_to_set_rate.param7 =0;
    MavlinkPassthrough::Result res = mavlink_passthrough.send_command_long(cmd_to_set_rate);
    qDebug()<<"MPT:: RC Rate Result : "<<(int)res<< "\n";


    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS, apilot::CallBack_RC_Channels);

    while(1)
    {

        // Check for ARM
        if (FLAG_ARM != FLAG_ARM_PREV)
        {
            if (FLAG_ARM)
            {
                if (action.arm() == Action::Result::Success) qDebug()<<"Arming Success";
                else qDebug()<<"Arming Failed";
            }
            else
            {
                if (action.disarm() == Action::Result::Success) qDebug()<<"Disarming Success";
                else qDebug()<<"Disrming Failed";
            }

            FLAG_ARM_PREV = FLAG_ARM;
        }
    }

    return 0;
}

void apilot::CallBack_FlightMode(Telemetry::FlightMode f)
{
    qDebug()<<"Flight Mode : " << (int) f;
}

void apilot::CallBack_Battery(Telemetry::Battery btry)
{
    qDebug()<<"Battery Voltage : " << btry.voltage_v;
}

void apilot::CallBack_RC_Channels(const mavlink_message_t msg_raw)
{
    qDebug() << "Recieved RC Transmission";
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


    //Channel 6 is for ARM/DISARM
    if (rc_channels.chan6_raw < 1100) //UP
    {
        FLAG_ARM = false;
    }
    else if (rc_channels.chan6_raw > 1800) // DOWN
    {
        FLAG_ARM = true;
    }
}
