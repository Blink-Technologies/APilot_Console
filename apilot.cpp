#include "apilot.h"

bool apilot::FLAG_ARM = false;
bool apilot::FLAG_ARM_PREV = false;
bool apilot::FLAG_AI_MODE = false;
int  apilot::FLAG_FLT_MODE = 0;
bool apilot::FLAG_AI_SUCCESS = false;

QFile *apilot::ff = nullptr;

apilot::apilot(QObject *parent)
    : QObject{parent}
{}


void apilot::Start()
{
    printh("APilot Started");
    printh("..........................");

    QString FileName = QString("%1.log").arg(QDateTime::currentDateTime().toString("ddMMyyyy-hhmmss"));

    ff = new QFile(FileName);

    if (ff->open(QIODevice::WriteOnly))
    {
        LogIntoFile("Welcome to APilot");
        LogIntoFile(FileName);
        printh("File Created :: " + FileName);

    }
    else  printh("File Creation Error :: " + FileName);



    InitMav();
}


int apilot::InitMav()
{
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_serial_connection(VEHICLE_CONNECTION_PATH, Mavsdk::DEFAULT_SERIAL_BAUDRATE, false, ForwardingOption::ForwardingOff);

    if (connection_result != ConnectionResult::Success) {
        printh("Adding Vechicle Failed");
        return -1;
    }
    printh("Vechicle added at /dev/ttyACM0");


    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        printh("Timed out waiting for system");
        return -2;
    }

    printh("Heartbeat Detected Successfully");


    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    auto mavlink_passthrough = MavlinkPassthrough{system.value()};
    auto failure = Failure(system.value());
    auto offboard = Offboard{system.value()};


    Telemetry::FlightMode fm =telemetry.flight_mode();
    printh("Inital Flight Mode : " + QString::number((int)fm));


    if (telemetry.set_rate_battery(1.0) != Telemetry::Result::Success)
        printh("Setting Rate failed: Battery");
    else
        printh("Setting Rate Success: Battery");

    if (telemetry.set_rate_attitude_euler(1.0) != Telemetry::Result::Success)
        printh("Setting Rate failed: Euler");
    else
        printh("Setting Rate Success: Euler");

    if (telemetry.set_rate_rc_status(1.0) != Telemetry::Result::Success)
        printh("Setting Rate failed: RC Status");
    else
        printh("Setting Rate Success: RC Status");


    telemetry.subscribe_flight_mode(apilot::CallBack_FlightMode);
    telemetry.subscribe_battery(apilot::CallBack_Battery);
    telemetry.subscribe_attitude_euler(apilot::CallBack_AttitudeEuler);
    telemetry.subscribe_rc_status(apilot::CallBack_RCStatus);
    telemetry.subscribe_health(apilot::CallBack_Health);


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
    printh("MPT:: RC Rate Result : " + QString::number((int)res));


    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS, apilot::CallBack_RC_Channels);



    while(1)
    {

        // Check for ARM
        if (FLAG_ARM != FLAG_ARM_PREV)
        {
            if (FLAG_ARM)
            {
                if (action.arm() == Action::Result::Success) qDebug()<<"Arming Success";
                else  printh("Arming Failed");
            }
            else
            {
                if (action.disarm() == Action::Result::Success) qDebug()<<"Disarming Success";
                else  printh("Disrming Failed");
            }

            FLAG_ARM_PREV = FLAG_ARM;
        }

        // AI MODE FSM

        /*
        if (FLAG_ARM)
        {
            switch (state)
            {
                case st_STABLIZED :
                {
                    if (FLAG_FLT_MODE == FLT_MODE_AI)
                    {
                        state = st_Start_AIMode;
                    }
                    else if (FLAG_FLT_MODE == FLT_MODE_STABLIZE)
                    {
                        state = st_STABLIZED;
                    }
                }

            break;

                case st_Start_AIMode :
                {
                    Offboard::Result res = offboard.start();
                    if (res == Offboard::Result::Success)
                    {
                        FLAG_AI_SUCCESS = true;
                        state = st_Run_AIMode;
                    }
                    else
                    {
                        // Go Back
                        FLAG_AI_SUCCESS = false;
                        state = st_STABLIZED;
                    }
                }
            break;

                case st_Run_AIMode :
                {
                    AIMode::UpdateParams(attitude_params);
                    ai_attitude.pitch_deg = attitude_params[0];
                    ai_attitude.roll_deg = attitude_params[1];
                    ai_attitude.thrust_value = attitude_params[2];
                    ai_attitude.yaw_deg = attitude_params[3];

                    offboard.set_attitude(ai_attitude);
                }
            break;

            default : break;

            }
        }
        */


        sleep_for(seconds(1));
    }

    return 0;
}


void apilot::CallBack_FlightMode(Telemetry::FlightMode f)
{
    printh("Flight Mode : " + QString::number((int) f));
}

void apilot::CallBack_Battery(Telemetry::Battery btry)
{
     printh("Battery ID : " + QString::number(btry.id));
     printh("Battery Temp : " + QString::number(btry.temperature_degc));
     printh("Battery Volt : " + QString::number(btry.voltage_v));
     printh("Battery Capacity Consumed : " + QString::number(btry.capacity_consumed_ah));
     printh("Battery Remaining Percentage : " + QString::number(btry.remaining_percent));

}

void apilot::CallBack_RC_Channels(const mavlink_message_t msg_raw)
{
    printh("Recieved RC Transmission");
    const mavlink_message_t* msg = &msg_raw;
    mavlink_rc_channels_t rc_channels;
    mavlink_msg_rc_channels_decode(msg, &rc_channels);
    /*
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
    */


    //Channel 6 is for ARM/DISARM
    if (rc_channels.chan6_raw < 1100) //UP
    {
        FLAG_ARM = false;
    }
    else if (rc_channels.chan6_raw > 1800) // DOWN
    {
        FLAG_ARM = true;
    }


    //Channel 5
    if (rc_channels.chan5_raw < 1100)
    {
        //UP
        FLAG_AI_MODE = false;
        FLAG_FLT_MODE = FLT_MODE_STABLIZE;
        printh("AI MODE DISABLED");
    }
    else
    {
        FLAG_AI_MODE = true;
        FLAG_FLT_MODE = FLT_MODE_AI;
        printh("AI MODE ENABLED");
    }

    //Channel 8
    if (rc_channels.chan8_raw < 1100)
    {
        // Dis-engage Target
        printh("Target Dis-Engaged");
    }
    else
    {
        // Engage Target
        printh("Target Engaged");

    }

}

void apilot::CallBack_AttitudeEuler(Telemetry::EulerAngle an)
{
     printh("Roll : " + QString::number(an.roll_deg));
     printh("Pitch : " + QString::number(an.pitch_deg));
     printh("Yaw : " + QString::number(an.yaw_deg));
}

void apilot::CallBack_RCStatus(Telemetry::RcStatus rc)
{
    printh("RC Status :: Is Available :: " + QString::number(rc.is_available));
    printh("RC Status :: Signal Strength :: " + QString::number(rc.signal_strength_percent));
}

void apilot::CallBack_Health(Telemetry::Health h)
{
    printh("Health :: Gyro Calibration :: " + QString::number(h.is_gyrometer_calibration_ok));
    printh("Health :: Accl Calibration :: " +  QString::number(h.is_accelerometer_calibration_ok));
    printh("Health :: Armable :: " +  QString::number(h.is_armable));
}

// /////////////////////////////

void apilot::printh(QString aa)
{
    qDebug() << aa;
    LogIntoFile(aa);
}

void apilot::LogIntoFile(QString aa)
{
    aa = aa + "\n";

    if (ff != nullptr && ff->isOpen())
        ff->write(aa.toUtf8());
}
