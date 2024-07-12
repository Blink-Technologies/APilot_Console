#include "a_telem.h"

A_TELEM::A_TELEM(Telemetry *tmt, MavlinkPassthrough *mav_pt) {

    telemetry = tmt;
    mavlink_passthrough = mav_pt;
}


void A_TELEM::Telemetry_Subscribe()
{
    Telemetry::FlightMode fm = telemetry->flight_mode();
    sharedVars::printh("Inital Flight Mode : " + QString::number((int)fm));


    if (telemetry->set_rate_battery(1.0) != Telemetry::Result::Success)
        sharedVars::printh("Setting Rate failed: Battery");
    else
        sharedVars::printh("Setting Rate Success: Battery");

    if (telemetry->set_rate_attitude_euler(1.0) != Telemetry::Result::Success)
        sharedVars::printh("Setting Rate failed: Euler");
    else
        sharedVars::printh("Setting Rate Success: Euler");

    if (telemetry->set_rate_rc_status(1.0) != Telemetry::Result::Success)
        sharedVars::printh("Setting Rate failed: RC Status");
    else
        sharedVars::printh("Setting Rate Success: RC Status");


    telemetry->subscribe_flight_mode(A_TELEM::CallBack_FlightMode);
    telemetry->subscribe_battery(A_TELEM::CallBack_Battery);
    telemetry->subscribe_attitude_euler(A_TELEM::CallBack_AttitudeEuler);
    telemetry->subscribe_rc_status(A_TELEM::CallBack_RCStatus);
    telemetry->subscribe_health(A_TELEM::CallBack_Health);

}

void A_TELEM::Telemetry_Subscribe_RC_Channels()
{
    // Set Rate of RC Channels
    MavlinkPassthrough::CommandLong cmd_to_set_rate;
    cmd_to_set_rate.target_sysid = mavlink_passthrough->get_target_sysid();
    cmd_to_set_rate.target_compid = mavlink_passthrough->get_target_compid();
    cmd_to_set_rate.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd_to_set_rate.param1 = MAVLINK_MSG_ID_RC_CHANNELS;
    cmd_to_set_rate.param2 = 1000000;  //1Hz
    cmd_to_set_rate.param3 =0;
    cmd_to_set_rate.param4 =0;
    cmd_to_set_rate.param5 =0;
    cmd_to_set_rate.param6 =0;
    cmd_to_set_rate.param7 =0;
    MavlinkPassthrough::Result res = mavlink_passthrough->send_command_long(cmd_to_set_rate);
    sharedVars::printh("MPT:: RC Rate Result : " + QString::number((int)res));


    mavlink_passthrough->subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS, A_TELEM::CallBack_RC_Channels);
}

void A_TELEM::CallBack_FlightMode(Telemetry::FlightMode f)
{
    sharedVars::printh("Flight Mode : " + QString::number((int) f));
}

void A_TELEM::CallBack_Battery(Telemetry::Battery btry)
{
    sharedVars::printh("Battery ID : " + QString::number(btry.id));
    sharedVars::printh("Battery Temp : " + QString::number(btry.temperature_degc));
    sharedVars::printh("Battery Volt : " + QString::number(btry.voltage_v));
    sharedVars::printh("Battery Capacity Consumed : " + QString::number(btry.capacity_consumed_ah));
    sharedVars::printh("Battery Remaining Percentage : " + QString::number(btry.remaining_percent));

}

void A_TELEM::CallBack_RC_Channels(const mavlink_message_t msg_raw)
{
    sharedVars::printh("Recieved RC Transmission");
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
        sharedVars::RC_FLAG_ARM = false;
    }
    else if (rc_channels.chan6_raw > 1800) // DOWN
    {
        sharedVars::RC_FLAG_ARM= true;
    }


    //Channel 5
    if (rc_channels.chan5_raw < 1100)
    {
        //UP
        sharedVars::RC_FLAG_FLT_MODE = 0;
        sharedVars::printh("AI MODE DISABLED");
    }
    else
    {
        sharedVars::RC_FLAG_FLT_MODE = 1;
        sharedVars::printh("AI MODE ENABLED");
    }

    //Channel 8
    if (rc_channels.chan8_raw < 1100)
    {
        // Dis-engage Target
        sharedVars::printh("Target Dis-Engaged");
    }
    else
    {
        // Engage Target
        sharedVars::printh("Target Engaged");

    }

}

void A_TELEM::CallBack_AttitudeEuler(Telemetry::EulerAngle an)
{
    sharedVars::printh("Roll : " + QString::number(an.roll_deg));
    sharedVars::printh("Pitch : " + QString::number(an.pitch_deg));
    sharedVars::printh("Yaw : " + QString::number(an.yaw_deg));
}

void A_TELEM::CallBack_RCStatus(Telemetry::RcStatus rc)
{
    sharedVars::printh("RC Status :: Is Available :: " + QString::number(rc.is_available));
    sharedVars::printh("RC Status :: Signal Strength :: " + QString::number(rc.signal_strength_percent));
}

void A_TELEM::CallBack_Health(Telemetry::Health h)
{
    sharedVars::printh("Health :: Gyro Calibration :: " + QString::number(h.is_gyrometer_calibration_ok));
    sharedVars::printh("Health :: Accl Calibration :: " +  QString::number(h.is_accelerometer_calibration_ok));
    sharedVars::printh("Health :: Armable :: " +  QString::number(h.is_armable));
}
