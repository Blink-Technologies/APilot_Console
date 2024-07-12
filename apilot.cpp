#include "apilot.h"


apilot::apilot(QObject *parent)
    : QObject{parent}
{}


void apilot::Start()
{
    qDebug()<<"Welcome to APilot Companion Computer";
    sharedVars::InitLogFile();

    flt_timer = new QTimer();
    flt_timer->setInterval(250);
    connect(flt_timer, SIGNAL(timeout()), this, SLOT(on_Flt_Timer_Tick()));

    InitMav();
}


int apilot::InitMav()
{
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_serial_connection(VEHICLE_CONNECTION_PATH, Mavsdk::DEFAULT_SERIAL_BAUDRATE, false, ForwardingOption::ForwardingOff);

    if (connection_result != ConnectionResult::Success) {
        sharedVars::printh("Adding Vechicle Failed");
        return -1;
    }
    sharedVars::printh("Vechicle added at /dev/ttyACM0");


    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        sharedVars::printh("Timed out waiting for system");
        return -2;
    }

    sharedVars::printh("Heartbeat Detected Successfully");


    // Instantiate plugins.
    telemetry = new Telemetry{system.value()};
    action = new Action{system.value()};
    mavlink_passthrough = new MavlinkPassthrough{system.value()};
    offboard = new Offboard{system.value()};

    _atelem = new A_TELEM(telemetry, mavlink_passthrough);


    while(1)
    {
        FSM();
        sleep_for(seconds(1));
    }

    return 0;
}

void apilot::FSM()
{
    // Check for ARM
    if (sharedVars::RC_FLAG_ARM != sharedVars::RC_FLAG_ARM_PREV)
    {
        if (sharedVars::RC_FLAG_ARM)
        {
            if (action->arm() == Action::Result::Success) qDebug()<<"Arming Success";
            else  sharedVars::printh("Arming Failed");
        }
        else
        {
            if (action->disarm() == Action::Result::Success) qDebug()<<"Disarming Success";
            else  sharedVars::printh("Disrming Failed");
        }

        sharedVars::RC_FLAG_ARM_PREV = sharedVars::RC_FLAG_ARM;
    }


    // If Bird is not armed, just retun
    if (!sharedVars::RC_FLAG_ARM) return;

    /*

    switch (state)
    {
        case st_STABLIZED :
        {
        if (sharedVars::RC_FLAG_FLT_MODE == FLT_MODE_AI)
            {
                state = st_Start_AIMode;
            }

        }

        break;

        case st_Start_AIMode :
        {
            Offboard::Result res = offboard->start();
            if (res == Offboard::Result::Success)
            {
                //FLAG_AI_SUCCESS = true;
                state = st_Run_AIMode;
            }
            else
            {
                // Go Back
                //FLAG_AI_SUCCESS = false;
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

            offboard->set_attitude(ai_attitude);
        }
        break;

        default : break;

    }
*/
}

void apilot::on_Flt_Timer_Tick()
{

}
