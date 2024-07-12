#include "aimode.h"

float AIMode::cx =0;
float AIMode::cy =0;
float AIMode::fcx =0;
float AIMode::fcy =0;


AIMode::AIMode(QObject *parent)
    : QObject{parent}
{}

void AIMode::CalculateTargetToCenter(float *error_x, float *error_y)
{
    if (cx && cy) {
        *error_x = cx - fcx;
        *error_y = cy - fcy;
    } else {
        *error_x = 0;
        *error_y = 0;
    }

}

void AIMode::CalculateFCXY()
{
    fcx = 640.0 / 2;
    fcy = 640.0 / 2;
}


void AIMode::UpdateParams(float *attitude_params)
{
    float error_x =0;
    float error_y =0;

    AIMode::CalculateFCXY();
    AIMode::CalculateTargetToCenter(&error_x, &error_y);


    if (error_x && error_y)
    {
        float speed_in_percentage = 0.75;
        float kp_pitch = 0.5;
        float kp_thrust = 0.5;
        float kp_yaw_rate = 0.5;

        float updated_pitch = speed_in_percentage * kp_pitch; // Multiply by percentage of wanted speed to minimize missing frame from vibration
        float updated_thrust = 0.5 + error_y * kp_thrust; // Example proportional controller
        float updated_yaw = error_x * kp_yaw_rate; // Example proportional controller
        float updated_roll =0;

        // Saturate the variables
        updated_pitch = std::clamp(updated_pitch, 0.0f, 45.0f); // Must be between 0 degree to 45 degree
        updated_thrust = std::clamp(updated_thrust, 0.0f, 1.0f); // Must be between 0 and 1
        updated_yaw = std::clamp(updated_yaw, -3.0f, 3.0f); // Must be between -3 degrees/second and 3 degrees/second


        attitude_params[0] = updated_pitch;
        attitude_params[1] = updated_yaw;
        attitude_params[2] = updated_roll;
        attitude_params[3] = updated_thrust;

    }
}
