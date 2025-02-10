#include "odometry.h"
#include "robot_config.h"
#include "robot.h"

Odometry::Odometry()
{
    x = 0;
    y = 0;
    h = 0;
    startH = 0;
    deltaX = 0;
    deltaY = 0;
    deltaH = 0;
    totalLDist = 0;
    totalRDist = 0;
    deltaLDist = 0;
    deltaRDist = 0;
    deltaAvgDist = 0;
    currentLMDPosition = 0;
    currentRMDPosition = 0;
    currentInertialRotation = 0;
    LeftMiddleDrive.resetPosition();
    RightMiddleDrive.resetPosition();
    Inertial.resetRotation();
}

double Odometry::getX()
{ return x; }

double Odometry::getY()
{ return y; }

double Odometry::getH()
{ return h; }

void Odometry::setPosition(double ax, double ay)
{
    x = ax;
    y = ay;
    startH = h;
    currentLMDPosition = 0;
    currentRMDPosition = 0;
    currentInertialRotation = h;
    LeftMiddleDrive.resetPosition();
    RightMiddleDrive.resetPosition();
    Inertial.setRotation(h, degrees);
}

void Odometry::setPosition(double ax, double ay, double ah)
{
    x = ax;
    y = ay;
    h = ah;
    startH = h;
    currentLMDPosition = 0;
    currentRMDPosition = 0;
    currentInertialRotation = h;
    LeftMiddleDrive.resetPosition();
    RightMiddleDrive.resetPosition();
    Inertial.setRotation(h, degrees);
}

void Odometry::resetHeading()
{
    h = 0;
    startH = 0;
    currentLMDPosition = 0;
    currentRMDPosition = 0;
    currentInertialRotation = 0;
    LeftMiddleDrive.resetPosition();
    RightMiddleDrive.resetPosition();
    Inertial.resetHeading();
}

double Odometry::getLeftEncoder()
{ return LeftMiddleDrive.position(degrees) * motorToWheelConverter; }

double Odometry::getRightEncoder()
{ return RightMiddleDrive.position(degrees) * motorToWheelConverter; }

double Odometry::getInertialRotation()
{ return Inertial.rotation(degrees); }

// Update the change in distance of both wheels as well as average of both wheels
void Odometry::updateDeltaDist()
{
    // Store previous positions in temporary variable
    double leftPrev = currentLMDPosition;
    double rightPrev = currentRMDPosition;
    // Update global previous variable
    currentLMDPosition = getLeftEncoder();
    currentRMDPosition = getRightEncoder();
    // Update total distance
    totalLDist = currentLMDPosition * degreeToInchConverter;
    totalRDist = currentRMDPosition * degreeToInchConverter;
    // Current value - Previous value equals change
    deltaLDist = (currentLMDPosition - leftPrev) * degreeToInchConverter;
    deltaRDist = (currentRMDPosition - rightPrev) * degreeToInchConverter;
    deltaAvgDist = ((deltaLDist + deltaRDist) / 2.0) * degreeToInchConverter;
}

// Update the Heading of the robot by averaging the inertial sensor and two wheel odometry values
void Odometry::updateHeading()
{
    double rotationPrev = currentInertialRotation;
    currentInertialRotation = Inertial.heading();
    h = ( (startH + ((totalLDist - totalRDist) / dist_from_center) * radianToDegreeConverter) + currentInertialRotation) / 2.0;
    deltaH = ( ( ((deltaLDist - deltaRDist) / dist_from_center) * radianToDegreeConverter) + (currentInertialRotation - rotationPrev) ) / 2.0;
}

// Set as a task during auton
// Always running in a loop
void Odometry::calculatePosition()
{
    // Calculate change in wheel positions
    updateDeltaDist();
    // Calculate overall heading and change in heading
    updateHeading();
    // Calculate the input for the cos and sin functions for the odometry equations
    double cos_sin_input = degreeToRadianConverter * (h + (deltaH / 2.0) );
    // Calculate the Change in x and change in y
    deltaX = deltaAvgDist * cos(cos_sin_input);
    deltaY = deltaAvgDist * sin(cos_sin_input);
    // Add to previous x and y values
    x += deltaX;
    y += deltaY;
}

// Global variable to store odometry object
Odometry odom = Odometry();

// Global variable to store task instance
thread odometryTask;

int odomPrintWait = 100;
// a task started by usercontrol
void odometryFunction()
{
    while(true)
    {
        odom.calculatePosition();
        if (odomPrintWait <= 100)
        {
            printf("x:%f y:%f h:%f \n", odom.getX(), odom.getY(), odom.getH());
        }
        odomPrintWait--;
        this_thread::sleep_for(10);
    }
}