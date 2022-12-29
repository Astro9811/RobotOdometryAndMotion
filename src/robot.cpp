#include "main.h"

using namespace pros;

//Rotations Sensors
Rotation Left(12);
Rotation Right(19, true);
Rotation Back(10);

//Motors
Motor FL(2, true);
Motor FR(9);
Motor BL(11, true);
Motor BR(20);

//////////////////////////////////////////////////////////////////////
//Adjust constants for odometry

//Constants that convert wheel revolutions to inches travelled
//LR for left and right tracking wheels, S for back tracking wheel
const double LR = 8.639379797;
const double B = 10.21017612;

//Distances from center of robot to tracking wheels
const double sl = 6.735;
const double sr = 6.735;
const double sb = 7.0;

//////////////////////////////////////////////////////////////////////

//Global variables for the last position of encoders
double lastL = 0.0;
double lastR = 0.0;
double lastB = 0.0;

//Array to hold the current position: cur[0] = x, cur[1] = y, cur[2] = current heading (angle)
double cur[3];

double power = 0.0;
double turn = 0.0;
double currentDistance = 0.0;
double endDistance = 0.0;
void test(){
	while(1){
		pros::lcd::print(5, "Distance left: %.3lf", (endDistance - currentDistance));
        pros::lcd::print(6, "Power: %.3lf", power);
		pros::delay(5);
	}
}

void trackPosition(){
    while(1){
        //Current encoder positions (add negatives to reverse values)
        double left = Left.get_position()/100;
        double right = Right.get_position()/100;
        double back = Back.get_position()/100;

        //Change in distance travelled (inches) for each tracking wheel
        double l = (left - lastL) / 360 * LR;
        double r = (right - lastR) / 360 * LR;
        double b = (back - lastB) / 360 * B;

        //Update the last position values
        lastL = left;
        lastR = right;
        lastB = back;

        double h; //hypotenuse that gives total displacement of robot
        double i; // Half of a, the increase in current heading
        double h2;
        double a = (l - r) / (sl + sr);

        if(a){

            double radius = r / a; //Radius of the circle that the robot travels with its right wheel.
            i = a / 2.0;
            double sinI = sin(i);
            h = 2.0 * ((radius + sr) * sinI);

            double radiusB = b / a; //Radius of the circle that the robot travels with its back wheel.
            h2 = 2.0 * ((radiusB + sb) * sinI);
        }

        else{

            h = r;
            i = 0;
            h2 = b;
        }

        double ma = i + cur[2]; //Angle of movement but not the final angle position
        double cosMA = cos(ma);
        double sinMA = sin(ma);

        //Updates the global position
        cur[1] += h * cosMA;
        cur[0] += h * sinMA;

        //Drift correction to the global position
        cur[1] += h2 * -sinMA;
        cur[0] += h2 * cosMA;

        cur[2] += a; //Updates the current global angle

        //Keeps our current heading between 180 and -180 degrees, convenient for moveTo function and allows robot to turn optimally
        double angleWrap = (cur[2] < -M_PI) ? cur[2] += 2 * M_PI : (cur[2] > M_PI) ? cur[2] -= 2 * M_PI : M_PI;

        //Prints global position to Brain
        lcd::print(1, "Current Coordinates:");
        lcd::print(2, "Y: %.3lf", cur[1]);
        lcd::print(3, "X: %.3lf", cur[0]);
        lcd::print(4, "heading: %.3lf", cur[2] * (180 / M_PI));
        
        //Delay to save resources when running as a task
        delay(5);
    }
}

void resetEncoders(){

    Left.set_position(0);
    Right.set_position(0);
    Back.set_position(0);
}

//////////////////////////////////////////////////////////////////////
//Adjust constants for PD loop
double kp = 1.0;
double kd = 0.0;
double min = 22.0;

double turnKP = 0.9;
//////////////////////////////////////////////////////////////////////
int counter = 0;
double prevError = 0.0;
int counterReset = 0;


double getValue(double error, double kp, double min = min){

    double derivative = (error - prevError);

    prevError = error;
    counter++;

    double speed = (kp * error) + (kd * derivative);
    double coefficient = (std::min(100, counter)) / 100;

    return coefficient * (abs(speed) > min) ? speed : (speed > 0.0) ? min : -min;
}

void reset(){
    counter = counterReset;
}

// void moveTo(double *target, bool purePursuit, double *speeds){

//     //Convert current angle to degrees
//     double heading = cur[2] * (180 / M_PI);

//     double xError = target[0] - cur[0];
//     double yError = target[1] - cur[1];
//     double headingError = target[2] - heading;

//     //Displacement error (error of h)
//     double hError = sqrt(pow(xError, 2) + pow(yError, 2) * 1.0);

//     int coefficient = 0;

//     while (abs(yError) > 5 || abs(xError) > 5 || abs(headingError) > 5){

//         double power = getValue(hError * speeds[0]);
//         double turn = getValue(headingError * speeds[1]);

//         drive(power, turn, false);

//         //Updating error values
//         double heading = cur[2] * (180 / M_PI);

//         double xError = target[0] - cur[0];
//         double yError = target[1] - cur[1];
//         double headingError = target[2] - heading;

//         if (purePursuit) return;
//         delay(5);

//     }

//     reset(); //Reset PD counter
//     brake(false); //Brake robot

// }

void moveStraight(double *target){

    double xError = target[0] - cur[0];
    double yError = target[1] - cur[1];

    double hError = sqrt(pow(xError, 2) + pow(yError, 2) * 1.0);

    //heading to move to
    double headingChange = atan2(xError, yError) * (180 / M_PI);

    double targetHeading = headingChange;

    //Turns robot to face target coordinates
    turnTo(targetHeading);

    double heading = cur[2] * (180 / M_PI);
    double headingError = targetHeading - heading;

    if (abs(headingError) > 180){
        headingError +=360;
    }

    currentDistance = (Left.get_position() + Right.get_position())/72000 * LR;
    endDistance = currentDistance + hError;

    int count = 0;

    while((endDistance - currentDistance) > 1 ){

        power = getValue(hError, kp, 40.0);
        turn = getValue(headingError, turnKP, 10.0);


        drive(power, turn, false);

        //Updating error values
        heading = cur[2] * (180 / M_PI);
        headingError = targetHeading - heading;

        if (abs(headingError) > 180){
            headingError +=360;
        }

        currentDistance = (Left.get_position() + Right.get_position())/72000 * LR;

        delay(5);
        count += 5;

    }
    
    reset(); //Reset PD counter
    brake(false); //Brake robot
    //For playtesting only
    //brake(true);

}

void moveTo(double *target, bool purePursuit, double *speeds){

    //Convert current angle to degrees
    double heading = cur[2] * (180 / M_PI);

    double xError = target[0] - cur[0];
    double yError = target[1] - cur[1];
    double headingError = target[2] - heading;

    if (abs(headingError) > 180){
        headingError +=360;
    }

    //Displacement error (error of h)
    double hError = sqrt(pow(xError, 2) + pow(yError, 2) * 1.0);


    while (abs(hError > 2)){

        power = getValue(hError * speeds[0], kp, 40.0);
        turn = getValue(headingError * speeds[1], turnKP, 10.0);

        drive(power, turn, false);

        //Updating error values
        heading = cur[2] * (180 / M_PI);

        xError = target[0] - cur[0];
        yError = target[1] - cur[1];
        hError = sqrt(pow(xError, 2) + pow(yError, 2) * 1.0);

        headingError = target[2] - heading;

        if (abs(headingError) > 180){
            headingError +=360;
        }

        if (purePursuit) return;
        delay(5);

    }

    reset(); //Reset PD counter
    brake(false); //Brake robot
    //For playtesting only
    brake(true);

}

//done for now
void turnTo(double target){

    //Convert current angle to degrees
    double heading = cur[2] * (180 / M_PI);
    double headingError = target - heading;
    int count = 0;

    //For cases, where you start at 180 degrees and need to turn right.
    if (abs(headingError) > 180){
        headingError +=360;
    }

    int coefficient = 0; 

    while (abs(headingError) > 1){

        double power = 0;
        double turn = getValue(headingError, turnKP);

        drive(power, turn, false);

        //Updating error values
        heading = cur[2] * (180 / M_PI);
        headingError = target - heading;

        if (abs(headingError) > 180){
            headingError +=360;
        }

        //Limits movement to 1.75 sec (preventing infinite spins)
        if (count > 2500){
            break;
        }

        delay(5);
        count += 5;

    }

    reset(); //Reset PD counter
    //Sets current heading to target to counter heading drift
    //cur[2] = target / (180 / M_PI);
    brake(false); //Brake robot
    //remove when finished playtesting
    brake(true);

}

// void turnTo(double target){

//     //Convert current angle to degrees
//     double heading = cur[2] * (180 / M_PI);
//     double headingError = target - heading;

//     //For cases, where you start at 180 degrees and need to turn right.
//     if (abs(headingError) > 180){
//         headingError +=360;
//     }

//     int coefficient = 0;


//     while (abs(headingError) > 1){

//         double power = 0;
//         double turn = 15.0;
//         bool min = true;

//         drive(power, turn, min);

//         //Updating error values
//         heading = cur[2] * (180 / M_PI);
//         headingError = target - heading;

//         if (abs(headingError) > 180){
//             headingError +=360;
//         }

//         delay(5);

//     }

//     reset(); //Reset PD counter
//     //Sets current heading to target to counter heading drift
//     //cur[2] = target / (180 / M_PI);
//     brake(); //Brake robot

// }

void drive(int power, int turn, bool min){

    if (min){
        FL.move_velocity(power + turn);
        BL.move_velocity(power + turn);
        FR.move_velocity(power - turn);
        BR.move_velocity(power - turn);
    }

    else{
        FL = (power + turn);
        BL = (power + turn);
        FR = (power - turn);
        BR = (power - turn);
    }

}

void brake(bool coast){

    if (coast){
        FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
        FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
    }
    else{
        FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    }

    FL.brake();
    FR.brake();
    BL.brake();
    BR.brake();

}

int sign(double x){
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

double getDegrees(std::vector<double> target, double *start){

    double x = target[0] - start[0];
    double y = target[1] - start[1];
    return atan2(-y, x) * 180 / M_PI;
}

std::vector<double> getIntersection(std::vector<double> start, std::vector<double> end, double *cur, double radius){

    std::vector<double> p1 {start[0] - cur[0], start[1] - cur[1]};
    std::vector<double> p2 {end[0] - cur[0], end[1] - cur[1]}; 

    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];

    float d = sqrt(pow(dx, 2) + pow(dy, 2) * 1.0);
    float D = p1[0] * p2[1] - p2[0] * p1[1];
    float discriminant = abs(pow(radius, 2) * pow(d, 2) - pow(D, 2));

    float x1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / pow(d, 2);
    float y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / pow(d, 2);
    float x2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / pow(d, 2);
    float y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / pow(d, 2);

    /* Above calculations can be explained and proven here: https://mathworld.wolfram.com/Circle-LineIntersection.html */

    float distance1 = sqrt(pow(p2[0] - x1, 2) + pow(p2[1] - y1, 2) * 1.0);
    float distance2 = sqrt(pow(p2[0] - x2, 2) + pow(p2[1] - y2, 2) * 1.0);


    if (distance1 < distance2){
        
        std::vector<double> calc1 {(x1 + cur[0]), (y1 + cur[1])};
        return calc1;
    }

    //if (distance1 > distance2)
    else{

        std::vector<double> calc2 {(x2 + cur[0]), (y2 + cur[1])};
        return calc2;
    } 

}

void movePurePursuit(std::vector<std::vector<double>> points, std::vector<double> final_point, std::vector<double> speeds){

    //To complete if moveTo function works
}