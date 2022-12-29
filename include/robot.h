#ifndef ROBOT_H
#define ROBOT_H

extern pros::Motor FL;
extern pros::Motor FR;
extern pros::Motor BL;
extern pros::Motor BR;

//Debugging Function
void test(void);

//Odometry related functions
void trackPosition(void);
void resetEncoders(void);

//PD related functions
double getValue(double error, double kp, double min);
void reset();

//Motion functions
void moveTo(double *target, bool purePursuit, double *speeds);
void moveStraight(double *target);
void turnTo(double target);
void drive(int power, int turn, bool min);
void brake(bool coast);

//Pure Pursuit Functions
int sign(double x);
double getDegrees(double *p1, double *p2);
std::vector<double> getIntersection(std::vector<double> start, std::vector<double> end, std::vector<double> cur, double radius);
void movePurePursuit(std::vector<std::vector<double>> points, std::vector<double> final_point, std::vector<double> speeds);

#endif 