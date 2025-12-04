#ifndef MotionPlanner_h
#define MotionPlanner_h

#include "Arduino.h"
#include "MyPID.h"

#define DELTA_TIME_DELAY 0.05
#define DISPLAY_TIME_DELAY 50
#define SLOW_KI 0.05
#define FAST_KI 0.25

enum MotionMode {
  POSITION = 1, 
  VELOCITY = 2
};

enum MotionStatus {
  REACHING = 1, 
  REACHED = 2, 
  REACHED_AND_HOLDING = 3, 
  OFF = 4, 
};

class MotionPlanner
{
  public:
    MotionPlanner(MotionMode motion_mode, double Kp, double Ki, double Kd, double alpha, double beta);
    void set(double start_value, double end_value, double target_duration, bool holding, bool reset);
    void tune(double Kp, double Ki, double Kd, double alpha, double beta);
    void stop();
    bool isActive();
    bool isReaching();
    bool isHolding();
    MotionStatus status();
    double abs(double value);
    short int sign(double value);
    double process(double value, bool debug);
  private:
  
    MyPID* __pid_regulator;
    double __Kp, __Ki, __Kd, __alpha, __beta;

    // POSITION variables
    double _start_position;
    double _curr_position;
    double _end_position;
    double _target_position;

    // VELOCITY variables
    double _start_velocity;
    double _curr_velocity;
    double _end_velocity;
    double _target_velocity;

    // TIME variables
    double _curr_duration;
    double _target_duration;

    // OUTPUT variables
    double _output;
    double _prev_output;

    unsigned long _start_time;
    unsigned long _curr_time;
    unsigned long _prev_time;
    unsigned long _prev_display_time;

    MotionMode _motion_mode;
    MotionStatus _motion_status;
    bool _reaching;
    bool _holding;
};

#endif
 