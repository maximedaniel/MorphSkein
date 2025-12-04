
#include <Arduino.h>
#include "MyPID.h"
#include "MotionPlanner.h"

short int MotionPlanner::sign(double value) {
  return (value < 0)? -1 : ((value > 0)? 1 : 0);
}

double MotionPlanner::abs(double value) {
	return (value > 0)? value:-value;
}


double linearAcceleration(float currentTime, float startVel, float endVel, float duration) {
  return  startVel + (endVel - startVel)  * _min(1, currentTime/duration);
}

double trapezoidalVelocity(float currentTime, float startPos, float endPos, float duration) {

  float totalDistance = endPos - startPos;
  float max_velocity = 1.5 * totalDistance/duration;
  float accelerationTime = duration / 3.0;
  float decelerationTime = accelerationTime;
  float max_acceleration = max_velocity/accelerationTime;
  float constantVelocityTime = duration - (2 * accelerationTime);

  if (currentTime < accelerationTime) {
    return  max_velocity * _min(1, currentTime/accelerationTime);
  } 
  else if (currentTime < accelerationTime + constantVelocityTime) {
    return  max_velocity;
  } 
  else if (currentTime < duration) {
    return max_velocity * _max(0, (duration - currentTime)/decelerationTime);
    } 
  else {
    return 0;
  }
}

double trapezoidalPosition(float currentTime, float startPos, float endPos, float duration) {

  float totalDistance = endPos - startPos;
  float max_velocity = 1.5 * totalDistance/duration;
  float accelerationTime = duration / 3.0;
  float decelerationTime = accelerationTime;
  float max_acceleration = max_velocity/accelerationTime;
  float constantVelocityTime = duration - (2 * accelerationTime);
  float totalDistanceAcceleration = 0.5 * max_velocity * accelerationTime;
  float totalDistanceDeceleration = totalDistanceAcceleration;
  float totalDistanceConstant = totalDistance - (totalDistanceAcceleration + totalDistanceDeceleration);

  if (currentTime < accelerationTime) {
    return startPos + totalDistanceAcceleration * _min(1, currentTime/accelerationTime);
  } 
  else if (currentTime < accelerationTime + constantVelocityTime) {
    return startPos + totalDistanceAcceleration + totalDistanceConstant * _min(1, (currentTime - accelerationTime)/constantVelocityTime);
  } 
  else if (currentTime < duration) {
    return startPos + totalDistanceAcceleration + totalDistanceConstant + totalDistanceDeceleration * _min(1, (currentTime - (accelerationTime + constantVelocityTime))/decelerationTime);
    } 
  else {
    return endPos;
  }
}


double sCurveProfilPosition(double t, double start_pos, double end_pos, double duration_max){
  double c = duration_max/2;
  double tolerance = 0.0001;
  double b = 1/c * log((1-tolerance)/tolerance);
  double pos = start_pos + (end_pos - start_pos) * (1/(1 + exp(-b * (t-c))));
  return pos;
}

double sCurveProfilVelocity(double t, double start_pos, double end_pos, double duration_max){
  double c = duration_max/2;
  double tolerance = 0.0001;
  double b = 1/c * log((1-tolerance)/tolerance);
  double speed = (end_pos - start_pos)  * ( (b * exp(-b * (t-c))) / pow(1 + exp(-b * (t-c)), 2) );
  return speed;
}



MotionPlanner::MotionPlanner(MotionMode motion_mode, double Kp, double Ki, double Kd, double alpha, double beta) {
  __pid_regulator = new MyPID(Kp, Ki, Kd, alpha, beta);
   MotionPlanner::tune(Kp, Ki, Kd, alpha, beta);
  _reaching = false;
  _holding = false;
  _motion_mode = motion_mode;
  _motion_status = MotionStatus::OFF;
  _output = _prev_output = 0;

}
MotionStatus MotionPlanner::status(){
  return _motion_status;
}

void MotionPlanner::tune(double Kp, double Ki, double Kd, double alpha, double beta){
  __Kp = Kp;
  __Ki = Ki;
  __Kd = Kd;
  __alpha = alpha;
  __beta = beta;
  __pid_regulator -> SetTunings(__Kp, __Ki, __Kd, __alpha, __beta);
  __pid_regulator -> Reset();

}


void MotionPlanner::set(double start_value, double end_value, double target_duration, bool holding, bool reset){

  _target_duration = target_duration;

  if (_motion_mode ==  MotionMode::POSITION){
      _prev_output = 0;
      _start_position = start_value;
      _end_position = end_value;
      _target_position = start_value;
      double max_velocity = 1.5 * MotionPlanner::abs(_end_position - _start_position)/_target_duration;
      __pid_regulator -> SetOutputRange(-max_velocity, max_velocity);
  }
  
  else if (_motion_mode ==  MotionMode::VELOCITY){
      _start_velocity = start_value;
      _end_velocity = end_value;
      _target_velocity = start_value;
      __pid_regulator -> SetOutputRange(0, 800); // NEED TO FIX THIS

  }
  if (reset){
    __pid_regulator -> Reset();
  }
  _start_time = _prev_time = millis();
  //_prev_time = _prev_time - DELTA_TIME_DELAY; // make sure a new output is computed the first time process is called
  _reaching = true;
  _holding = holding;
  _motion_status = MotionStatus::REACHING;
}

bool MotionPlanner::isReaching(){
  return _reaching;
}

bool MotionPlanner::isHolding(){
  return !_reaching & _holding;
}

bool MotionPlanner::isActive(){
  return MotionPlanner::isReaching() || MotionPlanner::isHolding();
}

void MotionPlanner::stop() {
  _reaching = _holding = false;
  _motion_status =  MotionStatus::OFF;
}

double MotionPlanner::process(double curr_value, bool debug){
  _curr_time = millis();
  _output = _prev_output;
  float delta_time = (_curr_time - _prev_time) / 1000.0;
  if (delta_time >= DELTA_TIME_DELAY){
      _curr_duration = (_curr_time - _start_time)/1000.0;
      if(_motion_mode == MotionMode::POSITION){
          _curr_position = curr_value; 
           double _pid_correction_velocity = 0;
            if (MotionPlanner::isReaching()){
              _target_velocity = trapezoidalVelocity(_curr_duration, _start_position, _end_position, _target_duration);
              _target_position =  trapezoidalPosition(_curr_duration, _start_position, _end_position, _target_duration);
              _pid_correction_velocity = __pid_regulator -> Compute((_curr_time - _prev_time), _curr_position, _target_position, false);
              _output =  _target_velocity + _pid_correction_velocity;
              if ((_end_position > _start_position && _curr_position >= _end_position) || (_end_position < _start_position && _curr_position <= _end_position)){
                __pid_regulator -> Reset();
                _target_position = _end_position;
                _output = 0;
                _reaching = false;
                _motion_status =  _holding ? MotionStatus::REACHED_AND_HOLDING : MotionStatus::REACHED;
              }
            }

            if (MotionPlanner::isHolding()){
               if (MotionPlanner::abs(_target_position - _curr_position) > 0.01){
                _pid_correction_velocity = __pid_regulator -> Compute((_curr_time - _prev_time), _curr_position, _target_position, false);
                _output =  _pid_correction_velocity;
               } else {
                 _output = 0;
               }

            }
          
            if (debug) {//&& _curr_time - _prev_display_time > DISPLAY_TIME_DELAY){
                Serial.print("_curr_duration:");
                Serial.print(_curr_duration);
                Serial.print(", _curr_velocity:");
                Serial.print(_curr_velocity);
                Serial.print(", _target_velocity:");
                Serial.print(_target_velocity);
                Serial.print(", _pid_correction_velocity:");
                Serial.print(_pid_correction_velocity);
                Serial.print(", _current_position:");
                Serial.print(_curr_position);
                Serial.print(", _target_position:");
                Serial.print(_target_position);
                Serial.print(", holding:");
                Serial.print(MotionPlanner::isHolding());
                Serial.println();
                _prev_display_time = _curr_time;
            }
      }
      else if (_motion_mode == MotionMode::VELOCITY){
            _curr_velocity = curr_value; 
            double _pid_correction_velocity = 0;
            if (MotionPlanner::isReaching()){

              _target_velocity = _start_velocity + (_end_velocity - _start_velocity)  * _min(1, _curr_duration/_target_duration);
              _pid_correction_velocity = __pid_regulator -> Compute((_curr_time - _prev_time), _curr_velocity, _target_velocity, false);
              _output = _target_velocity * __Kp + _pid_correction_velocity;

              if ((_end_velocity > _start_velocity && _curr_velocity >= _end_velocity) || (_end_velocity < _start_velocity && _curr_velocity <= _end_velocity)){
                _target_velocity = _end_velocity;
                _reaching = false;
                _motion_status =  _holding ? MotionStatus::REACHED_AND_HOLDING : MotionStatus::REACHED;
              }

            }
            if (MotionPlanner::isHolding()){
                _pid_correction_velocity = __pid_regulator -> Compute((_curr_time - _prev_time), _curr_velocity, _target_velocity, false);
                _output =  _target_velocity  * __Kp + _pid_correction_velocity;
            }
            
            if (debug && _curr_time - _prev_display_time > DISPLAY_TIME_DELAY){
                Serial.print("_curr_duration:");
                Serial.print(_curr_duration);
                Serial.print(", _curr_velocity:");
                Serial.print(_curr_velocity);
                Serial.print(", _target_velocity:");
                Serial.print(_target_velocity);
                Serial.print(", _pid_correction_velocity:");
                Serial.print(_pid_correction_velocity);
                Serial.print(", holding:");
                Serial.print(MotionPlanner::isHolding());
                Serial.println();
                _prev_display_time = _curr_time;
            }
      }
      _prev_output = _output;
      _prev_time = _curr_time;
  }

  return _output;
}

