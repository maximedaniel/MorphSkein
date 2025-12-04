/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "MyPID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
MyPID::MyPID(double Kp, double Ki, double Kd, double alpha, double beta)
{

  MyPID::SetTunings(Kp, Ki, Kd, alpha, beta);
  MyPID::SetOutputRange(0, 255);
   __samplingTime = 100;
   __prevTime = millis();
   __eD = 0;
   __prev_eD = 0;
   __prev_output = 0;
   __integral = 0;  
   __proportional = 0;
   __integral = 0;
   __derivative = 0;
}

void MyPID::SetSampleTime(unsigned long samplingTime)
{
   __samplingTime = samplingTime;
}

void MyPID::SetOutputRange(double outputMin, double outputMax)
{
   __outputMin = outputMin;
   __outputMax = outputMax;
}

void MyPID::SetTunings(double Kp, double Ki, double Kd, double alpha, double beta)
{
   __Kp = Kp;
   __Ki = Ki;
   __Kd = Kd;
   __alpha = alpha;
   __beta = beta;
}

void MyPID::Reset()
{
  __integral = 0;  
  __prev_eD = 0;
  __prev_output = 0;
}


double MyPID::Compute(unsigned long deltaTime, double inputValue, double targetValue, bool debug)
{

  double output = __prev_output;
  double deltaTime_s = double(deltaTime)/1000.0;

  // PID calculations
  __proportional = __Kp * (__alpha * targetValue - inputValue);
  __integral = __integral + __Ki * (targetValue - inputValue) * deltaTime_s;
  __eD =  __beta * inputValue - inputValue;
  __derivative = __Kd * (__eD - __prev_eD)/ deltaTime_s;
  output  =  __proportional + __integral + __derivative;
  
  // Constrain MV to range 0 to 100 for anti-reset windup
  if (output > __outputMax) output = __outputMax;
  if (output < __outputMin) output = __outputMin;

  __integral = output - __proportional - __derivative;
  __prev_eD = __eD;
  __prev_output = output;

  if (debug){
    Serial.print(", deltaTime_s:");
    Serial.print(deltaTime_s);
    Serial.print(", __proportional:");
    Serial.print(__proportional);
    Serial.print(", __integral:");
    Serial.print(__integral);
    Serial.print(", __derivative:");
    Serial.print(__derivative);
    Serial.print(", output:");
    Serial.println(output);

  }
  return output;
}

double MyPID::Compute(unsigned long deltaTime, double inputValue, double targetValue, double trackingValue)
{

  double output = __prev_output;
  double deltaTime_s = double(deltaTime)/1000.0;
  // compute PID
  __integral = trackingValue - __proportional - __derivative;
  // PID calculations
  __proportional = __Kp * (__alpha * targetValue - inputValue);
  __integral = __integral + __Ki * (targetValue - inputValue) * deltaTime_s;
  __eD =  __beta * inputValue - inputValue;
  __derivative = __Kd * (__eD - __prev_eD)/ deltaTime_s;
  output  =  __proportional + __integral + __derivative;

  // Constrain MV to range 0 to 100 for anti-reset windup
  if (output > __outputMax) output = __outputMax;
  if (output < __outputMin) output = __outputMin;
  __integral = output - __proportional - __derivative;
  __prev_eD = __eD;
  __prev_output = output;

  return output;
}


double MyPID::Compute(double inputValue, double targetValue)
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - __prevTime;
  double output = __prev_output;
  if(deltaTime >= __samplingTime){
    output = MyPID::Compute(deltaTime, inputValue, targetValue);
    __prevTime = currentTime;
  } 
  return output;
}

double MyPID::Compute(double inputValue, double targetValue, double trackingValue)
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - __prevTime;

  double output = __prev_output;
  
  if(deltaTime >= __samplingTime)
  {
    
    double deltaTime_s = double(deltaTime)/1000.0;
    // compute PID
    __integral = trackingValue - __proportional - __derivative;
    // PID calculations
    __proportional = __Kp * (__alpha * targetValue - inputValue);
    __integral = __integral + __Ki * (targetValue - inputValue) * deltaTime_s;
    __eD =  __beta * inputValue - inputValue;
    __derivative = __Kd * (__eD - __prev_eD)/ deltaTime_s;
    output  =  __proportional + __integral + __derivative;

    // Constrain MV to range 0 to 100 for anti-reset windup
    if (output > __outputMax) output = __outputMax;
    if (output < __outputMin) output = __outputMin;
    __integral = output - __proportional - __derivative;

    __prevTime = currentTime;
    __prev_eD = __eD;
    __prev_output = output;
  } 
  return output;
}

double MyPID::GetProportional(){return __proportional;}
double MyPID::GetIntegral(){return __integral;}
double MyPID::GetDerivative(){return __derivative;}