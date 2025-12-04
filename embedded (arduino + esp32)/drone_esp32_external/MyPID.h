#ifndef MyPID_h
#define MyPID_h

class MyPID
{
    public:
  //commonly used functions **************************************************************************
    MyPID(double, double, double, double, double);
    void SetSampleTime(unsigned long);
    double Compute(unsigned long, double, double, bool debug=false);
    double Compute(double, double);
    double Compute(unsigned long, double, double, double);
    double Compute(double, double, double);
    void SetTunings(double, double, double, double, double);
    void SetOutputRange(double, double);
    void Reset();
    double GetProportional();
    double GetIntegral();
    double GetDerivative();

    private:
    double __Kp;
    double __Ki;
    double __Kd;
    double __alpha;
    double __beta;
    unsigned long __samplingTime;
    // PID values
    double __proportional;
    double __integral;
    double __derivative;
    double __outputMin;
    double __outputMax;

    // useful
    double __eD;
    double __prev_eD;
    unsigned long __prevTime;
    double __prev_output;


};
#endif

