
#include <Motoron.h>
#include "Rotary.h"

#define CW 0
#define CCW 1

#define OUTA_PIN 0
#define OUTB_PIN 1
#define REDUCTION_FACTOR 380
#define SPINNER_NB_PULSE_PER_REVOLUTION 48
#define PULSE_PER_REVOLUTION 12
#define PULSE_COUNT_CORRECTION 4
#define SAMPLING_DURATION 10000
#define DELTA_TIME 0.05

// MOTOR CONTROLLER
MotoronI2C mc;

unsigned int motor_speed = 0;
unsigned int motor_direction = CW;
// ROTATION ENCODER
Rotary encoder = Rotary(OUTA_PIN, OUTB_PIN);
volatile long int encoderValue = 0;
bool start_sampling;
volatile unsigned long prev_sampling_time;
volatile unsigned long sampling_duration;
// motion
float target_duration;
int target_displacement;
bool start_moving;
unsigned long prev_moving_time;
unsigned long curr_time;
unsigned long prev_time;
float curr_position;
float prev_position;
float target_position;
float prev_target_position;
float target_velocity;
float prev_target_velocity;
float target_min_velocity;
float target_max_velocity;
float target_max_acceleration;
float target_acceleration_duration;
float target_total_time;
float min_velocity = 0.1; //0.5; // rev/s
float curr_velocity = 0; // rev/s
float min_motoron = 100; // units
float max_velocity = 1.41; //7.2; // rev/s
float max_motoron = 800; // units
float max_motoron_accel  = 800;
float position_tolerance = 0.01;
float last_motoron_control_time;
// calibration
bool start_calibrating;



float sCurveProfilPosition(float t, float start_pos, float end_pos, float duration_max){
  float c = duration_max/2;
  float position_tolerance = 0.01;
  float b = 1/c * log((1-position_tolerance)/position_tolerance);
  float pos = start_pos + (end_pos - start_pos) * (1/(1 + exp(-b * (t-c))));
  return pos;
}

float sCurveProfilVelocity(float t, float start_pos, float end_pos, float duration_max){
  float c = duration_max/2;
  float position_tolerance = 0.01;
  float b = 1/c * log((1-position_tolerance)/position_tolerance);
  float speed = (end_pos - start_pos)  * ( (b * exp(-b * (t-c))) / pow(1 + exp(-b * (t-c)), 2) );
  return speed;
}

void encode0(){
  unsigned char result = encoder.process();
  if (result == DIR_CW) encoderValue++;
   else if (result == DIR_CCW) encoderValue--;
}

void attachAllInterrupts(){
    attachInterrupt(digitalPinToInterrupt(OUTA_PIN), encode0, CHANGE);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
    attachInterrupt(digitalPinToInterrupt(OUTB_PIN), encode0, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachAllInterrupts(){
    detachInterrupt(OUTA_PIN); 
    detachInterrupt(OUTB_PIN);
}
void setup() {
  // put your setup code here, to run once:

  
  // ENCODER SETUP
  pinMode(OUTA_PIN, INPUT_PULLUP);
  pinMode(OUTB_PIN, INPUT_PULLUP);
  encoder.begin();
  start_sampling = false;
  prev_sampling_time = 0;
  sampling_duration = SAMPLING_DURATION;
  // MOTOR SETUP
  Wire.begin();

  // Reset the controller to its default settings, then disable
  // CRC.  The bytes for each of these commands are shown here
  // in case you want to implement them on your own without
  // using the library.
  mc.reinitialize();    // Bytes: 0x96 0x74
  mc.disableCrc();      // Bytes: 0x8B 0x04 0x7B 0x43
  // Clear the reset flag, which is set after the controller
  // reinitializes and counts as an error.
  mc.clearResetFlag();  // Bytes: 0xA9 0x00 0x04

  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.  You
  // can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();

  // Configure motor 1
  mc.setMaxAcceleration(1, max_motoron_accel);
  mc.setMaxDeceleration(1, max_motoron_accel);
  prev_time = 0;
  motor_speed = 0;
  motor_direction = CW;

  // CALIBRATION
  start_calibrating = false;
  // SERIAL SETUP
  Serial.begin(9600);
  Serial.println("STARTED!");
}

void loop() {
     // READ COMMAND FROM SERIAL
    if(Serial.available() > 0)
    {
        String str = Serial.readStringUntil('\n');
        Serial.println(str);
        // calibration
        String calibrationCmd = "calibration ";
        int calibrationIndex = str.indexOf(calibrationCmd);
        if(calibrationIndex != -1){
          Serial.println("Calibration...");
          start_calibrating = true;
          attachAllInterrupts();
        }


        String sampleCmd = "sample ";
        int sampleIndex = str.indexOf(sampleCmd);
        if(sampleIndex != -1){
          Serial.println("Sampling...");
          str = str.substring(sampleIndex + sampleCmd.length());
          unsigned long targetDuration = str.toInt();
          targetDuration *= 1000;
          Serial.print("targetDuration: ");
          Serial.println(targetDuration);
          sampling_duration = targetDuration;
          start_sampling = true;
          prev_sampling_time = millis();
          encoderValue = 0;
          Serial.print("encoderValue: ");
          Serial.println(encoderValue);
          attachAllInterrupts();
        }
        String speedCmd = "speed ";
        int speedIndex = str.indexOf(speedCmd);
        if(speedIndex != -1){
            Serial.println("Speed...");
            str = str.substring(speedIndex + speedCmd.length());
            int targetSpeed = str.toInt();
            Serial.print("targetSpeed: ");
            Serial.println(targetSpeed);
            motor_speed = targetSpeed;
        }
        String moveCmd = "move ";
        int moveIndex = str.indexOf(moveCmd);
        if(moveIndex != -1){
          str = str.substring(moveIndex + moveCmd.length());
          int targetDisplacement;
          float targetDuration;
          // fetch target displacement value
          int targetDisplacementIndex = str.indexOf(" ");
          if(targetDisplacementIndex != -1){
            targetDisplacement = str.substring(0, targetDisplacementIndex).toInt();
            target_displacement = targetDisplacement;
            Serial.print("targetDisplacement: ");
            Serial.println(targetDisplacement);
            // fetch target duration value
            int targetDurationIndex = str.indexOf(" ");
            if(targetDurationIndex != -1){
                targetDuration = str.substring(targetDurationIndex).toFloat();
                target_duration = targetDuration;
                Serial.print("targetDuration: ");
                Serial.println(targetDuration);
                // compute motion profil parameter
                target_max_velocity = 1.5 * target_displacement/target_duration;
                target_min_velocity = min_velocity;
                target_max_acceleration = target_max_velocity / ((1.0/3.0) * target_duration);
                target_acceleration_duration = target_max_velocity/target_max_acceleration;
                Serial.print("target_acceleration_duration: ");
                Serial.println(target_acceleration_duration);
                Serial.print("max_vel (max_accel): ");
                Serial.print(target_max_velocity);
                Serial.print("rev/s (");
                Serial.print(target_max_acceleration);
                Serial.print("rev/s²)");
                Serial.println();

            
                start_moving = true;
                last_motoron_control_time = prev_moving_time = curr_time = prev_time = millis();
                prev_position = curr_position  = 0;
                target_position = prev_target_position = 0;
                curr_velocity = target_velocity = prev_target_velocity = 0;
                target_total_time = 0;

                encoderValue = 0;
                Serial.print("encoderValue: ");
                Serial.println(encoderValue);
                attachAllInterrupts();
              }
            }
          }
    }

    if (start_calibrating){
      float calibration_min_value = 0;
      float calibration_max_value = 800;
      float calibration_step_value = 100;
      unsigned long calibration_duration_time = 10000;
      for (int motoron_value = calibration_min_value; motoron_value <= calibration_max_value; motoron_value+=calibration_step_value){

          Serial.print("motoron_value: ");
          Serial.print(motoron_value);
          Serial.print(" -> ");
          unsigned long calibration_start_time = millis();
          encoderValue = 0;
      
          while( millis() - calibration_start_time < calibration_duration_time){
            mc.setSpeed(1, motoron_value);
          }
          float nbRevolution = (encoderValue * PULSE_COUNT_CORRECTION) / float(REDUCTION_FACTOR * PULSE_PER_REVOLUTION);
          float speed = nbRevolution/(calibration_duration_time/1000.0); // rev/s
          Serial.print("speed: ");
          Serial.print(speed);
          Serial.println(" rev/s");
      }
      Serial.println("Calibration done!");
      detachAllInterrupts();
      start_calibrating = false;
      /*
      motoron_value: 0-> speed: 0.00 rev/s
      motoron_value: 100-> speed: 0.54 rev/s
      motoron_value: 200-> speed: 1.48 rev/s
      motoron_value: 300-> speed: 2.44 rev/s
      motoron_value: 400-> speed: 3.36 rev/s
      motoron_value: 500-> speed: 4.32 rev/s
      motoron_value: 600-> speed: 5.26 rev/s
      motoron_value: 700-> speed: 6.23 rev/s
      motoron_value: 800-> speed: 7.18 rev/s
      */
    }

    // MOVING RPM

    if (start_moving){

      //compute current position
      curr_position = (encoderValue * PULSE_COUNT_CORRECTION) / float(REDUCTION_FACTOR * PULSE_PER_REVOLUTION);   
      curr_position = (int)(curr_position * 100) / 100.0;

      if ((abs(target_displacement - curr_position) <= position_tolerance)){
        Serial.print("curr_position: ");
        Serial.print(curr_position);
        Serial.print(" > target_displacement: ");
        Serial.println(target_displacement);
        mc.setSpeed(1, 0);
        start_moving = false;
      } else {
        // compute delta time
        curr_time = millis();
        float delta_time = (curr_time - prev_time) / 1000.0;
        delta_time = (int)(delta_time * 100) / 100.0;

        if (delta_time >= DELTA_TIME){
          prev_time = curr_time;
          float curr_duration = (curr_time - prev_moving_time)/1000.0;
          Serial.print(curr_duration);
          Serial.print("s -> ");

          if (curr_duration > 0 && target_velocity < target_max_velocity){
              target_velocity =  target_max_velocity * min(1, curr_duration/target_acceleration_duration);
          }
          if (curr_duration > target_duration - target_acceleration_duration && target_velocity > 0){
              target_velocity =  target_max_velocity * max(0, (target_duration - curr_duration)/target_acceleration_duration);
          }

          target_position = min(target_position + target_velocity * delta_time, target_displacement);

          Serial.print(", target_total_time: ");
          Serial.print(target_total_time);
          Serial.print(", target_position: ");
          Serial.print(target_position);
          Serial.print(", target_velocity: ");
          Serial.print(target_velocity);
          Serial.print("/ curr_position: ");
          Serial.print(curr_position);
          float error_position = target_position - curr_position;
          float nb_time_since_last_motoron_control = ((curr_time - last_motoron_control_time)/1000.0)/DELTA_TIME;

          float error_velocity = abs(error_position)/delta_time * nb_time_since_last_motoron_control;
          Serial.print(", error_velocity: ");
          Serial.print(error_velocity);
          //Serial.print(", error_position: ");
          //Serial.print(error_position);
          //curr_velocity = target_velocity;
          if (error_position > 0){
           curr_velocity = error_velocity; //target_velocity; //
          } else{
            curr_velocity = curr_velocity - error_velocity; // DANGEROUS
          }
          Serial.print(", curr_velocity: ");
          Serial.print(curr_velocity);

          float motoron_value = 0;
          if (abs(curr_velocity) >= min_velocity){
            float clamped_velocity = min(max(min_velocity, abs(curr_velocity)), max_velocity); 
            Serial.print(", clamped_velocity: ");
            Serial.print(clamped_velocity);
            float coeff_velocity = (clamped_velocity - min_velocity)/(max_velocity - min_velocity);
            motoron_value = coeff_velocity * (max_motoron - min_motoron) + min_motoron;
            last_motoron_control_time = curr_time;
          }
          if (curr_velocity >= 0){
            mc.setSpeed(1, motoron_value);
          } else {
            Serial.println("BACKWARD!");
            mc.setSpeed(1, -motoron_value);
          }
          Serial.println();
          

        }
        
      }
      
    }

    // SAMPLING RPM
    if (start_sampling && (millis() - prev_sampling_time > sampling_duration) ){
          Serial.println(millis());
          Serial.println(prev_sampling_time);
          detachAllInterrupts();
          Serial.print("encoderValue: ");
          Serial.println(encoderValue);
          float nbRevolution = (encoderValue * PULSE_COUNT_CORRECTION) / float(REDUCTION_FACTOR * PULSE_PER_REVOLUTION);
          Serial.println(nbRevolution);
          start_sampling = false;
    }
    //mc.setSpeed(1, motor_speed);

}
