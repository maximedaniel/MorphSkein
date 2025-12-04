
#include <Motoron.h>
#include <ArduinoJson.h>
#include <QTRSensors.h>
#include "MotionPlanner.h"
#include "Rotary.h"
#include "MyPID.h"
#include <QuickStats.h>

#define DEBUG_CMD "debug "
#define DISPLAY_VERTICAL_RESOLUTION "display_vertical_resolution"
#define DISPLAY_HORIZONTAL_RESOLUTION "display_horizontal_resolution"
#define DISPLAY_ERROR "display_error"
#define CALIBRATION_MODE "calibration_mode"
#define STATE_CMD "state"
#define RESET_CMD "reset"
#define SPIN_CMD "spin "
#define SPIN_CMD_CMD "spinCmd "
#define TUNE_SPIN_CMD "tuneSpin "
#define HEIGHT_CMD "height "
#define HEIGHT_CMD_CMD "heightCmd "
#define TUNE_HEIGHT_CMD "tuneHeight "
#define LINE_CMD_CMD "lineCmd "
#define LINE_CMD "line "
#define LINES_CMD "lines "
#define LINES_BIS_CMD "lines_bis "

#define LINE_STATUS "status_line"
#define LINE_CURRENT "current_line"
#define LINE_TARGET "target_line"

#define HEIGHT_STATUS "status_height"
#define HEIGHT_COMMAND "command_height"
#define HEIGHT_CURRENT "current_height"
#define HEIGHT_TARGET "target_height"

#define SPIN_STATUS "status_spin"
#define SPIN_COMMAND "command_spin"
#define SPIN_CURRENT "current_spin"
#define SPIN_TARGET "target_spin"
#define TOPIC "topic"
#define PAYLOAD "payload"
#define STATUS "status"


#define SPINNER_MAX_RPM_CHANGE_PER_SECOND 15.0
#define SPIN_COMMAND_CHANGE_DELAY 100
#define THRESHOLD_TOLERANCE 100
#define HEIGHT_SENSOR_PIN A1 // A1
#define HEIGHT_MOTORON_MIN_CMD 100
#define HEIGHT_MOTORON_MAX_CMD 800
#define HEIGHT_MOTORON_STOP_CMD 0

#define HEIGHT_ROTATION_OUTA_PIN 5
#define HEIGHT_ROTATION_OUTB_PIN 4
#define HEIGHT_ROTATION_PERIMETER  38.39 // 12.22 mm * pi
#define HEIGHT_MAXIMAL_RPM_ALLOW 42.5 // 85 RPM MAX
#define HEIGHT_MAXIMAL_RPM 85 // 85 RPM MAX
#define HEIGHT_MAXIMAL_DISTANCE  300 // 12.22 mm * pi
#define HEIGHT_REDUCTION_FACTOR 379.17
#define HEIGHT_PULSE_PER_REVOLUTION 12
#define HEIGHT_PULSE_COUNT_CORRECTION 4
#define HEIGHT_DELTA_TIME 0.05
#define HEIGHT_RESET_COMMAND -200
// #define HEIGHT_QUARTER_PULSES_PER_REVOLUTION 1137.51 // 3 (nb rising front of a single channel over one revolution) * HEIGHT_GEAR_RATIO
// #define HEIGHT_HALF_PULSES_PER_REVOLUTION 2275.02 // 6 (nb rising front of a single channel over one revolution) * HEIGHT_GEAR_RATIO
// #define HEIGHT_PULSES_PER_REVOLUTION 4550.04 // 12 (nb rising front of a single channel over one revolution) * HEIGHT_GEAR_RATIO
#define HEIGHT_ENSTOP_THRESHOLD 1000
#define MAX_MOTORON_SPEED_CHANGE_FOR_HEIGHT 200
#define MAX_MOTORON_SPEED_CHANGE_FOR_LINE 200

#define HEIGHT_TO_LINE_DISTANCE_TOLERANCE 3

// LINE ENDSTOP
#define LINE0_LUMINANCE_PIN A2
#define LINE1_LUMINANCE_PIN A3
#define LINE2_LUMINANCE_PIN A4
#define LINE3_LUMINANCE_PIN A5

// LINE ENCODER
#define LINE0_OUTA_PIN 7
#define LINE0_OUTB_PIN 6
#define LINE1_OUTA_PIN 9
#define LINE1_OUTB_PIN 8
#define LINE2_OUTA_PIN 11
#define LINE2_OUTB_PIN 10
#define LINE3_OUTA_PIN 13
#define LINE3_OUTB_PIN 12

// LINE CONSTANTS
#define LINE_NB 4
#define LINE_NB_READINGS 6
#define LINE_MAXIMAL_DISTANCE 600
#define LINE_RESET_COMMAND -300
#define LINE_MOTORON_MIN_CMD 100
#define LINE_MOTORON_MAX_CMD 800
#define LINE_MOTORON_STOP_CMD 0
#define LINE_LUMINANCE_THRESHOLD 400 // 200
#define LINE_ROTATION_PERIMETER  49.98 // 15.90913 mm * pi
#define LINE_MAXIMAL_RPM_ALLOW 42.5 // 85 RPM MAX
#define LINE_MAXIMAL_RPM 85 // 85 RPM MAX
#define LINE_REDUCTION_FACTOR 379.17  //379.17 for the grey one and 248.98 for the gold one
#define LINE_PULSE_PER_REVOLUTION 12
#define LINE_PULSE_COUNT_CORRECTION 4


#define SERIAL_BAUDRATE 115200
#define PRINT_DELAY 1000

#define CW 0
#define CCW 1
#define SPINNER_OUTA_PIN 2
#define SPINNER_OUTB_PIN 3
#define SPINNER_PULSES_PER_REVOLUTION_WITHOUT_RATIO 24
#define SPINNER_PULSES_PER_REVOLUTION 116.16 // 12 (nb rising front of a single channel over one revolution) * 9.68 (gear ratio)
#define SPINNER_ENCODER_TIME 250
#define LINE_ENCODER_TIME 250
#define SPINNER_MAX_SPEED 800
#define MAX_MOTORON_SPEED_CHANGE 50
#define SPINNER_KEEP_ALIVE 10000

 
 // DEBUG
volatile bool debug = false;
// MOTOR CONTROLLER
MotoronI2C mc1(16);
MotoronI2C mc2(17);

// SPINNER ACTUATION AND SENSING
double slowKi = 0.05;
double fastKi = 0.05;
double consKp=0.0, consKi=slowKi, consKd=0.0, alpha=0.0, beta=0.0; 
int spin_command;
double current_spin;
double target_spin;
bool spin_command_enabled = false;
int target_spin_command;
volatile unsigned int spin_encoder_value_0;
volatile unsigned int spin_encoder_value_1;
unsigned long spin_prev_time;
unsigned long spin_prev_alive_time;
unsigned long  spin_command_prev_time;

// SPIN CALIBRATION
bool spin_calibration_enabled = false;
int spin_calibration_current_index = 0;
int spin_calibration_current_command;
int spin_calibration_nb_command;
int spin_calibration_min_command;
int spin_calibration_max_command;
int spin_calibration_duration;
int* spin_calibration_command_array;
double* spin_calibration_spin_array;
unsigned long spin_calibration_prev_time;
int spin_calibration_nb_sample;
int spin_calibration_sample_index;
unsigned long spin_calibration_sample_duration;
double* spin_calibration_samples;
bool waitAcceleration = false;
unsigned long spin_calibration_sample_delay;
int spin_calibration_prev_command;

// SPIN ACTUATION AND SENSING
MotionPlanner spin_motion_planner = MotionPlanner(MotionMode::VELOCITY, 0.75, 0.25, 0, 1.0, 1.0); 

// HEIGHT ROTATION ENCODER 
Rotary height_rotation_encoder = Rotary(HEIGHT_ROTATION_OUTA_PIN, HEIGHT_ROTATION_OUTB_PIN);
MotionPlanner height_motion_planner = MotionPlanner(MotionMode::POSITION, 1.0, 0.05, 0, 1.0, 1.0); 
QTRSensors height_endstop_sensor;
bool height_command_enabled = false;
int target_height_command;

// LINE ENCODERS
Rotary line_encoders [LINE_NB] = { 
  Rotary(LINE0_OUTA_PIN, LINE0_OUTB_PIN), 
  Rotary(LINE1_OUTA_PIN, LINE1_OUTB_PIN), 
  Rotary(LINE2_OUTA_PIN, LINE2_OUTB_PIN), 
  Rotary(LINE3_OUTA_PIN, LINE3_OUTB_PIN)
  };

// LINE MOTION PLANNERS
MotionPlanner line_motion_planners [LINE_NB] = {
  MotionPlanner(MotionMode::POSITION, 1.0, 0.05, 0, 1.0, 1.0),
  MotionPlanner(MotionMode::POSITION, 1.0, 0.05, 0, 1.0, 1.0),
  MotionPlanner(MotionMode::POSITION, 1.0, 0.05, 0, 1.0, 1.0),
  MotionPlanner(MotionMode::POSITION, 1.0, 0.05, 0, 1.0, 1.0),
};
// LINE ENDSTOPS
int line_endstop_pins [LINE_NB] = { LINE0_LUMINANCE_PIN, LINE1_LUMINANCE_PIN, LINE2_LUMINANCE_PIN, LINE3_LUMINANCE_PIN};
bool line_endstops [LINE_NB] = { false, false, false, false};

volatile int current_line_rotations [LINE_NB] = {0, 0, 0, 0};
volatile int current_lines [LINE_NB] = {0, 0, 0, 0};
volatile int target_lines [LINE_NB] = {0, 0, 0, 0};
unsigned long prev_line_luminance_reading_times [LINE_NB] = {0, 0, 0, 0};
unsigned long line_luminance_indexes [LINE_NB] = {0, 0, 0, 0};
int line_luminances [LINE_NB] = {0, 0, 0, 0};
int line_commands [LINE_NB] = {0, 0, 0, 0};
int target_line_commands [LINE_NB] =  {0, 0, 0, 0};
bool line_command_enableds [LINE_NB] = {false, false, false, false};
bool lineIsResets [LINE_NB] = {false, false, false, false};
bool lineToResets [LINE_NB] = {false, false, false, false};
MotoronI2C* lineToMotoronBoards [LINE_NB] = {&mc1, &mc2, &mc2, &mc2};
int lineToMotoronPorts [LINE_NB] = {3, 1, 2, 3};
float line_luminance_readings[LINE_NB][LINE_NB_READINGS] = {
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0}
};

const unsigned long LINE_LUMINANCE_DELAY = 50;
unsigned long line_prev_time;  

int stalling_count = 0;
float line0_prev_nb_revolution = 0;
float line0_revolution_speed = 0;
const double line_max_speed = LINE_MAXIMAL_RPM/60.0 * LINE_ROTATION_PERIMETER;
const double line_max_speed_allow = 20.0; // mm/s
const int line_command_range = LINE_MOTORON_MAX_CMD - LINE_MOTORON_MIN_CMD;


int prev_height_endstop_value = 0;
int current_height_endstop_value = 0;
volatile long int current_height_rotation_value = 0;
int start_height = 0;
volatile int current_height = 0;
int target_height = 0;
float target_height_duration = 0;
const double height_max_speed = HEIGHT_MAXIMAL_RPM/60.0 * HEIGHT_ROTATION_PERIMETER;
const double height_max_speed_allow = 20.0; // mm/s
const int height_command_range = HEIGHT_MOTORON_MAX_CMD - HEIGHT_MOTORON_MIN_CMD;
int heigth_command;

// GLOBAL VARIABLES
unsigned long curr_time;
unsigned long prev_print_time;


bool heightIsReset = false;
bool heightToReset = false;
bool heightAndLinesInterruptsToAttach = false;
unsigned long heightAndLinesInterruptsTime = 0;
unsigned long heightAndLinesInterruptsDelay = 250;
// COMMUNICATION
StaticJsonDocument<512> jsonDocument;
char buffer[512];
QuickStats stats;



void setupMotoron(MotoronI2C & mc)
{
  mc.reinitialize();
  mc.disableCrc();
  // Clear the reset flag, which is set after the controller
  // reinitializes and counts as an error.
  mc.clearResetFlag();

  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.  You
  // can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();
}

void encodeLine0(){
  unsigned char result = line_encoders[0].process();
  if (result == DIR_CW) current_line_rotations[0]++;
   else if (result == DIR_CCW)  current_line_rotations[0]--;
}

void encodeLine1(){
  unsigned char result = line_encoders[1].process();
  if (result == DIR_CW) current_line_rotations[1]++;
   else if (result == DIR_CCW)  current_line_rotations[1]--;
}

void encodeLine2(){
  unsigned char result = line_encoders[2].process();
  if (result == DIR_CW) current_line_rotations[2]++;
   else if (result == DIR_CCW)  current_line_rotations[2]--;
}

void encodeLine3(){
  unsigned char result = line_encoders[3].process();
  if (result == DIR_CW) current_line_rotations[3]++;
   else if (result == DIR_CCW)  current_line_rotations[3]--;
}

void encodeHeight(){
  unsigned char result = height_rotation_encoder.process();
  if (result == DIR_CW) current_height_rotation_value++;
   else if (result == DIR_CCW) current_height_rotation_value--;
}

void encodeSpinner0(){spin_encoder_value_0++;}

void encodeSpinner1(){spin_encoder_value_1++;}

void attachLineInterrupts(int index){
  switch(index){
    case 0:
     attachLine0Interrupts();
    break;
    case 1:
     attachLine1Interrupts();
    break;
    case 2:
     attachLine2Interrupts();
    break;
    case 3:
     attachLine3Interrupts();
    break;
    default:
    break;
  }
}

void detachLineInterrupts(int index){
  switch(index){
    case 0:
      detachLine0Interrupts();
    break;
    case 1:
      detachLine1Interrupts();
    break;
    case 2:
      detachLine2Interrupts();
    break;
    case 3:
      detachLine3Interrupts();
    break;
    default:
    break;
  }
}

void attachLine0Interrupts(){
    attachInterrupt(LINE0_OUTA_PIN, encodeLine0, CHANGE);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
    attachInterrupt(LINE0_OUTB_PIN, encodeLine0, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachLine0Interrupts(){
    detachInterrupt(LINE0_OUTA_PIN); 
    detachInterrupt(LINE0_OUTB_PIN); 
}

void attachLine1Interrupts(){
    attachInterrupt(LINE1_OUTA_PIN, encodeLine1, CHANGE);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
    attachInterrupt(LINE1_OUTB_PIN, encodeLine1, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachLine1Interrupts(){
    detachInterrupt(LINE1_OUTA_PIN); 
    detachInterrupt(LINE1_OUTB_PIN); 
}

void attachLine2Interrupts(){
    attachInterrupt(LINE2_OUTA_PIN, encodeLine2, CHANGE);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
    attachInterrupt(LINE2_OUTB_PIN, encodeLine2, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachLine2Interrupts(){
    detachInterrupt(LINE2_OUTA_PIN); 
    detachInterrupt(LINE2_OUTB_PIN); 
}


void attachLine3Interrupts(){
    attachInterrupt(LINE3_OUTA_PIN, encodeLine3, CHANGE);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
    attachInterrupt(LINE3_OUTB_PIN, encodeLine3, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachLine3Interrupts(){
    detachInterrupt(LINE3_OUTA_PIN); 
    detachInterrupt(LINE3_OUTB_PIN); 
}

void attachHeightInterrupts(){
    attachInterrupt(HEIGHT_ROTATION_OUTA_PIN, encodeHeight, CHANGE);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
    attachInterrupt(HEIGHT_ROTATION_OUTB_PIN, encodeHeight, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachHeightInterrupts(){
    detachInterrupt(HEIGHT_ROTATION_OUTA_PIN); 
    detachInterrupt(HEIGHT_ROTATION_OUTB_PIN); 
}

void attachSpinInterrupts(){
    attachInterrupt(SPINNER_OUTA_PIN, encodeSpinner0, RISING);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
    //attachInterrupt(SPINNER_OUTB_PIN, encodeSpinner1, RISING);  // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachSpinInterrupts(){
    detachInterrupt(SPINNER_OUTA_PIN); 
    //detachInterrupt(SPINNER_OUTB_PIN); 
}

void attachAllInterrupts(){
    attachSpinInterrupts();
    attachHeightInterrupts();
    attachLine0Interrupts();
    attachLine1Interrupts();
    attachLine2Interrupts();
    attachLine3Interrupts();
}
void detachAllInterrupts(){
    detachSpinInterrupts();
    detachHeightInterrupts();
    detachLine0Interrupts();
    detachLine1Interrupts();
    detachLine2Interrupts();
    detachLine3Interrupts();
}



void setup() {
  
    // HEIGHT ENDSTOP SETUP
  pinMode(HEIGHT_SENSOR_PIN, INPUT_PULLUP);
  height_endstop_sensor.setTypeAnalog();
  height_endstop_sensor.setSensorPins((const uint8_t[]){HEIGHT_SENSOR_PIN}, 1);

  // HEIGHT ENCODER SETUP
  height_rotation_encoder.begin();
  
  // LINE  ENCODER SETUP
  for (int i = 0; i < LINE_NB; i++){
    pinMode(line_endstop_pins[i], INPUT);
    line_encoders[i].begin();
  }

  // SPIN ENCODER SETUP
  pinMode(SPINNER_OUTA_PIN, INPUT_PULLUP);
  pinMode(SPINNER_OUTB_PIN, INPUT_PULLUP);

  // MOTOR SETUP
  Wire.begin();

  setupMotoron(mc1);
  setupMotoron(mc2);
  // Configure motor 1
  mc1.setMaxAcceleration(1, MAX_MOTORON_SPEED_CHANGE);
  mc1.setMaxDeceleration(1, MAX_MOTORON_SPEED_CHANGE);
  mc1.setMaxAcceleration(2, MAX_MOTORON_SPEED_CHANGE_FOR_HEIGHT);
  mc1.setMaxDeceleration(2, MAX_MOTORON_SPEED_CHANGE_FOR_HEIGHT);
  
  for (int i = 0; i < LINE_NB; i++){
      lineToMotoronBoards[i] -> setMaxAcceleration(lineToMotoronPorts[i], MAX_MOTORON_SPEED_CHANGE_FOR_LINE);
      lineToMotoronBoards[i] -> setMaxDeceleration(lineToMotoronPorts[i], MAX_MOTORON_SPEED_CHANGE_FOR_LINE);
  }

  spin_prev_time =  spin_prev_alive_time = spin_command_prev_time = millis();
  spin_encoder_value_0 = 0;
  spin_encoder_value_1 = 0;
  spin_command = 0;
  current_spin = 0;
  target_spin = 0;

  // HEIGHT CONTROL
  target_height = 0;
  current_height = 0;
  heigth_command = 0; 
  target_height_command = 0;
  height_command_enabled = true;

  
  heightIsReset = false;
  heightToReset = false;

  // SETUP INTERRUPT
  attachAllInterrupts();

  // SERIAL SETUP
  Serial.begin(SERIAL_BAUDRATE);
  delay(100);
  Serial1.begin(SERIAL_BAUDRATE, SERIAL_8N1);
  delay(100);
}

void leftShift(float arr[], int size, int shift){
  for (int i = 0; i < size - shift; i++){
    arr[i] = arr[i + shift];
  }
   for (int i = size - shift; i < size; i++){
    arr[i] = 0;
  }
}
void prettyPrintArray(float arr[], int size){
  Serial.print("[");
  for (int i = 0; i < size - 1; i++){
    Serial.print(arr[i]);
    Serial.print(", ");
  }
    Serial.print(arr[size-1]);
  Serial.print("]");
  Serial.println();
}

int nbGreater(float arr[], int size, float threshold){
  int count = 0;

  for (int i = 0; i < size; i++){
    if(arr[i] > threshold) count++;
  }
  return count;
}

bool updateLinePosition(unsigned long current_time, int line_index){

    // bool endstop = !analogRead(LINE0_LUMINANCE_PIN);
    // if (endstop && line0_command < 0) { //&& line0_luminance >= LINE0_LUMINANCE_THRESHOLD){
    //   Serial.println("LUMINANCE ENDSTOP!");
    //   current_line0_rotation = 0;
    //   current_line0 = 0;
    //   line0_command = 0;
    // } else {
    //   float line0_nb_revolution = (current_line0_rotation * LINE0_PULSE_COUNT_CORRECTION) / float(LINE0_REDUCTION_FACTOR * LINE0_PULSE_PER_REVOLUTION);
    //   current_line0 = line0_nb_revolution * LINE0_ROTATION_PERIMETER;
    // }
    // bool endstop = false;
    // float line0_nb_revolution = (current_line0_rotation * LINE0_PULSE_COUNT_CORRECTION) / float(LINE0_REDUCTION_FACTOR * LINE0_PULSE_PER_REVOLUTION);
    // current_line0 = line0_nb_revolution * LINE0_ROTATION_PERIMETER;
    // unsigned long line_delta_time = current_time - line_prev_time;
    // if ( line_delta_time > LINE_ENCODER_TIME){
    //   //float line0_target_velocity = 26.88 // 26 -> 200, 43 -> 300, 61 -> 400
    //   float line0_delta_nb_revolution = line0_nb_revolution - line0_prev_nb_revolution;
    //   line0_revolution_speed = line0_delta_nb_revolution * 60000 / double(line_delta_time);
    //   Serial.print("line0_revolution_speed:");
    //   Serial.println(line0_revolution_speed);
    //   line0_prev_nb_revolution = line0_nb_revolution;
    //   line_prev_time = current_time;
    //   if(abs(line0_revolution_speed) < 0.1 && line0_command < 0 ){
    //     stalling_count++;
    //     if(stalling_count > 3 ){
    //       Serial.println("MOTOR STALLING!");
    //       stalling_count = 0;
    //       current_line0_rotation = 0;
    //       current_line0 = 0;
    //       line0_command = 0;
    //       endstop = true;
    //     }
    //   } else {
    //     stalling_count = 0;
    //   }
    // }

  bool endstop = false;
  int line_next_luminance = analogRead(line_endstop_pins[line_index]);
  // if(line0_next_luminance != line0_luminance){
  //   line0_luminance = line0_next_luminance;
  //   //Serial.println(line0_luminance);
  // }
  if(current_time - prev_line_luminance_reading_times[line_index] > LINE_LUMINANCE_DELAY){
    leftShift(line_luminance_readings[line_index], LINE_NB_READINGS, 1);
    line_luminance_indexes[line_index] ++;
    prev_line_luminance_reading_times[line_index] = current_time;
  }
  
  line_luminance_readings[line_index][LINE_NB_READINGS - 1] = (float)line_next_luminance;
  
    if (line_luminance_indexes[line_index] > LINE_NB_READINGS - 1){
      //prettyPrintArray(line0_luminance_readings, line0_luminance_nb_readings);
      line_luminance_indexes[line_index] = 0;
    }
  line_luminances[line_index] = line_luminance_readings[line_index][LINE_NB_READINGS-1]; // stats.average(line_luminance_readings[line_index], LINE_NB_READINGS);
  if (line_commands[line_index] < 0 && line_luminances[line_index] >= LINE_LUMINANCE_THRESHOLD && !lineIsResets[line_index]){
    Serial.print("LUMINANCE");
    Serial.print(line_index);
    Serial.println(" ENDSTOP!");
    current_line_rotations[line_index] = 0;
    current_lines[line_index] = 0;
    line_commands[line_index] = 0;
    endstop = true;
  }
   else {
    // (8376 * 4) / (379.17 * 12) 
    float line_nb_revolution = current_line_rotations[line_index] / float(LINE_REDUCTION_FACTOR * LINE_PULSE_PER_REVOLUTION/LINE_PULSE_COUNT_CORRECTION);
    current_lines[line_index] = line_nb_revolution * LINE_ROTATION_PERIMETER;

    //Serial.print("current_lines[3]:");
    //Serial.println(current_lines[3]);
  }
  //int nbGreaterCount = nbGreater(line0_luminance_readings, line0_luminance_nb_readings, LINE0_LUMINANCE_THRESHOLD);

  // if (line0_command < 0 && nbGreaterCount > 2) { //&& line0_luminance >= LINE0_LUMINANCE_THRESHOLD){
  //   Serial.println("LUMINANCE ENDSTOP!");
  //   current_line0_rotation = 0;
  //   current_line0 = 0;
  //   line0_command = 0;
  //   endstop = true;
  // } else {
  // }
  return endstop;
}

bool updateSpinVelocity(unsigned long current_time){
  bool new_sample = false;
  unsigned long spin_delta_time = current_time - spin_prev_time;
  if ( spin_delta_time > SPINNER_ENCODER_TIME){
     // uses both edges
      // float spinner_target_nb_pulse = int(target_spin * 2 * SPINNER_PULSES_PER_REVOLUTION / 60000 * spin_delta_time);
      // int spinner_nb_pulse = spin_encoder_value_0 + spin_encoder_value_1;
      // float spinner_nb_revolution = spinner_nb_pulse /  (2 * SPINNER_PULSES_PER_REVOLUTION);
      // current_spin = spinner_nb_revolution  * 60000 / double(spin_delta_time);
      // spin_encoder_value_0 = spin_encoder_value_1 =  0;
      // using single edge
      float spinner_target_nb_pulse = int(target_spin * SPINNER_PULSES_PER_REVOLUTION / 60000 * spin_delta_time);
      int spinner_nb_pulse = spin_encoder_value_0;
      float spinner_nb_revolution = spinner_nb_pulse / SPINNER_PULSES_PER_REVOLUTION;
      current_spin = spinner_nb_revolution  * 60000 / double(spin_delta_time);
      spin_encoder_value_0 =  0;
      spin_prev_time = current_time;
      new_sample  = true;   
  }
  return new_sample;
}

bool updateHeightPosition(unsigned long current_time){
  bool endstop = false;
  uint16_t height_endstop_values[1];
  height_endstop_sensor.read(height_endstop_values);
  current_height_endstop_value = height_endstop_values[0];
  if (heigth_command < 0 && current_height_endstop_value < HEIGHT_ENSTOP_THRESHOLD){
    Serial.print(current_height_endstop_value);
    Serial.println(": HEIGHT_ENSTOP_THRESHOLD!");
    current_height_rotation_value = 0;
    current_height = 0;
    heigth_command = 0;
    endstop = true;
  } else {
    // (7975 * 4) / (379.17 * 12)
    float height_nb_revolution = current_height_rotation_value / float(HEIGHT_REDUCTION_FACTOR * HEIGHT_PULSE_PER_REVOLUTION/HEIGHT_PULSE_COUNT_CORRECTION);
    current_height = height_nb_revolution * HEIGHT_ROTATION_PERIMETER;
  }
  return endstop;
}

void calibrateSpin(int nbCommand, int minCommand, int maxCommand, int nbSample, unsigned long sampleDuration){
  spin_calibration_enabled = true;
  spin_calibration_current_index = 0;
  spin_calibration_prev_command = 0;
  spin_calibration_current_command = minCommand;
  spin_calibration_nb_command = nbCommand;
  spin_calibration_min_command = minCommand;
  spin_calibration_max_command = maxCommand;
  spin_calibration_nb_sample = nbSample;
  spin_calibration_sample_index = 0;
  spin_calibration_sample_duration = sampleDuration;
  spin_calibration_samples = new double[spin_calibration_nb_sample];
  waitAcceleration = true;
  for(int i = 0; i < spin_calibration_nb_sample; i++) spin_calibration_samples[i] = 0.0;

  spin_calibration_command_array = new int[spin_calibration_nb_command];
  Serial.print("calibration points: [");
  for(int i = 0; i < spin_calibration_nb_command; i++) {
    spin_calibration_command_array[i] =  ((double)i)/(spin_calibration_nb_command - 1) * (maxCommand - minCommand) + minCommand;
    Serial.print(spin_calibration_command_array[i]);
    Serial.print(", ");
  }
  Serial.println("]");

  spin_calibration_spin_array = new double[spin_calibration_nb_command];
  for(int i = 0; i < spin_calibration_nb_command; i++) spin_calibration_spin_array[i] = 0.0;
  
  spin_calibration_current_command = spin_calibration_command_array[spin_calibration_current_index];

  spin_calibration_prev_time = millis();
}

double average(double * array, int len)  // assuming array is int.
{
  double sum = 0.0 ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((double) sum) / len ;  // average will be fractional, so float may be appropriate.
}

void handleResetCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String resetCmd = RESET_CMD;
    int resetCmdIndex = str.indexOf(resetCmd);
    if(resetCmdIndex != -1){
      str = str.substring(resetCmdIndex + resetCmd.length());
      serial.print("reset!");
      height_motion_planner.stop();
      heightIsReset = false;
      heightToReset = true;
      for (int i = 0; i < LINE_NB; i++){
        line_motion_planners[i].stop();
        lineIsResets[i] = false;
        lineToResets[i] = false;
      }
      detachHeightInterrupts();
      detachLine0Interrupts();
      detachLine1Interrupts();
      detachLine2Interrupts();
      detachLine3Interrupts();
    }
}
void handleLineCmdCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String lineCmdCmd = LINE_CMD_CMD;
    int lineCmdIndex = str.indexOf(lineCmdCmd);
    if(lineCmdIndex != -1){
        str = str.substring(lineCmdIndex + lineCmdCmd.length());
        // fetch line index 
        int lineIndexIndex = str.indexOf(" ");
        if(lineIndexIndex != -1){
          int line_index  = str.substring(0, lineIndexIndex).toInt();
          serial.print("line_index: ");
          serial.println(line_index);
          // fetch line command 
          int lineCommandIndex = str.indexOf(" ");
          if(lineCommandIndex != -1){
              target_line_commands[line_index] = str.substring(lineCommandIndex).toInt();
              line_command_enableds[line_index] = true;
              serial.print("target_line_command: ");
              serial.println(target_line_commands[line_index]);
          }
        }
    }
}

void handleLineCmd(String message, Stream &serial){
    String str =  message.substring(0);
    //Serial.println(str);
    String lineCmd = LINE_CMD;
    int lineIndex = str.indexOf(lineCmd);
    if(lineIndex != -1){
      str = str.substring(lineIndex + lineCmd.length());
      // fetch line index 
      int lineIndexIndex = str.indexOf(" ");
      if(lineIndexIndex != -1){
        int line_index  = str.substring(0, lineIndexIndex).toInt();
        serial.print("line_index: ");
        serial.println(line_index);
        str = str.substring(lineIndexIndex + 1);
        // fetch target displacement value
        int targetLineIndex = str.indexOf(" ");
        if(targetLineIndex != -1){
          target_lines[line_index] = constrain(str.substring(0, targetLineIndex).toInt(), 0, LINE_MAXIMAL_DISTANCE);
          serial.print("target_line: ");
          serial.println(target_lines[line_index]);
          // fetch target duration value
          int targetLineDurationIndex = str.indexOf(" ");
          if(targetLineDurationIndex != -1){
              float target_line_duration = str.substring(targetLineDurationIndex).toFloat();
              float min_line_duration = abs(target_lines[line_index] - current_lines[line_index])/line_max_speed_allow;
              if(target_line_duration < min_line_duration) target_line_duration = min_line_duration;
              line_command_enableds[line_index] = false;
              serial.print("target_line_duration: ");
              serial.println(target_line_duration);
              //attachLine0Interrupts();
              //delay(20);
              line_motion_planners[line_index].set(current_lines[line_index], target_lines[line_index], target_line_duration, true, true);
            }
          }
        }
      }
}

void handleLinesCmd(String message, Stream &serial){
    String str =  message.substring(0);
    //Serial.println(str);
    String linesCmd = LINES_CMD;
    int linesIndex = str.indexOf(linesCmd);
    if(linesIndex != -1){
      str = str.substring(linesIndex + linesCmd.length());
        // fetch target displacement value
        int targetLinesIndex = str.indexOf(" ");
        if(targetLinesIndex != -1){
          int target_line = constrain(str.substring(0, targetLinesIndex).toInt(), 0, LINE_MAXIMAL_DISTANCE);
          serial.print("target_lines: ");
          serial.println(target_line);
          // fetch target duration value
          int targetLinesDurationIndex = str.indexOf(" ");
          if(targetLinesDurationIndex != -1){
              float target_line_duration = str.substring(targetLinesDurationIndex).toFloat();
              serial.print("target_line_durations: ");
              serial.println(target_line_duration);
              for (int i = 0; i < LINE_NB; i++){
                target_lines[i] = target_line;
                float min_line_duration = abs(target_lines[i] - current_lines[i])/line_max_speed_allow;
                if(target_line_duration > min_line_duration) min_line_duration = target_line_duration;
                 line_command_enableds[i] = false;
                 line_motion_planners[i].set(current_lines[i], target_lines[i], min_line_duration, true, true);
              }

              //attachLine0Interrupts();
              //delay(20);
            }
          }
      }
}

void handleHeightCmdCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String heightCmdCmd =  HEIGHT_CMD_CMD;
      int heightCmdIndex = str.indexOf(heightCmdCmd);
        if(heightCmdIndex != -1){
          str = str.substring(heightCmdIndex + heightCmdCmd.length());
          attachHeightInterrupts();
          delay(20);
          height_command_enabled = true;
          target_height_command  = str.toInt();
          height_motion_planner.stop();
          serial.print("target_height_command:");
          serial.println(heigth_command);
        }

}

void handleHeightCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String heightCmd = HEIGHT_CMD;
    int heightIndex = str.indexOf(heightCmd);
    if(heightIndex != -1){
      str = str.substring(heightIndex + heightCmd.length());
      // fetch target displacement value
      int targetHeightIndex = str.indexOf(" ");
      if(targetHeightIndex != -1){
        target_height = str.substring(0, targetHeightIndex).toInt();
        target_height = constrain(target_height, 0, HEIGHT_MAXIMAL_DISTANCE);
        serial.print("target_height: ");
        serial.println(target_height);
        // fetch target duration value
        int targetHeightDurationIndex = str.indexOf(" ");
        if(targetHeightDurationIndex != -1){
            target_height_duration = str.substring(targetHeightDurationIndex).toFloat();
            float min_height_duration = abs(target_height - current_height)/height_max_speed_allow;
            if(target_height_duration < min_height_duration) target_height_duration = min_height_duration;
            height_command_enabled = false;
            serial.print("target_height_duration: ");
            serial.println(target_height_duration);
            attachHeightInterrupts();
            delay(20);
            height_motion_planner.set(current_height, target_height, target_height_duration, true, true);
          }
        }
    }
}


void handleTuneHeightCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String heightTuningCmd =  TUNE_HEIGHT_CMD;
    int heightTuningIndex = str.indexOf(heightTuningCmd);
    if(heightTuningIndex != -1){
      str = str.substring(heightTuningIndex + heightTuningCmd.length());
      double Kp;
      double Ki;
      double Kd;
      double alpha;
      double beta;
      int KpIndex = str.indexOf(" ");
      if(KpIndex != -1){
          Kp = str.substring(0, KpIndex).toDouble();
          serial.print("Kp: ");
          serial.println(Kp);
          str = str.substring(KpIndex + 1);
          int KiIndex = str.indexOf(" ");
          if(KiIndex != -1){
            Ki = str.substring(0, KiIndex).toDouble();
            serial.print("Ki: ");
            serial.println(Ki);
            str = str.substring(KiIndex + 1);
            int KdIndex = str.indexOf(" ");
            if(KdIndex != -1){
              Kd = str.substring(0, KdIndex).toDouble();
              serial.print("Kd: ");
              serial.println(Kd);
              str = str.substring(KdIndex + 1);
              int alphaIndex = str.indexOf(" ");
              if(alphaIndex != -1){
                alpha = str.substring(0, alphaIndex).toDouble();
                serial.print("alpha: ");
                serial.println(alpha);
                beta = str.substring(alphaIndex + 1).toDouble();
                serial.print("beta: ");
                serial.println(beta);
                height_motion_planner.tune(Kp, Ki, Kd, alpha, beta);
              }
            }
          }
      }
    }
}

void handleSpinCmdCmd(String message, Stream &serial){
  String str =  message.substring(0);
  String spinCmdCmd =  SPIN_CMD_CMD;
  int spinCmdIndex = str.indexOf(spinCmdCmd);
  if(spinCmdIndex != -1){
      str = str.substring(spinCmdIndex + spinCmdCmd.length());
      spin_command_enabled = true;
      target_spin_command = str.toInt();
      serial.print("target_spin_command:");
      serial.println(target_spin_command);
      spin_prev_alive_time = curr_time;
  }
}

void handleSpinCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String spinCmd = SPIN_CMD;
    int spinIndex = str.indexOf(spinCmd);
    if(spinIndex != -1){
        str = str.substring(spinIndex + spinCmd.length());
        double spinner_new_target_rpm = str.toDouble();
        spin_command_enabled = false;
        if(spinner_new_target_rpm != target_spin){
          target_spin = spinner_new_target_rpm;
          double rpm_change_total = abs(target_spin - current_spin);
          double rpm_change_duration = rpm_change_total/SPINNER_MAX_RPM_CHANGE_PER_SECOND; // time in s
          spin_motion_planner.set(current_spin, target_spin, rpm_change_duration, true, false);
          serial.print("target_spin: ");
          serial.println(target_spin);
        }
        spin_prev_alive_time = curr_time;
    }
}

void handleTuneSpinCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String spinTuningCmd =  TUNE_SPIN_CMD;
    int spinTuningIndex = str.indexOf(spinTuningCmd);
    if(spinTuningIndex != -1){
      str = str.substring(spinTuningIndex + spinTuningCmd.length());
      double Kp;
      double Ki;
      double Kd;
      double alpha;
      double beta;
      int KpIndex = str.indexOf(" ");
      if(KpIndex != -1){
          Kp = str.substring(0, KpIndex).toDouble();
          serial.print("Kp: ");
          serial.println(Kp);
          str = str.substring(KpIndex + 1);
          int KiIndex = str.indexOf(" ");
          if(KiIndex != -1){
            Ki = str.substring(0, KiIndex).toDouble();
            serial.print("Ki: ");
            serial.println(Ki);
            str = str.substring(KiIndex + 1);
            int KdIndex = str.indexOf(" ");
            if(KdIndex != -1){
              Kd = str.substring(0, KdIndex).toDouble();
              serial.print("Kd: ");
              serial.println(Kd);
              str = str.substring(KdIndex + 1);
              int alphaIndex = str.indexOf(" ");
              if(alphaIndex != -1){
                alpha = str.substring(0, alphaIndex).toDouble();
                serial.print("alpha: ");
                serial.println(alpha);
                beta = str.substring(alphaIndex + 1).toDouble();
                serial.print("beta: ");
                serial.println(beta);
                spin_motion_planner.tune(Kp, Ki, Kd, alpha, beta);

              }
            }
          }
      }
    }
}

      
     
void handleStateCmd(String message, Stream &serial){
  String str =  message.substring(0);
  String stateCmd = STATE_CMD;
  int stateIndex = str.indexOf(stateCmd);
  if(stateIndex != -1){
    jsonDocument.clear();
    jsonDocument[TOPIC] = STATE_CMD;
    JsonObject payloadDocument = jsonDocument.createNestedObject(PAYLOAD);
    JsonArray line_status_array = payloadDocument.createNestedArray(LINE_STATUS);
    JsonArray line_current_array = payloadDocument.createNestedArray(LINE_CURRENT);
    JsonArray line_target_array = payloadDocument.createNestedArray(LINE_TARGET);
    for(int i = 0; i < LINE_NB; i++){
        line_status_array.add(line_motion_planners[i].status());
        line_current_array.add(current_lines[i]);
        line_target_array.add(target_lines[i]);
    }
    payloadDocument[HEIGHT_STATUS] = height_motion_planner.status();
    payloadDocument[HEIGHT_COMMAND] = heigth_command;
    payloadDocument[HEIGHT_CURRENT] = current_height;
    payloadDocument[HEIGHT_TARGET] = target_height;
    payloadDocument[SPIN_STATUS] = spin_motion_planner.status();
    payloadDocument[SPIN_COMMAND] = spin_command;
    payloadDocument[SPIN_CURRENT] = current_spin;
    payloadDocument[SPIN_TARGET] = target_spin;
    payloadDocument[DISPLAY_VERTICAL_RESOLUTION] = 0;
    payloadDocument[DISPLAY_HORIZONTAL_RESOLUTION] = 0;
    payloadDocument[DISPLAY_ERROR] = 0;
    serializeJson(jsonDocument, buffer);
    Serial1.flush();
    Serial1.println(buffer);
  }

}   

void handleDebugCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String debugCmd = DEBUG_CMD;
    int debugIndex = str.indexOf(debugCmd);
    if(debugIndex != -1){
        str = str.substring(debugIndex + debugCmd.length());
        debug = str.toInt();
    }
}

void loop() {
  
    // Get current time
    curr_time = millis();
    // check if we need to attach interrupts
    if (heightAndLinesInterruptsToAttach && curr_time - heightAndLinesInterruptsTime > heightAndLinesInterruptsDelay){
      attachHeightInterrupts();
      attachLine0Interrupts();
      attachLine1Interrupts();
      attachLine2Interrupts();
      attachLine3Interrupts();
      heightAndLinesInterruptsToAttach = false;
    }
    // control the lines
    for (int i = 0; i < LINE_NB; i++){
      line_endstops[i] = updateLinePosition(curr_time, i);
      if(lineIsResets[i]){
        if (line_command_enableds[i]){
            if((line_endstops[i] && target_line_commands[i] < 0)){
              Serial.print("LINE_ENDSTOP");
              Serial.print(i);
              Serial.println(" REACHED!");
              target_line_commands[i] = 0;
              line_command_enableds[i] = false;
            }
            else if((target_line_commands[i] < 0 && current_lines[i]  - current_height <= HEIGHT_TO_LINE_DISTANCE_TOLERANCE)){
              Serial.print("HEIGHT_TO_LINE_TOLERANCE");
              Serial.print(i);
              Serial.println(" REACHED!");
              target_line_commands[i] = 0;
              line_command_enableds[i] = false;
            }
            else if((target_line_commands[i] > 0 && current_lines[i] >= LINE_MAXIMAL_DISTANCE)){
              Serial.print("LINE_MAXIMAL_DISTANCE");
              Serial.print(i);
              Serial.println(" REACHED!");
              target_line_commands[i] = 0;
              line_command_enableds[i] = false;
            }
            line_commands[i] = target_line_commands[i];
            lineToMotoronBoards[i] -> setSpeed(lineToMotoronPorts[i], line_commands[i]);
        }
        else {
              if (line_motion_planners[i].isActive()){
                target_line_commands[i] = 0;
                float line_speed = line_motion_planners[i].process(current_lines[i], false); 
                if (line_speed != 0){
                  target_line_commands[i] = line_motion_planners[i].sign(line_speed) * LINE_MOTORON_MIN_CMD + line_speed/line_max_speed * line_command_range;
                }
                // if((line0_endstop && target_line_commands[i] < 0)){
                //   Serial.println("LINE_ENDSTOP REACHED!");
                //   target_line_commands[i] = 0;
                //   line0_motion_planner.stop();
                // }
                else if((target_line_commands[i] < 0 && current_lines[i]  - current_height <= HEIGHT_TO_LINE_DISTANCE_TOLERANCE)){
                  Serial.print("HEIGHT_TO_LINE_TOLERANCE");
                  Serial.print(i);
                  Serial.println(" REACHED!");
                  target_line_commands[i] = 0; 
                  line_motion_planners[i].stop();
                }
                else if((target_line_commands[i] > 0 && current_lines[i] >= LINE_MAXIMAL_DISTANCE)){
                  Serial.print("LINE_MAXIMAL_DISTANCE");
                  Serial.print(i);
                  Serial.println(" REACHED!");
                  target_line_commands[i] = 0;
                  line_motion_planners[i].stop();
                }
                line_commands[i] = target_line_commands[i];
                lineToMotoronBoards[i] -> setSpeed(lineToMotoronPorts[i], line_commands[i]);
            }
        }
      } else {
          if (line_endstops[i]){
            Serial.print("LINE");
            Serial.print(i);
            Serial.println(" IS RESET!");
            lineIsResets[i] = true;
            lineToResets[i] = false;
            // notice the ESP32 that the display is reset
            bool allLinesAreResets = true;
            for (int j = 0; j < LINE_NB; j++){
                if (lineIsResets[j] != true || lineToResets[j] != false) {
                  allLinesAreResets = false; 
                  break;
                }
            }
            if (allLinesAreResets){
              jsonDocument.clear();
              jsonDocument[TOPIC] = RESET_CMD;
              serializeJson(jsonDocument, buffer);
              Serial1.flush();
              Serial1.println(buffer);
              //serializeJson(jsonDocument, Serial1);
              Serial.println("RESET RESPONSE!");
              heightAndLinesInterruptsToAttach = true;
              heightAndLinesInterruptsTime = curr_time;
            }
          }
          line_commands[i] = 0;
          if (lineToResets[i]){
              line_commands[i] = LINE_RESET_COMMAND;
          }
          
          lineToMotoronBoards[i] -> setSpeed(lineToMotoronPorts[i], line_commands[i]);
        }
    }
    
    
    // Control the height
    bool height_endstop = updateHeightPosition(curr_time);

    // check if height is reset
    if(heightIsReset){
      // check if direct command is enabled
      if (height_command_enabled){
        if(height_endstop && target_height_command < 0){
          target_height_command = 0;
          height_command_enabled = false;
        }
        heigth_command = target_height_command;
        mc1.setSpeed(2, -heigth_command);
      } 
      else {
            if (height_motion_planner.isActive()){
              float height_speed = height_motion_planner.process(current_height, false); 
              heigth_command = 0;
              if (height_speed != 0){
                heigth_command = height_motion_planner.sign(height_speed) * HEIGHT_MOTORON_MIN_CMD + height_speed/height_max_speed * height_command_range;
              }
            }
            mc1.setSpeed(2, -heigth_command);
      }
    } else {
      if (height_endstop){
        Serial.println("HEIGHT ENDSTOP!");
        heightIsReset = true;
        heightToReset = false;
        for (int i = 0; i < LINE_NB; i++) lineToResets[i] = true; 
      }
      if (heightToReset){
          heigth_command = HEIGHT_RESET_COMMAND;
          mc1.setSpeed(2, -heigth_command);
      }
    }

    // Control the spin
    bool new_spin_velocity = updateSpinVelocity(curr_time);
    if (spin_calibration_enabled){
        // if first sample wait for x seconds 
        if(waitAcceleration){
            spin_calibration_current_command = spin_calibration_command_array[spin_calibration_current_index];
            if(spin_calibration_prev_command != spin_calibration_current_command){
                spin_calibration_sample_delay =  (double)((spin_calibration_current_command-spin_calibration_prev_command))/MAX_MOTORON_SPEED_CHANGE * 1000;
                Serial.print("waiting ");
                Serial.print(spin_calibration_sample_delay);
                Serial.print("ms...");
                Serial.print(" cmd:");
                Serial.print(spin_calibration_command_array[spin_calibration_current_index]);
                spin_calibration_prev_command =  spin_calibration_current_command;
            }
            if(curr_time - spin_calibration_prev_time > spin_calibration_sample_delay){
              Serial.print(", samples: [");
              spin_calibration_prev_time = curr_time;
              waitAcceleration = false;
            }

        }
        else {
          if(curr_time - spin_calibration_prev_time > spin_calibration_sample_duration){
              //f (spin_calibration_current_command != spin_calibration_command_array[spin_calibration_current_index]){
              //}
              spin_calibration_current_command = spin_calibration_command_array[spin_calibration_current_index];
              spin_calibration_samples[spin_calibration_sample_index] = current_spin;
              Serial.print(current_spin);
              Serial.print(", ");
              spin_calibration_sample_index++;
              if (spin_calibration_sample_index > spin_calibration_nb_sample - 1){
                // process samples
                double average_spin = average(spin_calibration_samples, spin_calibration_nb_sample);
                Serial.print("], avg spin:");
                Serial.println(average_spin);
                spin_calibration_sample_index = 0;
                waitAcceleration = true;
                spin_calibration_spin_array[spin_calibration_current_index] = current_spin;
                spin_calibration_current_index++;

                // check end of calibrration
                if (spin_calibration_current_index  > spin_calibration_nb_command - 1){
                    spin_calibration_current_index = 0;
                    spin_calibration_current_command = 0;
                    spin_calibration_enabled = false;
                }
              }
              
              spin_calibration_prev_time = millis();

          }
        }
        mc1.setSpeed(1, -spin_calibration_current_command);
    }
    
    else if (spin_command_enabled){
      if (curr_time - spin_command_prev_time > SPIN_COMMAND_CHANGE_DELAY){
        int spin_command_diff = target_spin_command - spin_command;
        if (spin_command_diff > 0) spin_command += 1;
        if (spin_command_diff < 0) spin_command -= 1;
        spin_command_prev_time = curr_time;
      }
      mc1.setSpeed(1, -spin_command);
      
    } else {
      if (new_spin_velocity){
          if (spin_motion_planner.isActive()){
            double spin_speed = spin_motion_planner.process(current_spin, false); 
            spin_command = spin_speed;
            mc1.setSpeed(1, -spin_command);
          }
      }
    }


    if(Serial.available() > 0){
      String str = Serial.readStringUntil('\n');
      handleDebugCmd(str, Serial);
      handleResetCmd(str, Serial);
      handleLineCmd(str, Serial);
      //handleLineCmdCmd(str, Serial);
      handleLinesCmd(str, Serial);
      handleHeightCmdCmd(str, Serial);
      handleHeightCmd(str, Serial);
      handleTuneHeightCmd(str, Serial);
      handleSpinCmdCmd(str, Serial);
      handleSpinCmd(str, Serial);
      handleTuneSpinCmd(str, Serial);
    }

    if(Serial1.available() > 0){
       //Serial.print("receiving from Serial1: ");
       String str = Serial1.readStringUntil('\n');
       handleDebugCmd(str, Serial);
       handleResetCmd(str, Serial);
       handleSpinCmdCmd(str, Serial);
       handleSpinCmd(str, Serial);
       handleHeightCmd(str, Serial);
       handleLineCmd(str, Serial);
       handleLinesCmd(str, Serial);
       handleStateCmd(str, Serial);
    }

    if (debug && !spin_calibration_enabled && curr_time - prev_print_time > PRINT_DELAY){
      
        Serial.print("HEIGHT: ");
        Serial.print(current_height);
        Serial.print("(");
        Serial.print(current_height_endstop_value);
        Serial.print(")");
        Serial.print(" -> ");
        Serial.print(target_height);
        Serial.print("(");
        Serial.print(heigth_command);
        Serial.print(")");
        Serial.print(" | SPIN: ");
        Serial.print(current_spin);
        Serial.print("(");
        Serial.print(spin_encoder_value_0 + spin_encoder_value_1);
        Serial.print(")");
        Serial.print(" -> ");
        Serial.print(target_spin);
        Serial.print("(");
        Serial.print(spin_command);
        Serial.print(")");

        Serial.print(" | LINES: [   ");
        for(int i = 0; i < LINE_NB; i++){
        Serial.print(current_lines[i]);
        Serial.print("(");
        Serial.print(line_luminances[i]);
        Serial.print(", ");
        Serial.print(current_line_rotations[i]);
        Serial.print(")");
        Serial.print(" -> ");
        Serial.print(target_lines[i]);
        Serial.print("(");
        Serial.print(line_commands[i]);
        Serial.print(")   ");
        }
        Serial.print("]");
        Serial.println();
        // if (current_lines[3] < 0){
        //   Serial.println(current_line_rotations[3] * LINE_PULSE_COUNT_CORRECTION);
        //   Serial.println(float(LINE_REDUCTION_FACTOR * LINE_PULSE_PER_REVOLUTION));
        //   Serial.println( (current_line_rotations[3] * LINE_PULSE_COUNT_CORRECTION) / float(LINE_REDUCTION_FACTOR * LINE_PULSE_PER_REVOLUTION));
        //   Serial.println((current_line_rotations[3] * LINE_PULSE_COUNT_CORRECTION) / float(LINE_REDUCTION_FACTOR * LINE_PULSE_PER_REVOLUTION) * LINE_ROTATION_PERIMETER);
        //   Serial.println(current_lines[3]);
        // }
        prev_print_time = curr_time;
    }

    if (curr_time - spin_prev_alive_time > SPINNER_KEEP_ALIVE){
        if (spin_command_enabled){
          if (target_spin_command != 0){
            target_spin_command = 0;
          }
        }
        else {
          if (target_spin != 0){
            target_spin = 0;
            double rpm_change_total = abs(target_spin - current_spin);
            double rpm_change_duration = rpm_change_total/SPINNER_MAX_RPM_CHANGE_PER_SECOND; // time in s
            spin_motion_planner.set(current_spin, target_spin, rpm_change_duration, false, false);
          }
        }
    }
}
