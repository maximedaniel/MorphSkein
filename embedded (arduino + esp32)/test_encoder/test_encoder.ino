

#include "Rotary.h"

#define OUTA_PIN 0
#define OUTB_PIN 1
#define REDUCTION_FACTOR 75
#define PULSE_PER_REVOLUTION 12
#define PULSE_COUNT_CORRECTION 4
#define SAMPLING_DURATION 10000

Rotary r = Rotary(OUTA_PIN, OUTB_PIN);

volatile long int encoderValue;
bool start_sampling;
volatile unsigned long prev_sampling_time;
volatile unsigned long sampling_duration;
volatile unsigned long target_displacement;
volatile unsigned long target_duration;

// INTERRUPTION METHODS

void rotate0(){
  unsigned char result = r.process();
  if (result == DIR_CW) {
    encoderValue++;
  } else if (result == DIR_CCW) {
    encoderValue--;
  } 
}

void attachAllInterrupts(){
    attachInterrupt(digitalPinToInterrupt(OUTA_PIN), rotate0, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
    attachInterrupt(digitalPinToInterrupt(OUTB_PIN), rotate0, CHANGE); // WARNING remove digitalPinToInterrupt for certain board or will not work.
}

void detachAllInterrupts(){
    detachInterrupt(OUTA_PIN); 
    detachInterrupt(OUTB_PIN);
}

void setup() {
  // ENCODER SETUP
  encoderValue = 0;
  start_sampling = false;
  prev_sampling_time = 0;
  sampling_duration = SAMPLING_DURATION;
  r.begin();
  // SERIAL SETUP
  Serial.begin(9600);
}


void loop() {
     // READ COMMAND FROM SERIAL
    if(Serial.available() > 0)
    {
        String str = Serial.readStringUntil('\n');
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
          attachAllInterrupts();
        }
    }

    // SAMPLING RPM
    if (start_sampling && (millis() - prev_sampling_time > sampling_duration) ){
          Serial.println(millis());
          Serial.println(prev_sampling_time);
          detachAllInterrupts();
          Serial.println(encoderValue);
          float nbRevolution = (encoderValue * PULSE_COUNT_CORRECTION) / float(REDUCTION_FACTOR * PULSE_PER_REVOLUTION);
          Serial.println(nbRevolution);
          start_sampling = false;
    }


}
