// This example shows a simple way to control the
// Motoron Motor Controller using I2C.
//
// The motors will stop but automatically recover if:
// - Motor power (VIN) is interrupted, or
// - A temporary motor fault occurs, or
// - A command timeout occurs.
//
// The motors will stop until you power cycle or reset your
// Arduino if:
// - The Motoron experiences a reset.
//
// If a latched motor fault occurs, the motors
// experiencing the fault will stop until you power cycle motor
// power (VIN) or cause the motors to coast.

#include <Motoron.h>

#define CW 0
#define CCW 1
#define LINE0_LUMINANCE_PIN A1

MotoronI2C mc1(16);
MotoronI2C mc2(17);

unsigned int prev_time;
unsigned int direction;


int motor_speed1 = 0;
int motor_speed2 = 0;

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


void setup()
{
  Wire.begin();
  setupMotoron(mc1);
  setupMotoron(mc2);
  // Configure motor 1
  mc1.setMaxAcceleration(1, 100);
  mc1.setMaxDeceleration(1, 100);
  mc1.setMaxAcceleration(2, 100);
  mc1.setMaxDeceleration(2, 100);
  mc1.setMaxAcceleration(3, 100);
  mc1.setMaxDeceleration(3, 100);
  // Configure motor 2
  mc2.setMaxAcceleration(1, 100);
  mc2.setMaxDeceleration(1, 100);
  mc2.setMaxAcceleration(2, 100);
  mc2.setMaxDeceleration(2, 100);
  mc2.setMaxAcceleration(3, 100);
  mc2.setMaxDeceleration(3, 100);
  prev_time = 0;
  direction = CW;
  pinMode(LINE0_LUMINANCE_PIN, INPUT); // PULLUP FOR SWITCH²
  Serial.begin(9600);
}

void loop()
{
   // READ COMMAND FROM SERIAL
    if(Serial.available() > 0)
    {
        String str = Serial.readStringUntil('\n');
        Serial.println(str);
      
        String speed1Cmd = "speed1 ";
        int speed1Index = str.indexOf(speed1Cmd);
        if(speed1Index != -1){
            Serial.println("Speed1...");
            str = str.substring(speed1Index + speed1Cmd.length());
            int target1Speed = str.toInt();
            Serial.print("target1Speed: ");
            Serial.println(target1Speed);
            motor_speed1 = target1Speed;
        }
        String speed2Cmd = "speed2 ";
        int speed2Index = str.indexOf(speed2Cmd);
        if(speed2Index != -1){
            Serial.println("Speed2...");
            str = str.substring(speed2Index + speed2Cmd.length());
            int target2Speed = str.toInt();
            Serial.print("target2Speed: ");
            Serial.println(target2Speed);
            motor_speed2 = target2Speed;
        }
    }
  if(millis() - prev_time >= 1000){
    Serial.print("endStop: ");
    Serial.println(analogRead(LINE0_LUMINANCE_PIN));
     prev_time = millis();
  }
   mc1.setSpeed(1, motor_speed1);
   mc2.setSpeed(1, motor_speed2);

}
