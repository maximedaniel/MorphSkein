#include <QTRSensors.h>


#include <Arduino.h>
#include <QTRSensors.h>
#ifdef SERIAL
#define USB_Serial Serial
constexpr auto USB_SERIAL_BAUD = 115200;
#endif // SERIAL

#define QTR_SENSOR_GPIO 35
#define QTR_EMITTER_GPIO 32
#define REFLECTANCE_THRESHOLD 30

int prevRefectanceValue = 0;
int currRefectanceValue = 0;

QTRSensors qtr_sensor;

int prev_time = 0;
int curr_time = 0;
int print_delay = 100;
unsigned int prev_speed_time = 0;
unsigned int curr_speed_time = 0;
float speed = 0.0;

void setup()
{
  pinMode(QTR_SENSOR_GPIO, INPUT);
  pinMode(QTR_EMITTER_GPIO, OUTPUT);
	USB_Serial.begin(USB_SERIAL_BAUD);
  qtr_sensor.setTypeAnalog();
  qtr_sensor.setSensorPins((const uint8_t[]){QTR_SENSOR_GPIO}, 1);
  qtr_sensor.setEmitterPin(QTR_EMITTER_GPIO);
}
void loop()
{
  curr_time = millis();
  uint16_t sensors[1];
  qtr_sensor.read(sensors);  // sensors.read(sensor_values);
  currRefectanceValue = sensors[0];
  // digitalWrite(QTR_EMITTER_GPIO, HIGH);
  // delay(10);
  // currRefectanceValue = analogRead(QTR_SENSOR_GPIO);
  // digitalWrite(QTR_EMITTER_GPIO, LOW);
  if (currRefectanceValue > REFLECTANCE_THRESHOLD)
    USB_Serial.println(currRefectanceValue);

  if (currRefectanceValue >= REFLECTANCE_THRESHOLD &&  prevRefectanceValue < REFLECTANCE_THRESHOLD){ // rising edge detected
      curr_speed_time = curr_time;
      speed =  60000 / (curr_speed_time - prev_speed_time); // RPM -> nb_milliseconds_in_1_minute / nb_milliseconds_for_one_turn
      prev_speed_time =  curr_speed_time;
  }
  prevRefectanceValue = currRefectanceValue;

  if (curr_time - prev_time > print_delay){
    //USB_Serial.print("speed:");
    //USB_Serial.print(speed);
    //USB_Serial.println("RPM");
    prev_time =  curr_time;
  }
}

// void IRAM_ATTR countTurn()
// {
//   digitalWrite(LED_pin, !digitalRead(LED_pin));
// }
// void setup()
// {
//   pinMode(pushButton_pin, INPUT_PULLUP);
//   attachInterrupt(pushButton_pin, countTurn, RISING);
// } 
// void loop()
// {
// }

//
// uint16_t read_SerialThrottle()
// {
// 	if (USB_Serial.available() > 0)
// 	{
// 		auto throttle_input = (USB_Serial.readStringUntil('\n')).toInt();
// 		return throttle_input;
// 	}
// 	else
// 	{
// 		return FAILSAVE_THROTTLE;
// 	}
// }










