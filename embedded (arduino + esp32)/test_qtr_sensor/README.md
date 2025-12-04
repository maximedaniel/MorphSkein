# POVDroneProject
POVDroneProject code
#include <WiFi.h>
#include <WebServer.h>

#include <QTRSensors.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include "DShotRMT.h"


#ifdef SERIAL
#define USB_Serial Serial
constexpr auto USB_SERIAL_BAUD = 9600;
#endif // SERIAL


// #define NUMERIC_TOLERANCE 0.01// A0 // 35
// #define THRESHOLD_TOLERANCE 100// A0 // 35
// #define QTR_SENSOR_GPIO 35// A0 // 35
// #define QTR_EMITTER_GPIO 32
// #define REFLECTANCE_THRESHOLD 1500
// #define ACCELERATION_DELAY 250 // 250 -> max speed in 1 min | 125 -> max speed in 30secs
// // 450RPM -> 12% -> 245

// DShotRMT dshot_01(GPIO_NUM_4, RMT_CHANNEL_0);
// DShotRMT dshot_02(GPIO_NUM_2, RMT_CHANNEL_1);
// volatile uint16_t curr_throttle_value = 0; // ...sending "48", the first throttle value
// volatile uint16_t target_throttle_value = 0; // ...sending "48", the first throttle value

// volatile int throttle_perc = 0; // ...sending "48", the first throttle value
// constexpr auto FAILSAVE_THROTTLE = 0x3E7;

// int prevRefectanceValue = 0;
// int currRefectanceValue = 0;

// QTRSensors qtr_sensor;

int prev_time = 0;
int curr_time = 0;
int print_delay = 100;
unsigned int prev_speed_time = 0;
unsigned int curr_speed_time = 0;
float curr_speed = 0.0;
float prev_speed = 0.0;
unsigned int prev_acceleration_time = 0;
float curr_accel = 1.0;
float prev_accel = 1.0;


// WIFI NETWORK PARAMETERS
const char* ssid = "POVDroneDisplay";
const char* password = "123456789";

DynamicJsonDocument jsonDocument(2048);
char buffer[2048];

// Set web server port number to 80
WebServer  server(80);

void setup_routing() {     
  server.on("/speed",  getSpeed);  
  //server.on("/speed", HTTP_POST, setSpeed);     
  server.on("/accel", getAcceleration);      
  //server.on("/accel",  HTTP_POST, setAcceleration);     
  server.begin();    
}
 
void create_json(char *tag, float value, char *unit) {  
  jsonDocument.clear();  
  jsonDocument["type"] = tag;
  jsonDocument["value"] = value;
  jsonDocument["unit"] = unit;
  serializeJson(jsonDocument, buffer);
}
 
void add_json_object(char *tag, float value, char *unit) {
  JsonObject obj = jsonDocument.createNestedObject();
  obj["type"] = tag;
  obj["value"] = value;
  obj["unit"] = unit; 
}

String getSpeed() {
  Serial.println("Get speed");
  create_json("speed", curr_speed, "RPM");
  server.send(200, "application/json", buffer);
}

String getAcceleration() {
  Serial.println("Get acceleration");
  create_json("acceleration", curr_accel, "RPM/s");
  server.send(200, "application/json", buffer);
}

// void handlePost() {
//   if (server.hasArg("plain") == false) {
//   }
//   String body = server.arg("plain");
//   deserializeJson(jsonDocument, body);

//   int red_value = jsonDocument["red"];
//   int green_value = jsonDocument["green"];
//   int blue_value = jsonDocument["blue"];

//   ledcWrite(redChannel, red_value);
//   ledcWrite(greenChannel,green_value);
//   ledcWrite(blueChannel, blue_value);

//   server.send(200, "application/json", "{}");
// }



void setup()
{

  
	USB_Serial.begin(USB_SERIAL_BAUD);
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  setup_routing();

	// // ...start the dshot generation
  // // setup Motor control through DSHOT
	// dshot_01.begin(DSHOT300);
	// dshot_02.begin(DSHOT300);
	// dshot_01.send_dshot_value(curr_throttle_value);
	// dshot_02.send_dshot_value(curr_throttle_value);

  // // setup speed sensing through an IR sensor
  // pinMode(QTR_SENSOR_GPIO, INPUT);
  // qtr_sensor.setTypeAnalog();
  // qtr_sensor.setSensorPins((const uint8_t[]){QTR_SENSOR_GPIO}, 1);


}

void loop()
{
  server.handleClient();  
  // curr_time = millis();
  // if (USB_Serial.available() > 0)
	// {
	// 	throttle_perc = (USB_Serial.readStringUntil('\n')).toInt();
  //   throttle_perc = constrain(abs(throttle_perc), 0, 100);
  // }

  // target_throttle_value = map(throttle_perc, 0, 100, 0, 2048);
  //   if (abs(target_throttle_value - curr_throttle_value) > 1){
  //       if (curr_time - prev_acceleration_time > ACCELERATION_DELAY){
  //         if (target_throttle_value > curr_throttle_value){
  //             curr_throttle_value += 1;            
  //         }
  //         if (target_throttle_value < curr_throttle_value){
  //             curr_throttle_value -= 1;
  //         }
  //         prev_acceleration_time = curr_time;
  //       }
  //   }
  //   dshot_01.send_dshot_value(curr_throttle_value);
  //   dshot_02.send_dshot_value(curr_throttle_value);
  
  // uint16_t sensors[1];
  // qtr_sensor.read(sensors);  // sensors.read(sensor_values);
  // currRefectanceValue = sensors[0];
  // if (currRefectanceValue) 
  //   currRefectanceValue = sensors[0]/THRESHOLD_TOLERANCE * THRESHOLD_TOLERANCE;
  // // digitalWrite(QTR_EMITTER_GPIO, HIGH);
  // // delay(10);
  // // currRefectanceValue = analogRead(QTR_SENSOR_GPIO);
  // // digitalWrite(QTR_EMITTER_GPIO, LOW);
  // //if (currRefectanceValue < REFLECTANCE_THRESHOLD)
  // //USB_Serial.println(currRefectanceValue);

  // if (currRefectanceValue <= REFLECTANCE_THRESHOLD &&  prevRefectanceValue > REFLECTANCE_THRESHOLD){ // rising edge detected
  //     curr_speed_time = curr_time;
  //     curr_speed =  60000 / (curr_speed_time - prev_speed_time); // RPM -> nb_milliseconds_in_1_minute / nb_milliseconds_for_one_turn
  //     prev_speed_time =  curr_speed_time;
  // }
  // prevRefectanceValue = currRefectanceValue;

  // if (curr_time - prev_time > print_delay && abs(curr_speed - prev_speed) > NUMERIC_TOLERANCE){
  //   USB_Serial.print("throttle:");
  //   USB_Serial.print(throttle_perc);
  //   USB_Serial.println("%");

  //   USB_Serial.print("speed:");
  //   USB_Serial.print(curr_speed);
  //   USB_Serial.println("RPM");
  //   prev_speed = curr_speed;
  //   prev_time =  curr_time;
  // }
}
