
#include <String.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <QTRSensors.h>
#include "DShotRMT.h"

#define USB_SERIAL_BAUD 9600
#define BUFFER_SIZE 1024
#define TCP_PORT 1234
#define SET_ACCELERATION_CMD "setAcceleration"
#define SET_VELOCITY_CMD "setVelocity"
#define GET_STATE_CMD "getState"

#define NUMERIC_TOLERANCE 0.01// A0 // 35
#define THRESHOLD_TOLERANCE 100// A0 // 35
#define QTR_SENSOR_GPIO 35// A0 // 35
#define QTR_EMITTER_GPIO 32
#define REFLECTANCE_THRESHOLD 1500
#define RPM_ERROR_THRESHOLD 50
#define MAX_THROTTLE_VALUE 256
#define MIN_THROTTLE_VALUE 0
#define ACCELERATION_DELAY 250 // 250 -> max speed in 1 min | 125 -> max speed in 30secs
// 450RPM -> 12% -> 245

DShotRMT dshot_01(GPIO_NUM_4, RMT_CHANNEL_0);
DShotRMT dshot_02(GPIO_NUM_2, RMT_CHANNEL_1);
volatile uint16_t curr_throttle_value = 0; // ...sending "48", the first throttle value
volatile uint16_t target_throttle_value = 0; // ...sending "48", the first throttle value

volatile int throttle_perc = 0; // ...sending "48", the first throttle value
constexpr auto FAILSAVE_THROTTLE = 0x3E7;

QTRSensors qtr_sensor;
int prevRefectanceValue = 0;
int currRefectanceValue = 0;

// WIFI NETWORK PARAMETERS
const char* ssid = "DeformableFlyingDisplay";
const char* password = "123456789";

WiFiServer server;
uint8_t buff[BUFFER_SIZE];


const int led = 13;

int prev_time = 0;
int curr_time = 0;
int print_delay = 100;
unsigned int target_velocity_time = 0;
unsigned int current_velocity_time = 0;
float current_velocity = 0.0;
float prev_velocity = 0.0;
float target_velocity = 0.0;
unsigned int prev_acceleration_time = 0;
float current_acceleration = 0.0;

// Set web server port number to 80
// Set your Static IP address
IPAddress ip(192, 168, 137, 170);
IPAddress dns(192, 168, 137,  1);
// Set your Gateway IP address
IPAddress gateway(192, 168, 137,  1);

IPAddress subnet(255, 255, 255, 0);


void setVelocity(float velocity) {
  target_velocity  = velocity;
}

void setAcceleration(float acceleration) {
  current_acceleration  = acceleration;
}



void setup(void) {
  // ...start the dshot generation
  // setup Motor control through DSHOT
	dshot_01.begin(DSHOT300);
	dshot_02.begin(DSHOT300);
	dshot_01.send_dshot_value(curr_throttle_value);
	dshot_02.send_dshot_value(curr_throttle_value);

  // setup speed sensing through an IR sensor
  //pinMode(QTR_SENSOR_GPIO, INPUT);
  qtr_sensor.setTypeAnalog();
  qtr_sensor.setSensorPins((const uint8_t[]){QTR_SENSOR_GPIO}, 1);

  Serial.begin(USB_SERIAL_BAUD);
  WiFi.mode(WIFI_STA);
  // Configures static IP address
  WiFi.config(ip, gateway, subnet, dns);
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
        Serial.print("connecting to WiFi network");
        delay(500);
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server = WiFiServer(TCP_PORT);
  server.begin();
  delay(1000);
  Serial.println("server started");
}

String processMessage(uint8_t* buff, int size){
  String response = "";
  String str_buff = (char*)buff;
  str_buff = str_buff.substring(0, size);
  
  if (str_buff.indexOf(GET_STATE_CMD) != -1){
    response = "{";
    response += "\"targetVelocity\":";
    response += target_velocity;
    response += ", \"currentVelocity\":";
    response += current_velocity;
    response += ", \"currentAcceleration\":";
    response += current_acceleration;
    response += "}";
  }
  else if (str_buff.indexOf(SET_VELOCITY_CMD) != -1){
    int start_parameter_index = str_buff.indexOf(" ");
    if (start_parameter_index != 1){
      //Serial.println(str_buff.substring(start_parameter_index));
      float velocity = str_buff.substring(start_parameter_index).toFloat();
      //Serial.println(velocity);
      setVelocity(velocity);
    } else {
      response = "INVALID PARAMETER";
    }
  }
  else if (str_buff.indexOf(SET_ACCELERATION_CMD) != -1){
    int start_parameter_index = str_buff.indexOf(" ");
    if (start_parameter_index != 1){
      float acceleration = str_buff.substring(start_parameter_index).toFloat();
      setAcceleration(acceleration);
    } else {
      response = "INVALID PARAMETER";
    }
  }
  else{
    response = "INVALID COMMAND";
  }
  return response;

}

void loop(void) {
   WiFiClient client = server.available();
   if (client){
    int size = 0;
     if (client.connected()){
        if((size = client.available())){
           size = (size >= BUFFER_SIZE ? BUFFER_SIZE : size);
           client.read(buff, size);
           //Serial.print("receive:");
           //Serial.write(buff, size);
           //Serial.println();

           String response = processMessage(buff, size);
           if (response != ""){
            response.toCharArray((char*)buff, BUFFER_SIZE);  
            client.write(buff, response.length());
           }
           //Serial.print("reply:");
           //Serial.write(buff, response.length());
           //Serial.println();
           //Serial.flush();
        }
     }
   }
  curr_time = millis();
  //throttle_perc = constrain(abs(throttle_perc), 0, 100);
  //curr_throttle_value = map(throttle_perc, 0, 100, 0, 2048);
  curr_throttle_value = constrain(curr_throttle_value, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  dshot_01.send_dshot_value(curr_throttle_value);
  dshot_02.send_dshot_value(curr_throttle_value);
  uint16_t sensors[1];
  qtr_sensor.read(sensors);  // sensors.read(sensor_values);
  currRefectanceValue = sensors[0];
  if (currRefectanceValue){
    currRefectanceValue = sensors[0]/THRESHOLD_TOLERANCE * THRESHOLD_TOLERANCE;
  }

  bool speed_sensing_error = false;
  if (currRefectanceValue <= REFLECTANCE_THRESHOLD &&  prevRefectanceValue > REFLECTANCE_THRESHOLD){ // rising edge detected
      current_velocity_time = curr_time;
      current_velocity =  60000 / (current_velocity_time - target_velocity_time); // RPM -> nb_milliseconds_in_1_minute / nb_milliseconds_for_one_turn
      if (abs(current_velocity - prev_velocity) > RPM_ERROR_THRESHOLD){
        speed_sensing_error = true;
        Serial.print("error detected:");
        Serial.print(prev_velocity);
        Serial.print("RPM ->");
        Serial.print(current_velocity);
        Serial.println("RPM");
      } else{
        prev_velocity = current_velocity;
      }
      if (!speed_sensing_error) 
      target_velocity_time =  current_velocity_time;
  }
  prevRefectanceValue = currRefectanceValue;

  if (!speed_sensing_error){
    if (current_velocity < target_velocity){
        if (curr_time - prev_acceleration_time > ACCELERATION_DELAY){
              curr_throttle_value += 1;            
        }
        prev_acceleration_time = curr_time;
    }  

    if (current_velocity > target_velocity){
        if (curr_time - prev_acceleration_time > ACCELERATION_DELAY){
              curr_throttle_value -= 1;            
        }
        prev_acceleration_time = curr_time;
    }
    
  }
  
}
