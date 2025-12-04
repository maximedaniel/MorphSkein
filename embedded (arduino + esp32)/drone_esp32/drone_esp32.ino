
#include <ESPAsyncWebSrv.h>
#include <ArduinoJson.h>
#include <xtensa/core-macros.h>
#include "MotionPlanner.h"
#include "Rotary.h"
#include "MyPID.h"
#include "DShotRMT.h"
#include <QTRSensors.h>

// Global parameters
#define USB_SERIAL_BAUD 9600

#define SERIAL_BAUDRATE 115200
#define SPIN_STATUS "status_spin"
#define SPIN_CURRENT "current_spin"
#define SPIN_TARGET "target_spin"
#define SPIN_CURRENT_CMD "current_spin_cmd"
#define SPIN_TARGET_CMD  "target_spin_cmd"
#define TOPIC "topic"
#define PAYLOAD "payload"
#define STATUS "status"


#define DISPLAY_RPM_CMD "display_rpm"
#define ENABLE_CMD "enable"
#define DISABLE_CMD "disable"
#define STATE_CMD "state"
#define SPIN_CMD "spin"
#define SPINCMD_CMD "spinCmd"
#define HEIGHT_CMD "height"
#define LINE_CMD "line"
#define DURATION_CMD "duration"
#define RESET_CMD "reset"
#define COLOR_CMD "color"
#define LIGHT_CMD "light"
#define LIGHTS_CMD "lights"

#define SPINNER_KEEP_ALIVE 10000
#define SPINNER_MAX_RPM_CHANGE_PER_SECOND 15.0


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


// Communication parameters
const char* ssid = "DeformableFlyingDisplay";
const char* password = "123456789";
IPAddress ip(192, 168, 137, 171);
IPAddress dns(192, 168, 137,  1);
IPAddress gateway(192, 168, 137,  1);
IPAddress subnet(255, 255, 255, 0);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// SPIN ACTUATION AND SENSING

DShotRMT dshot_01(GPIO_NUM_4, RMT_CHANNEL_0);
DShotRMT dshot_02(GPIO_NUM_2, RMT_CHANNEL_1);
MotionPlanner spin_motion_planner = MotionPlanner(MotionMode::VELOCITY, 0.75, 0.25, 0, 1.0, 1.0); 
QTRSensors qtr_sensor;
int prevRefectanceValue = 0;
int currRefectanceValue = 0;

// JSON data buffer
StaticJsonDocument<512> jsonDocument;
char buffer[512];

uint32_t  CPU_Frequency_Mhz;
uint32_t CPU_tick_ns;

volatile int current_spin = 0;
int target_spin  = 0;
int prev_spin  = 0;
int status_spin = 0;
volatile uint16_t target_spin_command = 0;
volatile uint16_t current_spin_command = 0;
unsigned long curr_time;
unsigned long spin_prev_time;
unsigned long spin_prev_alive_time;
bool spinner_keep_alive = false;
bool spin_command_enabled = false;


int prev_time = 0;
int print_delay = 100;
unsigned int target_spin_time = 0;
unsigned int current_spin_time = 0;
// float current_velocity = 0.0;
// float prev_velocity = 0.0;
// float target_velocity = 0.0;
unsigned int prev_acceleration_time = 0;
float current_acceleration = 0.0;


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo * info = (AwsFrameInfo*)arg;
  // fetch frame
  if(info->final && info->opcode == WS_BINARY){
          Serial.println("received binary data...");
  } 

  // AwsFrameInfo *info = (AwsFrameInfo*)arg;

  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    deserializeJson(jsonDocument, ((char*)data));
    //serializeJsonPretty(jsonDocument, Serial);
    String topic = jsonDocument["topic"];
    String payload = jsonDocument["payload"];
    // Serial.print("Received message: ");
    // Serial.print(topic);
    // Serial.print(" -> ");
    // Serial.print(payload);
    // Serial.println();

    if (topic == STATE_CMD){
      jsonDocument.clear();
      jsonDocument[TOPIC] = STATE_CMD;
      JsonObject payloadDocument = jsonDocument.createNestedObject(PAYLOAD);
      payloadDocument[SPIN_STATUS] = status_spin; //spin_motion_planner.status();
      payloadDocument[SPIN_CURRENT] = current_spin;
      payloadDocument[SPIN_TARGET] = target_spin;
      payloadDocument[SPIN_CURRENT_CMD] = current_spin_command;
      payloadDocument[SPIN_TARGET_CMD] = target_spin_command;
      serializeJson(jsonDocument, buffer);
      ws.textAll((char*)buffer);
    }

    else if (topic == SPINCMD_CMD){
      spin_command_enabled = true;
      target_spin_command = jsonDocument["payload"][SPINCMD_CMD];
      Serial.printf("Received command: spinCmd %d\n", target_spin_command);
    }


    else if (topic == SPIN_CMD){
      spin_command_enabled = false;
      if (target_spin == 0 && current_spin_command == 0){
          dshot_01.send_dshot_value(current_spin_command);
          dshot_02.send_dshot_value(current_spin_command);
      }
      target_spin = jsonDocument["payload"][SPIN_CMD];
      Serial.printf("Received command: spin %d\n", target_spin);

      // double spinner_new_target_rpm = target_spin_asked;
      // spin_command_enabled = false;
      // if(spinner_new_target_rpm != target_spin){
      //   target_spin = spinner_new_target_rpm;
      //   double rpm_change_total = abs(target_spin - current_spin);
      //   double rpm_change_duration = rpm_change_total/SPINNER_MAX_RPM_CHANGE_PER_SECOND; // time in s
      //   spin_motion_planner.set(current_spin, target_spin, rpm_change_duration, true, false);
      //   //Serial.print("target_spin: ");
      //   //Serial.println(target_spin);
      // }
      // spin_prev_alive_time = curr_time;
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
      Serial.printf("WS_EVT_PONG\n");
      break;
    case WS_EVT_ERROR:
      Serial.printf("WS_EVT_ERROR\n");
      break;
  }
}


void startWifiWebServer(){
  // setup the WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet, dns);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); 

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
}

void handleServer(){
  // CHECK WIFI CONNECTION
    if (WiFi.status() != WL_CONNECTED) {
          Serial.println("Wifi connection lost... stopping everything.");
          //Serial.println(WiFi.localIP()); 
          spinner_keep_alive = false;
          startWifiWebServer();
    } else {
      // CHECK FOR WEBSOCKET CLIENT
      ws.cleanupClients();
    }
}
int sign(double value) {
  return (value < 0)? -1 : ((value > 0)? 1 : 0);
}


void setup(void) {
  
  // ...start the dshot generation
  // setup Motor control through DSHOT
	dshot_01.begin(DSHOT300);
	dshot_02.begin(DSHOT300);
	//dshot_01.send_dshot_value(current_spin_command);
	//dshot_02.send_dshot_value(current_spin_command);
  // setup speed sensing through an IR sensor
  //pinMode(QTR_SENSOR_GPIO, INPUT);
  qtr_sensor.setTypeAnalog();
  qtr_sensor.setSensorPins((const uint8_t[]){QTR_SENSOR_GPIO}, 1);

  spinner_keep_alive = false;
  curr_time = spin_prev_alive_time = millis();
  
  CPU_Frequency_Mhz = getCpuFrequencyMhz();
  CPU_tick_ns = 1.0 / (CPU_Frequency_Mhz / 1000.0);
  curr_time = prev_acceleration_time = millis();
  // setup serial communication for debugging
  Serial.begin(SERIAL_BAUDRATE);
  // setup the WiFi network
  startWifiWebServer();
  
}

// bool updateSpinVelocity(unsigned long current_time){
//   bool new_sample = false;
//   unsigned long spin_delta_time = current_time - spin_prev_time;
//   if ( spin_delta_time > SPINNER_ENCODER_TIME){
//       float spinner_target_nb_pulse = int(target_spin * 2 * SPINNER_PULSES_PER_REVOLUTION / 60000 * spin_delta_time);
//       int spinner_nb_pulse = spin_encoder_value_0 + spin_encoder_value_1;
//       float spinner_nb_revolution = spinner_nb_pulse /  (2 * SPINNER_PULSES_PER_REVOLUTION);
//       current_spin = spinner_nb_revolution  * 60000 / double(spin_delta_time);
//       spin_encoder_value_0 = spin_encoder_value_1 =  0;
//       spin_prev_time = current_time;
//       new_sample  = true;   
//   }
//   return new_sample;
// }


bool updateSpinVelocity(unsigned long current_time){

  bool new_sample = false;
  uint16_t sensors[1];
  qtr_sensor.read(sensors);  // sensors.read(sensor_values);
  currRefectanceValue = sensors[0];
  if (currRefectanceValue){
    currRefectanceValue = sensors[0]/THRESHOLD_TOLERANCE * THRESHOLD_TOLERANCE;
  }

  bool speed_sensing_error = false;
  if (currRefectanceValue <= REFLECTANCE_THRESHOLD &&  prevRefectanceValue > REFLECTANCE_THRESHOLD){ // rising edge detected
      new_sample = true;
      current_spin_time = curr_time;
      current_spin =  60000 / (current_spin_time - target_spin_time); // RPM -> nb_milliseconds_in_1_minute / nb_milliseconds_for_one_turn
      if (abs(current_spin - prev_spin) > RPM_ERROR_THRESHOLD){
        speed_sensing_error = true;
        Serial.print("error detected:");
        Serial.print(prev_spin);
        Serial.print("RPM ->");
        Serial.print(current_spin);
        Serial.println("RPM");
      } else{
        prev_spin = current_spin;
      }
      //if (!speed_sensing_error) 
      target_spin_time =  current_spin_time;
  }
  prevRefectanceValue = currRefectanceValue;

  return new_sample;
}

void loop(){
  handleServer();
  curr_time = millis();
  bool newSpinVelocity = updateSpinVelocity(curr_time);
  if (spin_command_enabled){
    current_spin_command = constrain(target_spin_command, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
    dshot_01.send_dshot_value(current_spin_command);
    dshot_02.send_dshot_value(current_spin_command);
  } else {
    if (current_spin != target_spin){
          if (curr_time - prev_acceleration_time > ACCELERATION_DELAY){
                current_spin_command += sign(target_spin-current_spin);     
                current_spin_command = constrain(current_spin_command, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);  
                Serial.print("next spin command: ");  
                Serial.println(current_spin_command);   
                prev_acceleration_time = curr_time;     
          }
          dshot_01.send_dshot_value(current_spin_command);
          dshot_02.send_dshot_value(current_spin_command);
    }
     else {
      if (target_spin == 0){
          current_spin_command = 0;
          dshot_01.send_dshot_value(current_spin_command);
          dshot_02.send_dshot_value(current_spin_command);
      }
    }
  }
    // else {
    //   if (new_spin_velocity){
    //       if (spin_motion_planner.isActive()){
    //         double spin_speed = spin_motion_planner.process(current_spin, false); 
    //         spin_command = spin_speed;
    //         mc.setSpeed(1, spin_command);
    //       }
    //   }
    // }
  //bool new_spin_velocity = updateSpinVelocity(curr_time);

  // if (curr_time - spin_prev_alive_time > SPINNER_KEEP_ALIVE && target_spin != 0){
  //     target_spin = 0;
  //     double rpm_change_total = abs(target_spin - current_spin);
  //     double rpm_change_duration = rpm_change_total/SPINNER_MAX_RPM_CHANGE_PER_SECOND; // time in s
  //     spin_motion_planner.set(current_spin, target_spin, rpm_change_duration, false, false);
  // }
  // delay(100);
}
