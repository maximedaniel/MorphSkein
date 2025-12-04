
#include <ESPAsyncWebSrv.h>
#include <ArduinoJson.h>
#include <xtensa/core-macros.h>
#include "My_Adafruit_DotStar.h"

// Global parameters
#define USB_SERIAL_BAUD 9600

#define PRINT_DELAY 100
#define SERIAL_BAUDRATE 115200
#define SPIN_CURRENT "current_spin"
#define TOPIC "topic"
#define PAYLOAD "payload"
#define STATUS "status"


#define DEBUG_CMD "debug "
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
#define WIDTH_CMD "width"
#define COLOR_CMD "color"
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



#define VSPI_MISO   MISO // 19
#define VSPI_MOSI   MOSI // 23 
#define VSPI_SCLK   SCK // 18
#define VSPI_SS     SS // 5
#define SPI_FREQUENCY 20000000
#define NUMPIXELS 55
#define NUMPIXELS_LEFT_OFFSET 3
#define NUMPIXELS_RIGHT_OFFSET 2

// Communication parameters
const char* ssid = "DeformableFlyingDisplay";
const char* password = "123456789";
IPAddress ip(192, 168, 137, 173);
IPAddress dns(192, 168, 137,  1);
IPAddress gateway(192, 168, 137,  1);
IPAddress subnet(255, 255, 255, 0);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/light");

// JSON data buffer
StaticJsonDocument<512> jsonDocument;
char buffer[512];

uint32_t  CPU_Frequency_Mhz;
uint32_t CPU_tick_ns;

unsigned long curr_time;
bool debug = false;
unsigned long prev_print_time;
unsigned long spin_prev_time;
unsigned long spin_prev_alive_time;


int prev_time = 0;
int print_delay = 100;

SPIClass * vspi = NULL;
My_Adafruit_DotStar * dotstar0 = NULL;

int target_width = 0;
int target_light_width = 10; //3
int target_color = 0;
int target_light_color = 0x191919; //0xFFFFFF;
bool new_target_light = true;

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
    if (topic == LIGHT_CMD){
      target_light_color = jsonDocument["payload"][COLOR_CMD];
      target_light_width = jsonDocument["payload"][WIDTH_CMD];
      Serial.printf("Received command: light %06X %d\n", target_light_color, target_light_width);
      new_target_light = true;
    }
    if (topic == STATE_CMD){
      jsonDocument.clear();
      jsonDocument[TOPIC] = STATE_CMD;
      JsonObject payloadDocument = jsonDocument.createNestedObject(PAYLOAD);
      serializeJson(jsonDocument, buffer);
      ws.textAll((char*)buffer);
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
          startWifiWebServer();
    } else {
      // CHECK FOR WEBSOCKET CLIENT
      ws.cleanupClients();
    }
}
int sign(double value) {
  return (value < 0)? -1 : ((value > 0)? 1 : 0);
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
void setup(void) {
  curr_time  = millis();
  CPU_Frequency_Mhz = getCpuFrequencyMhz();
  CPU_tick_ns = 1.0 / (CPU_Frequency_Mhz / 1000.0);
  curr_time  = millis();
  vspi = new SPIClass(VSPI);
  vspi->begin();
  dotstar0 = new My_Adafruit_DotStar(NUMPIXELS,  DOTSTAR_BGR, vspi, SPI_FREQUENCY); 
  // setup serial communication for debugging
  Serial.begin(SERIAL_BAUDRATE);
  // setup the WiFi network
  //startWifiWebServer();
  
}
void setStripFromColor(My_Adafruit_DotStar * dotstar, int start_pixel_index, int end_pixel_index, int color){
    for (int i = end_pixel_index-1; i >= start_pixel_index; i--){
        int j = (end_pixel_index - 1) - i;
        dotstar -> setPixelColor(i, color);
    }
}

void handleLight(){
    if (new_target_light){
          target_color = target_light_color;
          target_width = target_light_width;
          new_target_light = false;
      }
      dotstar0 -> clear();
      setStripFromColor(dotstar0, NUMPIXELS_LEFT_OFFSET, target_width + NUMPIXELS_LEFT_OFFSET, target_color);
      setStripFromColor(dotstar0, NUMPIXELS -NUMPIXELS_RIGHT_OFFSET - target_width, NUMPIXELS - NUMPIXELS_RIGHT_OFFSET, target_color);
      //dotstar0 -> setPixelColor(NUMPIXELS0 - RESETPIXEL, 0x880000); 
      //dotstar1 -> setPixelColor(NUMPIXELS3 - RESETPIXEL, 0x440044); 
      dotstar0 -> show(); 
      delay(100);
}

void loop(){
  //handleServer();
  curr_time = millis();
  handleLight();
  // bool newSpinVelocity = updateSpinVelocity(curr_time);
  // if (debug && curr_time - prev_print_time > PRINT_DELAY){
  //   Serial.print("current_spin:");
  //   Serial.print(current_spin);
  //   Serial.print(", currRefectanceValue:");
  //   Serial.print(currRefectanceValue);
  //   Serial.println();
  //   prev_print_time = curr_time;
  // }

  if(Serial.available() > 0){
    String str = Serial.readStringUntil('\n');
    handleDebugCmd(str, Serial);
  }
}
