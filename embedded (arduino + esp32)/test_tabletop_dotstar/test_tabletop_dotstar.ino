/// @file    Blink.ino
/// @brief   Blink the first LED of an LED strip
/// @example Blink.ino
// SPI TUTORIAL https://randomnerdtutorials.com/esp32-spi-communication-arduino/

#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET
#include <xtensa/core-macros.h>


#define VSPI_MISO   MISO // 19
#define VSPI_MOSI   MOSI // 23 
#define VSPI_SCLK   SCK // 18
#define VSPI_SS     SS // 5
#define VSPI_MUX 4

#define HSPI_MISO   12  
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define HSPI_SS     15
#define HSPI_MUX    21

#define MAXPIXELS     20
#define NUMPIXELS     143
#define MAX_BRIGHTNESS   127

#define CPU_FREQUENCY 240 //MHz
#define MUX_NANOSECONDS_DELAY 200


//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;
//uninitalised pointers to SPI objects
Adafruit_DotStar * strip0 = NULL;
Adafruit_DotStar * strip1 = NULL;

uint32_t prevCycleCount = 0;
uint32_t currCycleCount = 0;
uint32_t  CPU_Frequency_Mhz;
uint32_t CPU_tick_ns;


//Adafruit_DotStar strip0(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
//Adafruit_DotStar strip0(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno = pin 11 for data, 13 for clock, other boards are different).
//Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BRG);

void selectMux(int mux_pin, int mux_channel){
  int A = bitRead(mux_channel,0);
  digitalWrite(mux_pin, A);
  prevCycleCount = currCycleCount = XTHAL_GET_CCOUNT();
  while((currCycleCount - prevCycleCount) * CPU_tick_ns < MUX_NANOSECONDS_DELAY){
    currCycleCount = XTHAL_GET_CCOUNT();
  }
}


void setup() {
  Serial.begin(115200);
  CPU_Frequency_Mhz = getCpuFrequencyMhz();
  CPU_tick_ns = 1.0 / (CPU_Frequency_Mhz / 1000.0);
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  vspi->begin();
  hspi->begin();
  strip0 = new Adafruit_DotStar(NUMPIXELS,  DOTSTAR_BGR, vspi); // 8Mhz -> if we fork the code we can easy push to 80Mhz by creating a new constructor
  strip1 = new Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR, hspi);
  // MUX
  pinMode(VSPI_MUX, OUTPUT); 
  digitalWrite(VSPI_MUX, LOW);
  pinMode(HSPI_MUX, OUTPUT); 
  digitalWrite(HSPI_MUX, LOW);
  strip0 -> begin(); 
  strip1 -> begin();
  strip0 -> show(); 
  strip1 -> show(); 
  
}

// Runs 10 LEDs at a time along strip, cycling through red, green and blue.
// This requires about 200 mA for all the 'on' pixels + 1 mA per 'off' pixel.

int      head  = 0, tail = -10; // Index of first 'on' and 'off' pixels
uint32_t green_color = 0x001100;      // 'On' color (starts red)
uint32_t red_color = 0x110000;      // 'On' color (starts red)
uint32_t blue_color = 0x000011;      // 'On' color (starts red)
uint32_t cyan_color = 0x110011;      // 'On' color (starts red)
uint32_t no_color = 0x000000;      // 'On' color (starts red)

unsigned int red = 0x00;
unsigned int blue = 0x00;
unsigned int green = 0x00;
unsigned int color = 0x000000;
int light_index = 0;
const int step = 8;
const unsigned long LIGHT_DELAY = 250;
unsigned long prev_light_time;


void makeColorGradient(float frequency1, float frequency2, float frequency3, float phase1, float phase2, float phase3, float center, float width, float len){
    if ( millis() - prev_light_time > LIGHT_DELAY){
      light_index = light_index + 1;
      if (light_index > len - 1) light_index = 0;
      red = (sin(frequency1 * light_index + phase1) * width + center) * MAX_BRIGHTNESS / 255;
      green = (sin(frequency2 * light_index + phase2) * width + center) * MAX_BRIGHTNESS / 255;
      blue = (sin(frequency3 * light_index + phase3) * width + center) * MAX_BRIGHTNESS / 255;
      color  = red << 16 | green << 8 | blue;
      for (int i = 0; i < NUMPIXELS; i++){
        strip0 -> setPixelColor(i, color); // 'On' pixel at head
      }
      strip0 -> show(); 
      prev_light_time = millis();
    }
}

void loop() {
  //makeColorGradient(.1, .1, .1, 0, 2, 4, 128, 127, 128);
  
  for (int i = 0; i < NUMPIXELS; i++){
    if (i >= NUMPIXELS - MAXPIXELS){
      strip0 -> setPixelColor(i, green_color); // 'On' pixel at head
      strip1 -> setPixelColor(i, red_color); // 'On' pixel at head
    }
     else {
      strip0 -> setPixelColor(i, no_color); // 'On' pixel at head
      strip1 -> setPixelColor(i, no_color); // 'On' pixel at head
    }
  }
  Serial.println("SHOW!");
  selectMux(VSPI_MUX, 0);
  strip0 -> show(); 
  selectMux(HSPI_MUX, 0);
  strip1 -> show(); 
  selectMux(VSPI_MUX, 1);
  strip0 -> clear(); 
  strip0 -> show(); 
  selectMux(HSPI_MUX, 1);
  strip1 -> clear(); 
  strip1 -> show(); 
  delay(500);

  for (int i = 0; i < NUMPIXELS; i++){
    if (i >= NUMPIXELS - MAXPIXELS){
      strip0 -> setPixelColor(i, blue_color); // 'On' pixel at head
      strip1 -> setPixelColor(i, cyan_color); // 'On' pixel at head
    }
     else {
       strip0 -> setPixelColor(i, no_color); // 'On' pixel at head
       strip1 -> setPixelColor(i, no_color); // 'On' pixel at head
    }
  }
  // selectMux(VSPI_MUX, 0);
  // strip0 -> show(); 
  // selectMux(HSPI_MUX, 0);
  // strip1 -> show(); 
  selectMux(VSPI_MUX, 1);
  strip0 -> show(); 
  selectMux(HSPI_MUX, 1);
  strip1 -> show(); 
  selectMux(VSPI_MUX, 0);
  strip0 -> clear(); 
  strip0 -> show(); 
  selectMux(HSPI_MUX, 0);
  strip1 -> clear(); 
  strip1 -> show(); 
  delay(500);
  
}