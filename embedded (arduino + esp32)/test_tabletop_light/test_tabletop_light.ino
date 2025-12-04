/// @file    Blink.ino
/// @brief   Blink the first LED of an LED strip
/// @example Blink.ino
// SPI TUTORIAL https://randomnerdtutorials.com/esp32-spi-communication-arduino/

#define HSPI_MISO   12
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define HSPI_SS     15

#define VSPI_MISO   19
#define VSPI_MOSI   23
#define VSPI_SCLK   18
#define VSPI_SS     5
#define FASTLED_ALL_PINS_HARDWARE_SPI
//#define FASTLED_ESP32_SPI_BUS HSPI
#define NUM_STRIPS 2
#define NUM_LEDS 10

#include <SPI.h>
#include <FastLED.h>


CLEDController *controllers[NUM_STRIPS];
uint8_t gBrightness = 255;

CRGB leds[NUM_LEDS];

void setup() { 
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);  
  controllers[0] = &FastLED.addLeds<DOTSTAR, VSPI_MOSI, VSPI_SCLK, BGR>(leds, NUM_LEDS);
  controllers[1] = &FastLED.addLeds<DOTSTAR, HSPI_MOSI, HSPI_SCLK, BGR>(leds, NUM_LEDS);
}

void loop() { 
  // Turn the LED on, then pause
  for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB( 50, 0, 0); // CRGB::Red;
  }
  controllers[0]->showLeds(gBrightness);
  controllers[1]->showLeds(gBrightness);
  delay(200);
  // Now turn the LED off, then pause
  for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB::Black;
  }
  controllers[0]->showLeds(gBrightness);
  controllers[1]->showLeds(gBrightness);
  delay(200);
}