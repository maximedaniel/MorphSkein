
#define LUMINANCE_PIN A0   // select the input pin for the potentiometer
int luminance_value;  // variable to store the value coming from the sensor

void setup() {
  luminance_value = 0;
  Serial.begin(9600);

}

void loop() {
  // read the value from the sensor:
  luminance_value = analogRead(LUMINANCE_PIN);
  Serial.print("luminance: ");
  Serial.println(luminance_value);
  delay(100);
}
