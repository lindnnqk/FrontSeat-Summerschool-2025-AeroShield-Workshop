#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(5, LOW);
  Serial.begin(9600);
  delay(2000);
  Serial.println("Welcome at LRS, Teensy!");
}

void loop() {
  uint16_t val;
  val = analogRead(A3)/4;
  digitalWrite(LED_BUILTIN, HIGH);
  analogWrite(5, val);
  delay(400);
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(5, val);
  delay(200);
  Serial.printf("AnalogRead A3: %i\n", val);
}
