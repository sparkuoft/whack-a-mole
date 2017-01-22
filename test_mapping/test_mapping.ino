#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

const int NUM_STEPS = 9;
const int indicatorLED = 13;
Adafruit_7segment highscoreseg = Adafruit_7segment();
Adafruit_7segment scoreseg = Adafruit_7segment();
Adafruit_7segment timeseg = Adafruit_7segment();

// Order:
// 0 - Indy
// 1 - Chem
// 2 - Min
// 3 - EngSci
// 4 - T1
// 5 - Material
// 6 - ECE
// 7 - Mech
// 8 - Civ

int resistances[] = {
  500, 500, 500, 500, 500, 500, 500, 500, 500
};
int stepResistance[] = {
  80, 80, 80, 80, 80, 80, 80, 80, 80
};
int noStepResistance[] = {
  100, 100, 100, 100, 100, 100, 100, 100, 100
};

bool stepState[NUM_STEPS];
bool posEdge[NUM_STEPS];
bool negEdge[NUM_STEPS];

int stepPins[] = {
  A7, A3, A5, A6, A2, A4, A8, A0, A1,
};

int ledPins[] = {
  39, 41, 33, 47, 35, 31, 37, 43, 45,
};

void setup() {
  // setup serial port
  Serial.begin(115200);
  Serial.setTimeout(1000L * 1000L);

  // setup analog pins for pads, and led strips
  for (int i = 0; i < NUM_STEPS; ++i) {
    pinMode(stepPins[i], INPUT);
    pinMode(ledPins[i], OUTPUT);
    // set to HIGH to turn on strip
    digitalWrite(ledPins[i], LOW);
  }
}

// LEDs
void leds() {
  Serial.print("Which LED to light up? ");
  String s = Serial.readStringUntil('.');
  int index = s[0] - '0';
  Serial.print("I heard: ");
  Serial.println(index);
  digitalWrite(ledPins[index-1], HIGH);
  Serial.println("Type . to stop");
  Serial.readStringUntil('.');
  digitalWrite(ledPins[index-1], LOW);
}

void steps() {
  Serial.print("Which step to read? ");
  String s = Serial.readStringUntil('.');
  int index = s[0] - '0';
  Serial.print("I heard: ");
  Serial.println(index);
  delay(1000);
  Serial.println("Type . to stop");
  while (Serial.read() == -1) {
    Serial.println(analogRead(stepPins[index-1]));
  }
}

void both() {
  Serial.print("Which step to test? ");
  String s = Serial.readStringUntil('.');
  int index = s[0] - '0';
  Serial.print("I heard: ");
  Serial.println(index);
  digitalWrite(ledPins[index-1], HIGH);
  delay(1000);
  Serial.println("Type . to stop");
  while (Serial.read() == -1) {
    Serial.println(analogRead(stepPins[index-1]));
  }
  digitalWrite(ledPins[index-1], LOW);
}

void loop() {
  //leds();
  //steps();
  both();
}


