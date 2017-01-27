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
// 1 - Indy
// 2 - Chem
// 3 - Min
// 4 - EngSci
// 5 - T1
// 6 - Material
// 7 - ECE
// 8 - Mech
// 9 - Civ

int stepValues[] = {
  80, 80, 80, 80, 80, 80, 80, 80, 80
};
int noStepValues[] = {
  100, 100, 100, 100, 100, 100, 100, 100, 100
};

bool stepState[NUM_STEPS];
bool posEdge[NUM_STEPS];
bool negEdge[NUM_STEPS];
long measurements[NUM_STEPS];

int stepPins[] = {
  A7, A3, A5, A6, A2, A4, A8, A0, A1,
};

int ledPins[] = {
  39, 41, 33, 47, 35, 31, 37, 43, 45,
};

// R1 is fixed, R2 is the step
// Assuming: Vcc--R2--Vo--R1--GND
// Vo = Vcc * (R1 / (R1 + R2))
// So, R2 = R1*(Vcc/Vo) - R1
// And Vo = analogRead / 1024 * Vcc
// So, R2 = R1*(1024/analogRead) - R1
void readStep(int index) {
  int Vcc = 5;
  int pin = stepPins[index];
  int stepValue = stepValues[index];
  int noStepValue = noStepValues[index];

  // Trying to keep all of the arithmetic integer
  long reading = analogRead(pin);
  measurements[index] = reading;

  // Compute the new state
  bool newState = stepState[index];
  if (!stepState[index] && reading <= stepValue) {
    newState = true;
  } else if (stepState[index] && reading >= noStepValue) {
    newState = false;
  }

  // Compute whether we got an edge
  posEdge[index] = newState && !stepState[index];
  negEdge[index] = !newState && stepState[index];

  // Update step state
  stepState[index] = newState;
}

void readSteps() {
  for (int i = 0; i < NUM_STEPS; ++i) {
    readStep(i);
  }
}

void initSteps() {
  for (int i = 0; i < NUM_STEPS; ++i) {
    stepState[i] = false;
    posEdge[i] = false;
    negEdge[i] = false;
  }
}

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

  initSteps();
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
    readSteps();
    Serial.println(measurements[index-1]);
  }
  digitalWrite(ledPins[index-1], LOW);
}

void loop() {
  //leds();
  //steps();
  both();
}


