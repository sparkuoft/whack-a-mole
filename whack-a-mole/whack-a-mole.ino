#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

const int NUM_STEPS = 9;
const int indicatorLED = 13;
Adafruit_7segment highscoreseg = Adafruit_7segment();
Adafruit_7segment scoreseg = Adafruit_7segment();
Adafruit_7segment timeseg = Adafruit_7segment();

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
  A1, A2, A3, A4, A5, A6, A7, A8, A9
};

int ledPins[] = {
  31, 32, 33, 34, 35, 36, 37, 38, 39
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
  long R1 = resistances[index];
  int stepRes = stepResistance[index];
  int noStepRes = noStepResistance[index];

  // Trying to keep all of the arithmetic integer
  long reading = analogRead(pin);
  if (reading == 0) reading = 1;
  long resistance = (R1*1024L)/reading - R1;
  //Serial.print(reading);
  //Serial.print(" ");
  //Serial.print(R1*1024);
  //Serial.print(" ");
  //Serial.print((R1*1024)/reading);
  //Serial.print(" ");
  //Serial.println(resistance);

  // Compute the new state
  bool newState = stepState[index];
  if (!stepState[index] && resistance <= stepRes) {
    newState = true;
  } else if (stepState[index] && resistance >= noStepRes) {
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

void writeTime(long longtime){
  // convert time from ms to s as an integer
  // don't bother rounding, too much effort
  int time = (int)(longtime/1000);
  // write the time to 7-seg display as an int
  timeseg.print(time);
  timeseg.writeDisplay();  
}

// this is the main function for gameplay
void start() {
  // total time allotted for play = 1 min
  long time = 60000;
  // start with round 1
  int roundnum = 1;
  int score = 0;
  int tile;
  long timer;
  
  // start gameplay
  while (time > 0) {
    // write the time
    writeTime(time);
    // randomly pick a tile to light up
    tile = random(0,9);
    digitalWrite(ledPins[tile], HIGH);
    // read steps for variable amount of time, depending on round. Get the score.
    timer = millis();
    score = score + getscore(time, roundnum, tile);
    time = time - (millis() - timer);
    // deactivate tile
    digitalWrite(ledPins[tile], LOW);
    // write score to 7-seg
    scoreseg.print(score, DEC);
    scoreseg.writeDisplay();
    // increment round
    roundnum++;
  }
  writeTime(0.0);
  // flash score
  scoreseg.blinkRate(1);
  // write and flash high score if applicable
  if (score > EEPROM.read(0)) {
    EEPROM.write(0, score);
    highscoreseg.print(score, DEC);
    highscoreseg.blinkRate(1);
  }
  // flash score for 5 seconds before exiting
  delay(5000);
  scoreseg.blinkRate(0);
  highscoreseg.blinkRate(0);
}

// this function waits for a step on the correct tile
int getscore(long time, int roundnum, int tile) {
  // every 4 rounds, the time interval goes down by .1 sec
  long timeleft = (long) max((1000 - (roundnum / 4)*100), 350);
  // poll the step
  readSteps();
  while (!posEdge[tile]) {
    readSteps();
    if ((millis() - time) > timeleft) {
      break;
    }
    // We might want to add this if the clock updates are too uneven
    // writeTime(time);
  }
  return posEdge[tile];
}

void setup() {
  // setup serial port
  Serial.begin(115200);
  // setup analog pins for pads, and led strips
  for (int i = 0; i < NUM_STEPS; ++i) {
    pinMode(stepPins[i], INPUT);
    pinMode(ledPins[i], OUTPUT);
    // set to HIGH to turn on strip
    digitalWrite(ledPins[i], LOW);
  }
  pinMode(indicatorLED, OUTPUT);
  initSteps();
  // setup 7-seg controllers:
  // high score 7-seg
  highscoreseg.begin(0x70);
  // score 7-seg
  scoreseg.begin(0x71);
  // time remaining 7-seg
  timeseg.begin(0x72);
  // initialize high score
  int highscore = 0;
  // setup start button
  // unconnected pin used for random seed
  randomSeed(analogRead(0));
}

void loop() {
  // wait for start button to be pressed
  // poll button
  start();
}
