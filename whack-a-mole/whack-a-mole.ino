#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

const int NUM_STEPS = 9;
const int indicatorLED = 13;
const int HIGH_SCORE_ADDR = 0x0;
const int TIME_ADDR = 0x10;
const int COUNTER_ADDR = 0x20;
const unsigned long ONE_HOUR_MS = 1000L * 60L * 60L;
const unsigned long TEN_MINUTES_MS = 1000L * 60L * 10L;
unsigned long previous_time = 0;
unsigned long last_serial_update = 0;
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

unsigned long startResetButton[3];
int startButton = 2;
bool reset = false;
bool startgame = false;

/*
 * This function checks if a mechanical switch has been activated. The time
 * constant, 'DEBOUNCE_DELAY' avoids activated a false signal due to bouncing in
 * the switch
 */
bool switchToggled(unsigned long switchPin, unsigned long* lastValue, unsigned long* lastActivated){
  unsigned long now = millis();
  int currentValue = digitalRead(switchPin) == HIGH ? 0 : 1;

  if( (now - *lastActivated) < 200){
    return false;
  }

  if(*lastValue != currentValue){
    // inverted because it's pulled HIGH by default
    *lastValue = currentValue;
    *lastActivated = now;
    return currentValue;
  }

  return false;
}

/*
 * Instantiates the initial state of the switch (think of an RC capacitor)
 */
void initializeButton(unsigned long *buttonObject, unsigned long pinID){
  buttonObject[0] = pinID;
  buttonObject[1] = 0;
  buttonObject[2] = millis();

  //note: default value is HIGH because of the INPUT_PULLUP state
  Serial.print(pinID);
  pinMode(pinID, INPUT_PULLUP);
}

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
  // reset the start button state
  startgame = false;
  reset = false;
  // total time allotted for play = 1 min
  long time = 60000;
  // start with round 1
  int roundnum = 1;
  int score = 0;
  int tile;
  long timer;
  
  // intro flash the time 7-seg
  flash_intro();
  
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
    if (reset) {
      return;
    }
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
  if (score > EEPROM.read(HIGH_SCORE_ADDR)) {
    EEPROM.write(HIGH_SCORE_ADDR, score);
    highscoreseg.print(score, DEC);
    highscoreseg.blinkRate(1);
  }
  // flash score for 3 seconds before exiting
  delay(3000);
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
    if (switchToggled(startResetButton[0], &startResetButton[1], &startResetButton[2])) {
      reset = true;
      break;
    }
    readSteps();
    if ((millis() - time) > timeleft) {
      break;
    }
    // We might want to add this if the clock updates are too uneven
    // writeTime(time);
  }
  return posEdge[tile];
}

void flash_intro() {
  // literally the worst function I have ever written
  long j;
  for (int i=0; i < 6; i++) {
    j = (long)i*1000;
    writeTime(j);
    delay(500);
  }
}

void updateEEPROMTime() {
  // Check if our clock has ticked forward ten minutes compared to the stored EEPROM value
  unsigned long eeprom_time;
  EEPROM.get(TIME_ADDR, eeprom_time);
  unsigned long current_time = previous_time + millis();
  if (current_time > eeprom_time + TEN_MINUTES_MS) {
    // If so, update the EEPROM time
    EEPROM.put(TIME_ADDR, current_time);
  }
}

// Get the number of plays stored in EEPROM for this hour
unsigned int getEEPROMCount(unsigned int hour) {
  unsigned int count; // 2 bytes
  EEPROM.get(COUNTER_ADDR + 2*hour, count);
  return count;
}

// Store count into the EEPROM at the given hour
void putEEPROMCount(unsigned int hour, unsigned int count) {
  EEPROM.put(COUNTER_ADDR + 2*hour, count);
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
  
  // setup start button
  initializeButton(startResetButton, startButton);
  
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
  // unconnected pin used for random seed
  randomSeed(analogRead(0));

  // Initialize the stats stuff
  EEPROM.get(TIME_ADDR, previous_time);
}

// TODO: can the score ever go over 255?
void loop() {
  // Per-game reset

  // reset all the LEDs in case the start button was pressed at an inopportune time and
  // one of the LEDs is still on.
  for (int i = 0; i < NUM_STEPS; ++i) {
    digitalWrite(ledPins[i], LOW);
  }

  // clear the timer
  writeTime(0); // or whatever

  // clear the score
  scoreseg.print(0, DEC); // or whatever
  scoreseg.writeDisplay();

  // read the high score and show that
  int high_score = EEPROM.read(HIGH_SCORE_ADDR);
  highscoreseg.print(high_score, DEC);

  // wait for start button to be pressed
  while (!startgame) {
    startgame = switchToggled(startResetButton[0], &startResetButton[1], &startResetButton[2]);

    // We need to periodically update the time stored in the EEPROM.
    // Since this loop is where we spend most of our idle time, let's do it here.
    updateEEPROMTime();

    // We also want to periodically (every 10 seconds) print out the stats to the serial port
    // so we can read them just by plugging in a USB.
    if (millis() - last_serial_update > 1000L * 10L) {
      unsigned long total = 0;
      unsigned long current_hour = (previous_time + millis()) / ONE_HOUR_MS;
      for (int i = 0; i <= current_hour; i++) {
        Serial.print("Hour ");
        Serial.print(i);
        Serial.print(": ");
        unsigned int count = getEEPROMCount(i);
        Serial.println(count);
        total += count;
      }
      Serial.print("Total: ");
      Serial.println(total);
      last_serial_update = millis();
    }
  }
  start();
  startgame = reset;

  // Check if we actually finished this game without reset
  if (!reset) {
    // Great! Count it
    unsigned long current_time = previous_time + millis();
    unsigned int hours_since_start = current_time / ONE_HOUR_MS;
    unsigned int old_count = getEEPROMCount(hours_since_start);
    putEEPROMCount(hours_since_start, old_count + 1);
    // Also update the EEPROM time
    updateEEPROMTime();
  }
}

