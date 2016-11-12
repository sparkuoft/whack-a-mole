
const int NUM_STEPS = 1;//9;
const int indicatorLED = 13;

int resistances[] = {
  680, 680, 680, 680, 680, 680, 680, 680, 680
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

void initSteps() {
  for (int i = 0; i < NUM_STEPS; ++i) {
    stepState[i] = false;
    posEdge[i] = false;
    negEdge[i] = false;
  }
}

void readSteps() {
  for (int i = 0; i < NUM_STEPS; ++i) {
    readStep(i);
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_STEPS; ++i) {
    pinMode(stepPins[i], INPUT);
  }
  pinMode(indicatorLED, OUTPUT);
}

void loop() {
  while (true) {
    readSteps();
    digitalWrite(indicatorLED, stepState[0] ? HIGH : LOW);
    if (posEdge[0]) {
      Serial.print("Positive edge: ");
      Serial.println(millis());
    }
    if (negEdge[0]) {
      Serial.print("Negative edge: ");
      Serial.println(millis());
    }
  }
}
