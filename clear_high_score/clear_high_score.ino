#include <EEPROM.h>

void setup() {
  unsigned long high_score = 0;
  EEPROM.put(0, high_score);
}

void loop() {
  // put your main code here, to run repeatedly:

}
