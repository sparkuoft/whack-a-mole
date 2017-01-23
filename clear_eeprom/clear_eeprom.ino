#include <EEPROM.h>

const int HIGH_SCORE_ADDR = 0x0;
const int TIME_ADDR = 0x10;
const int COUNTER_ADDR = 0x20;

void setup() {
  unsigned long high_score = 0;
  EEPROM.put(HIGH_SCORE_ADDR, high_score);

  unsigned long previous_time = 0;
  EEPROM.put(TIME_ADDR, previous_time);

  unsigned int count = 0;
  for (unsigned long addr = COUNTER_ADDR; addr < EEPROM.length(); addr += 2) {
    EEPROM.put(addr, count);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
