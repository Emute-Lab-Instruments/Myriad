#ifndef __DEBOUNCER_H
#define __DEBOUNCER_H

struct debouncer {
  debouncer() {
    ts = millis();
    val=0;
  }

  bool debounce(int pin) {
    auto now = millis();
    bool nextval = digitalRead(pin);
    if (nextval != val) {
      unsigned long gap = now - ts;
      if(gap > 50) {
        val = nextval;
        ts =now;
        Serial.printf("Debounce %d\n", val);
      }
    }
    return val;
  }
  unsigned long ts;
  bool val;
};

#endif
