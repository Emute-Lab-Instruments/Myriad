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
      size_t gap = ts < now ? now - ts : std::numeric_limits<unsigned long>::max() - ts + now;
      if(gap > 25) {
        val = nextval;
        ts = millis();
        Serial.printf("Debounce %d\n", val);
      }
    }
    return val;
  }
  unsigned long ts;
  bool val;
};

#endif
