#ifndef SYNTH_PARAMS_H
#define SYNTH_PARAMS_H

static queue_t __not_in_flash("mydata") coreCommsQueue;

struct queueItem {
  uint8_t idx;
  size_t value;
};

static int __not_in_flash("mydata") wavelen0 = 30000;
static int __not_in_flash("mydata") wavelen1 = 30010;
static int __not_in_flash("mydata") wavelen2 = 30020;
static int __not_in_flash("mydata") wavelen3 = 31100;
static int __not_in_flash("mydata") wavelen4 = 31500;
static int __not_in_flash("mydata") wavelen5 = 31900;


#endif