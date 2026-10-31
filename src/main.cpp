#include <Arduino.h>
#include "mode_main.h"
#include "mode_test.h"

// default RUN_MODE is 1
#ifndef RUN_MODE
  #define RUN_MODE 1   
#endif

// Select the setup function depending on the mode
void setup() {
#if RUN_MODE == 1
  setup_main();
#elif RUN_MODE == 2 
  setup_test();
#else
  #error "Unknown RUN_MODE"
#endif
}

// Select the loop function depending on the mode
void loop() {
#if RUN_MODE == 1
  loop_main();
#elif RUN_MODE == 2
  loop_test();
#else
  #error "Unknown RUN_MODE"
#endif
}
