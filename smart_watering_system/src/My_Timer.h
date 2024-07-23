#ifndef _MY_TIMER_H_
#define _MY_TIMER_H_

// you need setTimer as a condition for an 'if' or 'while' statement and startTimer inside the conditional statement for this to work.
class MyTimer {
  int timerPrevious = -999999;
  unsigned int resetInput;

  public: 
    // Use as a condition for an 'if' or 'while' statement to set the timer.
    bool setTimer(unsigned int timerInput) {
      return (millis() - timerPrevious) >= timerInput;
    }

    // Place inside the 'if' or 'while' statement's curly brackets to start the timer.
    int startTimer() {
      timerPrevious = millis();
      return timerPrevious;
    }
};
#endif