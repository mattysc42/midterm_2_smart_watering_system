#ifndef _MY_TIMER_H_
#define _MY_TIMER_H_

// you need setTimer as a condition for an 'if' or 'while' statement. Nested timers don't work!
class MyTimer {
  int timerPrevious = -99999;
  
  public: 
    // Use as a condition for an 'if' or 'while' statement to set the timer.
    bool setTimer(unsigned int timerInput) {
      bool checkTimer = (millis() - timerPrevious) >= timerInput;
      if(checkTimer == true) {
        timerPrevious = millis();
      }
      return checkTimer;
    }
};
#endif