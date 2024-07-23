#ifndef _MY_TOGGLE_H_
#define _MY_TOGGLE_H_

// create an object to toggle. Toggles when toggle method is called. Can be used as a condition in a conditional statement. Returns true by default
class MyToggle {
  public:
    bool toggle() {
      static bool toggleInput = false;
      toggleInput = !toggleInput;
      return toggleInput;
    }
};

#endif