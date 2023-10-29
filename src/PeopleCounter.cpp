// People Counter Class
// Author: Chip McClelland
// Date: May 2023
// License: GPL3
// In this class, we look at the occpancy values and determine what the occupancy count should be 
// Note, this code is limited to a single sensor with two zones
// Note, this code assumes that Zone 1 is the inner (relative to room we are measureing occupancy for) and Zone 2 is outer

#include "Particle.h"
#include "PeopleCounterConfig.h"
#include "ErrorCodes.h"
#include "PeopleCounter.h"
#include "TofSensor.h"
#include <StackArray.h>

// StackArray data structure:
// [ bottom, ... , ... , top ]
StackArray <int> stateStack;
StackArray <int> tempStack;

static int occupancyCount = 0;      // How many folks in the room or (if there is more than one door) - net occupancy through this door
static int occupancyLimit = DEFAULT_PEOPLE_LIMIT;

int magicalStateMap[4] = {3, 2, 1, 0};                                // Define impossible state transitions (Ex. newOccupancyState cannot equal impossibilityMap[lastOccupancyState])

PeopleCounter *PeopleCounter::_instance;

// [static]
PeopleCounter &PeopleCounter::instance() {
    if (!_instance) {
        _instance = new PeopleCounter();
    }
    return *_instance;
}

PeopleCounter::PeopleCounter() {
}

PeopleCounter::~PeopleCounter() {
}

void PeopleCounter::setup() {
  MOUNTED_INSIDE ? PeopleCounter::instance().setCount(1) : PeopleCounter::instance().setCount(0);
}

void PeopleCounter::loop(){                                               // This function is only called if there is a change in occupancy state
  int oldOccupancyCount = occupancyCount;
  int newOccupancyState = TofSensor::instance().getOccupancyState();
  
  if(newOccupancyState != stateStack.peek()){
    switch(stateStack.count()){
      case 0:                         
        stateStack.push(0);       // First value MUST be a 0
        #if PEOPLECOUNTER_DEBUG
          Log.info("[Line 55]: SEQUENCE [SIZE = %i]: [%i] <--- %i", stateStack.count(), stateStack.peekIndex(0), newOccupancyState);
        #endif                                           
        break;
      case 1:
        stateStack.push(newOccupancyState);                             // Push to the stack without checking for impossibilities
        #if PEOPLECOUNTER_DEBUG
          Log.info("[Line 61]: SEQUENCE [SIZE = %i]: [%i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), newOccupancyState);
        #endif
        break;                                                           
      case 2:
      case 3:                                                           // When the stateStack has 2 or 3 items, we must identify impossible patterns and fix them.
        applyMagicalStateMapCorrections(newOccupancyState);
        #if PEOPLECOUNTER_DEBUG
          switch (stateStack.count()){
            case 1:
              Log.info("[Line 70]: SEQUENCE [SIZE = %i]: [%i] <--- %i", stateStack.count(), stateStack.peekIndex(0), newOccupancyState);
              break;
            case 2:
              Log.info("[Line 73]: SEQUENCE [SIZE = %i]: [%i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), newOccupancyState);
              break;
            case 3:
              Log.info("[Line 76]: SEQUENCE [SIZE = %i]: [%i, %i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), stateStack.peekIndex(2), newOccupancyState);
              break;
            case 4:
              Log.info("[Line 79]: SEQUENCE [SIZE = %i]: [%i, %i, %i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), stateStack.peekIndex(2), stateStack.peekIndex(3), newOccupancyState);
              break;
          }
        #endif
        break;
      case 4:
        if(newOccupancyState != 0 && newOccupancyState != magicalStateMap[stateStack.peek()]){                                     // If the final occupancy state is NOT 0 ...
          while(stateStack.peek() != newOccupancyState){                        // ... until the top of the stack is equal to the new occupancy state ...
            stateStack.pop();                                                       // ... remove the top of the stack.
          }
          #if PEOPLECOUNTER_DEBUG
            switch (stateStack.count()){
              case 1:
                Log.info("[Line 92]: SEQUENCE [SIZE = %i]: [%i] <--- %i", stateStack.count(), stateStack.peekIndex(0), newOccupancyState);
                break;
              case 2:
                Log.info("[Line 95]: SEQUENCE [SIZE = %i]: [%i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), newOccupancyState);
                break;
              case 3:
                Log.info("[Line 98]: SEQUENCE [SIZE = %i]: [%i, %i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), stateStack.peekIndex(2), newOccupancyState);
                break;
              case 4:
                Log.info("[Line 101]: SEQUENCE [SIZE = %i]: [%i, %i, %i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), stateStack.peekIndex(2), stateStack.peekIndex(3), newOccupancyState);
                break;
            }
          #endif        
        } else {                                                        // If the new occupancy state is 0 ...
          (newOccupancyState == magicalStateMap[stateStack.peek()]) ? stateStack.push(0) : stateStack.push(newOccupancyState);                                   // ... push the final state.
        }
        break;
    }
  }
  
  if(stateStack.count() == 5){                                        // If the stack is finished ...
    char states[56];                                                          // ... turn it into a string by popping all values off the stack...
      #if PEOPLECOUNTER_DEBUG
        Log.info("[Line 115]: SEQUENCE [SIZE = %i]: [%i, %i, %i, %i, %i] <--- %i", stateStack.count(), stateStack.peekIndex(0), stateStack.peekIndex(1), stateStack.peekIndex(2), stateStack.peekIndex(3), stateStack.peekIndex(4), newOccupancyState);
      #endif 
      snprintf(states, sizeof(states), "%i%i%i%i%i", stateStack.pop(), stateStack.pop(), stateStack.pop(), stateStack.pop(), stateStack.pop());   
      if(strcmp(states, "01320")){
        MOUNTED_INSIDE ? occupancyCount-- : occupancyCount++;                                                           // ... if the sequence (backwards) matches the increment sequence then increment the count.
      } else if(strcmp(states, "02310")) {
        MOUNTED_INSIDE ? occupancyCount++ : occupancyCount--;                                                           // ... if the sequence (backwards) matches the decrement secuence then decrement the count.
      } else {
        Log.info("ERROR: Algorithm somehow produced states: %s", states);           // ... if the sequence does not match the decrement or increment sequence, do nothing.
      }
  }
  #if TENFOOTDISPLAY
      if (oldOccupancyCount != occupancyCount) printBigNumbers(occupancyCount);
  #else
    if (oldOccupancyCount != occupancyCount) Log.info("Occupancy %s %i",(occupancyCount > oldOccupancyCount) ? "increased to" : "decreased to", occupancyCount);
  #endif
}


int PeopleCounter::getCount(){
  Log.info("Occupancy count is %d",occupancyCount);
  return occupancyCount;

}

void PeopleCounter::setCount(int value){
  occupancyCount = value;
}

int PeopleCounter::getLimit(){
  return occupancyLimit;
}

void PeopleCounter::setLimit(int value){
  occupancyLimit = value;
}

void PeopleCounter::applyMagicalStateMapCorrections(int newOccupancyState){
  int needsCleanup = 0;
  tempStack.push(newOccupancyState);                              // Push the new occupancyState to the tempStack
  while(stateStack.count() > 1){                                  // Go through the stack containing prior states
    int current = stateStack.pop();                               
    int after = tempStack.peek();
    int before = stateStack.peek();
    if(before == after){                                          // If the state before current is equal to the state after current ...
      stateStack.push(current);                             // put current back
      needsCleanup = 1;
      break;
    }
    if(magicalStateMap[before] == current){                       // If the transition from before --> current is impossible, we must have failed to detect the person at some point ...
      tempStack.push(current);                                            // ... so push current ...
      int missedState = magicalStateMap[after];                               // ... consult the magical state map to determine what state was missed ...
      tempStack.push(missedState);                                                // ... then push that.
    } else if(magicalStateMap[current] == after) {                // If the transition from current --> after is impossible, we must have failed to detect the person at some point ...
      int missedState = magicalStateMap[before];                          // ... so consult the magical state map to determine what state was missed ...
      tempStack.push(missedState);                                            // ... push the missing state ...
      tempStack.push(current);                                                    // ... then push current.
    } else {                                                      // If the transition from before --> current AND from current --> after are possible ...
      tempStack.push(current);                                            // ... push current.
    }
  }
  while(!tempStack.isEmpty()){                                    // move everything in the tempStack back to the permanent stack
    stateStack.push(tempStack.pop());
  } 
  if(needsCleanup) { 
    do {                        // ... until the top of the stack is equal to the new occupancy state ...
      stateStack.pop();                                                       // ... remove the top of the stack.
    } while (stateStack.peek() != newOccupancyState); 
  } 
}

void PeopleCounter::printBigNumbers(int number) {
  Log.info("  ");

  switch (abs(number)) {
    case 0:
      Log.info("%s  0000  ", (number < 0) ? "      " : "");
      Log.info("%s 0    0 ", (number < 0) ? "      " : "");
      Log.info("%s0      0", (number < 0) ? "      " : "");
      Log.info("%s0      0", (number < 0) ? "------" : "");  
      Log.info("%s0      0", (number < 0) ? "      " : "");     
      Log.info("%s 0    0 ", (number < 0) ? "      " : "");
      Log.info("%s  0000  ", (number < 0) ? "      " : "");
    break;

    case 1:
      Log.info("%s    11  ", (number < 0) ? "      " : "");
      Log.info("%s   1 1  ", (number < 0) ? "      " : "");
      Log.info("%s     1  ", (number < 0) ? "      " : "");
      Log.info("%s     1  ", (number < 0) ? "------" : "");
      Log.info("%s     1  ", (number < 0) ? "      " : "");
      Log.info("%s     1  ", (number < 0) ? "      " : "");
      Log.info("%s   11111", (number < 0) ? "      " : "");
    break;

    case 2:
      Log.info("%s  2222  ", (number < 0) ? "      " : "");
      Log.info("%s 2    22", (number < 0) ? "      " : "");
      Log.info("%s     2  ", (number < 0) ? "      " : "");
      Log.info("%s   2    ", (number < 0) ? "------" : "");
      Log.info("%s  2     ", (number < 0) ? "      " : "");
      Log.info("%s22     2", (number < 0) ? "      " : "");
      Log.info("%s2222222 ", (number < 0) ? "      " : "");
      break;

      case 3:
      Log.info("%s  3333  ", (number < 0) ? "      " : "");
      Log.info("%s 3    3 ", (number < 0) ? "      " : "");
      Log.info("%s       3", (number < 0) ? "      " : "");
      Log.info("%s   333  ", (number < 0) ? "------" : "");
      Log.info("%s       3", (number < 0) ? "      " : "");
      Log.info("%s 3    3 ", (number < 0) ? "      " : "");
      Log.info("%s  3333  ", (number < 0) ? "      " : "");
      break;

      case 4:
      Log.info("%s4      4", (number < 0) ? "      " : "");
      Log.info("%s4      4", (number < 0) ? "      " : "");
      Log.info("%s4      4", (number < 0) ? "      " : "");
      Log.info("%s  4444  ", (number < 0) ? "------" : "");
      Log.info("%s       4", (number < 0) ? "      " : "");
      Log.info("%s       4", (number < 0) ? "      " : "");
      Log.info("%s       4", (number < 0) ? "      " : "");
      break;

      case 5:
      Log.info("%s  555555", (number < 0) ? "      " : "");
      Log.info("%s 5      ", (number < 0) ? "      " : "");
      Log.info("%s 555555 ", (number < 0) ? "      " : "");
      Log.info("%s      5 ", (number < 0) ? "------" : "");
      Log.info("%s       5", (number < 0) ? "      " : "");
      Log.info("%s      5 ", (number < 0) ? "      " : "");
      Log.info("%s 555555 ", (number < 0) ? "      " : "");
      break;

      case 6:
      Log.info("%s  666666", (number < 0) ? "      " : "");
      Log.info("%s 6      ", (number < 0) ? "      " : "");
      Log.info("%s  66666 ", (number < 0) ? "      " : "");
      Log.info("%s6      6", (number < 0) ? "------" : "");
      Log.info("%s6      6", (number < 0) ? "      " : "");
      Log.info("%s 6    6 ", (number < 0) ? "      " : "");
      Log.info("%s  6666  ", (number < 0) ? "      " : "");
      break;

      case 7:
      Log.info("%s  777777", (number < 0) ? "      " : "");
      Log.info("%s 7     7", (number < 0) ? "      " : "");
      Log.info("%s      7 ", (number < 0) ? "      " : "");
      Log.info("%s     7  ", (number < 0) ? "------" : "");
      Log.info("%s    7   ", (number < 0) ? "      " : "");
      Log.info("%s   7    ", (number < 0) ? "      " : "");
      Log.info("%s  7     ", (number < 0) ? "      " : "");
      break;

      case 8:
      Log.info("%s  8888  ", (number < 0) ? "      " : "");
      Log.info("%s 8    8 ", (number < 0) ? "      " : "");
      Log.info("%s8      8", (number < 0) ? "      " : "");
      Log.info("%s  8888  ", (number < 0) ? "------" : "");
      Log.info("%s8      8", (number < 0) ? "      " : "");
      Log.info("%s 8    8 ", (number < 0) ? "      " : "");
      Log.info("%s  8888  ", (number < 0) ? "      " : "");
      break;

      case 9:
      Log.info("%s 99999  ", (number < 0) ? "      " : "");
      Log.info("%s9     9 ", (number < 0) ? "      " : "");
      Log.info("%s9      9", (number < 0) ? "      " : "");
      Log.info("%s 99999  ", (number < 0) ? "------" : "");
      Log.info("%s       9", (number < 0) ? "      " : "");
      Log.info("%s      9 ", (number < 0) ? "      " : "");
      Log.info("%s 999999 ", (number < 0) ? "      " : "");
      break;

      default:
      Log.info("********");
      Log.info("********");
      Log.info("********");
      Log.info("********");
      Log.info("********");
      Log.info("********");
      Log.info("********");
      break;
  }
  Log.info("  ");
}
