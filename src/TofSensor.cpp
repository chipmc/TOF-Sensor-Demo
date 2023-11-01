// Time of Flight Sensor Class
// Author: Chip McClelland
// Date: May 2023
// License: GPL3
// This is the class for the ST Micro VL53L1X Time of Flight Sensor
// We are using the Sparkfun library which has some shortcomgings
// - It does not implement distance mode medium
// - It does not give access to the factory calibration of the optical center

#include "Particle.h"
#include "ErrorCodes.h"
#include "TofSensorConfig.h"
#include "PeopleCounterConfig.h"
#include "TofSensor.h"

/** Detection **/
int detectionBaselines[2] = {0, 0};                 // Upper and lower values that define a baseline "range" of signal strength that is considered as "nothing" (Max, Min)
int lastDetectionSignal = 0;                        // The most recent Detection zone Signal
int detectionState = 0;                             // The current detection state (detected someone or not)
int detectionMode = 1;                              // Flag for putting the device in "detection" mode, a state saving power by waiting to trigger an interrupt

/** Occupancy **/
uint8_t occupancyOpticalCenters[2] = {OCCUPANCY_FRONT_ZONE_CENTER, OCCUPANCY_BACK_ZONE_CENTER}; // Array of optical centers for the Occupancy zones (zone 1 (ones) and zone 2 (twos))
int zoneSignalPerSpad[2][2] = {{0, 0}, {0, 0}};     // Stores the average Signal strength of all SPADs in bursts of 2 readings for each zone to reduce noise (zone 1 (ones) and zone 2 (twos))
int occupancyBaselines[2][2] = {{0, 0}, {0, 0}};    // Upper and lower values that define a baseline "range" of signal strength that is considered as "nothing" ((zone 1(Min, Max)) , (zone 2(Min, Max)))
int lastSignal[2] = {0, 0};                         // The most recent Signal (zone 1 (ones) and zone 2 (twos))
int lastAmbient[2] = {0, 0};                        // The most recent Ambient (zone 1 (ones) and zone 2 (twos))
int occupancyState = 0;                             // The current occupancy state (occupied or not, zone 1 (ones) and zone 2 (twos))
int ready = 0;                                      // "ready" flag

TofSensor *TofSensor::_instance;

// [static]
TofSensor &TofSensor::instance() {
  if (!_instance) {
      _instance = new TofSensor();
  }
  return *_instance;
}

TofSensor::TofSensor() {
}

TofSensor::~TofSensor() {
}

SFEVL53L1X myTofSensor;

void TofSensor::setup(){
  if(myTofSensor.begin() != 0){
    Log.info("Sensor error reset in 10 seconds");
    delay(10000);
    System.reset();
  }
  else Log.info("Sensor init successfully");
  
  // Here is where we set the device properties
  myTofSensor.setSigmaThreshold(45);        // Default is 45 - this will make it harder to get a valid result - Range 1 - 16383
  myTofSensor.setSignalThreshold(1500);     // Default is 1500 raising value makes it harder to get a valid results- Range 1-16383
  myTofSensor.setTimingBudgetInMs(20);      // Was 20mSec

  // while (TofSensor::measure() == SENSOR_BUFFRER_NOT_FULL) {delay(10);}; // Wait for the buffer to fill up
  Log.info("Buffer is full - will now calibrate");

  if (TofSensor::performOccupancyCalibration() 
      && TofSensor::performDetectionCalibration()) Log.info("Calibration Complete");
  else {
    Log.info("Initial calibration failed - wait 10 secs and reset");
    delay(10000);
    System.reset();
  }

  // myTofSensor.setDistanceModeShort();                     // Once initialized, we are focused on the top half of the door

}

bool TofSensor::performOccupancyCalibration() {
  TofSensor::measure();
  occupancyBaselines[0][0] = lastSignal[0];     // Assign the first reading as max AND min for each zone
  occupancyBaselines[0][1] = lastSignal[0];
  occupancyBaselines[1][0] = lastSignal[1];
  occupancyBaselines[1][1] = lastSignal[1];
  for (int i=0; i<NUM_OCCUPANCY_CALIBRATION_LOOPS; i++) {   // Loop through a set number of times ... 
    TofSensor::measure();
    if(zoneSignalPerSpad[0][0] < occupancyBaselines[0][0]) occupancyBaselines[0][0] = zoneSignalPerSpad[0][0];    // ... and update the min and max readings each time
    if(zoneSignalPerSpad[0][1] > occupancyBaselines[0][1]) occupancyBaselines[0][1] = zoneSignalPerSpad[0][1];
    if(zoneSignalPerSpad[1][0] < occupancyBaselines[1][0]) occupancyBaselines[1][0] = zoneSignalPerSpad[1][0];
    if(zoneSignalPerSpad[1][1] > occupancyBaselines[1][1]) occupancyBaselines[1][1] = zoneSignalPerSpad[1][1];
  }
  #if DEBUG_COUNTER
    Log.info("[BASELINE RANGES] Zone1:{Min=%ikbps/SPAD, Max=%ikbps/SPAD} Zone2:{Min=%ikbps/SPAD, Max=%ikbps/SPAD}", occupancyBaselines[0][0], occupancyBaselines[0][1], occupancyBaselines[1][0], occupancyBaselines[1][1]);
    Log.info("[OCCUPANCY CHECK] Zone1:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} Zone2:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} - Occupancy State %i", zoneSignalPerSpad[0][0], zoneSignalPerSpad[0][1], zoneSignalPerSpad[1][0], zoneSignalPerSpad[1][1], occupancyState);
  #endif
  while (occupancyState != 0){
    Log.info("Occupancy zone not clear - try again");
    delay(80);
    TofSensor::loop();
  }
  //make sure we have a bit of room at the bottom of the range for black hair
  if(occupancyBaselines[0][0] < 5) occupancyBaselines[0][0] = 5;
  if(occupancyBaselines[1][0] < 5) occupancyBaselines[1][0] = 5;

  Log.info("Target zone is clear with zone1 range at {MIN: %ikcps/SPAD, MAX: %ikcps/SPAD} and zone2 range at {MIN: %ikcps/SPAD, MAX: %ikcps/SPAD}",occupancyBaselines[0][0],occupancyBaselines[0][1],occupancyBaselines[1][0],occupancyBaselines[1][1]);
  return TRUE;
}

bool TofSensor::performDetectionCalibration() {
  TofSensor::detect();
  detectionBaselines[0] = lastDetectionSignal;        // Assign the first reading as max AND min of baseline
  detectionBaselines[1] = lastDetectionSignal; 
  for (int i=0; i<NUM_DETECTION_CALIBRATION_LOOPS; i++) {       // Loop through a set number of times ... 
    TofSensor::detect();
    if(lastDetectionSignal < detectionBaselines[0]) detectionBaselines[0]= lastDetectionSignal;    // ... and update the min and max readings each time.
    if(lastDetectionSignal > detectionBaselines[1]) detectionBaselines[1] = lastDetectionSignal;
  }
  while (detectionState != 0){
    Log.info("Detection zone not clear - try again");
    delay(80);
    TofSensor::loop();
  }
  detectionBaselines[0] -= 5;
  detectionBaselines[0] += 5;

  Log.info("Detection zone is clear with range {MIN: %ikcps/SPAD, MAX: %ikcps/SPAD} ",detectionBaselines[0],detectionBaselines[1]);
  return TRUE;
}

int TofSensor::loop(){                         // This function will update the current detection or occupancy for each zone. Switches modes based on interrupt triggers. Returns error codes also.
  int result = 0;
  if(detectionMode){
    detectionState = 0;
    result = TofSensor::detect();
    detectionState += (lastDetectionSignal > detectionBaselines[1] || lastDetectionSignal < detectionBaselines[0]) ? 1 : 0;
    #if DEBUG_COUNTER
        Log.info("[DETECTION BASELINE RANGES] {Min=%ikbps/SPAD, Max=%ikbps/SPAD}", detectionBaselines[0], detectionBaselines[1]);
        Log.info("[DETECTION CHECK] Zone1:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} Zone2:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} - Occupancy State %i\n", zoneSignalPerSpad[0][0], zoneSignalPerSpad[0][1], zoneSignalPerSpad[1][0], zoneSignalPerSpad[1][1], occupancyState);
    #endif

  } else {
    occupancyState = 0;
    result = TofSensor::measure();
    occupancyState += ((zoneSignalPerSpad[0][0] > (occupancyBaselines[0][1]) && zoneSignalPerSpad[0][1] > (occupancyBaselines[0][1]))
                      || (zoneSignalPerSpad[0][0] <= (occupancyBaselines[0][0]) && zoneSignalPerSpad[0][1] <= (occupancyBaselines[0][0]))) ? 1 : 0;
    
    occupancyState += ((zoneSignalPerSpad[1][0] > (occupancyBaselines[1][1]) && zoneSignalPerSpad[1][1] > (occupancyBaselines[1][1]))
                      || (zoneSignalPerSpad[1][0] <= (occupancyBaselines[1][0]) && zoneSignalPerSpad[1][1] <= (occupancyBaselines[1][0]))) ? 2 : 0;
    #if DEBUG_COUNTER
        Log.info("[OCCUPANCY BASELINE RANGES] Zone1:{Min=%ikbps/SPAD, Max=%ikbps/SPAD} Zone2:{Min=%ikbps/SPAD, Max=%ikbps/SPAD}", occupancyBaselines[0][0], occupancyBaselines[0][1], occupancyBaselines[1][0], occupancyBaselines[1][1]);
        Log.info("[OCCUPANCY CHECK] Zone1:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} Zone2:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} - Occupancy State %i\n", zoneSignalPerSpad[0][0], zoneSignalPerSpad[0][1], zoneSignalPerSpad[1][0], zoneSignalPerSpad[1][1], occupancyState);
    #endif
  }
  return (result);     // Relay the result of taking our samples.
}

int TofSensor::measure(){
  ready = 0;
  unsigned long startedRanging;
  for (byte samples = 0; samples < 4; samples++){
    int zone = samples % 2;
    myTofSensor.stopRanging();
    myTofSensor.clearInterrupt();
    myTofSensor.setROI(OCCUPANCY_ROWS_OF_SPADS,OCCUPANCY_COLUMNS_OF_SPADS,occupancyOpticalCenters[zone]);
    delay(1);
    myTofSensor.startRanging();
    startedRanging = millis();
    while(!myTofSensor.checkForDataReady()) {
      if (millis() - startedRanging > SENSOR_TIMEOUT) {
        Log.info("Sensor Timed out");
        return SENSOR_TIMEOUT_ERROR;
      }
    }
    lastSignal[zone] = myTofSensor.getSignalPerSpad();
    lastAmbient[zone] = myTofSensor.getAmbientPerSpad();
    zoneSignalPerSpad[zone][samples < 2 ? 0 : 1] = lastSignal[zone];
    #if DEBUG_COUNTER
     if(samples >= 2){
      // Log.info("[READING] Zone%i (%dx%d %d SPADs with optical center %d)", zone+1, myTofSensor.getROIX(), myTofSensor.getROIY(), myTofSensor.getSpadNb(),opticalCenters[zone]);
      zone == 0 ? Log.info("{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD}", zoneSignalPerSpad[zone][0], zoneSignalPerSpad[zone][1]) : Log.info("                                    {1st=%ikbps/SPAD, 2nd=%ikbps/SPAD}", zoneSignalPerSpad[zone][0], zoneSignalPerSpad[zone][1]);
     }
    #endif
  }
  return(++ready);
}

int TofSensor::detect(){
  ready = 0;
  unsigned long startedRanging;
  myTofSensor.stopRanging();
  myTofSensor.clearInterrupt();
  myTofSensor.setROI(DETECTION_ROWS_OF_SPADS, DETECTION_COLUMNS_OF_SPADS, DETECTION_ZONE_CENTER);
  delay(1);
  myTofSensor.startRanging();
  startedRanging = millis();
  while(!myTofSensor.checkForDataReady()) {
    if (millis() - startedRanging > SENSOR_TIMEOUT) {
      Log.info("Sensor Timed out");
      return SENSOR_TIMEOUT_ERROR;
    }
  }
  lastDetectionSignal = myTofSensor.getSignalPerSpad();
  return(++ready);
}

int TofSensor::getDetectionMode() {
  return detectionMode;
}

void TofSensor::setDetectionMode(int mode) {
  detectionMode = mode;
}

int TofSensor::getDetectionZone() {
  return lastDetectionSignal;
}

int TofSensor::getDetectionState() {
  return detectionState;
}

int TofSensor::getOccupancyZone1() {
  return lastSignal[0];
}

int TofSensor::getOccupancyZone2() {
  return lastSignal[1];
}

int TofSensor::getOccupancyZone1Ambient() {
  return lastAmbient[0];
}

int TofSensor::getOccupancyZone2Ambient() {
  return lastAmbient[1];
}

int TofSensor::getOccupancyState() {
  return occupancyState;
}



