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

uint8_t opticalCenters[2] = {FRONT_ZONE_CENTER,BACK_ZONE_CENTER}; 
int zoneSignalPerSpad[2][3] = {{0, 0, 0},{0, 0, 0}};
int zoneBaselines[2] = {0,0};
int lastSignal[2] = {0,0};
int occupancyState = 0;      // This is the current occupancy state (occupied or not, zone 1 (ones) and zone 2 (twos))

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
  myTofSensor.setDistanceModeLong();
  myTofSensor.setSigmaThreshold(45);        // Default is 45 - this will make it harder to get a valid result - Range 1 - 16383
  myTofSensor.setSignalThreshold(1500);     // Default is 1500 raising value makes it harder to get a valid results- Range 1-16383
  myTofSensor.setTimingBudgetInMs(20);      // Was 20mSec

  while (TofSensor::loop() == SENSOR_BUFFRER_NOT_FULL) {delay(10);}; // Wait for the buffer to fill up
  Log.info("Buffer is full - will now calibrate");

  if (TofSensor::performCalibration()) Log.info("Calibration Complete");
  else {
    Log.info("Initial calibration failed - wait 10 secs and reset");
    delay(10000);
    System.reset();
  }

  // myTofSensor.setDistanceModeShort();                     // Once initialized, we are focused on the top half of the door

}

bool TofSensor::performCalibration() {
  for (int i=0; i<NUM_CALIBRATION_LOOPS; i++) {
    TofSensor::loop(); 
    int zonevalue1 = TofSensor::getZone1();                                         // Get the latest values
    int zonevalue2 = TofSensor::getZone2();
    zoneBaselines[0] += zonevalue1;
    zoneBaselines[1] += zonevalue2;
    if (TofSensor::getZone1() >= zonevalue1 + PERSON_THRESHOLD || TofSensor::getZone2() >= zonevalue2 + PERSON_THRESHOLD){
      Log.info("Target zone not clear - will wait 5 seconds and try again");
      delay(5000);
      TofSensor::loop();
      if (TofSensor::getZone1() >= zonevalue1 + PERSON_THRESHOLD || TofSensor::getZone2() >= zonevalue2 + PERSON_THRESHOLD) return FALSE;
    }
  }
  zoneBaselines[0] = zoneBaselines[0]/NUM_CALIBRATION_LOOPS;
  zoneBaselines[1] = zoneBaselines[1]/NUM_CALIBRATION_LOOPS;
  Log.info("Target zone is clear with zone1 at %ikcps/SPAD and zone2 at %ikcps/SPAD",zoneBaselines[0],zoneBaselines[1]);
  return TRUE;
}

int TofSensor::loop(){                         // This function will update the current distance / occupancy for each zone.  It will return true if occupancy changes                    
  byte ready = 0;
  occupancyState = 0;
  unsigned long startedRanging;

  for (byte samples = 0; samples < 6; samples++){
    int zone = samples % 2;
    myTofSensor.stopRanging();
    myTofSensor.clearInterrupt();
    myTofSensor.setROI(ROWS_OF_SPADS,COLUMNS_OF_SPADS,opticalCenters[zone]);
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
    #if DEBUG_COUNTER
    Log.info("Zone%d (%dx%d %d SPADs with optical center %d) = %ikcps/SPAD. Signal/SPAD: %d Ambient/SPAD: %d",zone+1,myTofSensor.getROIX(), myTofSensor.getROIY(), myTofSensor.getSpadNb(),opticalCenters[zone],lastSignal[zone],myTofSensor.getSignalPerSpad(), myTofSensor.getAmbientPerSpad());
    #endif
    zoneSignalPerSpad[zone][samples % 3] = lastSignal[zone];
  }
  occupancyState += ((zoneSignalPerSpad[0][0] >= (zoneBaselines[0] + PERSON_THRESHOLD) && zoneSignalPerSpad[0][1] >= (zoneBaselines[0] + PERSON_THRESHOLD) && zoneSignalPerSpad[0][2] >= (zoneBaselines[0] + PERSON_THRESHOLD))
                    || (zoneSignalPerSpad[0][0] <= (zoneBaselines[0] - PERSON_THRESHOLD) || zoneSignalPerSpad[0][1] <= (zoneBaselines[0] - PERSON_THRESHOLD) || zoneSignalPerSpad[0][2] <= (zoneBaselines[0] - PERSON_THRESHOLD))) ? 1 : 0;
  
  occupancyState += ((zoneSignalPerSpad[1][0] >= (zoneBaselines[1] + PERSON_THRESHOLD) && zoneSignalPerSpad[1][1] >= (zoneBaselines[0] + PERSON_THRESHOLD) && zoneSignalPerSpad[1][2] >= (zoneBaselines[0] + PERSON_THRESHOLD))
                    || (zoneSignalPerSpad[1][0] <= (zoneBaselines[1] - PERSON_THRESHOLD) || zoneSignalPerSpad[1][1] <= (zoneBaselines[1] - PERSON_THRESHOLD) || zoneSignalPerSpad[1][2] <= (zoneBaselines[1] - PERSON_THRESHOLD))) ? 2 : 0;
  
  ready = 1;

  return (ready);     // Let us know if the occupancy state has progressed in the algorithm.
}

int TofSensor::getZone1() {
  return lastSignal[0];
}

int TofSensor::getZone2() {
  return lastSignal[1];
}

int TofSensor::getOccupancyState() {
  return occupancyState;
}



