#include "PoseTracker.h"
#include <Wire.h>

PoseTracker::PoseTracker(double alphaImuFilterIn, int baseStationModeIn, bool simulateLighthouseIn) :

  OrientationTracker(alphaImuFilterIn, false),
  lighthouse(),
  simulateLighthouse(simulateLighthouseIn),
  simulateLighthouseCounter(0),
  position{0, 0, -500},
  baseStationPitch(0),
  baseStationRoll(0),
  baseStationMode(baseStationModeIn),
  position2D{0,0,0,0, 0,0,0,0},
  clockTicks{0,0,0,0, 0,0,0,0},
  numPulseDetections{0,0,0,0, 0,0,0,0},
  pulseWidth{0,0,0,0,0,0,0,0}

  {

}

int PoseTracker::processLighthouse() {

  if (simulateLighthouse) {
  //if in simulation mode, get data from external file
    for (int i = 0; i < 8; i++) {
      clockTicks[i] = clockTicksData[(simulateLighthouseCounter*8 + i) % nLighthouseSamples];
      numPulseDetections[i] = 0;
    }

    //base station pitch/roll values remain the same throughout the simulation
    if (simulateLighthouseCounter == 0) {
      baseStationPitch = baseStationPitchSim;
      baseStationRoll = baseStationRollSim;
    }

    //data wraps around after end of array is reached
    simulateLighthouseCounter = (simulateLighthouseCounter + 1) % nLighthouseSamples;

    //slight delay to simulate delay between sensor readings (not exactly 120 Hz)
    delay(1);

  } else {
    //check data is available
    if (!lighthouse.readTimings(baseStationMode, clockTicks, numPulseDetections, pulseWidth,
      baseStationPitch, baseStationRoll)) {
      return -2;
    }

    //check that all diodes have detections
    for (int i = 0; i < 8; i++) {
      if (numPulseDetections[i] == 0) {
        return -1;
      }
    }
  }

  return updatePose();

}


/**
 * Use the functions in PoseMath.h to get from clockTicks, to a new
 * position and quaternion estimate, in the base station frame, where
 * y is the normal of the top face of the base station, z points to the back
 * You should not do any math here; use the functions in PoseMath.h.
 *
 * You will need to access the following fields defined in this class:
 *  - clockTicks
 *  - position2D
 *  - positionRef
 *  - position
 *  - quaternionHm
 *
 * The position and quaternionHm variables should be updated to the
 * new estimate.
 *
 * @returns  0:if any errors occur (eg failed matrix inversion),
 *           1: if successful.
 */
int PoseTracker::updatePose() {

  // call functions in PoseMath.cpp to get a new position 
  // and orientation estimate. 
  //
  // you will need to use the following class variables:
  // - clockTicks
  // - position2D
  // - positionRef
  // - position
  // - quaternionHm
  // 
  // - position and quaternionHm should hold the your position
  // and orientation estimates at the end of this function
  //
  // return 0 if errors occur, return 1 if successful
  
  double A_matrix[8][8];
  double H_vector[8];
  double R_matrix[3][3];
  
  convertTicksTo2DPositions(clockTicks, position2D);
  formA(position2D, positionRef, A_matrix);
  if(!solveForH(A_matrix, position2D, H_vector)) return 0;
  getRtFromH(H_vector, R_matrix, position);
  quaternionHm = getQuaternionFromRotationMatrix(R_matrix);

  return 1;

}
