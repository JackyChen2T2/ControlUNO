#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <math.h>

// Instance of Pixy
Pixy2 pixy;

// Angle convertion constants
const double D2R = 0.01745329252;
const double R2D = 57.29577951;


// Pixy image-side constants
const double RESOLUTION_X = 315;
const double FOV_X_DEGREE = 59.58;
const double MIDPOINT_X = RESOLUTION_X / 2;
const double FOCAL_LENGTH_FOR_X = RESOLUTION_X / (2 * tan(FOV_X_DEGREE/2 * D2R));

const double RESOLUTION_Y = 207;
const double FOV_Y_DEGREE = 41.12;
const double MIDPOINT_Y = RESOLUTION_Y / 2;
const double FOCAL_LENGTH_FOR_Y = RESOLUTION_Y / (2 * tan(FOV_Y_DEGREE/2 * D2R));

const int iPINK = 0;
const int iGREEN = 1;
const int iBLUE = 2;
const int iORANGE = 3;


// Pixy object-side constants
const double CARD_SIDE = 0.075;
const int PINK = 1;
const int GREEN = 3;
const int BLUE = 4;
const int ORANGE = 2;


// Pixy image block and object variables
int numberOfBlocks = 0;
int blockIndexOf [4];
double blockX [4];
double blockY [4];
double blockWidth [4];
double blockHeight [4];
double objectX [4];
double objectY [4];
double objectRange [4];
double averageRange = 0;
double gapAngle [6];
double averageAngle = 0;


// Rover state, control variables and state constants
int thisBlock = 0;
int blockSignature = 0;
int i = 0;
int gap = 0;
int state = 0;
int attempt = 0;
bool hasCompleted = false;
const int STOPPED = 0;
const int LONG_RANGE = 1;
const int SHORT_RANGE = 2;
const int SLIDING = 3;
const int ROTATING = 4;
const int DRIVING = 5;
const int ELEVATING = 6;
const int INSERTING = 7;
const int COMPLETED = 8;
const int LOST = 9;
const int FAILURE = 10;


void setup() {
  Serial.begin(9600);

// Initialize Pixy camera
  pixy.init();
}


void loop() {
// Placeholder: start the robot and begin LONG RANGE detection automatically
// Next: LONG_RANGE
  if (state == STOPPED) {
    state = LONG_RANGE;
  }
  
// Process Pixy2 data: at LONG RANGE (at least 20cm away), expect 4 small Pixy2 blocks
// Next: LONG_RANGE, SHORT_RANGE, SLIDING, ROTATING, DRIVING, ELEVATING, LOST
  else if (state == LONG_RANGE) {
    ClearBlocksAndObjects();
    state = LongRangeAnalysis();
    Serial.println();
    delay(6000);
  }
  
// Process Pixy2 data: at SHORT RANGE (within 20cm), expect 2 large Pixy2 blocks
// Next: SHORT_RANGE, LONG_RANGE, SLIDING, ROTATING, DRIVING, ELEVATING, INSERTING, COMPLETED, LOST
  else if (state == SHORT_RANGE) {
    ClearBlocksAndObjects();
    state = ShortRangeAnalysis();
    Serial.println();
    delay(6000);
  }
  
// Placeholder: SLIDING command
// Next: LONG_RANGE, SHORT_RANGE
  else if (state == SLIDING) {
    Serial.println("Command: SLIDE to align with the Y-Z plane that contains the charger port vector...");
    Serial.println();
    delay(6000);
    attempt = 0;
    if (averageRange > 0.2) {
      state = LONG_RANGE;
    }
    else {
      state = SHORT_RANGE;
    }
  }

// Placeholder: ROTATING command
// Next: LONG_RANGE, SHORT_RANGE
  else if (state == ROTATING) {
    Serial.println("Command: ROTATE to align towards the charger port...");
    Serial.println();
    delay(6000);
    attempt = 0;
    if (averageRange > 0.2) {
      state = LONG_RANGE;
    }
    else {
      state = SHORT_RANGE;
    }
  }
  
// Placeholder: DRIVING command
// Next: LONG_RANGE, SHORT_RANGE
  else if (state == DRIVING) {
    Serial.println("Command: DRIVE forward until a state change is required...");
    Serial.println();
    delay(6000);
    attempt = 0;
    if (averageRange > 0.2) {
      state = LONG_RANGE;
    }
    else {
      state = SHORT_RANGE;
    }
  }
  
// Placeholder: ELEVATING command
// Next: LONG_RANGE, SHORT_RANGE
  else if (state == ELEVATING) {
    Serial.println("Command: ELEVATE to align with the X-Y plane that contains the charger port vector...");
    Serial.println();
    delay(6000);
    attempt = 0;
    if (averageRange > 0.2) {
      state = LONG_RANGE;
    }
    else {
      state = SHORT_RANGE;
    }
  }
  
// Placeholder: INSERTING command, consisting of an unlocking command and the DRIVING command with a constant parameter.
// Next: COMPLETED, LOST
  else if (state == INSERTING) {
    Serial.println("Command: INSERT the charger port - funnel unlocked, driving forward...");
    Serial.println();
    delay(6000);
    attempt = 0;
    ClearBlocksAndObjects();
    if (true) {
      state = COMPLETED;
    }
    else {
      state = LOST;
    }
  }
  
// Placeholder: COMPLETED command, wait, disengage the clip, DRIVE backward for a preset distance and STOP.
// Next: STOPPED, LOST
  else if (state == COMPLETED) {
    Serial.println("Command: COMPLETE the time hold, clip unlocked, returning...");
    Serial.println();
    delay(6000);
    attempt = 0;
    ClearBlocksAndObjects();
    if (true) {
      state = STOPPED;
    }
    else {
      state = LOST;
    }
  }
  
// Placeholder: LOST procedure, if has not COMPLETED, reverse to the previous state.
// Next: LONG_RANGE, STOPPED
  else if (state == LOST) {
    Serial.println("Command: LOST recovery...");
    Serial.println();
    delay(6000);
    attempt = 0;
    if (hasCompleted) {
      state = STOPPED;
    }
    else {
      state = LONG_RANGE;
    }
  }
  
// FATAL ERROR
  else if (state == FAILURE) {
    Serial.println("State-10 Fatal Error.");
    Serial.println();
    delay(60000);
  }
  
// UNEXPECTED ERROR
  else {
    Serial.println("Unknown State Error.");
    Serial.println(state);
    Serial.println();
    delay(60000);
  }
}


// Clear the garbage data and analyses from the previous attempt.
void ClearBlocksAndObjects() {
  attempt += 1;
  numberOfBlocks = 0;
  averageRange = 0;
  averageAngle = 0;
  for (i = 0; i < 4; i++)
    {
      blockIndexOf[i] = 0;
      blockX[i] = 0;
      blockY[i] = 0;
      blockWidth[i] = 0;
      blockHeight[i] = 0;
      objectX[i] = 0;
      objectY[i] = 0;
      objectRange[i] = 0;
    }
  for (i = 0; i<6; i++) {
    gapAngle[i] = 0;
  }
  Serial.println("Previous data cleared.");
}


int LongRangeAnalysis() {
  Serial.println("Command: LONG RANGE detection...");
  if (attempt > 3) {
    Serial.println("Continuous failure, proceed to LOST state.");
    return LOST;
  }
  
  numberOfBlocks = pixy.ccc.getBlocks();
  if (numberOfBlocks != 4) {
    Serial.print("Failure: blocks detection");
    return LONG_RANGE;
  } else {
    for (thisBlock = 0; thisBlock < 4; thisBlock++) {
      blockSignature = pixy.ccc.blocks[thisBlock].m_signature;
      if (blockSignature == PINK) {
        blockIndexOf[iPINK] = thisBlock;
      }
      else if (blockSignature == ORANGE) {
        blockIndexOf[iORANGE] = thisBlock;
      }
      else if (blockSignature == GREEN) {
        blockIndexOf[iGREEN] = thisBlock;
      }
      else if (blockSignature == BLUE) {
        blockIndexOf[iBLUE] = thisBlock;
      }
      else {
        Serial.print("Failure: unknown signature.");
        return LONG_RANGE;
      }
    }
    attempt = 0;
    for (i = iPINK; i <= iORANGE; i++) {
      
      if (i == iPINK) {
        Serial.println("Pink Report:");
      }
      else if (i == iGREEN) {
        Serial.println("Green Report:");
      }
      else if (i == iBLUE) {
        Serial.println("Blue Report:");
      }
      else {
        Serial.println("Orange Report:");
      }
      
      blockX[i] = pixy.ccc.blocks[blockIndexOf[i]].m_x;
      blockY[i] = pixy.ccc.blocks[blockIndexOf[i]].m_y;
      blockWidth[i] = pixy.ccc.blocks[blockIndexOf[i]].m_width;
      blockHeight[i] = pixy.ccc.blocks[blockIndexOf[i]].m_height;
      objectRange[i] = (FOCAL_LENGTH_FOR_X / blockWidth[i] + FOCAL_LENGTH_FOR_Y / blockHeight[i]) / 2 * CARD_SIDE;
      Serial.print("Range:\t");
      Serial.print(objectRange[i]);
      objectX[i] = objectRange[i] * (blockX[i] - MIDPOINT_X) / FOCAL_LENGTH_FOR_X;
      Serial.print("\tHorizontal Displacement:\t");
      Serial.print(objectX[i]);
      objectY[i] = objectRange[i] * (blockY[i] - MIDPOINT_Y) / FOCAL_LENGTH_FOR_Y;
      Serial.print("\tVertical Displacement:\t");
      Serial.println(objectY[i]);
      averageRange += objectRange[i];
    }
    averageRange = averageRange / 4;
    
    gapAngle[0] = atan((objectRange[iGREEN]-objectRange[iPINK]) / (objectX[iGREEN]-objectX[iPINK]));
    averageAngle += gapAngle[0];
    Serial.println();
    gapAngle[1] = atan((objectRange[iBLUE]-objectRange[iGREEN]) / (objectX[iBLUE]-objectX[iGREEN]));
    averageAngle += gapAngle[1];
    gapAngle[2] = atan((objectRange[iORANGE]-objectRange[iBLUE]) / (objectX[iORANGE]-objectX[iBLUE]));
    averageAngle += gapAngle[2];
    gapAngle[3] = atan((objectRange[iBLUE]-objectRange[iPINK]) / (objectX[iBLUE]-objectX[iPINK]));
    averageAngle += gapAngle[3];
    gapAngle[4] = atan((objectRange[iORANGE]-objectRange[iGREEN]) / (objectX[iORANGE]-objectX[iGREEN]));
    averageAngle += gapAngle[4];
    gapAngle[5] = atan((objectRange[iORANGE]-objectRange[iPINK]) / (objectX[iORANGE]-objectX[iPINK]));
    averageAngle += gapAngle[5];

    averageAngle = averageAngle / 6 * R2D;
    
    Serial.println("READABLE:");
    Serial.print("RANGE:\t");
    Serial.print(averageRange);
    Serial.println(" m");
    Serial.print("ANGLE:\t");
    Serial.print(averageAngle);
    Serial.println(" degrees");

//LONG_RANGE, SLIDING, ROTATING, DRIVING, ELEVATING, LOST
    if(averageRange < 0.3){
      Serial.println("Command Placeholder: INSERTING...");
      return SHORT_RANGE;
    }
    else if(abs(objectY[iPINK] + objectY[iGREEN] + objectY[iBLUE] + objectY[iORANGE])/4 > 0.05){
      return ELEVATING;
    }
    else if(averageAngle*(objectX[iPINK] + objectX[iGREEN] + objectX[iBLUE] + objectX[iORANGE])/4/abs(averageAngle) > 0.1){
      return SLIDING;
    }
    else if(abs(objectX[iPINK] + objectX[iGREEN] + objectX[iBLUE] + objectX[iORANGE])/4 > 0.1){
      return ROTATING;
    }
    else if ((abs(averageAngle) < 10) && (averageRange >0.3)){
      return DRIVING;
    }
    else
    {
      return LONG_RANGE;
    }
  }
  
}

// Placeholder: yet to differ from LONG RANGE detection. Require further testing.
int ShortRangeAnalysis() {
  state = LONG_RANGE;
}
