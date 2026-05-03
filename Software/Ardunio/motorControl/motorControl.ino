#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver* drivers[2] = {&pwm1, &pwm2};

const bool stepSequence[8][4] = {
  {1, 0, 0, 0}, {1, 0, 1, 0}, {0, 0, 1, 0}, {0, 1, 1, 0},
  {0, 1, 0, 0}, {0, 1, 0, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// --- TIMING CONSTANTS ---
const unsigned long TIMESTEP_MICROS = 100000; // 100ms timestep window
const unsigned long MIN_SAFE_DELAY_MICROS = 2000; // Physical speed limit to prevent stalling

// --- POWER LEVELS (0 to 4095) ---
const int MOVE_POWER = 4000; // ~97% power for moving
const int HOLD_POWER = 1228; // ~30% power for holding position

// --- UPGRADED MOTOR STRUCTURE ---
struct StepperMotor {
  String name;
  uint8_t driverIndex;
  uint8_t baseChannel;
  float gearRatio;
  float stepAngle; 
  
  long currentPositionHalfSteps;
  long targetPositionHalfSteps; 
  
  int currentStepPhase;
  unsigned long lastStepTimeMicros; 
  unsigned long currentDelayMicros; 
  
  bool isMoving;
};

// --- MOTOR CONFIGURATIONS ---
StepperMotor motors[7] = {
  // name, drv, chan, gear, stepAngle, currPos, targPos, phase, lastMicros, delayMicros, moving
  {"J1", 0, 0,  10.0, 0.225, 0, 0, 0, 0, 0, false}, 
  {"J2", 0, 4,  42.0, 0.225, 0, 0, 0, 0, 0, false}, 
  {"J3", 0, 8,  14.0, 0.225, 0, 0, 0, 0, 0, false}, 
  {"J4", 1, 12,  3.0, 0.900, 0, 0, 0, 0, 0, false}, 
  {"J5", 1, 8,   1.0, 0.900, 0, 0, 0, 0, 0, false}, 
  {"J6", 1, 4,   3.0, 0.900, 0, 0, 0, 0, 0, false}, 
  {"J7", 1, 0,   1.0, 0.900, 0, 0, 0, 0, 0, false}  
};

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // Very fast timeout for rapid serial reading
  
  pwm1.begin();
  pwm1.setPWMFreq(1000); 
  pwm2.begin();
  pwm2.setPWMFreq(1000); 
  Wire.setClock(400000);
  delay(10);
  
  for (int i = 0; i < 7; i++) motorOff(motors[i]);

  Serial.println("System Ready. Send 7 comma-separated angles (e.g., 10,20.5,-5,0,90,45,0)");
}

void setLogicState(StepperMotor &motor, uint8_t channelOffset, bool isHigh) {
  uint8_t targetPin = motor.baseChannel + channelOffset;
  if (isHigh) {
    int currentPower = motor.isMoving ? MOVE_POWER : HOLD_POWER;
    drivers[motor.driverIndex]->setPWM(targetPin, 0, currentPower); 
  } else {
    drivers[motor.driverIndex]->setPWM(targetPin, 0, 4096); 
  }
}

void stepMotor(StepperMotor &motor, int dir) {
  motor.currentStepPhase += dir;
  if (motor.currentStepPhase > 7) motor.currentStepPhase = 0;
  if (motor.currentStepPhase < 0) motor.currentStepPhase = 7;

  setLogicState(motor, 0, stepSequence[motor.currentStepPhase][0]);
  setLogicState(motor, 1, stepSequence[motor.currentStepPhase][1]);
  setLogicState(motor, 2, stepSequence[motor.currentStepPhase][2]);
  setLogicState(motor, 3, stepSequence[motor.currentStepPhase][3]);
}

void motorOff(StepperMotor &motor) {
  setLogicState(motor, 0, false); setLogicState(motor, 1, false);
  setLogicState(motor, 2, false); setLogicState(motor, 3, false);
}

// --- NEW MULTI-AXIS TARGET PARSER & TIMING CALCULATOR ---
void parseAndExecutePositions(String input) {
  float targetAngles[7];
  int commaIndex;
  
  // Parse comma-separated string into float array
  for (int i = 0; i < 6; i++) {
    commaIndex = input.indexOf(',');
    if (commaIndex == -1) return; // Malformed string
    targetAngles[i] = input.substring(0, commaIndex).toFloat();
    input = input.substring(commaIndex + 1);
  }
  targetAngles[6] = input.toFloat(); // Get the last value

  // Apply targets and calculate dynamic speeds
  unsigned long moveStartTime = micros();
  
  for (int i = 0; i < 7; i++) {
    float baseHalfSteps = targetAngles[i] / motors[i].stepAngle;
    long newTarget = round(baseHalfSteps * motors[i].gearRatio);
    
    if (newTarget != motors[i].currentPositionHalfSteps) {
      motors[i].targetPositionHalfSteps = newTarget;
      
      long stepsRequired = abs(motors[i].targetPositionHalfSteps - motors[i].currentPositionHalfSteps);
      
      // Calculate delay to fit perfectly within the 100ms timestep
      motors[i].currentDelayMicros = TIMESTEP_MICROS / stepsRequired;
      
      // Safety limits - prevent the motor from trying to step faster than physically possible
      if (motors[i].currentDelayMicros < MIN_SAFE_DELAY_MICROS) {
         motors[i].currentDelayMicros = MIN_SAFE_DELAY_MICROS;
      }
      
      motors[i].isMoving = true;
      motors[i].lastStepTimeMicros = moveStartTime; 
      
      stepMotor(motors[i], 0); // Energize coils at MOVE_POWER
    } else if (motors[i].isMoving) {
      // If commanded to stay still, drop to hold power
      motors[i].isMoving = false;
      stepMotor(motors[i], 0); 
    }
  }
}

// --- CONSTANT VELOCITY BACKGROUND ENGINE ---
void updateAllMotors() {
  unsigned long currentMicros = micros();

  for (int i = 0; i < 7; i++) {
    if (motors[i].isMoving) {
      long stepsRemaining = motors[i].targetPositionHalfSteps - motors[i].currentPositionHalfSteps;
      
      if (stepsRemaining != 0) {
        if (currentMicros - motors[i].lastStepTimeMicros >= motors[i].currentDelayMicros) {
          
          int dir = (stepsRemaining > 0) ? 1 : -1;
          stepMotor(motors[i], dir);
          
          motors[i].currentPositionHalfSteps += dir;
          
          // Add delay directly to last step time to prevent timing drift over the 100ms window
          motors[i].lastStepTimeMicros += motors[i].currentDelayMicros; 
        }
      } 
      else {
        // Motor has arrived exactly on time
        motors[i].isMoving = false; 
        stepMotor(motors[i], 0); // Drop coils to HOLD_POWER (1228)
      }
    }
  }
}

void loop() {
  updateAllMotors(); // Runs constantly to pulse the motors

  // Check for incoming trajectories
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    if (input.length() > 0) {
      parseAndExecutePositions(input);
    }
  }
}