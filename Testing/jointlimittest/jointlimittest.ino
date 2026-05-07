#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver* drivers[2] = {&pwm1, &pwm2};

const bool stepSequence[8][4] = {
  {1, 0, 0, 0}, {1, 0, 1, 0}, {0, 0, 1, 0}, {0, 1, 1, 0},
  {0, 1, 0, 0}, {0, 1, 0, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

g
const unsigned long MIN_DELAY_MICROS = 15000;  
const unsigned long MAX_DELAY_MICROS = 60000; 
const long MAX_ACCEL_STEPS = 400;             


const int MOVE_POWER = 4000; 
const int HOLD_POWER = 1228;


struct StepperMotor {
  String name;
  uint8_t driverIndex;
  uint8_t baseChannel;
  float gearRatio;
  float stepAngle; 
  
  long currentPositionHalfSteps;
  long targetPositionHalfSteps; 
  unsigned long totalStepsMoved; 
  

  long startPositionHalfSteps; 
  long accelSteps;             
  
  int currentStepPhase;
  unsigned long lastStepTimeMicros; 
  unsigned long currentDelayMicros; 
  
  bool isMoving;
};

StepperMotor motors[7] = {
  // name, drv, chan, gear, stepAngle, currPos, targPos, totalSteps, startPos, accelStp, phase, lastMicros, currMicros, moving
  {"J1", 0, 0,  10.0, 0.225, 0, 0, 0, 0, 0, 0, 0, 0, false}, // 4x smaller step size
  {"J2", 0, 4,  42.0, 0.225, 0, 0, 0, 0, 0, 0, 0, 0, false}, // 4x smaller step size
  {"J3", 0, 8,  14.0, 0.225, 0, 0, 0, 0, 0, 0, 0, 0, false}, // 4x smaller step size
  {"J4", 1, 12,  3.0, 0.900, 0, 0, 0, 0, 0, 0, 0, 0, false}, // Standard step size
  {"J5", 1, 8,   1.0, 0.900, 0, 0, 0, 0, 0, 0, 0, 0, false}, 
  {"J6", 1, 4,   3.0, 0.900, 0, 0, 0, 0, 0, 0, 0, 0, false}, 
  {"J7", 1, 0,   1.0, 0.900, 0, 0, 0, 0, 0, 0, 0, 0, false}  
};

enum InputState { WAITING_FOR_MOTOR, WAITING_FOR_ANGLE };
InputState currentState = WAITING_FOR_MOTOR;
int selectedMotorIndex = -1; 

void setup() {
 
  Serial.setTimeout(50); 
  
  Serial.println("--- 7-Axis Stepper Control (Accel, Microstep & Holding Fixes) ---");

  pwm1.begin();
  pwm1.setPWMFreq(1000); 
  pwm2.begin();
  pwm2.setPWMFreq(1000); 
  Wire.setClock(400000);
  delay(10);
  
  for (int i = 0; i < 7; i++) motorOff(motors[i]);

  Serial.println("System Ready.");
  Serial.println("\nWhich motor? (1-7):");
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

void releaseOtherMotors(StepperMotor &activeMotor) {
  for (int i = 0; i < 7; i++) {
    if (&motors[i] != &activeMotor && !motors[i].isMoving) {
      motorOff(motors[i]);
    }
  }
}


void setTargetAngle(StepperMotor &motor, float targetAngle) {
  float baseHalfSteps = targetAngle / motor.stepAngle;
  long newTarget = round(baseHalfSteps * motor.gearRatio);
  
  if (newTarget != motor.currentPositionHalfSteps) {
    
    releaseOtherMotors(motor);
    
    /
    motor.startPositionHalfSteps = motor.currentPositionHalfSteps;
    motor.targetPositionHalfSteps = newTarget;
    
    
    long totalSteps = abs(motor.targetPositionHalfSteps - motor.startPositionHalfSteps);
    motor.accelSteps = totalSteps / 3; 
    if (motor.accelSteps > MAX_ACCEL_STEPS) motor.accelSteps = MAX_ACCEL_STEPS;
    
    motor.isMoving = true;
    motor.currentDelayMicros = MAX_DELAY_MICROS; 
    motor.lastStepTimeMicros = micros();
    
    stepMotor(motor, 0); 
    
    Serial.print(motor.name);
    Serial.print(" target set to ");
    Serial.println(targetAngle);
  } else {
    Serial.println("Already at target. Holding torque maintained.");
  }
}


void updateAllMotors() {
  unsigned long currentMicros = micros();

  for (int i = 0; i < 7; i++) {
    long stepsRemaining = motors[i].targetPositionHalfSteps - motors[i].currentPositionHalfSteps;
    
    if (stepsRemaining != 0) {
      
      if (currentMicros - motors[i].lastStepTimeMicros >= motors[i].currentDelayMicros) {
        
        int dir = (stepsRemaining > 0) ? 1 : -1;
        stepMotor(motors[i], dir);
        
        motors[i].currentPositionHalfSteps += dir;
        motors[i].totalStepsMoved++; // 
        motors[i].lastStepTimeMicros = currentMicros; 
        
        /
        long stepsMoved = abs(motors[i].currentPositionHalfSteps - motors[i].startPositionHalfSteps);
        long absRemaining = abs(motors[i].targetPositionHalfSteps - motors[i].currentPositionHalfSteps);
        
        if (stepsMoved < motors[i].accelSteps) {
          
          float progress = (float)stepsMoved / motors[i].accelSteps;
          motors[i].currentDelayMicros = MAX_DELAY_MICROS - (progress * (MAX_DELAY_MICROS - MIN_DELAY_MICROS));
        } 
        else if (absRemaining < motors[i].accelSteps) {
          
          float progress = (float)absRemaining / motors[i].accelSteps;
          motors[i].currentDelayMicros = MAX_DELAY_MICROS - (progress * (MAX_DELAY_MICROS - MIN_DELAY_MICROS));
        } 
        else {
          
          motors[i].currentDelayMicros = MIN_DELAY_MICROS;
        }
      }
    } 
    else if (motors[i].isMoving) {
      motors[i].isMoving = false; 
      
     
      
      stepMotor(motors[i], 0); 
      releaseOtherMotors(motors[i]);
      
      Serial.print("\n>>> "); Serial.print(motors[i].name);
      Serial.println(" arrived and holding position.");
      
    
      Serial.print(">>> Total steps moved so far (Odometer): ");
      Serial.println(motors[i].totalStepsMoved);
      
      Serial.println("Which motor next? (1-7):");
    }
  }
}

void loop() {
  updateAllMotors();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    if (input.length() == 0) return; 
    
    if (currentState == WAITING_FOR_MOTOR) {
      int mNum = input.toInt();
      if (mNum >= 1 && mNum <= 7) {
        selectedMotorIndex = mNum - 1;
        Serial.print("Selected "); Serial.print(motors[selectedMotorIndex].name);
        Serial.println(". Enter target angle (degrees):");
        currentState = WAITING_FOR_ANGLE; 
      } else {
        Serial.println("Invalid input. Which motor? (1-7):");
      }
    } 
    else if (currentState == WAITING_FOR_ANGLE) {
      float targetDegree = input.toFloat();
      setTargetAngle(motors[selectedMotorIndex], targetDegree);
      
      currentState = WAITING_FOR_MOTOR;
      Serial.println("\nCommand sent! Which motor next? (1-7):");
    }
  }
}