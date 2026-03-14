#define LEFT_PWM_PIN    6
#define LEFT_IN1_PIN    7
#define LEFT_IN2_PIN    8
#define RIGHT_PWM_PIN   9
#define RIGHT_IN3_PIN   13
#define RIGHT_IN4_PIN   12

#define BACK_TRIG_PIN   2
#define BACK_ECHO_PIN   3
#define LEFT_TRIG_PIN   10
#define LEFT_ECHO_PIN   11
#define RIGHT_TRIG_PIN  4
#define RIGHT_ECHO_PIN  5

#define SHOOT_PWM_PIN   16
#define SHOOT_IN1_PIN   15
#define SHOOT_IN2_PIN   14
#define SHOOT_ENCODER_A 18      // yellow

#define MOTOR_SPEED     126
#define ROTATE_DURATION 1000
#define PAUSE_DURATION  1000
#define PRINT_INTERVAL  100

#define BACK_THRESHOLD  30.0
#define SIDE_SUM_TARGET 101.3
#define SIDE_SUM_BUFFER 1.5

#define BASE_RIGHT      135      //95 , 110
#define BASE_LEFT       155    // 120 , 135
#define BACK_FAR_THRESH 160.0       

#define RETREAT_SIDE_SPEED      138
#define RETREAT_OPP_SIDE_SPEED  140

#define KP 0.7                // 20
#define KD 0 // tune this — start low 12

#define STROBE_ON_MS  500
#define STROBE_OFF_MS 500

bool          strobeActive = true;
unsigned long strobeTimer  = 0;


float    lastAlignSum  = 0.0;
bool     alignCW       = true;   // start CW
#define  ALIGN_SPEED   120
#define  ALIGN_TARGET  101.3

#define  ALIGN_SHOOT_BUFFER  0.5

#define  ALIGN_DIFF_BUFFER    5



float lastError   = 0.0;
volatile bool controlReady = false;

const long SHOOT_PPR          = 690;
const long SHOOT_COUNTS_90    = (SHOOT_PPR + 2) / 4;
const int  SHOOT_PWM_FAST     = 205;
const int  SHOOT_PWM_SLOW     = 15;
const long SHOOT_SLOWDOWN_WIN = 100;

static int  shootCount   = 0;
static bool shootPausing = false;
static unsigned long shootPauseStart = 0;


enum RobotState {
  STATE_ROTATE_CCW,
  STATE_ROTATE_CCW2,
  STATE_LINE_FOLLOW,
  STATE_SHOOT,
  STATE_RETREAT,
  STATE_LOAD,
  STATE_ALIGN_SHOOT,
  STATE_STOPPED,
};

enum RotatePhase {
  PHASE_SPINNING,
  PHASE_STOPPED,
  PHASE_SPINNING2,
  PHASE_STOPPED2,
};


RobotState currentState = STATE_ROTATE_CCW;


//RobotState currentState = STATE_ROTATE_CCW;
RotatePhase rotatePhase = PHASE_SPINNING;

//RobotState currentState = STATE_LINE_FOLLOW;

unsigned long phaseStartTime = 0;
unsigned long lastPrintTime  = 0;

unsigned long loadStartTime = 0;

float backDistance  = 999.0;
float leftDistance  = 999.0;
float rightDistance = 999.0;

float error       = 0.0;
int   motorLspeed = 0;
int   motorRspeed = 0;

String detectedSide = "";
bool   shootAfterAlign = false;

volatile long shootEncoderCount = 0;
static long   shootStartCount   = 0;
static bool   shootStarted      = false;

void onShootEncoderA_Rise() {
  shootEncoderCount++;
}

static long readShootEncoderAtomic() {
  noInterrupts();
  long c = shootEncoderCount;
  interrupts();
  return c;
}

bool isBackClose() {
  return (backDistance <= BACK_THRESHOLD);
}

bool isSideSumInRange() {
  float sum = leftDistance + rightDistance;
  return (sum >= SIDE_SUM_TARGET - SIDE_SUM_BUFFER && sum <= SIDE_SUM_TARGET + SIDE_SUM_BUFFER);
}

bool isBackFar() {
  return (backDistance >= BACK_FAR_THRESH);
}

bool isPhaseTimeUp(unsigned long duration) {
  return (millis() - phaseStartTime >= duration);
}

bool isPrintDue() {
  return (millis() - lastPrintTime >= PRINT_INTERVAL);
}


ISR(TIMER1_COMPA_vect) {
  controlReady = true;
}



void setup() {
  Serial.begin(9600);

  pinMode(LEFT_PWM_PIN,  OUTPUT);
  pinMode(LEFT_IN1_PIN,  OUTPUT);
  pinMode(LEFT_IN2_PIN,  OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_IN3_PIN, OUTPUT);
  pinMode(RIGHT_IN4_PIN, OUTPUT);

  pinMode(BACK_TRIG_PIN,  OUTPUT);
  pinMode(BACK_ECHO_PIN,  INPUT);
  pinMode(LEFT_TRIG_PIN,  OUTPUT);
  pinMode(LEFT_ECHO_PIN,  INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  pinMode(SHOOT_PWM_PIN,  OUTPUT);
  pinMode(SHOOT_IN1_PIN,  OUTPUT);
  pinMode(SHOOT_IN2_PIN,  OUTPUT);
  stopShootMotor();

  pinMode(SHOOT_ENCODER_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(SHOOT_ENCODER_A), onShootEncoderA_Rise, RISING);

  motorsStop();

  Serial.println(F("turty ready!"));
  Serial.println(F("Entering Rotate CCW 1"));
  Serial.print(F("Shoot COUNTS_90="));
  Serial.println(SHOOT_COUNTS_90);

  enterSpinPhase();
  lastPrintTime = millis();

  // Timer1: CTC mode, prescaler 64, 100Hz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 2499;                          // 16MHz / 64 / 100Hz - 1
  TCCR1B |= (1 << WGM12);                // CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);  // prescaler 64
  TIMSK1 |= (1 << OCIE1A);              // enable interrupt
  sei();
}

void updateStrobe() {
  if (strobeActive && millis() - strobeTimer >= STROBE_ON_MS) {
    strobeActive = false;
    strobeTimer  = millis();
    motorsStop();
  } else if (!strobeActive && millis() - strobeTimer >= STROBE_OFF_MS) {
    strobeActive = true;
    strobeTimer  = millis();
  }
}

void loop() {
  updateStrobe();   // <-- add this line
  switch (currentState) {
    case STATE_ROTATE_CCW:
      handleRotateCCW();
      break;
    case STATE_ROTATE_CCW2:
      handleRotateCCW2();
      break;
    case STATE_LINE_FOLLOW:
      handleLineFollow();
      break;
    case STATE_SHOOT:
      handleShoot();
      break;
    case STATE_RETREAT:
      handleRetreat();
      break;
    case STATE_LOAD:
      handleLoad();
      break;
    case STATE_ALIGN_SHOOT:
      handleAlignShoot();
      break;
    case STATE_STOPPED:
      handleStopped();
      break;
  }
}

void onBackDetected() {
  motorsStop();
  Serial.println(F(">>> BACK SENSOR TRIGGERED – object <= 30 cm"));
  Serial.print(F("    Back distance: "));
  Serial.print(backDistance);
  Serial.println(F(" cm"));

  shootAfterAlign = false;
  currentState = STATE_ROTATE_CCW2;
  Serial.println(F("Entering State 2: Rotate CCW2 – Side Detection"));
  enterSpin2Phase();
}

void enterSpinPhase() {
  rotatePhase    = PHASE_SPINNING;
  phaseStartTime = millis();
  rotateCCW(MOTOR_SPEED);
  Serial.println(F("[CCW] Spinning..."));
}

void enterStopPhase() {
  rotatePhase    = PHASE_STOPPED;
  phaseStartTime = millis();
  motorsStop();
  Serial.println(F("[CCW] Paused."));
}

void enterSpin2Phase() {
  rotatePhase    = PHASE_SPINNING2;
  phaseStartTime = millis();
  rotateCCW(MOTOR_SPEED);
  Serial.println(F("[CCW2] Spinning..."));
}

void enterStop2Phase() {
  rotatePhase    = PHASE_STOPPED2;
  phaseStartTime = millis();
  motorsStop();
  Serial.println(F("[CCW2] Paused."));
}

void onSideDetected() {                                                                                                                                                                                                                                               
  motorsStop();

  if (leftDistance < rightDistance) {
    detectedSide = "left";
  } else {
    detectedSide = "right";
  }

  Serial.println(F(">>> SIDE SUM IN RANGE – aligned with wall"));
  Serial.print(F("    Left: "));  Serial.print(leftDistance);
  Serial.print(F(" cm | Right: ")); Serial.print(rightDistance);
  Serial.print(F(" cm | Sum: ")); Serial.println(leftDistance + rightDistance);
  Serial.print(F("    Closer side: ")); Serial.println(detectedSide);

  if (shootAfterAlign) {
    shootAfterAlign = false;
    currentState  = STATE_SHOOT;
    shootStarted  = false;
    shootCount    = 0;
    shootPausing  = false;
    Serial.println(F("Entering State: SHOOT (post-align)"));
  } else {
    currentState = STATE_LINE_FOLLOW;
    Serial.println(F("Entering State 3: Line Following (P-control)"));
  }
}

void onBackFarDetected() {
  // use the sum of both sides to align the bot
  motorsStop();
  Serial.println(F(">>> BACK SENSOR FAR – aligning for SHOOT"));

  lastAlignSum = leftDistance + rightDistance;
  alignCW      = true;
  lastError    = 0.0;

  currentState = STATE_ALIGN_SHOOT;
  Serial.println(F("Entering STATE_ALIGN_SHOOT"));

/* 
  motorsStop();
  Serial.println(F(">>> BACK SENSOR FAR – back >= 140 cm"));
  Serial.print(F("    Back distance: "));
  Serial.print(backDistance);
  Serial.println(F(" cm"));

  rotateCW(MOTOR_SPEED);        // <-- turn CW
  delay(250);                   // <-- for 250ms
  motorsStop();                 // <-- then stop

*/

/*
  shootAfterAlign = true;
  currentState = STATE_ROTATE_CCW2;
  Serial.println(F("Entering State: ROTATE_CCW2 – align before SHOOT"));
  enterSpin2Phase();

  currentState  = STATE_SHOOT;
  shootStarted  = false;
  shootCount    = 0;
  shootPausing  = false;
  Serial.println(F("Entering State: SHOOT"));
*/


}

void handleAlignShoot() {
  readAllSensors();

  float sum = leftDistance + rightDistance;

  // Stop if within target range
  if (sum >= ALIGN_TARGET - ALIGN_SHOOT_BUFFER && sum <= ALIGN_TARGET + ALIGN_SHOOT_BUFFER) {   // low
    motorsStop();
    Serial.print(F(">>> ALIGN DONE – sum: "));
    Serial.println(sum);

    
    currentState = STATE_SHOOT;
    shootStarted = false;
    shootCount   = 0;
    shootPausing = false;
    Serial.println(F("Entering STATE_SHOOT"));
    return;
  }

  // If sum increased since last check, we're going the wrong way — flip direction
  if (sum - lastAlignSum > ALIGN_DIFF_BUFFER) {
    alignCW = !alignCW;
    Serial.print(F("  [ALIGN] Sum increased ("));
    Serial.print(sum);
    Serial.println(F(") – flipping direction"));
  }
  lastAlignSum = sum;

  if (strobeActive) {
    if (alignCW) rotateCW(ALIGN_SPEED);
    else         rotateCCW(ALIGN_SPEED);
  }

  if (isPrintDue()) {
    Serial.print(F("[ALIGN_SHOOT]  Sum: "));  Serial.print(sum);
    Serial.print(F(" | Dir: "));              Serial.println(alignCW ? F("CW") : F("CCW"));
    lastPrintTime = millis();
  }
}

void onRetreatDone() {
  motorsStop();
  Serial.println(F(">>> BACK CLOSE – retreat complete"));
  Serial.print(F("    Back distance: "));
  Serial.print(backDistance);
  Serial.println(F(" cm"));

  loadStartTime = millis();
  currentState = STATE_LOAD;
  Serial.println(F("Waiting 10 seconds before LINE_FOLLOW..."));
}

void rotateCW(int speed) {
  motorLeft(speed,  true);
  motorRight(speed, false);
}

void handleRotateCCW() {
  readAllSensors();
  if (isBackClose()) { onBackDetected(); return; }
  if (strobeActive) rotateCCW(MOTOR_SPEED); else motorsStop();

  if (isPrintDue()) {
    Serial.print(F("[ROTATE_CCW]  Back: "));  Serial.print(backDistance);
    Serial.print(F(" cm | Left: ")); Serial.print(leftDistance);
    Serial.print(F(" cm | Right: ")); Serial.print(rightDistance);
    Serial.println(F(" cm"));
    lastPrintTime = millis();
  }
}

void handleRotateCCW2() {
  readAllSensors();
  if (isSideSumInRange()) { onSideDetected(); return; }
  if (strobeActive) rotateCCW(MOTOR_SPEED); else motorsStop();

  if (isPrintDue()) {
    Serial.print(F("[ROTATE_CCW2]  Back: "));  Serial.print(backDistance);
    Serial.print(F(" cm | Left: ")); Serial.print(leftDistance);
    Serial.print(F(" cm | Right: ")); Serial.print(rightDistance);
    Serial.print(F(" cm | Sum: ")); Serial.println(leftDistance + rightDistance);
    lastPrintTime = millis();
  }
}

void handleLineFollow() {
  readAllSensors();
  if (isBackFar()) { lastError = 0.0; onBackFarDetected(); return; }
  if (!strobeActive) { motorsStop(); return; }
  if (!controlReady) return;
  controlReady = false;

  error = leftDistance - rightDistance;
  float derivative = error - lastError;
  lastError = error;

  int rawL = BASE_LEFT  - (int)(error * KP) - (int)(derivative * KD);
  int rawR = BASE_RIGHT + (int)(error * KP) + (int)(derivative * KD);

  motorLspeed = constrain(rawL, 0, 160);
  motorRspeed = constrain(rawR, 0, 160);

  driveForward(motorLspeed, motorRspeed);

  if (isPrintDue()) {
    Serial.print(F("[LINE_FOLLOW]  L: "));  Serial.print(leftDistance);
    Serial.print(F(" cm | R: "));           Serial.print(rightDistance);
    Serial.print(F(" cm | err: "));         Serial.print(error);
    Serial.print(F(" | deriv: "));          Serial.print(derivative);
    Serial.print(F(" | Lspd: "));           Serial.print(motorLspeed);
    Serial.print(F(" | Rspd: "));           Serial.print(motorRspeed);
    Serial.print(F(" | Back: "));           Serial.print(backDistance);
    Serial.println(F(" cm"));
    lastPrintTime = millis();
  }
}

void handleRetreat() {
  readAllSensors();
  if (isBackClose()) { onRetreatDone(); return; }
  if (!strobeActive) { motorsStop(); return; }

  int leftSpd  = RETREAT_SIDE_SPEED;
  int rightSpd = RETREAT_SIDE_SPEED;

  if (detectedSide == "left") {
    leftSpd  = RETREAT_SIDE_SPEED + 25;
    rightSpd = RETREAT_OPP_SIDE_SPEED;
  } else if (detectedSide == "right") {
    rightSpd = RETREAT_SIDE_SPEED;
    leftSpd  = RETREAT_OPP_SIDE_SPEED + 25;
  }

  driveReverse(leftSpd, rightSpd);

  if (isPrintDue()) {
    Serial.print(F("[RETREAT]  Back: "));  Serial.print(backDistance);
    Serial.print(F(" cm | L: "));          Serial.print(leftDistance);
    Serial.print(F(" cm | R: "));          Serial.print(rightDistance);
    Serial.print(F(" cm | Side: "));       Serial.println(detectedSide);
    lastPrintTime = millis();
  }
}

void handleLoad() {
  if (millis() - loadStartTime >= 10000UL) {
    currentState = STATE_ROTATE_CCW;
    Serial.println(F(">>> WAIT DONE – entering STATE_ROTATE_CCW"));
  }

  if (isPrintDue()) {
    Serial.print(F("[LOAD]  "));
    Serial.print((millis() - loadStartTime) / 1000);
    Serial.println(F("s elapsed"));
    lastPrintTime = millis();
  }
}

void handleStopped() {
  readAllSensors();

  if (isPrintDue()) {
    Serial.print(F("[STOPPED]  Back: "));  Serial.print(backDistance);
    Serial.print(F(" cm | Left: "));       Serial.print(leftDistance);
    Serial.print(F(" cm | Right: "));      Serial.print(rightDistance);
    Serial.println(F(" cm"));
    lastPrintTime = millis();
  }
}

void handleShoot() {

  // Between-shot 0.5s pause
  if (shootPausing) {
    if (millis() - shootPauseStart >= 500UL) {
      shootPausing = false;
      shootStarted = false;   // arm next shot
    } else {
      return;
    }
  }

  if (!shootStarted) {
    shootStartCount = readShootEncoderAtomic();
    shootStarted    = true;
    setShootMotorPWM(SHOOT_PWM_FAST);
    Serial.print(F("  [SHOOT] Motor started, shot #"));
    Serial.println(shootCount + 1);
    return;
  }

  long current   = readShootEncoderAtomic();
  long moved     = labs(current - shootStartCount);
  long remaining = SHOOT_COUNTS_90 - moved;

  if (remaining <= 0) {
    stopShootMotor();
    shootStarted = false;
    shootCount++;
    Serial.print(F("  [SHOOT] Shot "));
    Serial.print(shootCount);
    Serial.println(F(" done"));

    if (shootCount >= 3) {
      currentState = STATE_STOPPED;
      Serial.println(F(">>> SHOOT DONE (3/3) – entering STATE_STOPPED"));
    } else {
      shootPausing     = true;
      shootPauseStart  = millis();
      Serial.println(F("  [SHOOT] Pausing 0.5s before next shot..."));
    }
    return;
  }

  int pwmCmd = SHOOT_PWM_FAST;
  if (remaining <= SHOOT_SLOWDOWN_WIN) {
    long num = (long)(SHOOT_PWM_FAST - SHOOT_PWM_SLOW) * remaining;
    pwmCmd   = SHOOT_PWM_SLOW + (int)(num / SHOOT_SLOWDOWN_WIN);
    pwmCmd   = constrain(pwmCmd, SHOOT_PWM_SLOW, SHOOT_PWM_FAST);
  }
  setShootMotorPWM(pwmCmd);
}

void stopShootMotor() {
  analogWrite(SHOOT_PWM_PIN, 0);
  digitalWrite(SHOOT_IN1_PIN, LOW);
  digitalWrite(SHOOT_IN2_PIN, LOW);
}

void setShootMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(SHOOT_IN1_PIN, LOW);
    digitalWrite(SHOOT_IN2_PIN, HIGH);
  } else {
    digitalWrite(SHOOT_IN1_PIN, HIGH);
    digitalWrite(SHOOT_IN2_PIN, LOW);
    pwm = -pwm;
  }
  analogWrite(SHOOT_PWM_PIN, pwm);
}

void motorLeft(int speed, bool forward) {
  if (forward) {
    digitalWrite(LEFT_IN1_PIN, HIGH);
    digitalWrite(LEFT_IN2_PIN, LOW);
  } else {
    digitalWrite(LEFT_IN1_PIN, LOW);
    digitalWrite(LEFT_IN2_PIN, HIGH);
  }
  analogWrite(LEFT_PWM_PIN, speed);
}

void motorRight(int speed, bool forward) {
  if (forward) {
    digitalWrite(RIGHT_IN3_PIN, HIGH);
    digitalWrite(RIGHT_IN4_PIN, LOW);
  } else {
    digitalWrite(RIGHT_IN3_PIN, LOW);
    digitalWrite(RIGHT_IN4_PIN, HIGH);
  }
  analogWrite(RIGHT_PWM_PIN, speed);
}

void motorsStop() {
  analogWrite(LEFT_PWM_PIN,  0);
  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(LEFT_IN1_PIN,  LOW);
  digitalWrite(LEFT_IN2_PIN,  LOW);
  digitalWrite(RIGHT_IN3_PIN, LOW);
  digitalWrite(RIGHT_IN4_PIN, LOW);
}

void rotateCCW(int speed) {
  motorLeft(speed,  false);
  motorRight(speed, true);
}

void driveForward(int leftSpd, int rightSpd) {
  motorLeft(leftSpd,   true);
  motorRight(rightSpd, true);
}

void driveReverse(int leftSpd, int rightSpd) {
  motorLeft(leftSpd,   false);
  motorRight(rightSpd, false);
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

void readAllSensors() {
  float newBack  = readUltrasonic(BACK_TRIG_PIN,  BACK_ECHO_PIN);
  float newLeft  = readUltrasonic(LEFT_TRIG_PIN,  LEFT_ECHO_PIN);
  float newRight = readUltrasonic(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  if (newBack  <= 300.0) backDistance  = newBack;
  if (newLeft  <= 300.0) leftDistance  = newLeft;
  if (newRight <= 300.0) rightDistance = newRight;
}