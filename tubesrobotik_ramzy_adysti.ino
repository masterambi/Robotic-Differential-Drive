#include <Servo.h>
#include <NewPing.h>        //Ultrasonic sensor function library. You must install this library
#include <MPU9250.h>        //Library MPU9250 dari borderflight

#define ENC_HIGH_DELAY 1

// sonar
#define SONAR_NUM 3
#define MAX_DISTANCE 120
#define PING_INTERVAL 50

unsigned long pingTimer;
unsigned int distances[SONAR_NUM]; // Store ping distances.
int distance;
uint8_t currentSensor = 0; // Which sensor is active.

NewPing sonar(8, 9, MAX_DISTANCE);

// our L298N control pins
const int LeftMotorForward = 13;
const int LeftMotorBackward = 12;
const int RightMotorForward = 11;
const int RightMotorBackward = 10;
const int enLeft = 6;
const int enRight = 5;
const int leftOpto = 0;
const int rightOpto = 1;
const double R = 3.25;
const double L = 25.1;
const double N = 20;
const int maxSpeed = 10;

long countKiri = 0;
long countKanan = 0;
long countKananNew, countKiriNew, deltaCountKanan = 0;
long countKananOld, countKiriOld, deltaCountKiri = 0;
volatile long lastRiseTimeR, lastRiseTimeL = 0;

double DL, DR, DC;
double totalDC;
double xNew, xOld;
double yNew, yOld;
double thetaNew, thetaOld;

double goalX, goalY;
double deltaX, deltaY;
double goalTheta, errorTheta;
double dt = 0.01;
double range = 8;
int pwm, angle;

double v0 = 3;
double vKiri, vKanan;
double error, errorOld, errorP, errorI, errorD;
double P, I, D, w = 0;
const double Kp = 2;
const double Ki = 0.1;
const double Kd = 0.01;

bool stateRightForward = false;
bool stateLeftForward = false;


double Mwf1_1, Mwf2_1, Mwf3_1, UI1_1, UI2_1, Uao1_1, Uao2_1;
double thetaAo, errorAo, errorAoP, errorAoI, errorAoD, oldErrorAo;
const double KpAo = 2;
const double KiAo = 0.1;
const double KdAo = 0.01;

MPU9250 IMU(Wire,0x68);
int status;
long currentTime, prevTime, deltaTime;
double gyroZ, v, prevAccelY;
double yOffset;

void setup() {
  // put your setup code here, to run once:
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  attachInterrupt(leftOpto, encoderL, RISING);
  attachInterrupt(rightOpto, encoderR, RISING);

  goalX = 250;
  goalY = 250;

  Serial.begin(115200);

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);

}


void getDistance() {
  if (millis() >= pingTimer + PING_INTERVAL) {
   distance = sonar.ping_cm();
   if (distance == 0) {
    distance = 120;
   }
  }
      
}

void loop() {
  // put your main code here, to run repeatedly:
  getDistance();
  if ((xNew < (goalX + range)) && (xNew > (goalX - range)) && (yNew < (goalY + range)) && (yNew > (goalY - range))) {
    moveStop();
  } else {
    if(distance < 60){
      executeObstacleAvoidance();
    } else {
      executeGoToGoal();
    }
  }


  Serial.print(xNew);
  Serial.print(" ");
  Serial.print(yNew);
  Serial.print(" ");
  Serial.print(thetaNew);
  Serial.print(" ");
  Serial.print(vKanan);
  Serial.print(" ");
  Serial.print(vKiri);
  Serial.print(" ");
  Serial.print(countKanan);
  Serial.print(" ");
  Serial.print(countKiri);
  Serial.print(" ");
  Serial.print(distance);
  Serial.print(" ");
  Serial.print(goalTheta);
  Serial.println(" ");

  delay(dt * 1000);
}



void executeGoToGoal() {
  countKananNew = countKanan;
  countKiriNew = countKiri;

  goToGoal();
  controlV();
  
  executeMotor();
  
}

void executeObstacleAvoidance() {
  countKananNew = countKanan;
  countKiriNew = countKiri;

  obstacleAvoidance();
  controlV();
  
  executeMotor();
}

void goToGoal() {
  estimatePosition();
  deltaX = goalX - xNew;
  deltaY = goalY - yNew;
  goalTheta = atan2(deltaY , deltaX);
  errorTheta = goalTheta - thetaNew;
  errorTheta = atan2(sin(errorTheta), cos(errorTheta));

  errorP = errorTheta;
  P = Kp * errorP;
  errorI = errorTheta + errorI * dt;
  I = Ki * errorI;
  errorD = (errorTheta - errorOld) / dt;
  D = Kd * errorD;

  errorOld = errorTheta;

  w = P + I + D;

  uniToDiff();
}

void obstacleAvoidance() {
  estimatePosition();
  distanceToWorldFrame();
  UI1_1 = Mwf1_1 - xNew;
  UI2_1 = Mwf2_1 - yNew;
  Uao1_1 = UI1_1;
  Uao2_1 = UI2_1;

  thetaAo = atan2(Uao2_1, Uao1_1);
  errorAo = -thetaNew + (thetaAo + PI/4);
  errorAo = atan2(sin(errorAo), cos(errorAo));

  errorAoP = errorAo;
  errorAoI = oldErrorAo + errorAo * dt;
  errorAoD = (errorAo - oldErrorAo) / dt;

  w = KpAo * errorAoP + KiAo * errorAoI + KdAo * errorAoD;
  uniToDiff();
}





void estimatePosition () {

  deltaCountKanan = countKananNew - countKananOld;
  deltaCountKiri = countKiriNew - countKiriOld;

  DR = 2 * PI * R * (deltaCountKanan / N) ;
  DL = 2 * PI * R * (deltaCountKiri / N) ;
  DC = (DR + DL) / 2 ;

  xNew = xOld + DC * cos (thetaOld);
  yNew = yOld + DC * sin (thetaOld);
  thetaNew = getTheta();

  countKananOld = countKananNew;
  countKiriOld = countKiriNew;;
  xOld = xNew;
  yOld = yNew;
  thetaOld = thetaNew;

}

void controlV() {
  double velRLMax = max(vKanan, vKiri);
  double velRLMin = min(vKanan, vKiri);
  if(velRLMax > maxSpeed) {
    vKanan = vKanan - (velRLMax - maxSpeed);
    vKiri = vKiri - (velRLMax - maxSpeed);
  } else if(velRLMin < -maxSpeed) {
    vKanan = vKanan - (velRLMin + maxSpeed);
    vKiri = vKiri - (velRLMin + maxSpeed);
  } else {
    vKanan = vKanan;
    vKiri = vKiri;
  }
}

double getTheta() {
  IMU.readSensor();
  currentTime = millis();
  deltaTime = currentTime - prevTime;
  double gyroZ_dt = IMU.getGyroZ_rads();
  gyroZ += gyroZ_dt * ((double)deltaTime/1000);
  prevTime = currentTime;
  return atan2(sin(gyroZ), cos(gyroZ));
}



void executeMotor() {
  if ((vKanan > 0) && (vKiri > 0)) {
    moveForward();
  } else if ((vKanan < 0) && (vKiri < 0)) {
    vKanan = -vKanan;
    vKiri = -vKiri;
    moveBackward();
  } else if ((vKanan < 0) && (vKiri > 0)) {
    vKanan = -vKanan;
    moveRight();
  } else if ((vKanan > 0) && (vKiri < 0)) {
    vKiri = -vKiri;
    moveLeft();
  }
}

void moveForward() {
  stateRightForward = true;
  stateLeftForward = true;
  vKiri  = map(vKiri , 0, maxSpeed, 120, 180);
  vKanan = map(vKanan, 0, maxSpeed, 120, 180);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  analogWrite(enRight, vKanan);
  analogWrite(enLeft, vKiri);
}

void moveBackward() {
  stateRightForward = false;
  stateLeftForward = false;
  vKiri  = map(vKiri , 0, maxSpeed, 120, 180);
  vKanan = map(vKanan, 0, maxSpeed, 120, 180);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorBackward, HIGH);
  analogWrite(enRight, vKanan);
  analogWrite(enLeft, vKiri);
}

void moveRight() {
  stateRightForward = false;
  stateLeftForward = true;
  vKiri  = map(vKiri , 0, maxSpeed, 120, 180);
  vKanan = map(vKanan, 0, maxSpeed, 120, 180);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  analogWrite(enRight, vKanan);
  analogWrite(enLeft, vKiri);
}

void moveLeft() {
  stateRightForward = true;
  stateLeftForward = false;
  vKiri  = map(vKiri , 0, maxSpeed, 120, 180);
  vKanan = map(vKanan, 0, maxSpeed, 120, 180);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
  analogWrite(enRight, vKanan);
  analogWrite(enLeft, vKiri);
}

void moveStop() {
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void uniToDiff() {
  vKanan = (2 * v0 + w * L) / (2 * R);
  vKiri = (2 * v0 - w * L) / (2 * R);
}

void distanceToWorldFrame() {
  double Mrf1_1, Mrf2_1, Mrf3_1;
  Mrf1_1 = cos(0)*distance - sin(0)*0 + 25*1;
  Mrf2_1 = sin(0)*distance + cos(0)*0 + 0*1;
  Mrf3_1 = 0*distance + 0*0 + 1*1;
  
  Mwf1_1 = cos(thetaNew)*Mrf1_1 - sin(thetaNew)*Mrf2_1 + xNew*Mrf3_1;
  Mwf2_1 = sin(thetaNew)*Mrf1_1 + cos(thetaNew)*Mrf2_1 + yNew*Mrf3_1;
  Mwf3_1 = 0*Mrf1_1 + 0*Mrf2_1 + 1*Mrf3_1;
}

void encoderL() {
  long interruptTimeL = millis();
  if (interruptTimeL - lastRiseTimeL > ENC_HIGH_DELAY) {
    if (stateLeftForward) {
      countKiri++; 
    } else {
      countKiri--;
    }
    
  }
  lastRiseTimeL = interruptTimeL;
}

void encoderR() {
  long interruptTimeR = millis();
  if (interruptTimeR - lastRiseTimeR > ENC_HIGH_DELAY) {
    if (stateRightForward) {
      countKanan++;
    } else {
      countKanan--;
    }
    
  }
  lastRiseTimeR = interruptTimeR;
}

static inline int8_t sgn(double val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
