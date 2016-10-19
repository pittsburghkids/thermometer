#include "digitalWriteFast.h" // encoder tracking
#include <AccelStepper.h> // dial motor driving

// motor driver
#define MOTOR_STEP_PIN A1
#define MOTOR_DIR_PIN A2

// encoder 
#define ENC_A_PIN 2
#define ENC_B_PIN 5
#define ENC_Z_PIN 6

// temp sensor
#define TEMP_PIN A5

volatile bool encBSet;
volatile long encPos= 500;

AccelStepper stepper(AccelStepper::DRIVER,A1,A2);

void setup() {
  
  // setup encoder
  pinMode(ENC_A_PIN, INPUT);      // sets pin A as input
  pinMode(ENC_B_PIN, INPUT);      // sets pin B as input
  pinMode(ENC_Z_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), HandleEncInterrupt, RISING);
  
  // setup motor
  stepper.setMaxSpeed(1500); // steps per sec
  
  //setup sensor
  pinMode(TEMP_PIN, INPUT);
  analogReference(EXTERNAL);
  
  interrupts();             // enable all interrupts
  
  Serial.begin(9600);

  homeMotor();
}

unsigned long lastPrint=0;
long targetPos= 0;

void loop(){

  // set target encoder position. 
  // 1024 = 360 deg.
  // 1 deg= 2.8444 enc steps
  // 1 deg. C = 6.96 deg
  // 1 deg. C = 19.8 steps
  // on TEMP_PIN, each inc = ~ 0.16 deg. C
  
  float degC= analogRead(TEMP_PIN) * 0.16 - 24.0;
  targetPos= degC * 19.8 + 0.00;

  // square wave test
  //if ((millis()/2000) % 2 == 0) targetPos= 0;
  //else targetPos= 10.0 * 19.8;
  
  long posError= targetPos - encPos;

  // take shortest route to target
  if( posError > 512 ) encPos+= 1024;
  if( posError < -512 ) encPos-= 1024;
  
  // deccelerate to target
  float spd= map(posError, 0, 100, 1, 1500);
  if (spd > 0){
    if (spd > 1500) spd= 1500;
    if (spd < 10) spd= 10; 
  }else{
    if (spd < -1500) spd= -1500;
    if (spd > -10) spd= -10; 
  }

  if (abs(posError) < 2){
    spd= 0.0;
  }
  stepper.setSpeed(spd);
  
  
  //if(stepper.distanceToGo() == 0) stepper.moveTo(random(-5000,5000));
  
  if(millis() > lastPrint+500){
    Serial.print(targetPos);
    Serial.print(" ");
    Serial.print(encPos-targetPos);
    Serial.println();
    lastPrint= millis();
  }

  stepper.runSpeed(); // accel not needed
}

void homeMotor(){
  stepper.setSpeed(-1000); // counter-clocwise to home
  while(digitalRead(ENC_Z_PIN)){ stepper.runSpeed(); }
  encPos=0;
}

// Interrupt service routine for the encoder
void HandleEncInterrupt()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  encBSet = digitalReadFast(ENC_B_PIN);   // read the input pin
 
  // and adjust counter - if A leads B
  encPos += encBSet ? -1 : +1;
}

