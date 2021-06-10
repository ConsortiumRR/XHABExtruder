// V2_01 -04/02/21 - tested
// updated PID values to be a little smoother, debugged pulse from robot
// V2_00 -03/31/21 - not tested 
// Updates to include empirically derived thermistor coefficients and override/auto switch for robot coms
//---------------------------------------Libraries-----------------------------------------//
#include <PID_v1.h>
#include <AccelStepper.h>
//<HardwareSerial.h> for serial output <SafeString.h> for millis delay
// LCD display?

//---------------------------------------TEMP-VARIABLES-----------------------------------------//
//Temp Reading Info
#define THERMISTORPIN A0
#define THERMISTORNOMINAL 100000  
#define TEMPERATURENOMINAL 25   
#define SERIESRESISTOR 5110
#define NUMSAMPLES 5
#define BCoeff 4561.92
#define c1 0.8999108057e-03
#define c2 2.086212521e-04
#define c3 0.3424927711e-07

//PWM pin for heater control
#define MOS_PIN 9

//PID parameters
//uint16_t temp_set = 290; // target temp for extruder 
// uint16 because it is always a whole positive number and may be above 255 degrees
double Input=25;, Output, Setpt;
//double aggKp=20, aggKi=2, aggKd=.2;
double consKp=19.5, consKi=.71, consKd=50;

PID myPID(&Input, &Output, &Setpt, consKp, consKi, consKd, DIRECT);

bool HeatOn = true;
bool extruderready = false;
//-------------------------------------------------------------------------------//

//---------------------------------------ROBOT-COMS-VARIABLES-----------------------------------------//
bool ModeIn_pulse = true; 
bool ModeOut_GO = true;

uint8_t numiBits = 4;
uint8_t numoBits = 4;
//input (ABB-> Hub) IO binary pins
uint8_t ibit0 = 2, ibit1 = 31, ibit2 = 32, ibit3 = 33;
//output (Hub->ABB) IO binary pins
uint8_t obit0 = 40, obit1 = 41, obit2 = 42, obit3 = 43;

int doPin[] = {obit0, obit1, obit2, obit3};
int diPin[] = {ibit0, ibit1, ibit2, ibit3};

//Pulse Read Vars
const byte READ_PIN = 2; // Pin to read pulses from (needs to be an interrupt pin)
volatile unsigned long pulselen;
uint8_t numPulse = 5;
int pulselengths[] = {5000, 10000, 15000, 20000, 25000}; //set pulse lengths in microseconds
const char  *pulsecommand[] = {"NONE","FORWARD", "RETRACT", "STOP", "CHECK HEAT", "SYSTEM OFF"}; 
// Coresponds to extrude forward, extrude reverse, stop extruder, is extruder ready (preheat), turn off extruder
uint16_t pulsetolerance = 500;
//-------------------------------------------------------------------------------//

//---------------------------------------STEPPER-VARIABLES-----------------------------------------//
uint8_t STEP_PIN = 6;
uint8_t DIR_PIN = 7;
//uint8_t F_RATE = 600; //val for forward extrusion rate steps per second 
uint8_t retractstep = -300; //val num steps to retract

uint8_t motormode = 0;
AccelStepper Extruder(1, STEP_PIN, DIR_PIN); 

//-------------------------------------------------------------------------------//

//---------------------------------------INTERFACE-VARIABLES-----------------------------------------//
// will put in some variables for lcd, buttons/knobs to allow direct user input on tool
//-------------------------------------------------------------------------------//

bool debugging = false; // a lil bool to print to serial when debugging
uint8_t OVERRIDE_PIN = 14; // toggle switch to use robot or override
uint8_t DRIVE_PIN=13; // direction for override
int READYLED_PIN = 12; // LED to show temp status

// DEBUGGING STAGE ->EDIT VARIABLES HERE<-
uint8_t F_RATE = 700; //val for forward extrusion rate steps per second 
uint16_t temp_set = 230; // target temp for extruder 
uint16_t displaytimestep = 1; // number of seconds to wait between display intervals
uint8_t inputquery = 0; 

//>>>>>SETUP<<<<<-----------------------------------------------------------------------//
void setup() {
  Serial.begin(115200);
  //Set up outputs for mosfet and stepper control
  pinMode(MOS_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(OVERRIDE_PIN, INPUT_PULLUP);
  pinMode(DRIVE_PIN, INPUT_PULLUP);
  pinMode(READYLED_PIN, OUTPUT);
  //set up reference and PID mode for temp control
  HeatOn = true; //for now turn the heat on as long as the arduino is on - will change
  Setpt = temp_set;
  myPID.SetMode(AUTOMATIC);
  analogReference(EXTERNAL);
  Extruder.setAcceleration(200); 
  Extruder.setMaxSpeed(1000);
  Extruder.setSpeed(F_RATE);
  motormode = 0;
  
  //Set inputs and outputs for robot comms
  if (ModeIn_pulse == true){//input
    pinMode(READ_PIN, INPUT);
    }
  else{
    for (int indexPin = 0; indexPin < numiBits; indexPin++) {
    pinMode(diPin[indexPin], INPUT);
    }
  }
  if (ModeOut_GO == true){  //Output 
    for (int indexPin = 0; indexPin < numiBits; indexPin++) {
      pinMode(doPin[indexPin], OUTPUT);
    }
  }
  attachInterrupt(digitalPinToInterrupt(READ_PIN), ReadPulse, CHANGE);
      //sets up interrupt when IO input occurs
}

//>>>>>LOOP<<<<<-----------------------------------------------------------------------//
void loop(){
  static unsigned long loopTime = 0;
  int wait = 5; // milliseconds between loops
  static unsigned long displaytimer;
  long displayinterval = 1000* displaytimestep; //display time interval (3 seconds)
  if (millis() - loopTime > wait) {
    ExtruderRun(motormode);
    loopTime = millis();
  }
  //display loop - can be used for debugging or LCD display
  if (millis() - displaytimer > displayinterval){
    TempControl();
    Serial.print(Setpt);
    Serial.print("\t");
    Serial.print(Input);
    Serial.print("\t");
    Serial.println(Output);
    if (Input < (Setpt-25)){
      extruderready = false;
      digitalWrite(READYLED_PIN,LOW);
    }
    else{
      extruderready = true;
      digitalWrite(READYLED_PIN, HIGH);
    }
    if (debugging){
      Serial.println(motormode);
      Serial.println(extruderready);
    }   
    displaytimer = millis();
  }
}

void TempControl(){
  if (HeatOn){
    Input = (readThermistor()+Input)*.5; //average of current and last temp read to smooth out
    double gap = abs(Setpt-Input); //distance away from setpoint
    myPID.SetTunings(consKp, consKi, consKd);
    myPID.Compute();
    if ((Setpt-Input)>10){
      Output = 255;
    }
    else if ((Input-Setpt)>10){
       Output = 0;
    }
    else{
    }
    analogWrite(MOS_PIN, Output);
  }
  else{
    analogWrite(MOS_PIN,0);
  }
}

void ExtruderRun(uint8_t in){
// Runs steppers at set speeds - case:  NONE, FORWARD, RETRACT, STOP,
  if (digitalRead(OVERRIDE_PIN)==HIGH){
    // if override is on then use the direction pin to set motor 
    if (digitalRead(DRIVE_PIN)==HIGH){
      in = 1; 
    }
    else{
      in = 0;
    }
  }
    switch(in){
      case 0: // Stopped (just ensures motor is stopped if this is called)
        Extruder.stop();
        break;
      case 1: //Forward
        Extruder.runSpeed();  
        break;
      case 2: //Reverse 
      // should add something to prevent this from running too long
        Extruder.run();
        if (Extruder.distanceToGo()==0){
          motormode = 0;  
        }
        break;    
      case 3: //Decelerate and stop motor
      // needs refinement currently just using quickstop
        Extruder.stop();
        motormode = 0;
        break;  
      }
  
}

void ReadPulse(){
  static unsigned long startTime = 0;
  static uint8_t lastState = 0; // state of the pin last time the interrupt was triggered
  uint8_t readState = (PINE & (0b00010000)); // port access is quicker than digitalRead

  if (readState != lastState) {
    if (readState) startTime = micros(); // if just went high, start timer
    else {
      pulselen = micros() - startTime; // if just went low, stop timer and count difference
      // drive motor based on pulse read
      for (int i = 0; i < numPulse; i++) {
        if ( abs(pulselen - pulselengths[i]) < pulsetolerance ) {
          inputquery = i+1;
          break; // no need to continue loop if it's already found a motorMode
        }
      }
      if (debugging) {
        Serial.print("pulse");
        Serial.print(pulselen);
        Serial.print("\tcommand = ");
        Serial.println(inputquery);
      }
    }
    lastState = readState;
  if (inputquery > 0 && inputquery < numPulse){
      ExecuteCommand(inputquery);
    }
  }
}

void ExecuteCommand(uint8_t command){
  if (debugging){
    Serial.print("Executing Command: ");
    Serial.println(pulsecommand[command]);
  }
  //Execute commands from robot: (excluding none) {NONE, FORWARD, RETRACT, STOP, READY, OFF}; 
  switch(command){
    case 1: //Forward extruder   
      motormode = 1;
      Extruder.setSpeed(F_RATE);     
      ExtruderRun(motormode);
      break;  
    case 2: //retract    
      motormode = 2;
      Extruder.moveTo(retractstep);
      ExtruderRun(motormode);
      break;    
    case 3: //stop 
      motormode = 3;
      ExtruderRun(motormode);
      break;  
    case 4: //ready- checks if extruder is ready to go (heat wise)
      Serial.println(extruderready);
      if (extruderready){
        //if the temp is within 10 degrees of being ready send a 1 to the robot(yes)
        ReturnBinary(1);
        
      }
      else{
        ReturnBinary(2);
        
      }
      break; 
    case 5: //shuts extruder down  
      Extruder.stop();
      motormode = 0;
      HeatOn = false;
      break;
    
  }
}

void ReturnBinary(byte sendval){
  //send a binary signal to the robot to be read from GI
  for (int i = 0; i < numoBits; i++) {
    if (bitRead(sendval, i) == 1) {
      digitalWrite(doPin[i], HIGH);
    }
    else {
      digitalWrite(doPin[i], LOW);
    }
  }
}

double readThermistor(void){
    //this code might need reworking
  int samples[NUMSAMPLES];
  uint8_t i;
  double average;
  double logR;
  float T;

  average =0;
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   average +=samples[i];
  }
  average /= NUMSAMPLES;
  average = (SERIESRESISTOR / ((1023 / average) - 1));// convert the value to resistance

  
  logR = log(average);
  T = (1.0 / (c1 + c2*logR + c3*logR*logR*logR));
  T -= 273.15;
  return T;
}
