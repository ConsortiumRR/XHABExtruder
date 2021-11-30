// V3_04 - using ATtiny85 over I2C
// formated some things to be constant between motormode and pulses
//tested 
//revert to V3_03 to return to no atTiny

//---------------------------------------Libraries-----------------------------------------//
#include <PID_v1.h>
#include <Wire.h> //for I2C/ SPI

//#include <HardwareSerial.h> //for UART
//#include <PubSubClient.h> //nodered
//#include <Wifi.h>
//#include <ArduinoJson.h> //nodered
//#include <AccelStepper.h>

// LCD display library

//---------------------------------------TEMP-VARIABLES-----------------------------------------//
//Temp Reading Info
#define THERMISTORPIN 36
#define THERMISTORNOMINAL 100000  
#define TEMPERATURENOMINAL 25
#define TEMPVCC 3.3
#define ADCSAMP 4095   
#define SERIESRESISTOR 330
#define NUMSAMPLES 5
#define BCoeff 4007.5
#define c1 0.7493611e-03
#define c2 2.0899615e-04
#define c3 1.3007668e-07
#define TEMP_VOUT 32 //IO as power pin for thermistor

//PWM pin for heater control
#define MOS_PIN 4
#define MOS_CHAN 0

//PID parameters
//uint16_t temp_set = 290; // target temp for extruder - move to bottom section
double Input=25, Output, Setpt;
//double aggKp=20, aggKi=2, aggKd=.2;
double consKp=20, consKi=.5, consKd=5;

PID myPID(&Input, &Output, &Setpt, consKp, consKi, consKd, DIRECT);

bool HeatOn = true;
bool extruderready = false;
//-------------------------------------------------------------------------------//

//---------------------------------------ROBOT-COMS-VARIABLES-----------------------------------------//
bool ModeIn_pulse = true; 
bool ModeOut_GO = true;

// vars for binary signal mode (can be removed for one way pulse operation)
#define numiBits 4
#define numoBits 4
//input (ABB-> Hub) IO binary pins
const uint8_t ibit0 = 16, ibit1 = 17, ibit2 = 18, ibit3 = 19;
uint8_t diPin[] = {ibit0, ibit1, ibit2, ibit3};
//output (Hub->ABB) IO binary pins
const uint8_t obit0 = 25, obit1 = 26, obit2 = 27, obit3 = 33;
uint8_t doPin[] = {obit0, obit1, obit2, obit3};

//Pulse Read Vars
const byte READ_PIN = ibit0; // Pin to read pulses from (needs to be an interrupt pin)
volatile unsigned long pulselen;
const uint8_t numPulse = 7;
const uint16_t pulselengths[numPulse] = {5000, 10000, 15000, 25000, 35000, 40000, 45000}; //set pulse lengths in microseconds
const char  *pulsecommand[] = {"NONE","STOP","FORWARD", "RETRACT", "LOWER SPEED", "RAISE SPEED", "CHECK HEAT", "SYSTEM OFF"}; 
uint16_t pulsetolerance = 700;

bool trigger = false;
//-------------------------------------------------------------------------------//

//---------------------------------------MOTOR-VARIABLES-----------------------------------------//
bool motorSwitchState = LOW; 
// this has been delegated to attiny85
uint8_t motormode = 0;
const byte motorSub = 0x9; 
//structure messages like "[Command Char]:["3CharPositiveValue"]" 
//eg "F:200" = forward 200 step/s, "S","STOP","S:000" stops stepper, "R:100" retract at 100 step/s
//"D", "E" disables and enables stepper driver (this shouldn't be done often)
//"I[char]:[positiveValue]" changes stepper varaibles, "IA" for acceleration, "ID" for default speed, "IM" for max speed 
//Request events with 5 bytes return motor state / feed rate, request with 12 bytes for current settings "accel, max speed, default speed"
//-------------------------------------------------------------------------------//

//---------------------------------------INTERFACE-VARIABLES-----------------------------------------//
// will put in some variables for lcd, buttons/knobs to allow direct user input on tool
//-------------------------------------------------------------------------------//

bool debugging = false; // a lil bool to print to serial when debugging
uint8_t OVERRIDE_PIN = 34; // toggle switch to use robot or override
uint8_t DRIVE_PIN=35; // direction for override
uint8_t READYLED_PIN = 23; // LED to show temp status

// -------> EDIT PRIMARY VARIABLES HERE<------------------------------------------------------------------------//
uint16_t STEP_RATE = 200; //val for forward extrusion rate steps per second 
uint16_t temp_set = 190; // target temp for extruder 
uint16_t displayinterval = 3000; // number of milliseconds to wait between display loop to prevent delays due to serial write
uint8_t inputquery = 0; 

//>>>>>SETUP<<<<<-----------------------------------------------------------------------//
void IRAM_ATTR PulseTrigger(){
  static uint8_t lastState;
  static unsigned long startTime = 0;
  uint8_t readState = digitalRead(READ_PIN);
  if (readState != lastState){
    if (readState==LOW) startTime=micros();
    else{
      pulselen = micros()-startTime; 
      trigger = true; 
    }
    lastState = readState; 
  }
}

//>>>>>SETUP<<<<<-----------------------------------------------------------------------//
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(OVERRIDE_PIN, INPUT);
  pinMode(DRIVE_PIN, INPUT);
  pinMode(READYLED_PIN, OUTPUT);
  pinMode(TEMP_VOUT, OUTPUT);
  
  //Set up outputs for mosfet
  ledcSetup(MOS_CHAN, 490, 8);
  ledcAttachPin(MOS_PIN, MOS_CHAN);
  //set up reference and PID mode for temp control
  HeatOn = true; //heat starts on when board is on - robot can disable
  Setpt = temp_set;
  myPID.SetMode(AUTOMATIC);
  digitalWrite(TEMP_VOUT, HIGH);
  
  motormode = 0;
  messageMotor(motormode); //make sure to tell the motor it's off
  
  //Set inputs and outputs for robot comms
  if (ModeIn_pulse == true){
    pinMode(READ_PIN,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(READ_PIN),PulseTrigger, CHANGE);
    }
  else{
    for (int indexPin = 0; indexPin < numiBits; indexPin++) {
      pinMode (diPin[indexPin], INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(diPin[indexPin]),InputTrigger, CHANGE);// this might be bad news may need to only interrrupt 1 pin
    }
  }
  if (ModeOut_GO == true){  //Output 
    for (int indexPin = 0; indexPin < numiBits; indexPin++) {
      pinMode(doPin[indexPin], OUTPUT);
    }
  }
}

//>>>>>LOOP<<<<<-----------------------------------------------------------------------//
void loop(){
  static unsigned long loopTimer;
  static unsigned long displaytimer;
  uint8_t  loopWait = 4; // milliseconds between check loop
  
  // trigger state if robotic command detected 
  if (trigger==true)ReadPulse();

  // checks override motor switch states
  if (digitalRead(OVERRIDE_PIN)==LOW){
    if (millis()-loopTimer > loopWait){
      if (digitalRead(DRIVE_PIN)!=motorSwitchState){
        motorSwitchState = digitalRead(DRIVE_PIN);
        if (motorSwitchState ==HIGH)motormode=1;
        else motormode=0;
        messageMotor(motormode);   
        }
        loopTimer = millis();
    }   
  }
  
  //display loop - for LED inidicators, LCD display, serial print and Temp control loop
  if (millis() - displaytimer > displayinterval){
    TempControl();
    if (debugging){
      Serial.print("Motormode:");
      Serial.println(pulsecommand[motormode+1]);
      Serial.print(Setpt);
      Serial.print("\t");
      Serial.print(Input);
      Serial.print("\t");
      Serial.println(Output);
    }
    if (Input < (Setpt-25)){
      extruderready = false;
      if (!debugging)motormode = 0; //prevent cold motor start when debugging is disabled
      digitalWrite(READYLED_PIN,LOW);
    }
    else{
      extruderready = true;
      digitalWrite(READYLED_PIN, HIGH);
    }
    displaytimer = millis();
  }
}

void TempControl(){
  if (HeatOn){
    Input = (readThermistor()+Input)*.5; //average of current and last temp read to smooth out (should be reworked lol)
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
    ledcWrite(MOS_CHAN, Output);
  }
  else{
    ledcWrite(MOS_CHAN,0);
  }
}

void messageMotor(uint8_t in){
 // sends a message to the i2c motor controller sub - this could be expanded with multiple subs
  char motorMessage[6]; 
  // construct message for motorsub
    switch(in){
      case 0: // Stopped (just ensures motor is stopped if this is called)
        sprintf(motorMessage, "S:000");
        break;
      case 1: //Forward
        sprintf(motorMessage,"F:%u", STEP_RATE);
        break;
      case 2: //Retract
        sprintf(motorMessage,"R:%u", STEP_RATE);
        break;    
      case 3: // Enable
        sprintf(motorMessage, "ENABLE");  
        break;  
      case 4: //Disable
        sprintf(motorMessage, "DISBLE"); 
        break;
      }
      //send to sub
      Wire.beginTransmission(motorSub);
      Wire.write(motorMessage);
      Wire.endTransmission();

      //print message to serial to check
      if (debugging){
        Serial.print("sent to motor sub: ");
        for (uint8_t i=0; i<sizeof(motorMessage);i++){
          Serial.print(motorMessage[i]);
        }
        Serial.println();
      }
      //wipe message
      memset(motorMessage, 0, sizeof(motorMessage)); 
  
}
void ReadPulse(){
  for (int i = 0; i < numPulse; i++) {
      if ( abs(pulselen - pulselengths[i]) < pulsetolerance ) {
          inputquery = i+1;
          break; // no need to continue loop if it's already found a motorMode
        }
        else{
        inputquery = 0;
      }
    }
    if (debugging){
      Serial.print("pulse");
      Serial.print(pulselen);
    }
    pulselen = 0; 
    trigger = false;
    
    // if pulse length corresponds with command then execute command
  if (inputquery > 0 && inputquery <= numPulse)ExecuteCommand(inputquery);
  
}

void ExecuteCommand(uint8_t command){
  if (debugging){
    Serial.print("\tcommand = ");
    Serial.println(inputquery);
    Serial.print("Executing Command: ");
    Serial.println(pulsecommand[command]);
  }
  //Execute commands from robot: (excluding none) {"NONE","STOP","FORWARD", "RETRACT", "LOWER SPEED", "RAISE SPEED", "CHECK HEAT", "SYSTEM OFF"}
  switch(command){
    case 1: //Stop extruder
      motormode = 0;  
      messageMotor(motormode);
      break;  
    case 2: //Forward    
      motormode = 1;
      messageMotor(motormode);
      break;    
    case 3: //Retract
      motormode = 2;
      messageMotor(motormode);
      break;
    case 4: // decrease step speed by 50 step/s
      STEP_RATE -= 50; 
      messageMotor(motormode); 
      break;
    case 5: //increase step speed by 50 step/s
      STEP_RATE += 50; 
      messageMotor(motormode); 
      break; 
    case 6: //ensure heat + stepper is on, check if heat is ready
      Serial.println(extruderready);
      HeatOn = true;
      messageMotor(3);
      // if binary output setup send signal to robot
      if (ModeOut_GO){
        if (extruderready)ReturnBinary(1); 
        else ReturnBinary(2);       
      }
      break; 
    case 7: //shuts extruder down  
      motormode = 0;
      messageMotor(4); 
      HeatOn = false;
      break;
  }
}

void ReturnBinary(byte sendval){
  //send a binary signal to the robot to be read from GI
  for (int i = 0; i < numoBits; i++) {
    if (bitRead(sendval, i) == 1) digitalWrite(doPin[i], HIGH);
    else digitalWrite(doPin[i], LOW);
  }
}

double readThermistor(void){
  // maybe rework to read from table instead of all this business
  uint8_t i;
  double average;
  double logR;
  float T;

  average =0;
  for (i=0; i< NUMSAMPLES; i++) {
   average += analogRead(THERMISTORPIN);
  }
  average /= NUMSAMPLES;
  average = (average*TEMPVCC)/ADCSAMP;//convert to volts
  average = SERIESRESISTOR*average/(TEMPVCC-average);
  
  logR = log(average);
  T = (1.0 / (c1 + c2*logR + c3*logR*logR*logR));
  if (T>1)  T -= 273.15; //for low series resistor + ESP32 ADC nonlinearity everything under about 50C reads as O
  return T;
}
