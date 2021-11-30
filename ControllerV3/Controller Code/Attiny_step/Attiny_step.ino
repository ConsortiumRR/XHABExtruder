// I2C Sub Stepper Control
//PHOEBE DEGROOT - Updated 11/25/21 
// todo:  add more functions (pin allocation, parameters etc) and reduce memory load might need to replace accelstepper due to bloat 

// This ATtiny85 acts as a simple stepper controller using the a stepper driver (eg A4988) and the AccelStepper library
// executes stepper commands-  "STOP" stop stepper, "F:[speed]" run stepper forward at speed
// "R:[speed]" stepper retract at speed, "E" enable stepper, "D" disable stepper

// general format is COMMAND:VALUE - command can be char or string, value is unsigned 16bit  integer
// Syntax rules: need to be under 16 char long, must start with correct command char (can be upper or lower)
// command not case sensitive- use just one char or multiple, ':' can be interchanged with or ','  just dont use twice!
// eg "F:200" , "for:200", "f,0200" all run stepper forward at 200 steps per second
// with no feed set "Forward" and "Retract" will use last feed used or 100step/s if none set
// retract only runs for specified number of steps to protect auger
//'I' intializes different variables with in for value A=accel, R=retract steps(positive!), M= maxspeed
 

#define I2C_SUB_ADDRESS 0x9
//This sub's address is 0x9 or int 9 
#include<TinyWireS.h>
#include<AccelStepper.h>

// ------ ------ ------ STEPPER VARS ------ ------ ------ //
#define LED_PIN 1
uint8_t DIR_PIN =4;
uint8_t STEP_PIN =3;
const uint8_t motorLoop = 1; //loop time for motor running
AccelStepper Extruder(1, STEP_PIN, DIR_PIN);
uint16_t StepSpeed; 
bool motorOn = true;
uint8_t motormode = 0; 
int16_t retractStep = -250;
uint16_t stepAccel = 400;
uint16_t stepMaxSpeed = 500;
uint8_t defaultSpeed = 100;   


// ------ ------ ------ COM VARS ------ ------ ------ //  
#define numChars 16 
char tempChars[numChars]; 
char msgChars[numChars]; 
char msgString[numChars] = {0};
uint16_t msgInt = 0;

bool newMsg = false;
#define cmdNdx 0 //location of first command in message - for quick reactions

// >>>>>> ------ ------ ------ I2C EVENTS ------ ------ ------ <<<<<< //
void receiveEvent(uint8_t num_bytes){
  // returns if number of bytes is too high or low 
  if (num_bytes<1){
    return;
  }
  else if (num_bytes> numChars){
    return;
  }
  
  // data reading - loads msgChars array with each byte
  for (uint8_t i = 0; i<num_bytes; i++){
    msgChars[i] = TinyWireS.receive();
  }
  newMsg = true; 
  // skips parsing if the first char is "S" for stop
  if (msgChars[cmdNdx]=='S' or msgChars[cmdNdx]=='s'){
    motormode = 0; // quick reaction for stoping 
    newMsg = false; 
    return;  
  }
}

// >>>>>> ------ ------ ------ SETUP ------ ------ ------ <<<<<< //
void setup() {
  TinyWireS.begin(I2C_SUB_ADDRESS);
  TinyWireS.onReceive(receiveEvent); 

  pinMode(DIR_PIN,OUTPUT);
  pinMode(STEP_PIN,OUTPUT);
  pinMode(LED_PIN, OUTPUT); 
  Extruder.setAcceleration(stepAccel); 
  Extruder.setMaxSpeed(stepMaxSpeed);
  Extruder.setSpeed(defaultSpeed); 

}

void loop() {
  static unsigned long motorTimer;
  
  TinyWireS_stop_check(); //need to double check use of this function - keep loop tight! tws_delay() instead of delay
    
  if (newMsg){
    //parse message if new
    strcpy(tempChars, msgChars);
    parseMessage(); 
    newMsg = false; 
  }
  // skip timer and motor run if motorOn is false - this should help motor start faster when motorOn is switched to true
  if(motorOn){
    if (millis()-motorTimer>motorLoop){
      runMotor(motormode);
      motorTimer = millis(); 
    }
  } 
}

void parseMessage(){
  // vars to store message command string and int 
  // this could be modified to parse more complex messages - for this stepper the format is String:Int eg F:250 or Forward:250
  char *msgNdx; // pointer index for parsing input message into tokens
  
  msgNdx = strtok(tempChars, ":,"); //split on : , - or space char
  strcpy(msgString, msgNdx); //store first part of message as char array 
  msgString[0] &= 0xDF; //ensure first char is uppercase
  msgString[1] &= 0xDF; //ensure second char is uppercase

  msgNdx = strtok(NULL, ":,"); //continue tokenizing message
  msgInt = atoi(msgNdx); //store next part of message as int 
  getCommand(); 
}

void getCommand(){
  // interprets message using the first char of command string:  STOP, FORWARD, RETRACT, ENABLE, DISABLE
  // could add distance or ability to initialize some variables eg "IA" =>intialize acceleration value
  switch(msgString[0]){
    case 'S':
      motormode = 0; 
      StepSpeed = 0; 
      motorOn = false; 
      break;
    case 'F':
      motormode = 1;
      if (msgInt>0){
        StepSpeed = msgInt;
        defaultSpeed = StepSpeed; 
      }
      else{
        StepSpeed = defaultSpeed;  
      }
      motorOn = true; 
      Extruder.setSpeed(StepSpeed);
      digitalWrite(LED_PIN, HIGH);     
      break; 
    case 'R':
      motormode = 2;
      if (msgInt>0){
        StepSpeed = msgInt;
        defaultSpeed = StepSpeed; 
      }
      else{
        StepSpeed = defaultSpeed;  
      }
      motorOn = true; 
      Extruder.setCurrentPosition(0); 
      Extruder.moveTo(retractStep);
      Extruder.setSpeed(StepSpeed);
      digitalWrite(LED_PIN, HIGH);      
      break; 
    case 'D':
      //disable stepper 
      motormode = 0; 
      motorOn = false; 
      Extruder.disableOutputs();
      break;
    case 'E':
      //enable stepper
      motorOn = true; 
      Extruder.enableOutputs(); 
      break;
    case 'I': //initialize different variables
      switch(msgString[1]){
        case 'A':
        if (msgInt>0)Extruder.setAcceleration(msgInt); 
        break;
        case 'R':
        if (msgInt>0)retractStep=-msgInt; 
        break;
        case 'M':
        if (msgInt>0)Extruder.setMaxSpeed(msgInt); 
        break;
      }
  }
  runMotor(motormode);
}

void runMotor(uint8_t in){ 
  switch(in){
    case 0: //stop
      Extruder.stop();
      digitalWrite(LED_PIN, LOW);
      motorOn = false; 
      break;
    case 1://forward
      Extruder.runSpeed();  
      break;
    case 2://retract
      Extruder.run();
      if (Extruder.distanceToGo()==0){
        Extruder.stop();
        motormode = 0;
      }
      break;
  } 
}
