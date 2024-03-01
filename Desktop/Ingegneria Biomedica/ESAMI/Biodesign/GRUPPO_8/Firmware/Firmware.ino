//LIBRARIES DEFINITION

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Math.h>


//FS1012-1001-LQ FLOW SENSOR PIN DEFINITION

const int pinFlowsensor = A2;

//Parameters for sensor output conversion

const int lsb = 5/(2^10); //0.0049 [V]
const int gain = 40;

//FLOW SENSOR PARAMETERS

const float t_response = 0.005;// 5 [ms] -> 0.005 [s] 
const float k = -694.3; //curve fitting slope
const float offset = 93.3; //curve fitting offset

const float v_fs_min = 0; //[mL] 
const float v_fs_max = 0.8; //[mL] 

//MOTOR PIN DEFINITION

const int pinDir = 8; //PIN direction of motor revolution (clockwise and counterclockwise)
const int pinStep = 9; //PIN PWM to control the motor

//MANUAL MOTOR CONTROL PARAMETERS DEFINITION

int microstepping_number; 
int numStepMotor; //numStepMotor * microstepping_number -> 200 step for a motor revolution [datasheet]
long speed; //desired_speed * microstepping_number [rpm]

long delaySteptoStep; //delay in [us] (60*pow(10,6))
long t; //=delaySteptoStep/2 -> time between a raising edge and a falling edge

//AUTOMATIC MOTOR CONTROL PARAMETERS

float v_des; //[mL]
float flow_rate; //[ml/s] 

//The proposed device has a Flow_rate_min = 0 mL/s  and a Flow_rate_max = 0.8 mL/s (properly chosen referring to the literature). 
//These values are in compliance with typical flow rate values used in real infusion pumps for druge delivery


//SYRINGE PARAMETERS DEFINITION

float v_resid = 5; //[mL] -> Supposing to use a 5 mL syringe

int screw_feed; //screw_feed = screw_pitch * num_screw_principles [mm] -> screw_feed = 8 [mm] for a motor revolution
const long pi = 3.1415926525; 
const float diameter = 13; //[mm]
const long surface = pi*pow(diameter/2,2); //Syringe surface

//PIN BUZZER DEFINITION 
const int pinBuzzer = 10;

//PIN MICROSWITCH DEFINITION

const int microSwitch = 2;

//INTERRUPT VARIABLE DEFINITION
volatile bool Switch_state = HIGH;

//PIN KEYPAD DEFINITION
const byte ROWS = 4; //Number of Keypad rows
const byte COLS = 4; //Number of Keypad columns

//Virtual Keypad creation
char hexaKeys[ROWS][COLS] = {

  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}

};

byte rowPins[ROWS] = {0, 1, 2, 3};
byte colPins[COLS] = {4, 5, 6, 7};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

//LCD DEFINITION

LiquidCrystal_I2C lcd(0x27,20,4);

//MOTOR TURNING OFF FUNCTION

void motorDisable() {

  digitalWrite(pinStep,LOW);

}

//FUNCTION TO MAKE A MOTOR STEP 

void makeStep(){

  digitalWrite(pinStep, HIGH);
  delayMicroseconds(t);
  digitalWrite(pinStep,LOW);
  delayMicroseconds(t);

}

//MOTOR MANUAL CONTROL FUNCTION FOR SYRINGE PLACEMENT (MICROSWITCH)

void toPlace(){

  microstepping_number = 2;
  
  speed = 10 * microstepping_number; // speed_desired = 10 [rpm] arbitrary chosen 
  long delayStepToStep=(60*pow(10,6)) / (speed * 200); 
  long t = delayStepToStep/2; 

 

  while (Switch_state == HIGH) {

    digitalWrite(pinDir, HIGH); //clockwise revolution (backward)
    makeStep();

    lcd.clear(); 
    lcd.setCursor(1,0); 
    lcd.print("Syringe Placement");
    lcd.setCursor(3,2);
    lcd.print("Please Wait..");
    delay(2000);

  }

  motorDisable();

  lcd.clear(); 
  lcd.setCursor(6,0); 
  lcd.print("Ready to");
  lcd.setCursor(6,2);
  lcd.print("Infuse");
  delay(2000);

}
  

//AUTOMATIC VOLUME INFUSION PARAMETERS SETTING FUNCTION

float settings(){

  lcd.clear(); 
  lcd.setCursor(2,0); 
  lcd.print("Available Volume:");
  lcd.setCursor(6,2);
  lcd.print(v_resid);
  lcd.setCursor(13,2);
  lcd.print("mL")

  delay(2000);

  lcd.clear();
  lcd.setCursor(4,0); 
  lcd.print("Volume to");
  lcd.setCursor(4,1); 
  ldc.print("Infuse: ")
  delay(2000);
  
  //VOLUME_TARGET TO INFUSE DEFINITION

  char commandKey1 = customKeypad.getKey(); //Returns the key that is pressed, if any. This function is non-blocking.
  delay(1000);
  char commandKey2 = customKeypad.getKey(); //Returns the key that is pressed, if any. This function is non-blocking.
  char command_volume[] = {commandKey1, '.', commandKey2};
  v_des = atof(command_volume);
  
  lcd.clear();
  lcd.setCursor(4,1);
  lcd.print(v_des);
  lcd.setCursor(11,1);
  lcd.print("mL");
  delay(2000);

  if(v_des <= v_resid) {
    
    //VOLUME TO INFUSE CONFIRM

    lcd.clear();
    lcd.setCursor(2,0); 
    lcd.print("Please, digit '*'");
    lcd.setCursor(2,2); 
    ldc.print("to confirm")
    delay(2000);

    char commandKey = customKeypad.getKey(); //Returns the key that is pressed, if any. This function is non-blocking

    if( (commandKey == '*'){

      lcd.clear();
      lcd.setCursor(6,0); 
      lcd.print("Volume");
      lcd.setCursor(6,1); 
      ldc.print("Confirmed");
      delay(2000);

      v_resid = v_resid - v_des;
  
  } 
  }
  
  else {

    //BUZZER ALERT SOUND

    tone(pinBuzzer, 988,100);    // Nota B5
    delay(100);
    tone(pinBuzzer,494,850);    // Nota B4
    delay(800);
    noTone(pinBuzzer);


    lcd.clear();
    lcd.setCursor(4,0); 
    lcd.print("ERROR");
    lcd.setCursor(4,2); 
    ldc.print("Infusion");
    lcd.setCursor(4,3);
    lcd.print("Not Possible");
    delay(2000);

    v_des = settings();

  }

  //t_min & t_max definition in order to guarantee FS2012 correct working for a v_des chosen [see Attachment_Report]

  float t_min = t_response;
  float t_max = (1 * v_des)/ v_fs_max;

  lcd.clear();
  lcd.setCursor(2,0); 
  lcd.print("Please, set");
  lcd.setCursor(2,2); 
  ldc.print("Infusing time")
  delay(2000);

  char commandKey_time1 = customKeypad.getKey(); //Returns the key that is pressed, if any. This function is non-blocking.
  delay(1000);
  char commandKey_time2 = customKeypad.getKey(); //Returns the key that is pressed, if any. This function is non-blocking.
  char command_time[] = {commandKey_time1, '.', commandKey_time2};
  t_infusion = atof(command_time);

  if (t_min <= t_infusion <= t_max){

    flow_rate = v_des/t_infusion; //portare in us

    lcd.clear();
    lcd.setCursor(6,0); 
    lcd.print("Infusion");
    lcd.setCursor(6,1); 
    ldc.print("Confirmed");
    delay(2000);

  }

  else {

    tone(pinBuzzer, 988,100);    // Nota B5
    delay(100);
    tone(pinBuzzer,1319,850);    // Nota E6
    delay(800);
    noTone(pinBuzzer);


    lcd.clear();
    lcd.setCursor(4,0); 
    lcd.print("ERROR");
    lcd.setCursor(4,2); 
    ldc.print("Infusing time");
    lcd.setCursor(4,3);
    lcd.print("Not Consistent");
    delay(2000);

    v_des = settings();

  }

  return v_des;
  return flow_rate;
}

//FUNCTION FOR VOLUME INFUSION

void infusion(float v_des, float flow_rate) {

  microstepping_number = 2; 
  screw_feed = 8;
  
  //formula portata da mL/s a rpm

  float numStepMotor = (1000*v_des*200*microstepping_number)/(surface*screw_feed); 
  numStepMotor = round(numStepMotor);

  speed_ms = (flow_rate*microstepping_number)/surface;//[m/s]

  //speed[rpm]=60/(2*pi*(diameter/2))*speed[m/s]

  speed = speed_ms*(60/2*pi*(diameter/2));
  
  long delayStepToStep = (60*pow(10,6)) / (speed * 200); 
  long t = delayStepToStep/2; 

  float x = 0; //Variable used to increase motor's step during the automatic volume infusion

  while (x<numStepMotor){

    float v_fs=0;

    if (v_fs < v_des){

      digitalWrite(pinDir, LOW); //counterclockwise rotation
      makeStep();

      output = analogRead(pinFlowsensor);

      float output_mV = ((output*lsb)/gain)*1000;

      float flow_sensor_rate = abs(((output_mV-offset)/k)*(1000/60)); //[L/min] -> 10^3 mL/60s
      v_fs =+ flow_sensor_rate*t_response;
      x++;

      lcd.clear();
      lcd.setCursor(4,2); 
      ldc.print("Infusing...");
      delay(2000);

    }
    else {

      break;
      motorDisable();

    }

  }

  //ALERT SOUND FOR INFUSION STOP

  tone(pinBuzzer, 988,100);    // Nota B5
  delay(100);
  tone(pinBuzzer,1319,850);    // Nota E6
  delay(800);
  noTone(pinBuzzer);

  lcd.clear();
  lcd.setCursor(4,0); 
  lcd.print("Infusion");
  lcd.setCursor(4,2);
  lcd.print("Completed");
  delay(2000);

  lcd.clear();
  lcd.setCursor(4,0); 
  lcd.print("Volume");
  lcd.setCursor(4,1);
  lcd.print("Infused: ");
  lcd.setCursor(4,3);
  lcd.print(v_fs);
  lcd.setCursor(11,3);
  lcd.print("mL");
  delay(2000);

  if (v_resid > 0) {
    
    //NEW VOLUME INFUSION SETTING

    lcd.clear();
    lcd.setCursor(1,0); 
    lcd.print("To set a new volume");
    lcd.setCursor(1,1);
    lcd.print("please, press 'D'");
    lcd.setCursor(1,2);
    lcd.print("To stop infusing");
    lcd.setCursor(1,3);
    lcd.print("please, press 'C'");
    delay(2000);

    char commandKey = customKeypad.getKey();

    if(commandKey == 'D'){

      settings();

    }

    else {

      motorDisable();

      lcd.clear();
      lcd.setCursor(4,0); 
      lcd.print("Please, remove");
      lcd.setCursor(4,1);
      lcd.print("the syringe");
      delay(1000);

      toPlace();

    }

  }

  else{
    
    motorDisable();

    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("No Volume");
    lcd.setCursor(1,2);
    lcd.print("Available");
    delay(2000);

    lcd.clear();
    lcd.setCursor(4,0); 
    lcd.print("Please, remove");
    lcd.setCursor(4,1);
    lcd.print("the syringe");
    delay(1000);

    toPlace();


  }

}

//INTERRUPT FUNCTION

void stop_stepper(){

  Switch_state != Switch_state;
  
}

void setup() {

  //MOTOR PIN DEFINITION AS OUTPUT -> This allows to control the motor state (giving a PWM or not)
  
  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);
  
  pinMode(microSwitch, INPUT_PULLUP); //Inizialitation of microSwitch pin as an internal pull-up input. It doesn't need to use an external pull-up/pull-down resistor

  //BUZZER PIN DEFINITION AS OUTPUT -> This allows to controlo the buzzer state (high or low)

  pinMode(pinBuzzer, OUTPUT);

  //LCD INITIALIZATION

  lcd.init(); 
  lcd.backlight(); //This function allows to turn the backlight on

  //INTERRUPT DEFINITION

  attachInterrupt(digitalPinToInterrupt(2), stop_stepper, FALLING); //DETECT THE FALLING EDGE (PIN FROM HIGH --> LOW)

  //STARTING WITH MOTOR OFF
  motorDisable();

}

void loop() {

  float v_des = settings();

  //ALERT SOUND FOR INFUSION START
  
  tone(pinBuzzer, 330,100);    // Nota E4
  delay(100);
  tone(pinBuzzer,587,850);    // Nota D5
  delay(800);
  noTone(pinBuzzer);


  infusion(v_des); //INFUSING..

}
