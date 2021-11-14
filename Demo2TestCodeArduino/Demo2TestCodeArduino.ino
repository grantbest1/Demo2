//Include motorshield library
#include "DualMC33926MotorShield.h"
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>

DualMC33926MotorShield md;

//RIGHT MOTOR IS M1
//LEFT MOTOR IS M2

#define SLAVE_ADDRESS 0x04
#define clk_left 2
#define clk_right 3
#define dt_left 5
#define dt_right 6

float master_time;
double tNew;
double tOld;

int moveFlag = 0;
bool moving = false;


//left variables
int counter_left = 0;
int oldCount_left = 0;
int newCount_left = 0;
int deltaCount_left = 0;
float rad_left;
int currentState_left;
int lastState_left;
String currentDir_left;
double tNew_left;
double tOld_left;
double deltaT_left;
int speedM2 = 0;


//right variables
int counter_right = 0;
int oldCount_right = 0;
int newCount_right = 0;
int deltaCount_right = 0;
float rad_right;
int currentState_right;
int lastState_right;
String currentDir_right;
double tOld_right;
double tNew_right;
double deltaT_right;
int speedM1 = 0;

float r = 0.25; // radius of wheel as a fraction of one foot
float b = 0.91666; // distance between wheels as a fraction of one foot

// angular velocity
double thetaDot_left; 
double thetaDot_right;

bool flag = false;
float dist = 0;
double phi = 0; // current angle from straight


//int number = 0;
int tate = 0;
int offset = 0;
char data[64] = "";
int i = 0;
float angle = 0.0;
float distance = 0.0;
char dis[64] = "";
int sign = 1;
void recieveData(int);
void sendData();
char tape[64];
int halt = 0;

//if the motor shield has a fault it will tell the user
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}


void setup() {
  pinMode(3, OUTPUT);
  Serial.begin(9600); // start serial for output
  //initiaze encoder inputs
  pinMode(clk_left, INPUT);
  pinMode(clk_right, INPUT);
  pinMode(dt_left, INPUT);
  pinMode(dt_right, INPUT);
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  md.init();
  tOld_left = millis();
  tOld_right = millis();


  lastState_left = digitalRead(clk_left);
  lastState_right = digitalRead(clk_right);

  
  attachInterrupt(digitalPinToInterrupt(2), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), updateRightEncoder, CHANGE);
  

  Serial.println("Ready!");
}



void loop() {
  static state_t state = ID;

  //prints and does calculations
  if(flag == true){
    dist = (counter_left + counter_right)*1.57/(2*800);
    phi = float((-36.0*(counter_right))/(157.0));
      Serial.print(counter_right);
      Serial.print(" ");
      Serial.print(counter_left);
      Serial.print(" ");
      Serial.print(phi);
      Serial.print(" ");
      Serial.print(moveFlag);
      Serial.println();
      flag = false;
  }

  //case statement
  switch(state){
    case ID:
      //run comp vision
      moving = false;
      if (tape == 1){
        state = TAPE_FOUND;
        counter_left = 0;
        counter_right = 0;
      }else{
        rotate(10);
      }
      break;
    case TAPE_FOUND:
      if(angle != 0.0){
        state = ROTATE;
      }else{
        state = ID;
      }
      break;
    case ROTATE:
      if(phi!=angle){
        rotate(angle);
      }else{
        state = CHECK_ANGLE;
      }
      break;
    case CHECK_ANGLE:
      if(phi<=angle+2 && phi>=angle-2){
        state = ROTATE;
      }else{
        state = MOVE;
      }
      break;
    case MOVE:
      if(moving == false){
        moving = true;
        counter_left = 0;
        counter_right = 0;
      }
      if(tape == 1){
        moveDistance(1);
      }else{
        //done;
      }
      break;
  }
}



// callback for received data
void receiveData(int byteCount) {
  Wire.read();
  while (Wire.available()) {
    data[i] = Wire.read();
    //Breaks off angle when space is found
    if (data[i] == 32) {
      break;
    }    
    i++;
  }
  i = 0;
  while (Wire.available()) {
    //Defines tape as either one or zero (one if found)
    tape[i] = Wire.read();
    i++;
  }
  i = 0;
  angle = atof(data);
  //distance = atof(dis);
  Serial.println(angle);
  Serial.print("Distance is: ");
  Serial.println(distance);


  Wire.read();
  Wire.flush();
}


void rotate(float desAngle){
  if(phi<desAngle+1 && phi>desAngle-1){
    speedM1 = 0;
    speedM2 = 0;
    moveFlag++;
  }else if(phi<desAngle){
    speedM1 = -65;
    speedM2 = 65;
  }else if(phi>desAngle){
    speedM1 = 65;
    speedM2 = -65;
  }
    md.setM1Speed(speedM1);
    md.setM2Speed(speedM2);
}

void moveDistance(float desDist){
  if(dist<desDist){
      speedM1=100;
      speedM2=100;
    md.setM1Speed(speedM1);
    md.setM2Speed(speedM2);
  }else{
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
}


//this is the ISR for the left encoder
void updateLeftEncoder(){
  currentState_left = digitalRead(clk_left);


  if(currentState_left != lastState_left && currentState_left == 1){
    tNew_left = micros();
    deltaT_left = (tNew_left - tOld_left);
    deltaCount_left = newCount_left - oldCount_left;
    //thetadot is the angular velocity
    thetaDot_left = (deltaCount_left*1000000*2*3.1415)/((deltaT_left)*800);

    
    
    oldCount_left = counter_left;
    if(digitalRead(dt_left) != currentState_left){
      counter_left ++;
      currentDir_left = "CCW";
    }else{
      
      counter_left --;
      currentDir_left = "CW";
    }
    rad_left = (counter_left*2*PI)/800;
    newCount_left = counter_left;
    tOld_left = micros();
    flag = true;
    
  }
  lastState_left = currentState_left;
  
}


//this is the ISR for the right encoder
void updateRightEncoder(){
  currentState_right = digitalRead(clk_right);


  if(currentState_right != lastState_right && currentState_right == 1){
    tNew_right = micros();
    deltaT_right = tNew_right - tOld_right;
    deltaCount_right = newCount_right-oldCount_right;
    //thetadot is the angular velocity
    thetaDot_right = (deltaCount_right*1000*2*PI)/((deltaT_right)*800);
      
    
    oldCount_right = counter_right;
    if(digitalRead(dt_right) != currentState_right){
      counter_right --;
      currentDir_right = "CCW";
    }else{
      counter_right ++;
      currentDir_right = "CW";
    }
    rad_right = (counter_right*2*PI)/800;
    newCount_right = counter_right;
    tOld_right=micros();
    flag = true;
    
  }
  lastState_right = currentState_right;
  
}
