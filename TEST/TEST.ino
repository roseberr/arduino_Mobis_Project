#include "LedControl.h"
#include <SoftwareSerial.h>
#include <AFMotor.h>
#define sensor A4

// 0 - WHITE, 1 - BLACK
// motor : 2 - left, 1 - right
// sensor : a0(val1) - left, a5(val2) - right
// motor_L speed :105 ,motor_R spedd:85
int left_val,right_val;
AF_DCMotor motor_L(3);
AF_DCMotor motor_R(4);
LedControl dot = LedControl(15,17,16,1);

byte heart[] = {
  B00000000,
  B01101100,
  B11111110,
  B11111110,
  B11111110,
  B01111100,
  B00111000,
  B00010000
};

byte x[] = {
  B11000011,
  B11100111,
  B01111110,
  B00111100,
  B00111100,
  B01111110,
  B11100111,
  B11000011
};


byte smile[] = {
  B00111100,
  B01000010,
  B10100101,
  B10000001,
  B10100101,
  B10011001,
  B01000010,
  B00111100
};  


byte forward[] = {
  B00011000,
  B00111100,
  B01111110,
  B11111111,
  B00011000,
  B00011000,
  B00011000,
  B00011000
};

byte left_curve[] = {
  B00000000,
  B00100000,
  B01100000,
  B11111110,
  B11111110,
  B01100110,
  B00100110,
  B00000110
};

byte right_curve[] = {
  B00000000,
  B00000100,
  B00000110,
  B01111111,
  B01111111,
  B01100110,
  B01100100,
  B01100000
};

byte backward[] = {
  B00011000,
  B00011000,
  B00011000,
  B00011000,
  B11111111,
  B01111110,
  B00111100,
  B00011000
};

void setup() {
  // setting for motor
  motor_L.setSpeed(230);
  motor_L.run(RELEASE);
  motor_R.setSpeed(200);
  motor_R.run(RELEASE);

  // setting for dot matrix
  dot.shutdown(0, false);
  dot.setIntensity(0,5);
  dot.clearDisplay(0);

  // setting for IR sensor
  
//  Serial.begin(9600);
//  pinMode(A0, INPUT);
//  pinMode(A5, INPUT);
}

void loop() {
  left_val=digitalRead(A0); 
  right_val=digitalRead(A5);

//  Serial.print(val1);
//  Serial.println(val2);
//  delay(1000);

  /*
  int sensorValue = analogRead(sensor);
  // blocked by something
  if (sensorValue > 100){
    motor_L.run(RELEASE);
    motor_R.run(RELEASE);
    showmatrix(x);
  }*/

  // go forward
  if (left_val == 0 && right_val == 0){
      motor_L.run(FORWARD);
      motor_R.run(FORWARD);
      showmatrix(forward);
      //delay(10);
      }

  // RIGHT CURVE
  else if (left_val == 0 && right_val == 1){
      motor_L.run(FORWARD);
      motor_R.run(BACKWARD);
      //motor_R.run(RELEASE); 
      showmatrix(right_curve);
      //delay(10);
  }

  // LEFT CURVE
  else if (left_val == 1 && right_val == 0){
      motor_L.run(BACKWARD);
      //motor_L.run(RELEASE);
      motor_R.run(FORWARD);

      showmatrix(left_curve);
      //delay(10);
  }
  else{
    showmatrix(smile);
    //delay(50);
  }
  //delay(50);

  /*
  else if (left_val == 1 && right_val == 1){
      motor_L.run(RELEASE);
      motor_R.run(RELEASE);
     // delay(200);
  }*/
}

void showmatrix_left(byte arr[]){
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr[i]);
  }
  for(int i=0; i<8; i++){
    if((arr[i] & 0b10000000)>0){
      arr[i]=arr[i]<<1; // shift left
      arr[i]+=B00000001;  // append bit 1 to right
    }
    else{
      arr[i] = arr[i]<<1; 
    }
  }
}

void showmatrix_right(byte arr[]){
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr[i]);
  }
  for(int i=0; i<8; i++){
    if((arr[i] & 0b00000001)>0){
      arr[i]=arr[i]>>1;
      arr[i]+=B10000000;
    }
    else{
      arr[i] = arr[i]>>1;
    }
  }
}


void showmatrix_up(byte arr[]){
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr[i]);
  }

  for(int i=0; i<8; i++){
     arr[i] = arr[(i+1)%8];
  }
}

/*
void showmatrix_down(byte arr[]){
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr[i]);
  }

  for(int i=0; i<8; i++){
     arr[i] = arr[(i+7)%8];
  }
}*/

void showmatrix(byte arr[]){
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr[i]);
  }
  delay(10);
}
