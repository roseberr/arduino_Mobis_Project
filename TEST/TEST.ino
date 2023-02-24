#include "LedControl.h"

#define sensor A4

LedControl dot = LedControl(12,10,11,1);

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
  B00011000,
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
  B00011000,
  B01111110,
  B00111100,
  B00011000
};

void setup(){
  dot.shutdown(0, false);
  dot.setIntensity(0,5);
  dot.clearDisplay(0);
  Serial.begin(9600); // Start serial
}

void loop(){
  Serial.print("1.sensor :");
  int sensorValue = analogRead(sensor);
  Serial.println(sensorValue);
  //delay(1000);
  
  showmatrix_right(right_curve);
  delay(200);
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

/*
void showmatrix_up(byte arr[]){
  int n = 0;
  for(int i=n; i<8; i++){
    dot.setRow(0,i,arr[i]);
  }
  
}*/

void blinkmatrix(byte arr[]){
  delay(1000);
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr[i]);
  }
  delay(500);
  dot.clearDisplay(0);
}
