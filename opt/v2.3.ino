#include <avr/io.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
//#include <AFMotor.h>

// C/C++ 프로그래밍에서 정수(integer) 타입과 관련된 매크로 상수, 
//함수 등을 제공하는 헤더 파일(header file)입니다.
#include <inttypes.h>


// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

// Constants that the user passes in to the motor calls
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

//레지스터 제어 
#define LATCH 4
#define LATCH_DDR DDRB
#define LATCH_PORT PORTB

#define CLK_PORT PORTD
#define CLK_DDR DDRD
#define CLK 4

#define ENABLE_PORT PORTD
#define ENABLE_DDR DDRD
#define ENABLE 7

#define SER 0
#define SER_DDR DDRB
#define SER_PORT PORTB
//레지스터 제어 

// Arduino pin names for interface to 74HCT595 latch
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8


//pwm 조절 define
#define MICROSTEPS 16                       // 8 or 16

#define MOTOR12_64KHZ _BV(CS20)             // no prescale
#define MOTOR12_8KHZ _BV(CS21)              // divide by 8
#define MOTOR12_2KHZ _BV(CS21) | _BV(CS20)  // divide by 32
#define MOTOR12_1KHZ _BV(CS22)              // divide by 64

#define MOTOR34_64KHZ _BV(CS00)             // no prescale
#define MOTOR34_8KHZ _BV(CS01)              // divide by 8
#define MOTOR34_1KHZ _BV(CS01) | _BV(CS00)  // divide by 64

#define DC_MOTOR_PWM_RATE   MOTOR34_8KHZ    // PWM rate for DC motors
#define STEPPER1_PWM_RATE   MOTOR12_64KHZ   // PWM rate for stepper 1
#define STEPPER2_PWM_RATE   MOTOR34_64KHZ   // PWM rate for stepper 2


#define leftMotorNum 3
#define rightMotorNum 4
#define leftPwmFreq DC_MOTOR_PWM_RATE
#define rightPwmFreq DC_MOTOR_PWM_RATE

int latch_state=0;

void setSpeed(int motorNum,int speed);
void latch_tx();
inline void initPWM1(uint8_t freq);
inline void initPWM2(uint8_t freq);
inline void initPWM3(uint8_t freq);
inline void initPWM4(uint8_t freq);

inline void setPWM1(uint8_t s);
inline void setPWM2(uint8_t s);
inline void setPWM3(uint8_t s);
inline void setPWM4(uint8_t s);

void run(int motorNum,int cmd);

void motorConstruct();






//AF_DCMotor motor_L(1);              // 모터드라이버 L293D  1: M1에 연결,  4: M4에 연결
//AF_DCMotor motor_R(4); 

void setup() {
  Serial.begin(9600);              // PC와의 시리얼 통신속도
  Serial.println("Eduino Smart Car Start!");

  motorConstruct();

  // turn on motor

  setSpeed(leftMotorNum,200);
  run(leftMotorNum,RELEASE);
  setSpeed(rightMotorNum,200);
  run(rightMotorNum,RELEASE);
  
  //motor_L.setSpeed(230);              // 왼쪽 모터의 속도   
  //motor_L.run(RELEASE);
  //motor_R.setSpeed(230);              // 오른쪽 모터의 속도   
  //motor_R.run(RELEASE);
}

void loop() {
    int val1 = digitalRead(A0);    // 라인센서1
    int val2 = digitalRead(A5);    // 라인센서2   
    
      if (val1 == 0 && val2 == 0) {                   // 직진
       //motor_L.run(FORWARD); 
       //motor_R.run(FORWARD);
       
        setSpeed(leftMotorNum,200);
        setSpeed(rightMotorNum,200);
        run(leftMotorNum , FORWARD);
        run(rightMotorNum , FORWARD);
       
      }
      else if (val1 == 0 && val2 == 1) {              // 우회전
       //motor_L.run(FORWARD); 
       //motor_R.run(RELEASE);
        setSpeed(leftMotorNum,160);
        setSpeed(rightMotorNum,140);
        run(leftMotorNum , FORWARD);
        run(rightMotorNum , BACKWARD);
      }
      else if (val1 == 1 && val2 == 0) {              // 좌회전
        //motor_L.run(RELEASE); 
        //motor_R.run(FORWARD);
        setSpeed(leftMotorNum,140);
        setSpeed(rightMotorNum,160);
        run(leftMotorNum,BACKWARD);
        run(rightMotorNum , FORWARD);
      } 
      else if (val1 == 1 && val2 == 1) {              // 정지
        //motor_L.run(RELEASE); 
        //motor_R.run(RELEASE);
        
       // run(leftMotorNum , RELEASE);
      //  run(rightMotorNum , RELEASE);
      }           
}
    
void latch_tx() { //reset
  int i;

  //LATCH_PORT &= ~_BV(LATCH);
  digitalWrite(MOTORLATCH, LOW);

  //SER_PORT &= ~_BV(SER);
  digitalWrite(MOTORDATA, LOW);

  for (i=0; i<8; i++) {
    //CLK_PORT &= ~_BV(CLK);
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & _BV(7-i)) {
      //SER_PORT |= _BV(SER);
      digitalWrite(MOTORDATA, HIGH);
    } else {
      //SER_PORT &= ~_BV(SER);
      digitalWrite(MOTORDATA, LOW);
    }
    //CLK_PORT |= _BV(CLK);
    digitalWrite(MOTORCLK, HIGH);
  }
  //LATCH_PORT |= _BV(LATCH);
  digitalWrite(MOTORLATCH, HIGH);
}

inline void initPWM1(uint8_t freq){
    // use PWM from timer2A on PB3 (Arduino pin #11)
    TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
    TCCR2B = freq & 0x7;
    OCR2A = 0;
}
inline void setPWM1(uint8_t s){
    // use PWM from timer2A on PB3 (Arduino pin #11)
    OCR2A = s;
}

inline void initPWM2(uint8_t freq) {
      // use PWM from timer2B (pin 3)
    TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
    TCCR2B = freq & 0x7;
    OCR2B = 0;
    
    pinMode(3, OUTPUT);
    
}

inline void setPWM2(uint8_t s) {
    // use PWM from timer2A on PB3 (Arduino pin #11)
    OCR2B = s;
}

inline void initPWM3(uint8_t freq) {
    // use PWM from timer0A / PD6 (pin 6)
    TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on OC0A
    //TCCR0B = freq & 0x7;
    OCR0A = 0;
  
    pinMode(6, OUTPUT);
}

inline void setPWM3(uint8_t s) {
  
    // use PWM from timer0A on PB3 (Arduino pin #6)
    OCR0A = s;
}
inline void initPWM4(uint8_t freq){
    // use PWM from timer0B / PD5 (pin 5)
    TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0a
    //TCCR0B = freq & 0x7;
    OCR0B = 0;

    pinMode(5, OUTPUT);
}

inline void setPWM4(uint8_t s) {
    
    // use PWM from timer0A on PB3 (Arduino pin #6)
    OCR0B = s;
  
}

void setSpeed(int motorNum,int speed){
   switch (motorNum) {
  case 1:
    setPWM1(speed); break;
  case 2:
    setPWM2(speed); break;
  case 3:
    setPWM3(speed); break;
  case 4:
    setPWM4(speed); break;
  }

}


void run(int motorNum,int cmd) {
  int a, b;
  switch (motorNum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    latch_tx();
    break;
  }
}

void motorConstruct(){

  //afmotor controller default constructor start
  // setup the latch
  
  LATCH_DDR |= _BV(LATCH);
  ENABLE_DDR |= _BV(ENABLE);
  CLK_DDR |= _BV(CLK);
  SER_DDR |= _BV(SER);
  

  // pinMode(MOTORLATCH, OUTPUT);
  // pinMode(MOTORENABLE, OUTPUT);
  // pinMode(MOTORDATA, OUTPUT);
  // pinMode(MOTORCLK, OUTPUT);

  latch_tx();  // "reset"



  //ENABLE_PORT &= ~_BV(ENABLE); // enable the chip outputs!
  digitalWrite(MOTORENABLE, LOW);
  
  //afmotor controller default constructor end
  
  //left motor control 
  switch (leftMotorNum) {
    case 1:
      latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
      latch_tx();
      initPWM1(leftPwmFreq);
      break;
    case 2:
      latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
      latch_tx();
      initPWM2(leftPwmFreq);
      break;
    case 3:
      latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
      latch_tx();
      initPWM3(leftPwmFreq);
      break;
    case 4:
      latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
      latch_tx();
      initPWM4(leftPwmFreq);
      break;
  }
  
    //right motor control 
  switch (rightMotorNum) {
    case 1:
      latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
      latch_tx();
      initPWM1(rightPwmFreq);
      break;
    case 2:
      latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
      latch_tx();
      initPWM2(rightPwmFreq);
      break;
    case 3:
      latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
      latch_tx();
      initPWM3(rightPwmFreq);
      break;
    case 4:
      latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
      latch_tx();
      initPWM4(rightPwmFreq);
      break;
  }


  //motor construct end
  
  
}
