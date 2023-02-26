#define sensor A4 // IR input port
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // Start serial

  byte siren_1[] = {
  B00111100,
  B01000010,
  B10100101,
  B10011001,
  B10100101,
  B10000001,
  B11111111,
  B11111111
};

byte siren_2[] = {
  B00111100,
  B01000010,
  B10011001,
  B10111101,
  B10011001,
  B10000001,
  B11111111,
  B11111111
};
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sensorValue > 100){
    showmatrix_siren(siren_1, siren_2) // 해당 분기문 내에서 계속 siren_1과 siren_2가 반복되도록 수정 필요
    motor_L.run(FORWARD);
    motor_R.run(BACKWARD); // 전방 차량 감지로 인한 우측 조향
    delay(300); // 현재 모터 속도에 따라 몇도 틀어지는지 직접 실험 필요 
    motor_R.run(FORWARD); // 어느 정도 각도 틀었으면 직진
    delay(300);
    motor_L.run(BACKWARD); // 다시 라인 진입 위한 좌측 조향
    delay(300);
    motor_L.run(FORWARD); // 라인으로 직진
    /*
    고민해야할 것이 여기 아래 코드인데 어떻게 다시 라인으로 복귀를 하냐
    우선은 일정 시간 직진 후에 다시 loop문 들어가서
    라인을 탐색하는 방식으로 했는데 이 경우 마지막 라인의
    delay 시간을 경험을 통해 계산을 해야함.
    */
    //delay(300); // 하드코딩
    while(true) {
      // 라인 우측에서 약 45도 각도로 진입하며 계속 직진할 때
      // 오른쪽 센서가 라인, 왼쪽이 라인 왼쪽 흰바탕이라면
      // 센서가 라인 내에 진입 했다고 판단하고 while문 탈출
      // 이후 바로 continue로 다시 loop()문 진입하여 주행
      left_val=digitalRead(A0); 
      right_val=digitalRead(A5);
      if (left_val==0 & right_val==1) {
        break;
      }
    }
    // 위의 코드가 딜레이로 인해 라인 안으로 들어가버린다면 아래로 수정
    /*
    while(true) {
      // 왼쪽만 1이 되도 딜레이 감안하여 while문 탈출
      left_val=digitalRead(A0); 
      right_val=digitalRead(A5);
      if (left_val==1) {
        break;
      }
    }
    */
    continue;
  }

  void showmatrix_siren(byte arr1[], byte arr2[]){ // while문 없어도 계속 두
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr1[i]);
  }
  delay(100);
  for(int i=0; i<8; i++){
    dot.setRow(0,i,arr2[i]);
  }
}