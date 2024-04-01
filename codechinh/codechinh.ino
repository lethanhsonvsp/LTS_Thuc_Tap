//{233, 215, 197, 179, 143};
float pTerm, iTerm, dTerm;
int error, previousError;
float kp, ki, kd;
float output;
int integral, derivative;
int motor1newSpeed;
int motor2newSpeed;

int sensorPins[5] = {A1, A2, A3, A4, A5};
int motorPins[6] = {12, 6, 13, 11, 3, 5};
int s;// max 120
int motor1Speed; //Default r
int motor2Speed; //Default l
unsigned long previousTime = 0;
unsigned long interval = 65000;  // Khoảng thời gian 1 giây (1000ms)
int a=0;

void setup() {
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }

  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
  }


  Serial.begin(9600);
}


void calculateError() {
  int sensorValues[5];
  sensorValues[0] = digitalRead(A1);
  sensorValues[1] = digitalRead(A2);
  sensorValues[2] = digitalRead(A3);
  sensorValues[3] = digitalRead(A4);
  sensorValues[4] = digitalRead(A5);
  
  int errorValues[13][5] = {
    {0, 1, 1, 1, 1},   // 6
    {0, 0, 1, 1, 1},   // 5
    {0, 0, 0, 1, 1},   // 4
    {0, 0, 0, 0, 1},   // 3
    {1, 0, 1, 1, 1},   // 2
    {1, 0, 0, 1, 1},   // 1
    {1, 1, 0, 1, 1},   // 0
    {1, 1, 0, 0, 1},   // -1
    {1, 1, 1, 0, 1},   // -2
    {1, 0, 0, 0, 0},   // -3
    {1, 1, 0, 0, 0},   // -4
    {1, 1, 1, 0, 0},   // -5
    {1, 1, 1, 1, 0},   // -6  100 30
  };
  
  for (int i = 0; i < 13; i++) {
    bool match = true;
    for (int j = 0; j < 5; j++) {
      if (sensorValues[j] != errorValues[i][j]) {
        match = false;
        break;
      }
    }
    if (match) {
      error = i - 6;
      break;
    }
  }
}
void pidCalculations()  {
  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  output  = pTerm + iTerm + dTerm;
  previousError = error;
}

void changeMotorSpeed() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;  // Cập nhật thời điểm trước đó
    a=0;
  }
  if(currentTime<14700&&currentTime>2500){
    
    s=90;
    kp = 11; ki = 0; kd = 11;
  } 
  else {
    kp = 16; ki = 0; kd = 16;
    s=115;
  }
  motor1Speed = s;
  motor2Speed = s;
  Serial.println(currentTime);
  // Kiểm tra nếu tất cả giá trị đọc từ cảm biến là 0
  if ((digitalRead(A1) == 0) && (digitalRead(A2) == 0) && (digitalRead(A3) == 0) && (digitalRead(A4) == 0) && (digitalRead(A5) == 0)) {
    // Tắt động cơ bằng cách gán tốc độ đầu ra là 0 và đảo chiều quay
    analogWrite(3, 0);
    analogWrite(5, 0);
    digitalWrite(12, 0);
    digitalWrite(6, 0);
    digitalWrite(13, 0);
    digitalWrite(11, 0);
  } else if ((digitalRead(A1) == 1) && (digitalRead(A2) == 1) && (digitalRead(A3) == 1) && (digitalRead(A4) == 1) && (digitalRead(A5) == 1)&&a<10&&currentTime>2000) {
    analogWrite(3, 110);
    analogWrite(5, 0);
    digitalWrite(12, 1);
    digitalWrite(6, 1);
    digitalWrite(13, 0);
    digitalWrite(11, 0);
    a++;
    a=a;
  } 
   else {
    // Change motor speed of both motors accordingly
    motor2newSpeed = motor2Speed + output;
    motor1newSpeed = motor1Speed - output;

    int speedThresholds[] = {156, 145, 134, 101, 112};//108
    // Vòng lặp để kiểm tra và điều chỉnh lại tốc độ động cơ nếu nó trùng với một ngưỡng
    for (int i = 0; i < sizeof(speedThresholds) / sizeof(speedThresholds[0]); i++) {
      if (motor1newSpeed == speedThresholds[i]) {
        motor1newSpeed = s;
      }
      if (motor2newSpeed == speedThresholds[i]) {
        motor2newSpeed = s;
      }
    }
    int speedThresholds1[] = {211, 195, 179, 163, 131, 147};
    // Vòng lặp để kiểm tra và điều chỉnh lại tốc độ động cơ nếu nó trùng với một ngưỡng
    for (int i = 0; i < sizeof(speedThresholds1) / sizeof(speedThresholds1[0]); i++) {
      if (motor1newSpeed == speedThresholds1[i]) {
        motor1newSpeed = s;
      }
      if (motor2newSpeed == speedThresholds1[i]) {
        motor2newSpeed = s;
      }
    }
    // Constrain the new speed of motors to be between the range 0-255
    motor2newSpeed = constrain(motor2newSpeed, 0, 255);
    motor1newSpeed = constrain(motor1newSpeed, 0, 255);
    // Set new speed and run motors in the forward direction
    analogWrite(5, motor2newSpeed);
    analogWrite(3, motor1newSpeed);
    digitalWrite(12, 1);
    digitalWrite(6, 1);
    digitalWrite(13, 0);
    digitalWrite(11, 0);
  }
}


void  loop() {
  calculateError();
  pidCalculations();
  changeMotorSpeed();
  Serial.print("motor1newSpeed: ");
  Serial.println(motor1newSpeed);
  Serial.print("motor2newSpeed: ");
  Serial.println(motor2newSpeed);
  
  //delay(1000);
}